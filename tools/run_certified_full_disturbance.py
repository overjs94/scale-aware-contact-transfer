"""Execute complete tasks with a certified stationary load interval after lift.

The original accepted motion intervals are retained and shifted in time. An
explicit stationary conic-QP hold is inserted; the object remains dynamic.
Zero-load controls and challenged runs share the same insertion and controller.
"""
import argparse
from concurrent.futures import ThreadPoolExecutor
from copy import copy,deepcopy
from dataclasses import replace
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import time

import numpy as np
from scipy.spatial.transform import Rotation

from contact_transfer.certified_disturbance import physical_hold,schedule_membership
from contact_transfer.full_assessment import assess,measured_contact_flags
from contact_transfer.full_execution import FullEpisode,run_full_episode
from contact_transfer.full_recording import write_json
from contact_transfer.full_scene import build_scene
from contact_transfer.full_static import stationary_interval
from contact_transfer.full_trajectory import interval_contacts
from export_full_recording import load_recorded_plan


def inserted_hold(original,k,directory,duration=6.):
    p=copy(original.problem);p.directory=Path(directory);p.directory.mkdir(parents=True,exist_ok=True)
    goals=deepcopy(list(p.task_definition.keyframes));insert=deepcopy(goals[k]);insert.update(id='certified_load_hold',time=goals[k]['time']+duration)
    for g in goals[k+1:]:g['time']+=duration
    goals.insert(k+1,insert);p.task_definition=replace(p.task_definition,keyframes=tuple(goals))
    if hasattr(p,'_case'):del p._case
    key=deepcopy(original.keyframes[k])
    if key.environment_reactions:raise ValueError('Choose a pose before fixture or inter-object support')
    contacts=tuple(c for c in interval_contacts(p,k,key,16) if c.robot_index is not None)
    hold,contacts,_=stationary_interval(p,key,key,contacts,duration,16)
    keys=(*original.keyframes[:k+1],deepcopy(key),*original.keyframes[k+1:])
    rs=(*original.trajectories[:k],hold,*original.trajectories[k:])
    cs=(*original.contacts_by_interval[:k],contacts,*original.contacts_by_interval[k:])
    episode=FullEpisode(p,keys,rs,cs)
    return episode,float(episode.preparation+episode.offsets[k])


def hold_response(episode,trace,scene,k,start):
    obj=episode.problem.task_definition.object;q=episode.keyframes[k].q[episode.problem.system.object_q]
    mask=(trace.time>=start)&(trace.time<=start+6.)
    frames=list(obj.names) if hasattr(obj,'names') else ['base','moving'] if episode.problem.task_definition.id=='box' else [obj.tree.root]
    pe=np.zeros(len(trace.time));re=pe.copy()
    for frame in frames:
        T=obj.fk(q,frame);index=trace.metadata['object_names'].index(scene.object_frames[frame]);actual=trace.object_pose[:,index]
        pe=np.maximum(pe,np.linalg.norm(actual[:,:3]-T[:3,3],axis=1));re=np.maximum(re,(Rotation.from_matrix(T[:3,:3]).inv()*Rotation.from_quat(actual[:,3:])).magnitude())
    contacts=measured_contact_flags(episode,trace,scene)
    before=(trace.time>=start+1.)&(trace.time<start+2.);after=(trace.time>=start+5.)&(trace.time<=start+6.)
    idx=np.flatnonzero(mask)[::max(1,int(.02/trace.metadata['timestep']))]
    return dict(preload_reference_tracking_passed=bool(pe[before].max()<=.008 and re[before].max()<=.06),
        preload_all_contact_fraction=float(contacts[before].all(axis=1).mean()),
        hold_peak_position_error_m=float(pe[mask].max()),hold_peak_rotation_error_rad=float(re[mask].max()),
        hold_recovered=bool(pe[after].max()<=.008 and re[after].max()<=.06),
        time_s=trace.time[idx].tolist(),position_error_m=pe[idx].tolist(),rotation_error_rad=re[idx].tolist(),supporting_contacts=contacts[idx].sum(axis=1).tolist(),
        mathematical_divergence_claimed=False)


def run(args,task):
    protocol_path=args.probes/task/'protocol.json';started=time.monotonic()
    while not protocol_path.exists():
        if time.monotonic()-started>7200:raise TimeoutError('Static probe protocol did not become available')
        time.sleep(2)
    protocol=json.loads(protocol_path.read_text());k=protocol['optimized_keyframe_index'];out=args.output/task
    coefficient=np.asarray(protocol['cases']['on_interior']);rho=protocol['rho_on_optimized_and_independently_checked'];results=[]
    matched={}
    for mode,variant in [('on','full'),('off','no_target_robustness')]:
        p,keys,rs,cs,parent=load_recorded_plan(args.root,task,variant,model_directory=out/mode/'models')
        original=FullEpisode(p,keys,rs,cs);episode,start=inserted_hold(original,k,out/mode/'reference')
        scene=build_scene(episode.problem,out/mode/'reference-scene.xml')
        matched[mode]=dict(scene=hashlib.sha256(scene.path.read_bytes()).hexdigest(),controller=asdict(p.task_definition.configuration.execution),q=keys[k].q[p.system.object_q].tolist())
        if mode=='off' and matched['on']!=matched['off']:raise RuntimeError('Physical scene, controller and object pose must match')
        for case,vector in [('nominal',np.zeros_like(coefficient)),('challenge',coefficient)]:
            pulses=physical_hold(episode.problem,episode.keyframes[k],scene,vector,start=start+2.) if np.linalg.norm(vector) else ()
            certificate=schedule_membership(episode.problem,episode.keyframes[k].q[episode.problem.system.object_q],pulses,rho,scene)
            if not certificate['inside']:raise RuntimeError('Refusing an outside-ON validation wrench')
            folder=out/mode/case
            trace,controller,native,geometry=run_full_episode(episode,folder/'native',disturbances=pulses)
            report=assess(episode,trace,controller,native,geometry=geometry);analysis=hold_response(episode,trace,native,k,start)
            report.update(certified_full_disturbance=dict(mode=mode,case=case,optimized_keyframe_index=k,reference_hold_start_s=start,
                stationary_hold_inserted_s=6.,original_accepted_motion_intervals_preserved=True,
                inside_on_reference_certificate=certificate,response=analysis,protocol=protocol,
                reference_pose_only_certificate=True,dynamic_robustness_certificate=False),
                full_paper_pipeline_executed=False)
            write_json(folder/'result.json',report)
            results.append(dict(mode=mode,case=case,execution_passed=report['execution_passed'],task_metrics=report['task_metrics'],hold_response=analysis,inside_on_certificate=certificate,
                scene_sha256=trace.metadata['scene_sha256'],controller_configuration=trace.metadata['controller_configuration'],native_trace_sha256=hashlib.sha256((folder/'native/execution.npz').read_bytes()).hexdigest()))
            write_json(out/'summary.json',dict(task=task,results=results))
            print(task,mode,case,report['execution_passed'],analysis['hold_peak_position_error_m'],flush=True)
            if args.web is not None and case=='challenge':
                from contact_transfer.full_view import export_full
                export_full(episode,trace,report,native,args.web,dataset_id=f'research_loaded_{task}_{mode}',
                    recording_metadata=dict(title=f'{task.title()} · full task with inside-margin load · {mode.upper()}',
                        scenario='certified_full_sequence',full_task_recording=True,comparison_recording=True,study_kind='full_target_robustness',parent_task=task,
                        description='Complete accepted motion with an inserted six-second stationary load interval after lift. The load is inside the optimized ON static box at that posture; dynamic closed-loop robustness is measured, not certified.'))


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--root',type=Path,default=Path('outputs/research-study/full-common'))
    p.add_argument('--probes',type=Path,default=Path('outputs/research-study/probes-v1'));p.add_argument('--output',type=Path,default=Path('outputs/research-study/full-load-v1'))
    p.add_argument('--tasks',nargs='+',default=['panel','box','assembly']);p.add_argument('--workers',type=int,default=3);p.add_argument('--web',type=Path)
    args=p.parse_args()
    if len(args.tasks)>1:
        def launch(task):
            log=args.output/(task+'.log');log.parent.mkdir(parents=True,exist_ok=True)
            cmd=[sys.executable,'-u',__file__,'--root',str(args.root),'--probes',str(args.probes),'--output',str(args.output),'--tasks',task]
            if args.web is not None:cmd+=['--web',str(args.web)]
            with log.open('w') as handle:result=subprocess.run(cmd,stdout=handle,stderr=subprocess.STDOUT)
            return dict(task=task,exit_code=result.returncode,log=str(log))
        with ThreadPoolExecutor(max_workers=args.workers) as pool:rows=list(pool.map(launch,args.tasks))
        write_json(args.output/'workers.json',rows)
    else:run(args,args.tasks[0])


if __name__=='__main__':main()
