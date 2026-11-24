"""Matched native robustness probes of accepted optimized mid-task keyframes.

One pose per full task is probed separately from the full-motion recording.
Direction selection uses a declared finite set and independent static margins
before any native probe runs. All static candidates and native failures remain.
"""
import argparse
from concurrent.futures import ThreadPoolExecutor
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import time
from types import SimpleNamespace

import numpy as np
from scipy.spatial.transform import Rotation

from contact_transfer.certified_disturbance import probe_episode,physical_hold,schedule_membership
from contact_transfer.full_assessment import assess,measured_contact_flags
from contact_transfer.full_execution import FullEpisode,run_full_episode,reference_geometry
from contact_transfer.full_recording import write_json
from contact_transfer.full_scene import build_scene
from contact_transfer.full_validation import numeric_contact
from contact_transfer.generalized_conic import fixed_generalized_margin
from contact_transfer.trajectory import Contact
from export_full_recording import load_recorded_plan


def directional_margin(episode,direction):
    p=episode.problem;key=episode.keyframes[0];sys=p.system
    load=p.task_load(key.q[sys.object_q]);delta=load.E@load.D@direction
    ray=SimpleNamespace(nominal=load.nominal,E=delta[:,None],D=np.eye(1),vertices=np.ones((1,1)))
    maps=[];tmaps=[];caps=[]
    for i,(f,frame,cap,point,R) in enumerate(zip(p.task_definition.features,p.frames,p.capabilities,key.positions,key.rotations)):
        c=Contact(f.id,point,R[:,2],cap,robot_index=i,frame=frame,object_frame=f.frame,rotation_object=R)
        _,J,_,_=numeric_contact(p,c,key.q);X=np.c_[np.zeros((sys.nv,3)),J.T] if cap.kind=='P' else J.T
        maps.append(-X[sys.object_v]);tmaps.append(np.asarray(sys.B).T@X);caps.append(cap)
    gravity=np.concatenate([r.gravity(key.q[s]) for r,s in zip(p.robots,sys.q_slices)])
    result=fixed_generalized_margin(maps,ray,caps,torque_gravity=gravity,torque_maps=tmaps,effort=sys.effort)
    return dict(certified=result['certificate'],maximum_coefficient=result['rho'] if result['certificate'] else None,status=result['status'],residual=result['primal_residual'])


def response(episode,trace,scene,pulses):
    p=episode.problem;desired=episode.keyframes[0].q[p.system.object_q];obj=p.task_definition.object
    frames=list(obj.names) if hasattr(obj,'names') else ['base','moving'] if p.task_definition.id=='box' else [obj.tree.root]
    position=np.zeros(len(trace.time));rotation=position.copy()
    for frame in frames:
        target=obj.fk(desired,frame);index=trace.metadata['object_names'].index(scene.object_frames[frame]);actual=trace.object_pose[:,index]
        position=np.maximum(position,np.linalg.norm(actual[:,:3]-target[:3,3],axis=1))
        rotation=np.maximum(rotation,(Rotation.from_matrix(target[:3,:3]).inv()*Rotation.from_quat(actual[:,3:])).magnitude())
    contact=measured_contact_flags(episode,trace,scene);tail=trace.time>=trace.time[-1]-1.
    recovered=bool(np.max(position[tail])<=.008 and np.max(rotation[tail])<=.06 and np.mean(contact[tail].all(axis=1))>=.99)
    lost=bool(np.mean((position[tail]>.05)|(rotation[tail]>.3))>=.9 and np.mean(contact[tail].all(axis=1))<=.1)
    idx=np.unique(np.linspace(0,len(trace.time)-1,401).round().astype(int))
    return dict(recovered=recovered,persistent_loss_of_support=lost,mathematical_divergence_claimed=False,
        maximum_position_error_m=float(position[trace.time>=episode.preparation].max()),
        maximum_rotation_error_rad=float(rotation[trace.time>=episode.preparation].max()),
        tail_position_error_m=float(position[tail].max()),tail_rotation_error_rad=float(rotation[tail].max()),
        time_s=trace.time[idx].tolist(),position_error_m=position[idx].tolist(),rotation_error_rad=rotation[idx].tolist(),
        supporting_contacts=contact[idx].sum(axis=1).tolist(),
        force_end_s=max((x.start+x.duration for x in pulses),default=4.))


def run(args,task_id):
    started=time.monotonic()
    required=args.root/task_id/'no_target_robustness/result.json'
    while not required.exists():
        if time.monotonic()-started>args.wait_seconds:raise TimeoutError(f'Missing nominal-only plan: {required}')
        time.sleep(2)
    out=args.output/task_id;out.mkdir(parents=True,exist_ok=True);episodes={};checks={};scenes={};key_ids={};parent_pass={}
    for mode,variant in [('on','full'),('off','no_target_robustness')]:
        p,keys,rs,cs,report=load_recorded_plan(args.root,task_id,variant,model_directory=out/mode/'models')
        parent_pass[mode]=report['execution_passed']
        if mode=='on' and not report['execution_passed']:raise RuntimeError(f'{task_id}/{variant}: full ON physical baseline must pass first')
        key_ids[mode]=p.task_definition.keyframes[args.keyframe]['id']
        original=FullEpisode(p,keys,rs,cs)
        episode,check=probe_episode(original,args.keyframe,out/mode/'reference')
        episodes[mode]=episode;checks[mode]=check;scenes[mode]=build_scene(episode.problem,out/mode/'probe-scene.xml')
    on,off=episodes['on'],episodes['off'];onp,offp=on.problem,off.problem
    q_on=on.keyframes[0].q[onp.system.object_q];q_off=off.keyframes[0].q[offp.system.object_q]
    np.testing.assert_allclose(q_on,q_off,atol=1e-12)
    if asdict(onp.task_definition.configuration.execution)!=asdict(offp.task_definition.configuration.execution):raise RuntimeError('Controller policies must be identical')
    scene_hash={mode:hashlib.sha256(scene.path.read_bytes()).hexdigest() for mode,scene in scenes.items()}
    if len(set(scene_hash.values()))!=1:raise RuntimeError('Native physical scenes differ between ON/OFF')
    rho_on=min(on.keyframes[0].rho,checks['on']['frozen_conic']['rho']);rho_off=checks['off']['frozen_conic']['rho']
    if rho_on<=0 or not checks['on']['passed']:raise RuntimeError('A positive optimized ON certificate is required')
    load=onp.task_load(q_on);other=offp.task_load(q_off)
    np.testing.assert_array_equal(load.E,other.E);np.testing.assert_array_equal(load.D,other.D)
    dimension=load.E.shape[1];axes=[0,1,2] if dimension==4 else [3,4,5]
    candidates=[]
    for axis in axes:
        for sign in [-1,1]:
            direction=np.zeros(dimension);direction[axis]=sign
            a=directional_margin(on,direction);b=directional_margin(off,direction)
            gap=args.fraction*rho_on/max(b['maximum_coefficient'],1e-12) if b['certified'] else None
            candidates.append(dict(direction=direction.tolist(),on=a,off=b,challenge_to_off_directional_bound=gap))
    known=[c for c in candidates if c['challenge_to_off_directional_bound'] is not None]
    selected=max(known,key=lambda x:x['challenge_to_off_directional_bound']) if known else candidates[0]
    direction=np.asarray(selected['direction']);common=.25*max(0.,min(rho_on,rho_off))
    cases={'nominal':np.zeros(dimension)}
    if common>1e-8:cases['common_interior']=common*direction
    cases.update(on_interior=args.fraction*rho_on*direction,opposite_interior=-args.fraction*rho_on*direction)
    protocol=dict(schema_version=1,task=task_id,parent_full_plan=str(args.root/task_id/'full/result.json'),
        optimized_keyframe_index=args.keyframe,optimized_keyframe_id=key_ids['on'],parent_full_execution_passed=parent_pass,
        source='accepted mid-task keyframe from independently validated complete plans',
        target_robustness_only_ablated=True,candidate_fixed=True,physical_scene_sha256=scene_hash,
        controller=asdict(onp.task_definition.configuration.execution),
        independent_keyframe_checks=checks,
        rho_on_optimized_and_independently_checked=rho_on,rho_off_independently_checked=rho_off,
        reported_margins_are_certified_lower_bounds=True,
        failed_maximum_solve_is_not_zero_capacity=True,
        direction_selection='maximum challenge/OFF certified directional bound over the six signed force axes before native trials; failed ray solves excluded',
        certified_direction_selection_available=bool(known),nonzero_common_control_available=common>1e-8,
        directional_candidates=candidates,selected_direction=selected,cases={k:v.tolist() for k,v in cases.items()},
        hold_duration_s=6.,load_start_s=4.,load_duration_s=3.,ramp_s=.5,
        certificate_scope='fixed optimized reference pose; not a dynamic or closed-loop stability guarantee',
        nominal_trials_per_condition=1,statistical_success_rate_claimed=False)
    write_json(out/'protocol.json',protocol);results=[];nominal_pass={}
    for name,coefficient in cases.items():
        pulses=physical_hold(onp,on.keyframes[0],scenes['on'],coefficient) if np.linalg.norm(coefficient) else ()
        certificate=schedule_membership(onp,q_on,pulses,rho_on,scenes['on'])
        if not certificate['inside']:raise RuntimeError('Refusing native validation outside the optimized ON box')
        for mode,episode in episodes.items():
            folder=out/mode/name
            trace,controller,scene,geometry=run_full_episode(episode,folder/'native',disturbances=pulses)
            report=assess(episode,trace,controller,scene,geometry=geometry)
            # This separate hold probe does not perform placement/insertion.
            passed=report['tracking_passed'] and report['contact_retention_passed'] and report['minimum_actuator_margin_Nm']>=-1e-8 and all(x['passed'] for x in report['jaw_force'].values()) and report['sampled_interface_wrench_capability']['passed'] and geometry['passed'] and not report['unexpected_loaded_contact_penetrations_m']
            analysis=response(episode,trace,scene,pulses)
            outcome='bounded_tracking' if passed and analysis['recovered'] else 'persistent_loss_of_support' if analysis['persistent_loss_of_support'] else 'transient_failure_recovered' if analysis['recovered'] else 'tracking_failure'
            if name=='nominal':nominal_pass[mode]=bool(passed)
            ownrho=rho_on if mode=='on' else rho_off
            own=schedule_membership(episode.problem,episode.keyframes[0].q[episode.problem.system.object_q],pulses,ownrho,scene)
            report.update(execution_passed=bool(passed),task_passed=bool(passed),task_metrics=dict(task='optimized_midtask_keyframe_hold',full_task_completion_tested_here=False),
                robustness_probe=dict(mode=mode,case=name,protocol=protocol,inside_on_certificate=certificate,own_reference_certificate=own,response=analysis,outcome=outcome,load_effect_comparison_eligible=bool(len(nominal_pass)==2 and all(nominal_pass.values()))),
                full_paper_pipeline_executed=False,full_task_recording=False)
            write_json(folder/'result.json',report)
            results.append(dict(task=task_id,mode=mode,case=name,execution_passed=bool(passed),outcome=outcome,inside_on_certificate=certificate,own_reference_certificate=own,response=analysis,scene_sha256=scene_hash[mode],native_trace_sha256=hashlib.sha256((folder/'native/execution.npz').read_bytes()).hexdigest()))
            write_json(out/'summary.json',dict(protocol=protocol,results=results))
            print(task_id,mode,name,outcome,analysis['maximum_position_error_m'],flush=True)
            if args.web is not None and name=='on_interior':
                from contact_transfer.full_view import export_full
                export_full(episode,trace,report,scene,args.web,dataset_id=f'research_probe_{task_id}_{mode}',
                    recording_metadata=dict(title=f'{task_id.title()} · inside-margin keyframe probe · {mode.upper()}',
                        scenario='certified_keyframe_probe',full_task_recording=False,comparison_recording=True,
                        study_kind='target_robustness',parent_task=task_id,
                        description='Separate stationary probe of an optimized mid-task pose. Physical scene, controller and applied wrench histories are matched. See the reference-pose certificate and response; this is not a full-motion robustness guarantee.'))
    return results


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root',type=Path,default=Path('outputs/research-study/full-v1'))
    parser.add_argument('--output',type=Path,default=Path('outputs/research-study/probes-v1'))
    parser.add_argument('--tasks',nargs='+',choices=['panel','box','assembly'],default=['panel','box','assembly'])
    parser.add_argument('--keyframe',type=int,default=2)
    parser.add_argument('--fraction',type=float,default=.8)
    parser.add_argument('--workers',type=int,default=3)
    parser.add_argument('--web',type=Path)
    parser.add_argument('--wait-seconds',type=float,default=7200)
    args=parser.parse_args()
    if not 0<args.fraction<=1:parser.error('Load fraction must be within the ON margin')
    if len(args.tasks)>1:
        def launch(task):
            log=args.output/(task+'.log');log.parent.mkdir(parents=True,exist_ok=True)
            cmd=[sys.executable,'-u',__file__,'--root',str(args.root),'--output',str(args.output),'--tasks',task,'--keyframe',str(args.keyframe),'--fraction',str(args.fraction),'--wait-seconds',str(args.wait_seconds)]
            if args.web is not None:cmd+=['--web',str(args.web)]
            with log.open('w') as handle:result=subprocess.run(cmd,stdout=handle,stderr=subprocess.STDOUT)
            return dict(task=task,exit_code=result.returncode,log=str(log))
        with ThreadPoolExecutor(max_workers=args.workers) as pool:results=list(pool.map(launch,args.tasks))
        write_json(args.output/'workers.json',results)
        return 0 if all(x['exit_code']==0 for x in results) else 1
    run(args,args.tasks[0]);return 0


if __name__=='__main__':raise SystemExit(main())
