#!/usr/bin/env python3
"""Export a finished full-task trace without rerunning its physical experiment."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
from contact_transfer.execution import ExecutionTrace
from contact_transfer.full_scenario import FullTask,FullProblem
from contact_transfer.full_recording import load_key,load_interval
from contact_transfer.full_trajectory import interval_contacts,initial_interval
from contact_transfer.full_validation import validate_keyframe,validate_interval
from contact_transfer.full_scene import build_scene,support_contacts
from contact_transfer.paper_execution import CoupledEpisode
from contact_transfer.full_view import export_full


def original_identity(path):
    with np.load(path,allow_pickle=False) as a:return json.loads(str(a['metadata_json']))['identity']


def load_recorded_plan(root,task_id,variant='full'):
    root=Path(root);folder=root/task_id/variant;report=json.loads((folder/'result.json').read_text());selection=report['selection'];task=FullTask.load(task_id,root/task_id/'task');planning=root/task_id/(variant if variant in {'full','no_source_refinement','fixed_contact_types'} else 'full')/'planning';candidate=planning/(''.join(selection['selected_types'])+'_'+''.join(map(str,selection['selected_assignment'])))
    if (folder/'plan-origin.json').exists():candidate=Path(json.loads((folder/'plan-origin.json').read_text())['planning_directory'])
    p=FullProblem.create(task,candidate,selection['selected_types'],selection['selected_assignment'],refine_source=selection['source_refinement_enabled']);keys=[];rs=[];cs=[]
    for k,goal in enumerate(task.keyframes):
        path=candidate/f'key{k}.npz';key=load_key(path,original_identity(path));env=tuple(c for c in support_contacts(p,goal['phase'],1) if c.name in key.environment_reactions)
        if np.max(abs(key.q[p.system.object_q]-goal['object_q']))>2e-5 or not validate_keyframe(p,key,env)['passed']:raise RuntimeError(f'Current task/independent keyframe gate rejected {k}')
        keys.append(key)
    for k,(left,right) in enumerate(zip(keys[:-1],keys[1:])):
        path=candidate/f'interval{k}.npz';r=load_interval(path,original_identity(path));contacts=interval_contacts(p,k,left,len(r.dt))
        if not validate_interval(p,r,contacts)['passed']:raise RuntimeError(f'Current independent interval gate rejected {k}')
        if variant=='fixed_path':
            steps=len(r.dt);duration=float(r.dt.sum());contacts=interval_contacts(p,k,left,steps);r=initial_interval(p,k,left,right,steps,duration,contacts)
        rs.append(r);cs.append(contacts)
    return p,tuple(keys),tuple(rs),tuple(cs),report


def export_recording(root,task_id,variant,web):
    root=Path(root);folder=root/task_id/variant;p,keys,rs,cs,report=load_recorded_plan(root,task_id,variant)
    # Playback needs the accepted phase schedule, not a second dense IK solve.
    episode=CoupledEpisode(p,tuple(keys),tuple(rs),contacts_by_interval=tuple(cs),control_mode='impedance_feedforward',jaw_force_fraction=.7);trace=ExecutionTrace.load(folder/'native/execution.npz');scene=build_scene(p,folder/'native/export-scene-check.xml',timestep=trace.metadata['timestep'],contacts_enabled=variant!='contact_disabled')
    if hashlib.sha256(scene.path.read_bytes()).hexdigest()!=trace.metadata['scene_sha256']:raise RuntimeError('Current native scene differs from recorded physics')
    if abs(episode.duration-trace.time[-1])>trace.metadata['timestep']*1.1:raise RuntimeError('Recorded horizon differs from accepted schedule')
    matrix=json.loads((root/task_id/'ablation-results.json').read_text())['variants'];data=export_full(episode,trace,report,scene,web,variant=variant,ablation_results=matrix);print(data['id'],report['execution_passed'],len(data['times']))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--root',type=Path,required=True);parser.add_argument('--task',choices=['panel','box','assembly'],required=True);parser.add_argument('--variant',default='full');parser.add_argument('--web',type=Path,default=Path('web'));args=parser.parse_args();export_recording(args.root,args.task,args.variant,args.web)
