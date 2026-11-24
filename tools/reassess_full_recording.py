#!/usr/bin/env python3
"""Apply current independent execution gates to an unchanged saved native trace."""
import argparse,json,hashlib,fcntl
from pathlib import Path
from types import SimpleNamespace
from export_full_recording import load_recorded_plan
from contact_transfer.execution import ExecutionTrace
from contact_transfer.full_execution import FullEpisode,reference_geometry
from contact_transfer.full_scene import build_scene
from contact_transfer.full_assessment import assess
from contact_transfer.full_recording import write_json
from contact_transfer.full_view import export_full


def reassess(root,task,variant,web):
 root=Path(root);folder=root/task/variant;p,keys,rs,cs,previous=load_recorded_plan(root,task,variant);trace=ExecutionTrace.load(folder/'native/execution.npz');fraction=trace.metadata.get('controller_configuration',{}).get('jaw_force_fraction',previous.get('jaw_force_fraction',.7));episode=FullEpisode(p,keys,rs,cs,force_feedforward=previous['arm_reaction_feedforward'],jaw_force_fraction=fraction)
 if episode.identity()!=(trace.metadata['case_id'],trace.metadata['plan_id']):raise RuntimeError('Current reconstructed plan differs from the recorded native trial')
 scene=build_scene(p,folder/'native/reassessment-scene.xml',timestep=trace.metadata['timestep'],contacts_enabled=trace.metadata['contacts_enabled'])
 if hashlib.sha256(scene.path.read_bytes()).hexdigest()!=trace.metadata['scene_sha256']:raise RuntimeError('Current native scene differs from the recording')
 controller=SimpleNamespace(robots=p.full_robots,logs=json.loads((folder/'native/control.json').read_text()));geometry=reference_geometry(episode,scene);report={**previous,**assess(episode,trace,controller,scene,geometry=geometry), 'assessment_replayed_from_saved_trace':True};write_json(folder/'result.json',report)
 with (root/task/'.ablation-results.lock').open('w') as lock:
  fcntl.flock(lock,fcntl.LOCK_EX);matrix_path=root/task/'ablation-results.json';matrix=json.loads(matrix_path.read_text());matrix['variants'][variant]=report;write_json(matrix_path,matrix)
 export_full(episode,trace,report,scene,web,variant=variant,ablation_results=matrix['variants']);print(task,variant,report['execution_passed'],flush=True)
 return report


if __name__=='__main__':
 parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--root',type=Path,required=True);parser.add_argument('--task',choices=['panel','box','assembly'],required=True);parser.add_argument('--variants',nargs='+',default=['full']);parser.add_argument('--web',type=Path,default=Path('web'));args=parser.parse_args()
 for variant in args.variants:reassess(args.root,args.task,variant,args.web)
