"""Independently audit all task plans and attempt the literal Panda first lift.

A failed local solve is retained; it is not a global infeasibility proof.
The uploaded-PDF equation audit defines independent post-step integration as
the literal reference. Articulated/multi-object cases remain explicit domains.
"""
import argparse
from copy import deepcopy
from pathlib import Path
from time import perf_counter

from contact_transfer.equation_contract import audit_episode,literal_integration
from contact_transfer.full_execution import FullEpisode,reference_geometry
from contact_transfer.full_pipeline import scenario_solver_settings
from contact_transfer.full_recording import write_json,save_interval
from contact_transfer.full_scene import build_scene
from contact_transfer.full_trajectory import solve_interval,initial_interval,interval_contacts
from contact_transfer.full_validation import validate_interval
from export_full_recording import load_recorded_plan


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--root',type=Path,default=Path('outputs/research-study/full-common'))
    p.add_argument('--output',type=Path,default=Path('outputs/research-study/literal-audit'))
    p.add_argument('--attempt',action='store_true')
    p.add_argument('--steps',nargs='+',type=int,default=[16,32])
    p.add_argument('--cpu-seconds',type=float,default=120.)
    args=p.parse_args();audits={}
    for task in ['panel','box','assembly']:
        problem,keys,rs,cs,report=load_recorded_plan(args.root,task,model_directory=args.output/task/'models')
        episode=FullEpisode(problem,keys,rs,cs);audits[task]=audit_episode(episode)
        write_json(args.output/(task+'-equation-audit.json'),audits[task])
    write_json(args.output/'summary.json',audits)
    if not args.attempt:return
    p,keys,rs,cs,_=load_recorded_plan(args.root,'panel',model_directory=args.output/'literal/models')
    p.directory=args.output/'literal';p.directory.mkdir(parents=True,exist_ok=True)
    index=next(i for i,(a,b) in enumerate(zip(keys[:-1],keys[1:])) if max(abs(a.q[p.system.object_q]-b.q[p.system.object_q]))>1e-8)
    left,right=keys[index:index+2];duration=p.task_definition.keyframes[index+1]['time']-p.task_definition.keyframes[index]['time'];attempts=[]
    for nodes in args.steps:
        started=perf_counter();folder=args.output/'literal'/str(nodes);folder.mkdir(exist_ok=True)
        settings=scenario_solver_settings(solver_cpu_seconds=args.cpu_seconds);settings.diagnostics_path=str(folder/'nlp-failure.json')
        try:
            contacts=interval_contacts(p,index,left,nodes)
            seed=deepcopy(rs[index]) if nodes==len(rs[index].dt) else initial_interval(p,index,left,right,nodes,duration,contacts)
            result,contacts,_=solve_interval(p,index,left,right,settings,steps=nodes,configuration_update='independent_se3',initial_guess=seed)
            independent=validate_interval(p,result,contacts)
            local=FullEpisode(p,(left,right),(result,),(contacts,));scene=build_scene(p,folder/'native-reference.xml');geometry=reference_geometry(local,scene)
            accepted=independent['passed'] and geometry['passed']
            save_interval(folder/'interval.npz',result,'literal-first-lift-qualification')
            record=dict(nodes=nodes,accepted=accepted,independent=independent,geometry=geometry,literal_integration=literal_integration(p,result))
        except (RuntimeError,ValueError) as error:record=dict(nodes=nodes,accepted=False,failure=str(error),global_infeasibility_claimed=False)
        record.update(wall_time_s=perf_counter()-started,new_physical_trial_executed=False);attempts.append(record);write_json(args.output/'literal/attempts.json',attempts)
        print(record,flush=True)


if __name__=='__main__':main()
