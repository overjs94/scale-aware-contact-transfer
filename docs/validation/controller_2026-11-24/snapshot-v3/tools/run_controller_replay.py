"""Replay accepted plans with a declared controller change, never claim new NLP."""
import argparse
from dataclasses import asdict, replace
import hashlib
import json
from pathlib import Path
import time

import numpy as np

from export_full_recording import load_recorded_plan
from run_certified_probes import response, directional_margin
from contact_transfer.certified_disturbance import probe_episode,physical_hold,schedule_membership
from contact_transfer.configuration import TaskConfiguration
from contact_transfer.full_assessment import assess
from contact_transfer.full_execution import FullEpisode,run_full_episode
from contact_transfer.full_recording import write_json
from contact_transfer.full_scene import build_scene


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--task',choices=['box','panel','assembly'],default='box')
    parser.add_argument('--root',type=Path,default=Path('outputs/research-study/full-common'))
    parser.add_argument('--output',type=Path,default=Path('outputs/controller-study'))
    parser.add_argument('--mode',choices=['baseline','preload','allocation','adaptive'],required=True)
    parser.add_argument('--case',choices=['nominal','primary','opposite','full','early'],default='opposite')
    parser.add_argument('--variant',default='full')
    parser.add_argument('--web',type=Path)
    parser.add_argument('--export-only',action='store_true',help='Export a completed measured trace without a new physical trial')
    args=parser.parse_args();started=time.perf_counter();out=args.output/args.task/args.mode/args.case;out.mkdir(parents=True,exist_ok=True)
    if (out/'result.json').exists() and not args.export_only:raise FileExistsError(f'Refusing to overwrite a completed physical trial: {out}')
    if args.export_only and (not args.web or not (out/'result.json').exists()):parser.error('--export-only requires --web and a completed result')
    import contact_transfer
    package=Path(contact_transfer.__file__).parent
    if not args.export_only:write_json(out/'code-manifest.json',dict(package={path.name:hashlib.sha256(path.read_bytes()).hexdigest() for path in package.glob('*.py')},runner_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()))
    p,keys,rs,cs,parent=load_recorded_plan(args.root,args.task,args.variant,model_directory=out/'models')
    original=FullEpisode(p,keys,rs,cs)
    if args.case in {'full','early'}:episode=original;check=None
    else:episode,check=probe_episode(original,2,out/'reference')
    p=episode.problem;policy=p.task_definition.configuration.execution
    changes=dict(online_allocation=args.mode in {'allocation','adaptive'},adaptive_jaw_force=args.mode=='adaptive',allocation_grip_fraction=.6 if args.mode=='adaptive' else policy.jaw_force_fraction)
    if args.mode=='preload':changes['jaw_force_fraction']=.75
    policy=replace(policy,**changes);p.task_definition=replace(p.task_definition,configuration=replace(p.task_definition.configuration,execution=policy))
    if hasattr(p,'_case'):del p._case
    episode=FullEpisode(p,episode.keyframes,episode.trajectories,episode.contacts_by_interval)
    scene=build_scene(p,out/'scene-check.xml');pulses=()
    protocol=json.loads((Path('outputs/research-study/probes-v1')/args.task/'protocol.json').read_text())
    if args.case=='early':
        from contact_transfer.perturbations import WrenchHold
        from contact_transfer.full_validation import validate_keyframe
        with np.load(args.root/args.task/'external_push/native/execution.npz',allow_pickle=False) as a:old_metadata=json.loads(str(a['metadata_json']))
        if old_metadata['plan_id']!=original.identity()[1]:raise RuntimeError('Early-load baseline used a different plan')
        if old_metadata['scene_sha256']!=hashlib.sha256(scene.path.read_bytes()).hexdigest():raise RuntimeError('Early-load baseline scene differs')
        pulses=tuple(WrenchHold(**{k:value[k] for k in ['body','start','duration','force_world_N','point_body_m','moment_world_Nm','ramp_duration_s']}) for value in old_metadata['external_disturbances'])
        check=validate_keyframe(p,episode.keyframes[0]);rho=min(episode.keyframes[0].rho,check['frozen_conic']['rho'])
    elif args.case not in {'nominal','full'}:
        coefficient=np.array(protocol['cases']['on_interior' if args.case=='primary' else 'opposite_interior'])
        pulses=physical_hold(p,episode.keyframes[0],scene,coefficient)
    if args.case!='early':rho=protocol['rho_on_optimized_and_independently_checked']
    certificate=schedule_membership(p,episode.keyframes[0].q[p.system.object_q],pulses,rho,scene)
    if args.export_only:
        from contact_transfer.execution import ExecutionTrace
        from contact_transfer.full_view import export_full
        trace=ExecutionTrace.load(out/'native/execution.npz');report=json.loads((out/'result.json').read_text())
        if hashlib.sha256(scene.path.read_bytes()).hexdigest()!=trace.metadata['scene_sha256']:raise RuntimeError('Recorded scene differs')
        if episode.identity()!=(trace.metadata['case_id'],trace.metadata['plan_id']):raise RuntimeError('Recorded plan/controller policy differs')
        data=export_full(episode,trace,report,scene,args.web,dataset_id=f'research_controller_{args.task}_{args.mode}_{args.case}',recording_metadata=dict(title=f'{args.task.title()} · {args.mode} controller · {args.case}',scenario='controller_replay',full_task_recording=args.case in {'full','early'},comparison_recording=True,study_kind='controller_replay',parent_task=args.task,description='Accepted-plan controller replay. No new source/target/trajectory optimization; physical load unchanged. See explicit controller policy and force limits.'))
        print(data['id'],report['execution_passed']);return
    write_json(out/'protocol.json',dict(controller=asdict(policy),parent_plan=str(args.root/args.task/args.variant),new_source_or_target_NLP_executed=False,old_hardware_certificate=certificate,certificate_scope='reference pose and original hardware; operating allocation envelope is separately derated',independent_keyframe=check,case=args.case,scene_sha256=hashlib.sha256(scene.path.read_bytes()).hexdigest()))
    trace,controller,scene,geometry=run_full_episode(episode,out/'native',disturbances=pulses,progress=lambda t,d:print(f'{t:.1f}/{d:.1f}s',flush=True) if round(t*1000)%1000==0 else None)
    report=assess(episode,trace,controller,scene,geometry=geometry)
    if args.case not in {'full','early'}:
        passed=report['tracking_passed'] and report['contact_retention_passed'] and report['minimum_actuator_margin_Nm']>=-1e-8 and all(x['passed'] for x in report['jaw_force'].values()) and report['sampled_interface_wrench_capability']['passed'] and geometry['passed'] and not report['unexpected_loaded_contact_penetrations_m']
        report.update(execution_passed=bool(passed),task_passed=bool(passed),task_metrics=dict(task='controller_hold_replay',full_task_completion_tested_here=False),response=response(episode,trace,scene,pulses))
    records=controller.allocation_logs
    report['controller_replay']=dict(mode=args.mode,policy=asdict(policy),new_NLP_executed=False,all_solves_accepted=all(x['accepted'] for x in records) if records else None,allocation_count=len(records),rejected_solves=sum(not x['accepted'] for x in records),latency_p50_s=float(np.median([x['wall_time_s'] for x in records])) if records else None,latency_p99_s=float(np.percentile([x['wall_time_s'] for x in records],99)) if records else None,wall_time_s=time.perf_counter()-started,trace_sha256=hashlib.sha256((out/'native/execution.npz').read_bytes()).hexdigest())
    write_json(out/'result.json',report)
    if args.web:
        from contact_transfer.full_view import export_full
        export_full(episode,trace,report,scene,args.web,dataset_id=f'research_controller_{args.task}_{args.mode}_{args.case}',recording_metadata=dict(title=f'{args.task.title()} · {args.mode} · {args.case}',scenario='controller_replay',full_task_recording=args.case in {'full','early'},comparison_recording=True,study_kind='controller_replay',parent_task=args.task,description='Accepted-plan controller replay. No new source/target/trajectory optimization; physical load unchanged. See explicit controller policy and force limits.'))
    print(json.dumps(dict(passed=report['execution_passed'],errors=report['object_tracking'],controller_replay=report['controller_replay'])),flush=True)


if __name__=='__main__':main()
