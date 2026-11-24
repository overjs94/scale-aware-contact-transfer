"""A fresh physical trial with unchanged optimized plan and explicit control/dt.

This is not a fresh source/target/CITO solve. Original plan/trial files are not
modified, and failed physical trials remain failed in the new result.
"""
import argparse
from dataclasses import replace
import hashlib
import json
from pathlib import Path
from time import perf_counter

from export_full_recording import load_recorded_plan
from contact_transfer.full_assessment import assess
from contact_transfer.full_execution import FullEpisode,run_full_episode
from contact_transfer.full_recording import write_json


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root',type=Path,required=True)
    parser.add_argument('--task',choices=['panel','box','assembly'],required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--control-mode',choices=['impedance_feedforward','hybrid_feedback','hybrid_feedforward'])
    parser.add_argument('--timestep',type=float)
    parser.add_argument('--force-normal-damping',type=float,help='Common contact-normal velocity damping for a documented controller qualification trial')
    parser.add_argument('--no-force-feedforward',action='store_true')
    parser.add_argument('--disable-contacts',action='store_true')
    parser.add_argument('--allow-failed-baseline',action='store_true',help='Retry an independently validated plan whose previous physical execution failed; preserve that failure')
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=False);started=perf_counter()
    report_path=args.root/args.task/'full'/'result.json'
    problem,keys,trajectories,contacts,baseline=load_recorded_plan(args.root,args.task,model_directory=args.output/'models')
    if not baseline['execution_passed'] and not args.allow_failed_baseline:raise RuntimeError('A passed full-method baseline is required, or explicitly allow a failed physical baseline')
    cfg=problem.task_definition.configuration;changes={}
    if args.control_mode is not None:changes['control_mode']=args.control_mode
    if args.timestep is not None:changes['physics_dt_s']=args.timestep
    if args.force_normal_damping is not None:changes['force_d']=(*cfg.execution.force_d[:5],args.force_normal_damping)
    problem.task_definition.configuration=replace(cfg,execution=replace(cfg.execution,**changes))
    problem.directory=args.output/'reference';problem.directory.mkdir()
    if hasattr(problem,'_case'):del problem._case
    write_json(args.output/'resolved-execution-policy.json',problem.task_definition.configuration.to_dict())
    episode=FullEpisode(problem,keys,trajectories,contacts,force_feedforward=not args.no_force_feedforward)
    try:
        trace,controller,scene,geometry=run_full_episode(episode,args.output/'native',contacts_enabled=not args.disable_contacts,
            progress=lambda t,total:print(f'{args.task}: {t:.0f}/{total:.1f}s',flush=True) if int(t)%5==0 else None)
        report=assess(episode,trace,controller,scene,geometry=geometry)
        report.update(new_physical_trial_executed=True)
    except (RuntimeError,ValueError) as error:
        report=dict(execution_passed=False,failure=str(error),new_physical_trial_executed=False)
    report.update(task=args.task,baseline_execution_passed=baseline['execution_passed'],plan_source=str(report_path),plan_source_sha256=hashlib.sha256(report_path.read_bytes()).hexdigest(),
                  new_target_solver_executed=False,new_cito_solver_executed=False,full_paper_pipeline_executed=False,
                  controller=episode.control_mode,physics_dt_s=problem.case.data['simulation']['timestep'],
                  force_damping_contact=problem.task_definition.configuration.execution.force_d,
                  force_feedforward=not args.no_force_feedforward,contacts_enabled=not args.disable_contacts,
                  trial_wall_time_s=perf_counter()-started)
    write_json(args.output/'result.json',report)
    print(json.dumps({k:report.get(k) for k in ['task','execution_passed','failure','controller','physics_dt_s','trial_wall_time_s']},indent=2))
    return 0 if report['execution_passed'] else 1


if __name__=='__main__':raise SystemExit(main())
