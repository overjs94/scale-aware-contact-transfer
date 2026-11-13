#!/usr/bin/env python3
"""Validate saved optimized stages, then execute a fresh native physical trial.

This path reuses archived optimization certificates. To solve optimization
again, use tools/run_full_scenarios.py instead.
"""
import argparse,hashlib,json,shutil,subprocess,sys
from pathlib import Path
from export_full_recording import load_recorded_plan
from contact_transfer.full_recording import write_json

if __name__=='__main__':
 parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--task',choices=['panel','box','assembly'],required=True);parser.add_argument('--bundle',type=Path,default=Path('examples/recordings/full_tasks'));parser.add_argument('--output',type=Path,default=Path('outputs/bundled_full_tasks'));parser.add_argument('--web',type=Path,help='Optionally replace the corresponding viewer recording');parser.add_argument('--validate-only',action='store_true');args=parser.parse_args();source=args.bundle/args.task;manifest=json.loads((source/'manifest.json').read_text())
 for name,digest in manifest['files'].items():
  path=(source/name).resolve()
  if not path.is_relative_to(source.resolve()) or hashlib.sha256(path.read_bytes()).hexdigest()!=digest:raise RuntimeError('Bundle integrity rejected: '+name)
 target=args.output/args.task
 if target.exists():raise SystemExit('Choose a new --output directory to preserve the existing trial')
 shutil.copytree(source,target);report_path=target/'full/result.json';report=json.loads(report_path.read_text());report.update(execution_passed=False,new_physical_trial_executed=False,new_target_solver_executed=False,bundle_source_trial_passed=True);write_json(report_path,report);write_json(target/'ablation-results.json',dict(task=args.task,variants={'full':report},nominal_trials_per_variant=1,statistical_success_rate_claimed=False));p,keys,rs,cs,_=load_recorded_plan(args.output,args.task);
 for robot in p.full_robots:
  if hashlib.sha256(robot.path.read_bytes()).hexdigest()!=manifest['native_robot_urdf_sha256'][robot.path.name]:raise RuntimeError('Current URDF differs from accepted model: '+robot.path.name)
 print(f'{args.task}: {len(keys)} keyframes and {len(rs)} intervals passed current independent mechanics',flush=True)
 if not args.validate_only:
  command=[sys.executable,'-u',str(Path(__file__).with_name('replay_full_plan.py')),'--root',str(args.output),'--task',args.task,'--jaw-force-fraction',str(manifest['controller_jaw_force_fraction'])]
  if args.web is not None:command.extend(['--web',str(args.web)])
  raise SystemExit(subprocess.run(command).returncode)
