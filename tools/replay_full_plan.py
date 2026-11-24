#!/usr/bin/env python3
"""Rerun native control from a complete independently revalidated plan.

A rejected physical trial is retained in diagnostics. This does not rerun or
claim new target/trajectory optimization, and never changes model force limits.
"""
import argparse,json,hashlib,shutil
from datetime import datetime,timezone
from pathlib import Path
from time import perf_counter
from export_full_recording import load_recorded_plan
from contact_transfer.full_execution import FullEpisode,run_full_episode
from contact_transfer.full_assessment import assess
from contact_transfer.full_recording import write_json
from contact_transfer.full_view import export_full

if __name__=='__main__':
 parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--root',type=Path,required=True);parser.add_argument('--task',choices=['panel','box','assembly'],required=True);parser.add_argument('--web',type=Path,help='Optionally export this new trial to a viewer checkout');parser.add_argument('--jaw-force-fraction',type=float);args=parser.parse_args();start=perf_counter()
 p,keys,rs,cs,previous=load_recorded_plan(args.root,args.task);episode=FullEpisode(p,keys,rs,cs,jaw_force_fraction=args.jaw_force_fraction);folder=args.root/args.task/'full';stamp=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ');backup=args.root/args.task/'diagnostics'/('full-before-control-replay-'+stamp);backup.mkdir(parents=True)
 for name in ['native','result.json']:
  if (folder/name).exists():shutil.move(str(folder/name),backup/name)
 trace,controller,scene,geometry=run_full_episode(episode,folder/'native',progress=lambda t,total:print(f'{args.task}/full replay: {t:.0f}/{total:.1f}s',flush=True) if int(t)%5==0 else None)
 report=assess(episode,trace,controller,scene,geometry=geometry);report.update(variant='full',selection=previous['selection'],full_variant_pipeline_executed=True,all_intervals_cito_optimized=previous['all_intervals_cito_optimized'],interval_methods=previous['interval_methods'],fixed_path_integration_residuals=None,new_physical_trial_executed=True,trial_wall_time_s=perf_counter()-start,control_replay=dict(previous_trial=str(backup),previous_result_sha256=hashlib.sha256((backup/'result.json').read_bytes()).hexdigest(),new_target_solver_executed=False,model_force_limits_changed=False))
 write_json(folder/'result.json',report);matrix_path=args.root/args.task/'ablation-results.json';matrix=json.loads(matrix_path.read_text());matrix['variants']['full']=report;write_json(matrix_path,matrix)
 if args.web is not None:export_full(episode,trace,report,scene,args.web,ablation_results=matrix['variants'])
 print(args.task,report['execution_passed'],flush=True)
 raise SystemExit(0 if report['execution_passed'] else 1)
