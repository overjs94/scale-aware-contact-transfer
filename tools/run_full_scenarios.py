#!/usr/bin/env python3
"""Run genuine complete-task P/G selection, native execution and ablations."""
import argparse
from pathlib import Path
from contact_transfer.full_pipeline import ABLATIONS,run_task,scenario_solver_settings


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--tasks',nargs='+',choices=['panel','box','assembly'],default=['panel','box','assembly'])
    parser.add_argument('--variants',nargs='+',choices=ABLATIONS,default=list(ABLATIONS))
    parser.add_argument('--steps',type=int,default=16)
    parser.add_argument('--max-steps',type=int,default=64)
    parser.add_argument('--iterations',type=int,default=1000)
    parser.add_argument('--solver-cpu-seconds',type=float,default=180.,help='Identical per-NLP CPU budget for every method and adaptive grid attempt')
    parser.add_argument('--output',type=Path,default=Path('outputs/full_scenarios'))
    parser.add_argument('--web',type=Path)
    parser.add_argument('--keyframe-archive',type=Path,help='Explicitly revalidate earlier optimized target keyframes against current mechanics, task and source prior')
    parser.add_argument('--interval-archive',type=Path,help='Explicitly revalidate earlier sticking-only intervals with unchanged physical constraints')
    parser.add_argument('--verbose',action='store_true')
    args=parser.parse_args();settings=scenario_solver_settings(args.iterations,args.verbose,args.solver_cpu_seconds);passed=True
    for task in args.tasks:
        result=run_task(task,args.output/task,variants=args.variants,steps=args.steps,max_steps=args.max_steps,settings=settings,web_directory=args.web,keyframe_archive=args.keyframe_archive,interval_archive=args.interval_archive);passed=passed and all(r['execution_passed'] for v,r in result.items() if v=='full');print(task,{v:r['execution_passed'] for v,r in result.items()},flush=True)
    return 0 if passed else 1


if __name__=='__main__':raise SystemExit(main())
