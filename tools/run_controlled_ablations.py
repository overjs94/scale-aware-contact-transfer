#!/usr/bin/env python3
"""Resume fresh controlled physical trials from an accepted complete full plan."""
import argparse,json,subprocess,sys
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from datetime import datetime,timezone
from export_full_recording import load_recorded_plan
from contact_transfer.full_execution import FullEpisode
from contact_transfer.full_pipeline import run_task,ABLATIONS
from contact_transfer.optimization import SolverSettings

if __name__=='__main__':
 parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--root',type=Path,required=True);parser.add_argument('--task',choices=['panel','box','assembly'],required=True);parser.add_argument('--variants',nargs='+',choices=ABLATIONS,default=list(ABLATIONS[1:]));parser.add_argument('--web',type=Path,default=Path('web'));parser.add_argument('--workers',type=int,default=1,help='Independent Python trial workers; each variant keeps its own plan/control files');args=parser.parse_args()
 if args.workers<1:parser.error('--workers must be positive')
 if args.workers>1:
  def trial(variant):
   return subprocess.run([sys.executable,'-u',__file__,'--root',str(args.root),'--task',args.task,'--variants',variant,'--web',str(args.web)],check=True).returncode
  with ThreadPoolExecutor(max_workers=args.workers) as pool:list(pool.map(trial,args.variants))
  directory=args.root/args.task;matrix=json.loads((directory/'ablation-results.json').read_text())['variants']
  from contact_transfer.full_recording import write_json
  for path in (args.web/'data').glob('full_'+args.task+'*.json'):
   data=json.loads(path.read_text());data['report']['ablation_results']=matrix;write_json(path,data,compact=True)
  print({name:r['execution_passed'] for name,r in matrix.items()},flush=True);raise SystemExit(0)
 p,keys,rs,cs,report=load_recorded_plan(args.root,args.task)
 if not report['execution_passed']:raise SystemExit('The complete full-method physical trial must pass first')
 episode=FullEpisode(p,keys,rs,cs);directory=args.root/args.task;stamp=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
 for variant in args.variants:
  old=directory/variant
  if old.exists() and (old/'result.json').exists():
   backup=directory/'diagnostics'/f'{variant}-before-controlled-replay-{stamp}';backup.parent.mkdir(parents=True,exist_ok=True);old.rename(backup)
 result=run_task(args.task,directory,variants=args.variants,accepted_full=(episode,report['selection']),settings=SolverSettings(**report['selection']['solver_configuration']),web_directory=args.web)
 print({name:r['execution_passed'] for name,r in result.items()},flush=True)
