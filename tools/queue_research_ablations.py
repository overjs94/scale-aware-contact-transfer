"""Run controlled trials as soon as each full physical baseline is available."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import json
import os
from pathlib import Path
import subprocess
import sys
import time


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root',type=Path,required=True);parser.add_argument('--snapshot',type=Path,required=True)
    parser.add_argument('--tasks',nargs='+',default=['panel','box','assembly'])
    parser.add_argument('--workers',type=int,default=3);args=parser.parse_args()
    def task(name):
        report=args.root/name/'full/result.json';started=time.monotonic()
        while not report.exists():
            if time.monotonic()-started>7200:return dict(task=name,failed='Timed out waiting for a full baseline')
            time.sleep(2)
        baseline=json.loads(report.read_text())
        if not baseline['execution_passed']:return dict(task=name,failed='Full baseline failed; controlled comparison not qualified')
        env=dict(os.environ,PYTHONPATH=str(args.snapshot.resolve()/'src'))
        cmd=[sys.executable,'-u',str(args.snapshot.resolve()/'tools/run_research_ablation.py'),'--root',str(args.root),'--tasks',name,'--phase','ablations']
        log=args.root/(name+'-ablations.log')
        with log.open('w') as handle:result=subprocess.run(cmd,env=env,stdout=handle,stderr=subprocess.STDOUT)
        return dict(task=name,exit_code=result.returncode,log=str(log),snapshot=str(args.snapshot))
    with ThreadPoolExecutor(max_workers=args.workers) as pool:results=list(pool.map(task,args.tasks))
    (args.root/'ablation-workers.json').write_text(json.dumps(results,indent=2)+'\n')
    print(json.dumps(results,indent=2))


if __name__=='__main__':main()
