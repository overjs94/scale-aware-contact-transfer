#!/usr/bin/env python3
"""Run a matched inside-margin sustained-load and recovery comparison."""
import argparse
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import subprocess
import sys
from contact_transfer.margin_hold_study import CASES,run_mode

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,default=Path('outputs/margin_platform_v1'))
    p.add_argument('--web',type=Path,default=Path('web'))
    p.add_argument('--mode',choices=['on','off'])
    p.add_argument('--case',choices=list(CASES),action='append')
    p.add_argument('--object',choices=['flat','loaded','compact'],default='loaded')
    args=p.parse_args()
    if args.mode:
        run_mode(args.output,args.mode=='on',args.case,args.web,object_variant=args.object)
    else:
        def run(mode):
            cmd=[sys.executable,'-u',__file__,'--output',str(args.output),'--web',str(args.web),'--mode',mode,'--object',args.object]
            for case in args.case or []:cmd+=['--case',case]
            subprocess.run(cmd,check=True)
        with ThreadPoolExecutor(max_workers=2) as pool:list(pool.map(run,['on','off']))
