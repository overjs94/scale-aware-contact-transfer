#!/usr/bin/env python3
"""Execute a prespecified, matched three-arm source-robustness ablation."""
import argparse,subprocess,sys
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from contact_transfer.robustness_study import PROTOCOL,run_trial
from contact_transfer.full_recording import write_json

if __name__=='__main__':
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,default=Path('outputs/source_robustness_v2'));p.add_argument('--web',type=Path,default=Path('web'));p.add_argument('--mode',choices=['polytope','nominal']);p.add_argument('--force',type=float);p.add_argument('--workers',type=int,default=2);a=p.parse_args()
 if a.mode:
  if a.force not in PROTOCOL['force_levels_N']:p.error('Use a prespecified force level')
  run_trial(a.output,a.mode=='polytope',a.force,a.web)
 else:
  write_json(a.output/'protocol.json',PROTOCOL)
  # Each mode runs sequentially so authored URDF/plan files are never raced.
  # The summary checks identical native case/plan IDs across force levels.
  def mode(name):
   for force in PROTOCOL['force_levels_N']:
    subprocess.run([sys.executable,'-u',__file__,'--output',str(a.output),'--web',str(a.web),'--mode',name,'--force',str(force)],check=True)
  with ThreadPoolExecutor(max_workers=min(a.workers,2)) as pool:list(pool.map(mode,['polytope','nominal']))
