"""Postprocess local raw native controls, using the bundled accepted plans.

Large control.json traces remain in outputs; their SHA-256 values are reported.
"""
from pathlib import Path
import json,hashlib
import numpy as np
base=Path(__file__).resolve().parents[4];evidence=base/'docs/validation/framework_upgrade_2026-11-23';results={};control_hashes={}
for task in ['panel','box','assembly']:
 folder=evidence/'plans'/task;report=json.loads((folder/'full/result.json').read_text());selection=report['selection'];candidate=folder/'full/planning'/(''.join(selection['selected_types'])+'_012');
 with np.load(candidate/'key0.npz',allow_pickle=False) as a:groups=[entry['group_id'] for entry in json.loads(str(a['metadata_json']))['interfaces']]
 intervals=[];phases=[];offsets=[0.]
 for i in range(len(selection['actual_interval_nodes'])):
  with np.load(candidate/f'interval{i}.npz',allow_pickle=False) as a:intervals.append({k:a[k].copy() for k in a.files if k=='dt' or k.startswith('impulse_')})
  offsets.append(offsets[-1]+intervals[-1]['dt'].sum())
  with np.load(candidate/f'key{i}.npz',allow_pickle=False) as a:phases.append(json.loads(str(a['metadata_json']))['phase'])
 for suffix in ['conditioned','hybrid-d160']:
  name=task+'-'+suffix;path=base/'outputs/framework-upgrade/control-variants'/name/'native/control.json';control_hashes[name]=hashlib.sha256(path.read_bytes()).hexdigest();logs=json.loads(path.read_text());times=np.array([row['time'] for row in logs]);err=np.array([row['force_error'] for row in logs]);local=np.clip(times-2.,0,offsets[-1]);which=np.clip(np.searchsorted(offsets,local,side='right')-1,0,len(intervals)-1);out={}
  for arm,kind in enumerate(selection['selected_types']):
   if kind!='P':continue
   mask=np.zeros(len(times),bool)
   for i,r in enumerate(intervals):
    if phases[i] in ['unload','released']:continue
    idx=np.flatnonzero((times>=2.1)&(which==i));k=np.clip(np.searchsorted(np.r_[0.,np.cumsum(r['dt'])],local[idx]-offsets[i],side='right')-1,0,len(r['dt'])-1);mask[idx]=r['impulse_'+groups[arm]][k,2]/r['dt'][k]>.1
   values=err[mask,arm,5];out[groups[arm]]=dict(samples=len(values),rmse_N=float(np.sqrt(np.mean(values**2))),maximum_absolute_error_N=float(np.max(abs(values))))
  results[name]=out
report=dict(raw_control_sha256=control_hashes,scope='Postprocessing existing raw native contact force errors; P interfaces only; t>=preparation 2s + 0.1s, holding phase and planned normal load>0.1N; not an additional acceptance gate or physical trial',results=results)
(evidence/'normal-force-errors.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
