"""Current-code full tasks and controlled ablations with explicit provenance."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import hashlib
from importlib.resources import files
import json
from pathlib import Path
import subprocess
import sys

from contact_transfer.full_execution import FullEpisode
from contact_transfer.full_pipeline import run_task,scenario_solver_settings
from contact_transfer.full_recording import write_json
from export_full_recording import load_recorded_plan


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--root',type=Path,default=Path('outputs/research-study/full-v1'))
    p.add_argument('--tasks',nargs='+',choices=['panel','box','assembly'],default=['panel','box','assembly'])
    p.add_argument('--phase',choices=['full','ablations'],default='full')
    p.add_argument('--variants',nargs='+',default=['no_source_refinement','fixed_contact_types','no_target_robustness','fixed_path','external_push'])
    p.add_argument('--workers',type=int,default=3)
    p.add_argument('--web',type=Path)
    p.add_argument('--configuration-update',choices=['independent_midpoint','contact_coordinate_midpoint','independent_se3'],default='contact_coordinate_midpoint')
    args=p.parse_args()
    if len(args.tasks)>1:
        def task(name):
            cmd=[sys.executable,'-u',__file__,'--root',str(args.root),'--tasks',name,'--phase',args.phase,'--variants',*args.variants,'--workers','1','--configuration-update',args.configuration_update]
            if args.web is not None:cmd+=['--web',str(args.web)]
            log=args.root/(name+'-'+args.phase+'.log');log.parent.mkdir(parents=True,exist_ok=True)
            with log.open('w') as handle:
                result=subprocess.run(cmd,stdout=handle,stderr=subprocess.STDOUT)
            return dict(task=name,exit_code=result.returncode,log=str(log))
        with ThreadPoolExecutor(max_workers=max(1,args.workers)) as pool:results=list(pool.map(task,args.tasks))
        write_json(args.root/(args.phase+'-workers.json'),results)
        return 0 if all(r['exit_code']==0 for r in results) else 1
    task_id=args.tasks[0];folder=args.root/task_id
    configuration=json.loads(Path(files('contact_transfer').joinpath(f'scenarios/certified_sources/{task_id}-configuration.json')).read_text())
    configuration.setdefault('transition',{})['configuration_update']=args.configuration_update
    source=Path(files('contact_transfer'))
    protocol=dict(version=1,task=task_id,phase=args.phase,
        configuration=configuration,
        formulation_sha256={f.name:hashlib.sha256(f.read_bytes()).hexdigest() for f in source.glob('*.py')},
        algorithms='same source/target/type-selection/restricted-transition/native gates for every task',
        physical_inputs_and_acceptance_thresholds_ablated=False,
        nominal_trials_per_variant=1,statistical_success_rate_claimed=False)
    write_json(folder/(args.phase+'-protocol.json'),protocol)
    accepted=None
    if args.phase=='ablations':
        problem,keys,rs,contacts,baseline=load_recorded_plan(args.root,task_id,model_directory=folder/'analysis-models')
        if not baseline['execution_passed']:raise RuntimeError('Full native baseline must pass before controlled ablations')
        if not all(k.source.get('source_certificate') for k in keys):raise RuntimeError('Source certificates are required for this study')
        accepted=(FullEpisode(problem,keys,rs,contacts),baseline['selection'])
    results=run_task(task_id,folder,variants=['full'] if args.phase=='full' else args.variants,
        configuration=configuration,settings=scenario_solver_settings(),accepted_full=accepted,web_directory=args.web)
    print({v:r['execution_passed'] for v,r in results.items()},flush=True)
    return 0 if args.phase!='full' or results['full']['execution_passed'] else 1


if __name__=='__main__':raise SystemExit(main())
