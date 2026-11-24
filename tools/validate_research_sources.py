"""Validate every task source pose, retaining the legacy single-P failures."""
import argparse
from pathlib import Path
from time import perf_counter

from contact_transfer.full_pipeline import source_solver_settings
from contact_transfer.full_recording import write_json
from contact_transfer.full_scenario import FullTask,FullProblem


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=Path('outputs/research-study/sources'))
    parser.add_argument('--task',choices=['panel','box','assembly'],action='append')
    args=parser.parse_args();report={};success=True
    for task_id in args.task or ['panel','box','assembly']:
        for mode in (['representative'] if task_id=='panel' else ['representative','functional_sites']):
            folder=args.output/task_id/mode
            override={} if mode=='representative' else {'source':{'representation':'functional_group_sites','site_file':f'package:scenarios/certified_sources/{task_id}.json','failure_policy':'error'}}
            task=FullTask.load(task_id,folder/'task',configuration=override)
            problem=FullProblem.create(task,folder/'planning',task.configuration.default_types,
                source_settings=source_solver_settings())
            rows=[]
            for key in task.keyframes:
                started=perf_counter()
                try:record=problem.source(key)['record']
                except (RuntimeError,ValueError) as error:record=dict(source_certificate=False,source_failure=str(error))
                row=dict(keyframe=key['id'],wall_time_s=perf_counter()-started,**record)
                rows.append(row);write_json(folder/'result.json',dict(task=task_id,mode=mode,keyframes=rows))
                print(task_id,mode,key['id'],record.get('source_certificate'),record.get('source_rho',record.get('source_rho_after')),flush=True)
            report[task_id+'/'+mode]=dict(certified=sum(bool(r['source_certificate']) for r in rows),total=len(rows),rows=rows)
            if (mode=='functional_sites' or task_id=='panel') and not all(r['source_certificate'] for r in rows):success=False
    write_json(args.output/'summary.json',report)
    return 0 if success else 1


if __name__=='__main__':raise SystemExit(main())
