#!/usr/bin/env python3
"""Force an alternative hardware-compatible type through the complete planner.

Retain local solver failures separately from physical failures. The selected
baseline is never relabelled or replayed as an alternative candidate.
"""
import argparse,json
from importlib.resources import files
from pathlib import Path
from time import perf_counter
from contact_transfer.full_scenario import FullTask
from contact_transfer.full_pipeline import FullPlanner,scenario_solver_settings
from contact_transfer.full_execution import run_full_episode
from contact_transfer.full_assessment import assess
from contact_transfer.full_recording import write_json
from contact_transfer.full_view import export_full

if __name__=='__main__':
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--task',choices=['panel','box','assembly'],required=True);p.add_argument('--types',required=True);p.add_argument('--archive',type=Path,help='Optional earlier keyframe seeds, always independently revalidated');p.add_argument('--output',type=Path,default=Path('outputs/type_counterfactual_v1'));p.add_argument('--web',type=Path);p.add_argument('--research-profile',action='store_true');a=p.parse_args();folder=a.output/a.task/a.types;start=perf_counter();folder.mkdir(parents=True,exist_ok=True)
 try:
  configuration=None
  if a.research_profile:
   configuration=json.loads(Path(files('contact_transfer').joinpath(f'scenarios/certified_sources/{a.task}-configuration.json')).read_text())
   configuration.setdefault('transition',{})['configuration_update']='contact_coordinate_midpoint'
  task=FullTask.load(a.task,folder/'task',configuration=configuration);write_json(folder/'resolved-task-configuration.json',task.configuration.to_dict());planner=FullPlanner(task,folder/'planning',settings=scenario_solver_settings(),keyframe_archive=a.archive);episode,selection=planner.search(forced_types=tuple(a.types),forced_assignment=(0,1,2));write_json(folder/'selection.json',selection)
  trace,controller,scene,geometry=run_full_episode(episode,folder/'native',progress=lambda t,total:print(f'{a.task}/{a.types}: {t:.0f}/{total:.1f}s',flush=True) if int(t)%5==0 else None);report=assess(episode,trace,controller,scene,geometry=geometry);report.update(selection=selection,comparison_stage='native_execution',full_variant_pipeline_executed=True,new_physical_trial_executed=True)
  if a.web is not None:export_full(episode,trace,report,scene,a.web,dataset_id=f'counterfactual_{a.task}_{a.types}',recording_metadata=dict(title=f'{a.task.title()} · forced types {" / ".join(a.types)}',scenario='type_counterfactual_study',full_task_recording=True,comparison_recording=True,study_kind='contact_types',description='Same source, objective, assignment, mechanics and controller; full planning and native execution of a forced alternative type. Rejections remain in the report.'))
 except (RuntimeError,ValueError) as error:
  report=dict(execution_passed=False,new_physical_trial_executed=False,comparison_stage='planning',failure=str(error),local_solver_failure_is_not_global_infeasibility=True,full_paper_pipeline_executed=False)
  ledger=folder/'planning/candidate-ledger.json'
  if ledger.exists():report['candidate_rejections']=json.loads(ledger.read_text())
 report.update(task=a.task,forced_types=list(a.types),assignment=[0,1,2],wall_time_s=perf_counter()-start);write_json(folder/'result.json',report);print(a.task,a.types,report.get('comparison_stage'),report['execution_passed'],flush=True)
