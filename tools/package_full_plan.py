#!/usr/bin/env python3
"""Package compact accepted stage certificates; raw native traces stay external."""
import argparse,hashlib,json,shutil,re
from pathlib import Path
from export_full_recording import load_recorded_plan
from contact_transfer.full_recording import write_json

if __name__=='__main__':
 parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--root',type=Path,required=True);parser.add_argument('--task',choices=['panel','box','assembly'],required=True);parser.add_argument('--destination',type=Path,default=Path('examples/recordings/full_tasks'));args=parser.parse_args();p,keys,rs,cs,report=load_recorded_plan(args.root,args.task)
 if not report['execution_passed']:raise SystemExit('Package only an accepted full native trial')
 target=args.destination/args.task;candidate=target/'full/planning'/p.directory.name;candidate.mkdir(parents=True,exist_ok=True)
 for source in p.directory.iterdir():
  if source.is_file() and (source.suffix=='.urdf' or re.fullmatch(r'(key|interval)\d+\.npz',source.name)):shutil.copyfile(source,candidate/source.name)
 provenance=target/'provenance';provenance.mkdir(exist_ok=True)
 for source in p.task_definition.original.path.parent.iterdir():
  if source.is_file() and (source.name.startswith('LICENSE') or 'provenance' in source.name):shutil.copyfile(source,provenance/source.name)
 for source,name in [(Path('.cache/robotiq_2f85/LICENSE'),'LICENSE.robotiq'),(Path('.cache/robotiq_2f85/provenance.json'),'robotiq-provenance.json'),(Path('src/contact_transfer/models/robotiq_2f85/settings.json'),'robotiq-simulation-settings.json'),(Path('src/contact_transfer/models/tools/LICENSE.authored'),'LICENSE.spherical-tool')]:
  if source.is_file():shutil.copyfile(source,provenance/name)

 for source in Path('src/contact_transfer/models/tools').glob('*'):
  if source.is_file():shutil.copyfile(source,provenance/('spherical-tool-'+source.name))
 write_json(target/'full/result.json',report);write_json(candidate.parent/'selection.json',report['selection']);manifest=dict(task=args.task,native_robot_urdf_sha256={r.path.name:hashlib.sha256(r.path.read_bytes()).hexdigest() for r in p.full_robots},canonical_point_tool=next((spec['point_tool_profile'] for spec in p.specs if spec['kind']=='P'),None),source_trial=str(args.root/args.task/'full'),source_execution_sha256=hashlib.sha256((args.root/args.task/'full/native/execution.npz').read_bytes()).hexdigest(),source_native_execution_passed=True,native_trace_included=False,optimization_reexecuted_by_replay=False,fresh_native_replay_required=True,controller_jaw_force_fraction=report.get('jaw_force_fraction',.7),files={str(path.relative_to(target)):hashlib.sha256(path.read_bytes()).hexdigest() for path in sorted(target.rglob('*')) if path.is_file() and path.name!='manifest.json'});write_json(target/'manifest.json',manifest);print(args.task,len(manifest['files']),'files',flush=True)
