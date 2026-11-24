"""Collect compact, portable research evidence without native trajectory blobs."""
import argparse
from datetime import datetime,timezone
import hashlib
import json
from pathlib import Path
import shutil
import subprocess

from contact_transfer.full_recording import write_json


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root',type=Path,default=Path('outputs/research-study'))
    parser.add_argument('--output',type=Path,default=Path('docs/validation/research_2026-11-24'))
    args=parser.parse_args();root=args.root;out=args.output;out.mkdir(parents=True,exist_ok=True)
    copied=[]
    def copy(source,target):
        if not source.exists():return
        destination=out/target;destination.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(source,destination);copied.append(destination)
    for label in ['sources-final','source-diagnosis','literal-audit','probes-v1','full-load-v1','source-margin-v1','types-v1']:
        for f in sorted((root/label).rglob('*')):
            if not f.is_file() or 'native' in f.parts:continue
            if label in {'probes-v1','full-load-v1','source-margin-v1','types-v1'}:
                keep=f.name in {'result.json','summary.json','protocol.json','selection.json','workers.json','candidate-ledger.json'}
                if label=='types-v1':keep=keep or f.name.endswith('-failure.json')
            else:keep=f.suffix in {'.json','.npz','.urdf'} and 'witness-models' not in f.parts
            if keep:copy(f,Path(label)/f.relative_to(root/label))
    records={}
    for task in ['panel','box','assembly']:
        folder=root/'full-common'/task;records[task]={}
        for f in folder.glob('*.json'):copy(f,Path('plans')/task/f.name)
        for variant in ['full','no_source_refinement','fixed_contact_types','no_target_robustness','fixed_path','external_push']:
            source=folder/variant
            for f in source.glob('*.json'):copy(f,Path('plans')/task/variant/f.name)
            for f in (source/'planning').rglob('*'):
                if f.is_file() and (f.suffix in {'.npz','.json','.urdf'}):copy(f,Path('plans')/task/variant/'planning'/f.relative_to(source/'planning'))
            if (source/'plan-origin.json').exists():
                file=out/'plans'/task/variant/'plan-origin.json';origin=json.loads(file.read_text());origin['bundle_relative_planning_directory']='../full/planning/'+Path(origin['planning_directory']).name;write_json(file,origin)
            result=source/'result.json'
            if result.exists():
                d=json.loads(result.read_text());trace=source/'native/execution.npz';selection=d.get('selection',{})
                audit=source/'post-release-equation-audit.json'
                current=json.loads(audit.read_text()) if audit.exists() else None
                records[task][variant]=dict(execution_passed=d['execution_passed'],task_passed=d.get('task_passed'),
                    full_framework_pipeline_certified=bool(current['full_framework_stage_execution'] and d['execution_passed']) if current is not None else None,
                    as_run_full_framework_pipeline_certified=d.get('full_framework_pipeline_certified',False),
                    full_paper_pipeline_executed=d.get('full_paper_pipeline_executed',False),
                    selected_types=selection.get('selected_types'),assignment=selection.get('selected_assignment'),
                    sequence_objective=selection.get('objective'),cold_planning_wall_time_s=selection.get('planning_wall_time_s'),
                    objective_comparable_to_full=variant not in {'fixed_path','no_target_robustness'},
                    object_tracking=d.get('object_tracking'),contact_retention=d.get('maintained_contact_fraction'),
                    failure=d.get('failure'),external_disturbance_certificate=d.get('external_disturbance_certificate'),
                    scene_sha256=d.get('scene_sha256'),native_trace_sha256=sha(trace) if trace.exists() else None,
                    evidence=str(Path('plans')/task/variant/'result.json'))
    diagnostic=root/'full-v1/assembly/diagnostic-stop.json';copy(diagnostic,Path('diagnostics/assembly-independent-midpoint-stop.json'))
    snapshot=root/'snapshot-v1/manifest.json'
    if not snapshot.exists():snapshot=root/'snapshot-v1/snapshot-manifest.json'
    copy(snapshot,Path('as-run-snapshot.json'))
    source=Path('src/contact_transfer');write_json(out/'release-source-snapshot.json',dict(parent_commit=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),files={str(p):sha(p) for p in sorted(source.rglob('*')) if p.is_file() and p.suffix in {'.py','.json','.urdf'}}))
    lognames=['research-full-pytest','research-release-core','research-dual-certificate-test','research-stage-badge-test','research-web-tests','research-demo-refinement','research-dual-arm','research-fixed-wrench','research-three-arm','research-installed-wheel','research-literal-audit']
    for name in lognames:
        for suffix in ['.log','.json']:
            copy(Path('/tmp')/(name+suffix),Path('checks')/(name+suffix))
    copy(Path('/tmp/research-installed-wheel-check.py'),Path('checks/installed-wheel-check.py'))
    copy(Path('/tmp/research-web-current-probes.log'),Path('checks/current-web-probes.log'))
    copy(Path('/tmp/research-web-current-primary.log'),Path('checks/current-web-primary.log'))
    for name in ['research-web-current-all.log','research-final-rank-test.log','research-review-distribution-tests.log','research-literal-component-tests.log','research-anonymous-export.log','research-web-presentation-final.log','research-web-disturbance-final.log','research-real-render.log']:
        copy(Path('/tmp')/name,Path('checks')/name)
    copy(root/'study-summary.json',Path('study-results.json'))
    summary=dict(schema_version=1,release='0.2.0a2',created_utc=datetime.now(timezone.utc).isoformat(),
        manuscript_reference='existing supplied-PDF equation audit; PDF not reopened',
        source_qualification=json.loads((root/'sources-final/summary.json').read_text()),
        full_tasks_and_ablations=records,
        scope=dict(supplied_contacts=True,learned_inference_executed=False,grouped_source_is_extension=True,full_paper_literal_verification_completed=False,continuous_geometry_certified=False,CRISP_executed=False),
        compact_artifacts=True,full_native_npz_distributed=False)
    write_json(out/'summary.json',summary)
    manifest={str(p.relative_to(out)):sha(p) for p in sorted(out.rglob('*')) if p.is_file() and p.name!='files.sha256.json'}
    write_json(out/'files.sha256.json',manifest);print(len(manifest),'artifacts',sum(p.stat().st_size for p in out.rglob('*') if p.is_file()),'bytes')


if __name__=='__main__':main()
