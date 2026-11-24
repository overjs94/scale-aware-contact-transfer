"""Keep every controller outcome and compact evidence, including failed pilots."""
import hashlib
import json
from pathlib import Path
import shutil
import numpy as np

from contact_transfer.full_recording import write_json


def main():
    target=Path('docs/validation/controller_2026-11-24');target.mkdir(parents=True,exist_ok=True);trials=[]
    sources=['controller-study','controller-final','controller-v3','controller-v5']
    for name in sources:
        root=Path('outputs')/name
        for source in sorted(root.glob('*/*/*/result.json')):
            relative=source.parent.relative_to(root);folder=target/name/relative;folder.mkdir(parents=True,exist_ok=True)
            for pattern in ['result.json','protocol.json','code-manifest.json','export-identity-migration.json','native/allocation.json','native/allocation-scope.json','native/reference-geometry.json','native/scene.xml']:
                path=source.parent/pattern
                if path.exists():(folder/Path(pattern).name).write_bytes(path.read_bytes())
            with np.load(source.parent/'native/execution.npz',allow_pickle=False) as archive:metadata=json.loads(str(archive['metadata_json']))
            if 'articulated_state' in metadata:
                state=metadata['articulated_state'];metadata['articulated_state']={k:v for k,v in state.items() if k not in {'q','v'}};metadata['articulated_state']['recorded_samples']=len(state.get('q',[]))
            write_json(folder/'trace-metadata.json',metadata)
            result=json.loads(source.read_text());allocation=source.parent/'native/allocation.json';rows=json.loads(allocation.read_text()) if allocation.exists() else []
            active_bad=[x for x in rows if not x['accepted'] and any(x['active'])]
            task,mode,case=relative.parts
            trials.append(dict(study=name,task=task,mode=mode,case=case,passed=result['execution_passed'],maximum_position_error_m=max(x['maximum_position_error_m'] for x in result['object_tracking'].values()),peak_jaw_force_N=max((max(x['all_time_maximum_per_jaw_N']) for x in result['jaw_force'].values()),default=0),allocation_count=len(rows),rejected_solves=sum(not x['accepted'] for x in rows),active_rejections=len(active_bad),partial_rank_updates=sum(not x.get('full_dynamics_equality_enforced',True) for x in rows),report=str(folder/'result.json'),trace_sha256=result['controller_replay']['trace_sha256']))
    for study in ['controller-study','controller-final']:
        root=Path('outputs')/study/'envelope'
        for source in root.glob('*/certificate.json'):
            folder=target/study/'envelope'/source.parent.name;folder.mkdir(parents=True,exist_ok=True);shutil.copyfile(source,folder/source.name)
    for version in ['snapshot','snapshot-v3','snapshot-v5']:
        snapshot=Path('outputs/controller-final')/version
        for name in ['configuration.py','full_scenario.py','paper_execution.py','full_execution.py','full_assessment.py','online_wrench.py','__init__.py']:
            path=snapshot/'src/contact_transfer'/name;folder=target/version/'src/contact_transfer';folder.mkdir(parents=True,exist_ok=True);shutil.copyfile(path,folder/name)
        for name in ['run_controller_replay.py','certify_controller_envelope.py']:
            folder=target/version/'tools';folder.mkdir(parents=True,exist_ok=True);shutil.copyfile(snapshot/'tools'/name,folder/name)
        shutil.copyfile(snapshot/'manifest.json',target/version/'manifest.json')
    interrupted=Path('outputs/controller-v3/assembly/adaptive/full/interrupted.json');folder=target/'controller-v3/assembly/adaptive/full';folder.mkdir(parents=True,exist_ok=True);shutil.copyfile(interrupted,folder/interrupted.name)
    checks=target/'checks';checks.mkdir(exist_ok=True)
    for name in ['controller-all-tests.log','controller-final-tests.log','controller-projection-tests.log','controller-build-final.log','controller-wheel-check.log','controller-final-export.log','controller-web-tests.log','controller-web-final.log','controller-real-mesh.log','controller-real-mesh-initial.log','controller-evidence-check.log','controller-review-export.log']:
        path=Path('/tmp')/name
        if path.exists():shutil.copyfile(path,checks/name)
    screenshots=target/'screenshots';screenshots.mkdir(exist_ok=True)
    for path in Path('outputs/controller-screenshots').glob('*.png'):shutil.copyfile(path,screenshots/path.name)
    write_json(target/'summary.json',dict(schema_version=1,trials=trials,baseline_parent_revision='21b1edfae4405014ab1d9363ecf3363e31544ecd',full_NLP_reoptimized=False,all_outcomes_retained=True))
    # Only selected representatives enter the viewer table; the full ledger
    # above preserves repeats, intermediate failures and their exact policies.
    preferred={('box','allocation','opposite'):('controller-final','Box · fixed 24 N + allocation'),('box','adaptive','opposite'):('controller-final','Box · adaptive jaw + allocation'),('box','baseline','opposite'):('controller-v5','Box · original fixed 24 N'),('box','preload','opposite'):('controller-v5','Box · 30 N preload only'),('panel','adaptive','early'):('controller-final','Panel · original early disturbance'),('assembly','adaptive','early'):('controller-final','Assembly · original early disturbance'),('panel','adaptive','full'):('controller-v3','Panel · complete nominal task'),('box','adaptive','full'):('controller-v3','Box · complete nominal task'),('assembly','adaptive','full'):('controller-v5','Assembly · complete nominal task')}
    displayed=[]
    for row in trials:
        selection=preferred.get((row['task'],row['mode'],row['case']))
        if selection and row['study']==selection[0]:displayed.append(dict(row,title=selection[1],recording_id='research_controller_'+row['task']+'_'+row['mode']+'_'+row['case']))
    original=json.loads(Path('outputs/research-study/probes-v1/box/on/opposite_interior/result.json').read_text())
    if not any(r.get('mode')=='baseline' for r in displayed):displayed.insert(0,dict(title='Box · original fixed 24 N',task='box',passed=original['execution_passed'],maximum_position_error_m=original['robustness_probe']['response']['maximum_position_error_m'],allocation_count=0,rejected_solves=0,recording_id='research_probe_box_on_opposite'))
    write_json(Path('web/data/controller-study-summary.json'),dict(trials=displayed,all_outcomes=str(target/'summary.json'),new_NLP_executed=False))
    write_json(target/'files.sha256.json',{str(p.relative_to(target)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(target.rglob('*')) if p.is_file() and p.name!='files.sha256.json'})
    print(len(trials),'physical trials collected')


if __name__=='__main__':main()
