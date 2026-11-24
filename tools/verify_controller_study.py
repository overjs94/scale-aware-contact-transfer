"""Independent integrity/matching checks; never turn solver status into success."""
import hashlib
import json
from pathlib import Path


def read(path):return json.loads(Path(path).read_text())


def main():
    archive=Path('docs/validation/controller_2026-11-24');manifest=read(archive/'files.sha256.json')
    for path,digest in manifest.items():assert hashlib.sha256((archive/path).read_bytes()).hexdigest()==digest,path
    trials=read(archive/'summary.json')['trials'];assert len(trials)==19
    assert sum(x['passed'] for x in trials)==15
    box=[]
    for study,mode in [('controller-v5','baseline'),('controller-v5','preload'),('controller-final','allocation'),('controller-final','adaptive')]:
        folder=archive/study/'box'/mode/'opposite';box.append(read(folder/'trace-metadata.json'))
    for field in ['scene_sha256','plan_id','external_disturbances']:
        assert all(x[field]==box[0][field] for x in box),field
    for task in ['panel','assembly']:
        old=read(f'web/data/research_full_{task}_external_push.json');new=read(archive/'controller-final'/task/'adaptive/early/trace-metadata.json')
        for field in ['scene_sha256','plan_id']:assert old['provenance'][field]==new[field],(task,field)
        assert old['external_disturbances']==new['external_disturbances'],task
    for study,task in [('controller-v3','panel'),('controller-v3','box'),('controller-v5','assembly')]:
        folder=archive/study/task/'adaptive/full';report=read(folder/'result.json');rows=read(folder/'allocation.json')
        assert report['execution_passed'] and all(x['accepted'] for x in rows)
        assert max(x['residual'] for x in rows)<2e-5
        if task=='assembly':assert sum(x['controlled_dynamics_rank']==6 for x in rows)==144
    for version in ['snapshot','snapshot-v3','snapshot-v5']:
        folder=archive/version;files=read(folder/'manifest.json')['files']
        for path in folder.rglob('*.py'):assert hashlib.sha256(path.read_bytes()).hexdigest()==files[str(path.relative_to(folder))],path
    for trial in trials:
        path=Path('outputs')/trial['study']/trial['task']/trial['mode']/trial['case']/'native/execution.npz'
        assert hashlib.sha256(path.read_bytes()).hexdigest()==trial['trace_sha256'],path
    print(f'Verified {len(manifest)} evidence hashes, all 19 raw traces, matched scenes/plans/loads, and complete-task QP scope.')


if __name__=='__main__':main()
