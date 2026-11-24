"""Export completed controller trials with fresh scene/plan identity checks."""
from copy import copy
from dataclasses import replace
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace

from export_full_recording import load_recorded_plan
from run_controller_replay import emit_replay
from contact_transfer.certified_disturbance import probe_episode
from contact_transfer.configuration import ExecutionPolicy
from contact_transfer.execution import ExecutionTrace
from contact_transfer.full_execution import FullEpisode
from contact_transfer.full_scene import build_scene


def main():
    cases={'box':[('controller-final','allocation','opposite'),('controller-final','adaptive','opposite'),('controller-v5','baseline','opposite'),('controller-v5','preload','opposite'),('controller-v3','adaptive','full')],
           'panel':[('controller-v3','adaptive','full'),('controller-final','adaptive','early')],
           'assembly':[('controller-v5','adaptive','full'),('controller-final','adaptive','early')]}
    output=Path('outputs/controller-export');output.mkdir(exist_ok=True)
    for task,rows in cases.items():
        p,keys,rs,cs,_=load_recorded_plan('outputs/research-study/full-common',task,model_directory=output/task/'models')
        original=FullEpisode(p,keys,rs,cs);probe=None
        for study,mode,case in rows:
            folder=Path('outputs')/study/task/mode/case;report=json.loads((folder/'result.json').read_text());policy=ExecutionPolicy(**report['controller_replay']['policy'])
            if case=='opposite':
                if probe is None:probe,_=probe_episode(original,2,output/task/'probe')
                template=probe
            else:template=original
            problem=copy(template.problem);problem.task_definition=replace(problem.task_definition,configuration=replace(problem.task_definition.configuration,execution=policy));problem.directory=output/task/'references'/mode/case;problem.directory.mkdir(parents=True,exist_ok=True)
            if hasattr(problem,'_case'):del problem._case
            episode=FullEpisode(problem,template.keyframes,template.trajectories,template.contacts_by_interval)
            trace=ExecutionTrace.load(folder/'native/execution.npz');scene=build_scene(problem,output/task/'export-scene.xml',timestep=trace.metadata['timestep'])
            if episode.identity()!=(trace.metadata['case_id'],trace.metadata['plan_id']):raise RuntimeError(f'{task}/{mode}/{case}: immutable plan/controller identity differs')
            if hashlib.sha256(scene.path.read_bytes()).hexdigest()!=trace.metadata['scene_sha256']:raise RuntimeError('Native scene differs')
            data=emit_replay(episode,trace,report,scene,SimpleNamespace(task=task,mode=mode,case=case,web=Path('web')))
            print(data['id'],report['execution_passed'],flush=True)


if __name__=='__main__':main()
