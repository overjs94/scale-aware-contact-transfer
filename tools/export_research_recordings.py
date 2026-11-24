"""Export current qualified trials under new IDs, preserving historical records."""
import argparse
from dataclasses import replace
import hashlib
import json
from pathlib import Path

import numpy as np

from contact_transfer.certified_disturbance import probe_episode
from contact_transfer.execution import ExecutionTrace
from contact_transfer.equation_contract import audit_episode
from contact_transfer.full_execution import FullEpisode
from contact_transfer.full_recording import load_key,load_interval,write_json
from contact_transfer.full_scene import build_scene
from contact_transfer.full_trajectory import initial_interval,interval_contacts
from contact_transfer.full_validation import validate_keyframe,validate_interval
from contact_transfer.full_view import export_full
from contact_transfer.robustness_study import make_plan
from export_full_recording import load_recorded_plan,original_identity
from run_certified_full_disturbance import inserted_hold


TITLES={'panel':'Panel reorientation, seating and release','box':'Free articulated box opening','assembly':'Tabletop–leg insertion and release'}


def emit(episode,folder,report,web,dataset,title,**metadata):
    trace=ExecutionTrace.load(folder/'native/execution.npz')
    scene=build_scene(episode.problem,folder/'export-scene-check.xml',timestep=trace.metadata['timestep'])
    if hashlib.sha256(scene.path.read_bytes()).hexdigest()!=trace.metadata['scene_sha256']:raise RuntimeError(f'{dataset}: physical scene differs')
    if abs(episode.duration-trace.time[-1])>1.1*trace.metadata['timestep']:raise RuntimeError(f'{dataset}: reference duration differs')
    data=export_full(episode,trace,report,scene,web,dataset_id=dataset,
        recording_metadata=dict(title=title,**metadata))
    print(dataset,report['execution_passed'],flush=True)
    return data,trace


def full_tasks(root,web,tasks=TITLES):
    for task in tasks:
        folder=root/'full-common'/task
        if not (folder/'ablation-results.json').exists():raise RuntimeError('Complete ablations before export')
        matrix=json.loads((folder/'ablation-results.json').read_text())['variants']
        p,keys,rs,cs,report=load_recorded_plan(root/'full-common',task,model_directory=folder/'export-models')
        full=FullEpisode(p,keys,rs,cs)
        ids={variant:f'research_full_{task}'+('' if variant=='full' else '_'+variant) for variant in matrix}
        for variant,r in matrix.items():
            r=dict(r)
            episode=full
            if variant in {'no_source_refinement','no_target_robustness'}:
                other,kk,rr,cc,_=load_recorded_plan(root/'full-common',task,variant,model_directory=folder/variant/'export-models')
                episode=FullEpisode(other,kk,rr,cc)
            elif variant=='fixed_path':
                rr=[];cc=[]
                for i,(left,right,old) in enumerate(zip(keys[:-1],keys[1:],rs)):
                    contact=interval_contacts(p,i,left,len(old.dt));rr.append(initial_interval(p,i,left,right,len(old.dt),float(old.dt.sum()),contact));cc.append(contact)
                episode=FullEpisode(p,keys,tuple(rr),tuple(cc))
            post=audit_episode(episode)
            write_json(folder/variant/'post-release-equation-audit.json',post)
            r.update(post_release_equation_contract=post,as_run_full_framework_pipeline_certified=r.get('full_framework_pipeline_certified'),full_framework_pipeline_certified=bool(post['full_framework_stage_execution'] and r['execution_passed']))
            data,_=emit(episode,folder/variant,r,web,ids[variant],TITLES[task]+(' · '+variant if variant!='full' else ''),
                study_recordings=ids,full_task_recording=True,scenario='full_'+task)
            data['report']['ablation_results']=matrix
            write_json(web/'data'/(ids[variant]+'.json'),data,compact=True)


def source_margin(root,web):
    trials=[];folder=root/'source-margin-v1';protocol=json.loads((folder/'protocol.json').read_text())
    for mode in ['on','off']:
        plan=folder/mode/'plan';original=json.loads((plan/'plan.json').read_text())['source']
        object_path=Path('src/contact_transfer/scenarios/margin_platform/panel.urdf')
        template,_=make_plan(folder/mode/'export-models',mode=='on',hold_duration_s=13.,object_urdf=object_path)
        p=template.problem;positions=np.asarray(original['target_positions_m'])
        features=tuple(replace(f,surface=point,tcp=point) for f,point in zip(p.task_definition.features,positions))
        p.task_definition=replace(p.task_definition,features=features)
        keys=tuple(load_key(plan/f'key{i}.npz',original_identity(plan/f'key{i}.npz')) for i in range(3))
        rs=tuple(load_interval(plan/f'interval{i}.npz',original_identity(plan/f'interval{i}.npz')) for i in range(2))
        cs=tuple(interval_contacts(p,i,keys[i],len(r.dt)) for i,r in enumerate(rs));episode=FullEpisode(p,keys,rs,cs)
        if not all(validate_keyframe(p,k)['passed'] for k in keys) or not all(validate_interval(p,r,c)['passed'] for r,c in zip(rs,cs)):raise RuntimeError('Restored source-margin plan rejected')
        for case in ['nominal','common_up1','slow_up4']:
            path=folder/mode/case;r=json.loads((path/'result.json').read_text());dataset=f'research_source_margin_{mode}_{case}'
            data,trace=emit(episode,path,r,web,dataset,f'Source robustness {mode.upper()} · '+protocol['protocol']['cases'][case]['title'],
                scenario='source_robustness_study',full_task_recording=False,comparison_recording=True,study_kind='source_robustness')
            summary=json.loads((path/'summary.json').read_text());summary.update(recording_id=dataset,external_disturbances=trace.metadata['external_disturbances']);trials.append(summary)
    protocol['protocol']['cases']={k:v for k,v in protocol['protocol']['cases'].items() if k in {'nominal','common_up1','slow_up4'}}
    controllers=[r['controller_configuration'] for r in trials]
    if any(c!=controllers[0] for c in controllers):raise RuntimeError('Source margin controllers are not matched')
    for case in ['nominal','common_up1','slow_up4']:
        pair=[r for r in trials if r['case']==case]
        if pair[0]['native_case_id']!=pair[1]['native_case_id'] or pair[0]['external_disturbances']!=pair[1]['external_disturbances']:raise RuntimeError('Source margin scene/load differs')
    write_json(web/'data/research-source-margin-comparison.json',dict(schema_version=1,protocol=protocol,trials=trials,
        design_history='Current-code reproduction of the declared tall-payload source-refinement diagnostic. All six new trials and failures are retained. Historical exploratory studies remain available in the original comparison.',flat_panel_exploratory_trials=[],compact_payload_exploratory_trials=[]))


def probes(root,web,loaded=False,tasks=TITLES,probe_case='on_interior'):
    study='full-load-v1' if loaded else 'probes-v1'
    for task in tasks:
        protocol=json.loads((root/'probes-v1'/task/'protocol.json').read_text());k=protocol['optimized_keyframe_index']
        for mode,variant in [('on','full'),('off','no_target_robustness')]:
            folder=root/study/task/mode
            p,kk,rr,cc,_=load_recorded_plan(root/'full-common',task,variant,model_directory=folder/'export-parent')
            original=FullEpisode(p,kk,rr,cc)
            if loaded:episode,_=inserted_hold(original,k,folder/'export-reference')
            else:episode,_=probe_episode(original,k,folder/'export-reference')
            case='challenge' if loaded else probe_case;path=folder/case;r=json.loads((path/'result.json').read_text());suffix='_opposite' if not loaded and case=='opposite_interior' else ''
            emit(episode,path,r,web,f'research_{"loaded" if loaded else "probe"}_{task}_{mode}'+suffix,TITLES[task]+f' · {"full task with load" if loaded else "opposite load probe" if suffix else "keyframe load probe"} · {mode.upper()}',
                scenario='certified_full_sequence' if loaded else 'certified_keyframe_probe',full_task_recording=loaded,comparison_recording=True,
                study_kind='full_target_robustness' if loaded else 'target_robustness',parent_task=task)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--root',type=Path,default=Path('outputs/research-study'));parser.add_argument('--web',type=Path,default=Path('web'));parser.add_argument('--part',choices=['full','source','probes','loaded','types'],required=True);parser.add_argument('--tasks',nargs='+',choices=list(TITLES),default=list(TITLES));parser.add_argument('--probe-case',choices=['on_interior','opposite_interior'],default='on_interior');args=parser.parse_args()
    if args.part=='full':full_tasks(args.root,args.web,args.tasks)
    elif args.part=='source':source_margin(args.root,args.web)
    elif args.part=='types':
        for task in args.tasks:
            for path in sorted((args.root/'types-v1'/task).glob('*/result.json')):
                r=json.loads(path.read_text())
                if not r.get('new_physical_trial_executed'):continue
                p,kk,rr,cc,_=load_recorded_plan(args.root/'types-v1',task,path.parent.name,model_directory=path.parent/'export-models');episode=FullEpisode(p,kk,rr,cc)
                emit(episode,path.parent,r,args.web,f'research_type_{task}_{path.parent.name}',TITLES[task]+' · forced '+path.parent.name,scenario='type_counterfactual_study',full_task_recording=True,comparison_recording=True,study_kind='contact_types',parent_task=task)
    else:probes(args.root,args.web,loaded=args.part=='loaded',tasks=args.tasks,probe_case=args.probe_case)
