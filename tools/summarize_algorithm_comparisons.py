#!/usr/bin/env python3
"""Publish completed matched comparisons with plan identity and common gates."""
import argparse,json,hashlib
import numpy as np
from pathlib import Path
from scipy.spatial.transform import Rotation
from contact_transfer.execution import ExecutionTrace
from contact_transfer.full_recording import write_json
from contact_transfer.robustness_study import PROTOCOL


def failure_checks(r):
    names=[k for k in ('tracking_passed','task_passed','contact_retention_passed') if r.get(k) is False]
    if any(not j['passed'] for j in r.get('jaw_force',{}).values()):names.append('jaw_force')
    if r.get('sampled_interface_wrench_capability',{}).get('passed') is False:names.append('wrench_capability')
    if r.get('minimum_actuator_margin_Nm',0)<-1e-8:names.append('joint_torque')
    if r.get('unexpected_loaded_contact_penetrations_m'):names.append('unexpected_loaded_contact')
    return names


def summarize(root,types_root,capacities,web):
    trials=[];identities={};controllers={}
    for mode in ('polytope','nominal'):
        mode_ids=set();configs=set()
        for force in PROTOCOL['force_levels_N']:
            folder=root/mode/('force_'+str(int(force)));r=json.loads((folder/'result.json').read_text());trace=ExecutionTrace.load(folder/'native/execution.npz');native=trace.metadata;mode_ids.add((native['case_id'],native['plan_id']));configs.add(json.dumps(native['controller_configuration'],sort_keys=True));source=r['source'];e=r['object_tracking']['panel'];forces=native['external_disturbances']
            if force:
                assert len(forces)==1 and forces[0]['force_world_N']==(force*np.array(PROTOCOL['force_direction_world'])).tolist()
                assert forces[0]['point_body_m']==PROTOCOL['force_application_point_body_m'] and forces[0]['start']==PROTOCOL['pulse_start_s'] and forces[0]['duration']==PROTOCOL['pulse_duration_s']
            else:assert not forces
            coefficient=abs(force)/PROTOCOL['target_force_box_scales_N'][2]
            angles=Rotation.from_quat(trace.object_pose[:,0,3:]).magnitude();angles[trace.time<2.]=0.;peak_time=float(trace.time[int(np.argmax(angles))])
            recording_id=f'robust_refinement_{mode}_{int(force)}N';recording_path=web/'data'/f'{recording_id}.json';recording=json.loads(recording_path.read_text())
            if any(recording['provenance'][key]!=native[key] for key in ('case_id','plan_id','scene_sha256')):raise RuntimeError('Viewer and raw trial identity differ')
            recording['events']=[event for event in recording['events'] if event['id'] not in {'과도응답 비교','최대 자세 오차'}]
            if force:recording['events'].append(dict(id='과도응답 비교',time=4.7))
            recording['events'].append(dict(id='최대 자세 오차',time=peak_time));write_json(recording_path,recording,compact=True)
            trials.append(dict(recording_id=f'robust_refinement_{mode}_{int(force)}N',force_N=force,robustness_enabled=mode=='polytope',execution_passed=r['execution_passed'],source_rho=source['source_rho_after'],target_rho=source['target_rho'],source_optimization_vertex_count=source['optimization_vertex_count'],maximum_position_error_m=e['maximum_position_error_m'],maximum_rotation_error_rad=e['maximum_rotation_error_rad'],final_position_error_m=e['final_position_error_m'],minimum_contact_fraction=min(v for v in r['maintained_contact_fraction'].values() if v is not None),failed_checks=failure_checks(r),normalized_force_box_coefficient=coefficient,inside_initial_static_box=coefficient<=source['target_rho']+1e-8,native_case_id=native['case_id'],native_plan_id=native['plan_id'],native_scene_sha256=native['scene_sha256'],native_trace_sha256=hashlib.sha256((folder/'native/execution.npz').read_bytes()).hexdigest()))
        if len(mode_ids)!=1:raise RuntimeError('Force sweep changed a plan: '+mode)
        if len(configs)!=1:raise RuntimeError('Force sweep changed a controller: '+mode)
        identities[mode]=list(next(iter(mode_ids)));controllers[mode]=next(iter(configs))
    if controllers['polytope']!=controllers['nominal']:raise RuntimeError('ON/OFF controller settings differ')
    assert {t['source_optimization_vertex_count'] for t in trials if t['robustness_enabled']}=={8}
    assert {t['source_optimization_vertex_count'] for t in trials if not t['robustness_enabled']}=={1}
    trials.sort(key=lambda t:(PROTOCOL['force_levels_N'].index(t['force_N']),not t['robustness_enabled']))
    groups={}
    for task in ('panel','box','assembly'):
        group=json.loads((capacities/task/'capacity-comparison.json').read_text());alternatives=[]
        for path in sorted((types_root/task).glob('*/result.json')):
            r=json.loads(path.read_text());s=r.get('selection',{});errors=r.get('object_tracking',{});dataset=f'counterfactual_{task}_'+''.join(r['forced_types']);recording=web/'data'/f'{dataset}.json'
            failure=r.get('failure');ledger=path.parent/'planning/candidate-ledger.json'
            if r['comparison_stage']=='planning' and ledger.exists():
                details=json.loads(ledger.read_text());failure=' · '.join(dict.fromkeys(v['reason'] for v in details)) or failure
            alternatives.append(dict(types=r['forced_types'],assignment=r['assignment'],stage=r['comparison_stage'],execution_passed=r['execution_passed'],new_physical_trial_executed=r['new_physical_trial_executed'],sequence_objective=s.get('objective'),maximum_position_error_m=max((e['maximum_position_error_m'] for e in errors.values()),default=None),maximum_rotation_error_rad=max((e['maximum_rotation_error_rad'] for e in errors.values()),default=None),failed_checks=failure_checks(r),failure=failure,recording_id=dataset if recording.exists() else None))
        group['counterfactuals']=alternatives;groups[task]=group
    result=dict(schema_version=1,source_robustness=dict(protocol=PROTOCOL,trials=trials,matched_plan_ids=identities,controller_settings_equal=True,nominal_trials_per_condition=1,stationary_diagnostic_not_full_motion_CITO=True,source_objective='geometry_weight * sum squared displacement - rho',nominal_objective='same geometry cost, nominal equilibrium only; rho term and disturbed vertices removed'),contact_types=groups,algorithm_scope=dict(network_inference=False,supplied_contracted_groups=True,source_fallbacks_explicit=True,full_motion_integration='independent/contact-coordinate midpoint variants; not literal manuscript Eq.17',arm_control='impedance + planned reaction feedforward; Sf=0',hardware_calibration=False,crisp_cross_validation_of_new_tasks=False,full_paper_pipeline_executed=False))
    write_json(web/'data/algorithm-comparisons.json',result);write_json(Path('docs/algorithm-comparison-results.json'),result);return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--root',type=Path,default=Path('outputs/source_robustness_v2'));p.add_argument('--types-root',type=Path,default=Path('outputs/type_counterfactual_v1'));p.add_argument('--capacities',type=Path,default=Path('outputs/type_capacity_v1'));p.add_argument('--web',type=Path,default=Path('web'));a=p.parse_args();result=summarize(a.root,a.types_root,a.capacities,a.web);print('Verified',len(result['source_robustness']['trials']),'matched robustness trials and',sum(len(c['counterfactuals']) for c in result['contact_types'].values()),'forced-type planning/execution comparisons')
