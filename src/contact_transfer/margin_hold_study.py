"""Matched sustained/repeated loads inside the optimized ON box.

Finite-time loss of support is distinguished from recovered transients and
from mathematical divergence. No controller or physical parameter is ablated.
"""
from pathlib import Path
import hashlib
import json
import numpy as np
from scipy.spatial.transform import Rotation
from .contacts import map_reaction
from .generalized_load import GeneralizedTaskLoad
from .generalized_conic import fixed_generalized_margin
from .perturbations import WrenchHold, WrenchPulse
from .robustness_study import PROTOCOL, make_plan, hold_report
from .full_execution import run_full_episode
from .full_assessment import measured_contact_flags
from .full_recording import write_json


CASES = {
    'nominal': dict(title='No external load', force_N=0., profile='smooth_hold'),
    'common_up2': dict(title='Common interior · 2 N upward hold', force_N=-2., profile='smooth_hold'),
    'common_up1': dict(title='Common interior · 1 N with slow rise', force_N=-1., profile='smooth_hold',ramp_duration_s=2.,duration_s=8.),
    'sustained_up4': dict(title='4 N upward · sustained load', force_N=-4., profile='smooth_hold'),
    'sustained_down4': dict(title='4 N downward · sustained load', force_N=4., profile='smooth_hold'),
    'repeated_up4': dict(title='4 N upward · four repeated pulses', force_N=-4., profile='sin_squared'),
    'slow_up4': dict(title='4 N upward · slow 2 s rise and fall', force_N=-4., profile='smooth_hold',ramp_duration_s=2.,duration_s=8.),
    'diagonal': dict(title='Diagonal hold · 4.5 N up, 1.4 N in x/y', force_N=-4.5,force_world_N=[1.4,-1.4,4.5],profile='smooth_hold',ramp_duration_s=2.,duration_s=8.),
}
EVALUATION_CASES=['nominal','common_up1','common_up2','slow_up4','sustained_up4','repeated_up4']
HOLD_PROTOCOL = dict(
    version=1, hold_reference_duration_s=13., onset_s=4., hold_load_duration_s=5.,
    ramp_duration_s=.5, repeated_onsets_s=[4.,6.,8.,10.], repeated_duration_s=1.,
    recovery_window_s=1., persistent_position_error_m=.05,
    persistent_rotation_error_rad=.3, persistent_exceedance_fraction=.9,
    persistent_all_support_fraction_max=.1, trials_per_condition=1,
    statistical_success_rate_claimed=False, mathematical_divergence_claimed=False,
    primary_comparison='slow_up4', cases=CASES,
    only_ablation='source box-vertex refinement ON/OFF; target material sites fixed',
    certificate_scope='initial-pose static force box; not a closed-loop stability certificate',
    historical_single_pulse_records_preserved=True,
)


def disturbances(body, case):
    cfg=CASES[case]
    if not cfg['force_N']: return ()
    force=tuple(cfg.get('force_world_N',cfg['force_N']*np.array(PROTOCOL['force_direction_world'])))
    args=dict(body=body,force_world_N=force,point_body_m=tuple(PROTOCOL['force_application_point_body_m']))
    if cfg['profile']=='sin_squared':
        return tuple(WrenchPulse(start=start,duration=HOLD_PROTOCOL['repeated_duration_s'],**args)
                     for start in HOLD_PROTOCOL['repeated_onsets_s'])
    return (WrenchHold(start=HOLD_PROTOCOL['onset_s'],duration=cfg.get('duration_s',HOLD_PROTOCOL['hold_load_duration_s']),
                       ramp_duration_s=cfg.get('ramp_duration_s',HOLD_PROTOCOL['ramp_duration_s']),**args),)


def initial_certificate(source, pulses):
    E,D=np.array(source['target_E']),np.array(source['target_D'])
    rho=float(min(source['target_rho'],source['independent_target']['frozen_conic']['rho']))
    # All defined pulses are disjoint. Check their complete wrenches, including
    # the declared body-frame lever arm at the identity reference orientation.
    worst=residual=0.
    for pulse in pulses:
        force=np.asarray(pulse.force_world_N)
        wrench=np.r_[np.cross(pulse.point_body_m,force)+pulse.moment_world_Nm,force]
        u=np.linalg.lstsq(E@D,wrench,rcond=None)[0]
        residual=max(residual,float(np.max(abs(E@D@u-wrench))))
        worst=max(worst,float(np.max(abs(u))))
    return dict(rho_certified=rho,normalized_peak_coefficient=worst,
                load_to_margin_ratio=worst/rho,subspace_residual=residual,
                inside_initial_static_box=residual<=1e-9 and worst<=rho+1e-8,
                vertical_bound_N=float(rho*D[2,2]),
                nominal_reference_pose_only=True)


def directional_equilibrium(episode,pulses):
    """Independent object-only SOCP at the initial pose and peak signed force.

    Infeasibility already excludes any arm allocation at these material sites.
    Feasibility alone does not certify robot torques; the full ON box does.
    """
    p=episode.problem;key=episode.keyframes[0];load=p.task_load(key.q[p.system.object_q])
    force=np.asarray(pulses[0].force_world_N) if pulses else np.zeros(3)
    required=np.asarray(load.nominal)+np.asarray(load.E)@force
    task=GeneralizedTaskLoad(required,np.asarray(load.E)[:,[2]],np.ones((1,1)))
    maps=[np.asarray(map_reaction(point,R,np.eye(6))) for point,R in zip(key.positions,key.rotations)]
    result=fixed_generalized_margin(maps,task,p.capabilities,fixed_rho=0.)
    return dict(feasible=result['certificate'],status=result['status'],
                residual=result['primal_residual'],reference_wrench=required.tolist(),
                object_only=True,robot_torques_certified_here=False)


def response(episode,trace,scene,pulses):
    t=trace.time;pose=trace.object_pose[:,0];reference=episode.desired_object_pose(0)
    pe=np.linalg.norm(pose[:,:3]-reference[:3],axis=1)
    re=(Rotation.from_quat(reference[3:]).inv()*Rotation.from_quat(pose[:,3:])).magnitude()
    contacts=measured_contact_flags(episode,trace,scene);all_support=contacts.all(axis=1)
    onset=min((p.start for p in pulses),default=HOLD_PROTOCOL['onset_s'])
    end=max((p.start+p.duration for p in pulses),default=onset)
    tail=t>=t[-1]-HOLD_PROTOCOL['recovery_window_s']
    recovered=bool(np.all(pe[tail]<=PROTOCOL['pass_position_error_m']) and
                   np.all(re[tail]<=PROTOCOL['pass_rotation_error_rad']) and
                   np.mean(all_support[tail])>=PROTOCOL['pass_contact_fraction'])
    exceeded=(pe>HOLD_PROTOCOL['persistent_position_error_m'])|(re>HOLD_PROTOCOL['persistent_rotation_error_rad'])
    persistent=bool(pulses and np.mean(exceeded[tail])>=HOLD_PROTOCOL['persistent_exceedance_fraction'] and
                    np.mean(all_support[tail])<=HOLD_PROTOCOL['persistent_all_support_fraction_max'] and not recovered)
    loaded=(t>=onset)&(t<=end);first=loaded&(t<onset+.5);last=loaded&(t>end-.5)
    indices=np.unique(np.linspace(0,len(t)-1,801).round().astype(int))
    force=np.array([sum((pulse.scale(time)*np.asarray(pulse.force_world_N) for pulse in pulses),start=np.zeros(3)) for time in t[indices]])
    return dict(recovered=recovered,persistent_loss_of_support=persistent,
                tail_all_support_fraction=float(np.mean(all_support[tail])),
                tail_position_error_m=float(np.max(pe[tail])),
                tail_rotation_error_rad=float(np.max(re[tail])),
                loaded_initial_rms_position_m=float(np.sqrt(np.mean(pe[first]**2))) if first.any() else 0.,
                loaded_final_rms_position_m=float(np.sqrt(np.mean(pe[last]**2))) if last.any() else 0.,
                peak_position_error_m=float(pe.max()),peak_rotation_error_rad=float(re.max()),
                force_end_s=end,recovery_observation_s=float(t[-1]-end),
                mathematical_divergence_claimed=False,
                time_s=t[indices].tolist(),position_error_m=pe[indices].tolist(),
                rotation_error_rad=re[indices].tolist(),height_m=pose[indices,2].tolist(),
                supporting_contacts=contacts[indices].sum(axis=1).tolist(),
                force_world_N=force.tolist())


def run_mode(directory,enabled,cases=None,web=None,*,object_variant='flat'):
    root=Path(directory);mode='on' if enabled else 'off';out=root/mode
    object_path=Path(__file__).with_name('scenarios')/('margin_platform_compact/panel.urdf' if object_variant=='compact' else 'margin_platform/panel.urdf') if object_variant!='flat' else None
    episode,source=make_plan(out/'plan',enabled,hold_duration_s=HOLD_PROTOCOL['hold_reference_duration_s'],object_urdf=object_path)
    protocol={**HOLD_PROTOCOL,'object_variant':object_variant,
              'object_description':'Rigid platform with a fixed upper payload' if object_path else 'Original flat panel',
              'target_com_object_m':source['target_com_object_m'],
              'source_and_target_com_from_urdf':True}
    write_json(root/'protocol.json',dict(protocol=protocol,source_protocol=PROTOCOL))
    results=[]
    for case in (EVALUATION_CASES if cases is None else cases):
        folder=out/case;body='object_urdf_panel';pulses=disturbances(body,case)
        certificate=initial_certificate(source,pulses)
        if enabled and not certificate['inside_initial_static_box']:
            raise RuntimeError('Refusing an ON validation load outside its optimized certificate')
        static=directional_equilibrium(episode,pulses)
        print(f'{mode}/{case}: ratio={certificate["load_to_margin_ratio"]:.4f}, static={static["status"]}',flush=True)
        trace,controller,scene,geometry=run_full_episode(episode,folder/'native',disturbances=pulses,
            progress=lambda t,total:print(f'{mode}/{case} {t:.0f}/{total:.0f}s',flush=True) if int(t)%2==0 else None)
        report=hold_report(episode,trace,controller,scene,geometry)
        analysis=response(episode,trace,scene,pulses)
        outcome='bounded_tracking' if report['execution_passed'] and analysis['recovered'] else 'persistent_loss_of_support' if analysis['persistent_loss_of_support'] else 'transient_failure_recovered' if analysis['recovered'] else 'tracking_failure'
        report.update(source=source,force_N=CASES[case]['force_N'],source_robustness_enabled=enabled,
                      margin_hold=dict(case=case,certificate=certificate,directional_equilibrium=static,
                                       response=analysis,outcome=outcome,protocol=protocol))
        write_json(folder/'result.json',report)
        result=dict(recording_id=f'{"margin_compact" if object_variant=="compact" else "margin_platform" if object_variant=="loaded" else "margin_hold"}_{mode}_{case}',case=case,object_variant=object_variant,robustness_enabled=enabled,
                    execution_passed=report['execution_passed'],outcome=outcome,
                    initial_certificate=certificate,directional_equilibrium=static,response=analysis,
                    native_case_id=trace.metadata['case_id'],native_plan_id=trace.metadata['plan_id'],
                    controller_configuration=trace.metadata['controller_configuration'],
                    native_trace_sha256=hashlib.sha256((folder/'native/execution.npz').read_bytes()).hexdigest())
        write_json(folder/'summary.json',result);results.append(result)
        if web is not None:
            from .full_view import export_full
            label='ON' if enabled else 'OFF'
            object_label={'flat':'Flat panel','loaded':'Tall payload','compact':'Compact payload'}[object_variant]
            data=export_full(episode,trace,report,scene,web,dataset_id=result['recording_id'],
                recording_metadata=dict(title=f'{object_label} · {CASES[case]["title"]} · robustness {label}',
                    scenario='source_robustness_study',full_task_recording=False,comparison_recording=True,
                    study_kind='source_robustness',display=dict(description=f'{object_label} · {CASES[case]["title"]} · source refinement {label}',
                        scene_label=f'MATCHED MARGIN VALIDATION · {label} · P / P / P',
                        geometry='Three Panda arms, identical spherical tools, and a native free panel',
                        chart_note='Identical controller and load; optimized ON certificate checked before execution')))
            data['events']=[dict(id='External force onset',time=HOLD_PROTOCOL['onset_s']),
                            dict(id='Peak external force · arrow',time=pulses[0].start+(pulses[0].ramp_duration_s if isinstance(pulses[0],WrenchHold) else pulses[0].duration/2)),
                            dict(id='External force end',time=analysis['force_end_s']),
                            dict(id='Recovery observation',time=trace.time[-1]-1.)] if pulses else []
            write_json(Path(web)/'data'/(data['id']+'.json'),data,compact=True)
        print(f'{mode}/{case}: {outcome}, final={analysis["tail_position_error_m"]:.4f}m/{analysis["tail_rotation_error_rad"]:.4f}rad',flush=True)
    return results
