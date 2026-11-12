"""Controlled III-B robustness ablation with genuine three-arm native physics.

Only source box-vertex robustness is ablated. Target contact geometry is fixed
to the transferred source to isolate refinement; target posture/force/torque
are solved identically. This stationary diagnostic is not full-motion CITO.
"""
from copy import deepcopy
from dataclasses import replace
from pathlib import Path
import hashlib,json
import numpy as np
from scipy.spatial.transform import Rotation
from .contacts import Capability,TaskWrench,gravity_wrench,contact_basis
from .refinement import refine_contacts
from .conic import fixed_wrench_margin
from .paper_panel import planar_patch
from .paper_keyframe import Interface
from .robot import Robot
from .full_scenario import FullTask,FullProblem,Feature,point_pose_ik,polish_keyframe,nearest_box
from .full_static import fixed_geometry_keyframe,stationary_interval
from .full_trajectory import interval_contacts
from .full_validation import validate_keyframe,validate_interval,cross
from .full_scene import build_scene
from .full_execution import FullEpisode,run_full_episode
from .full_assessment import assess
from .full_recording import write_json,save_key,save_interval
from .full_pipeline import scenario_solver_settings
from .generalized_load import GeneralizedTaskLoad
from .perturbations import WrenchPulse


PROTOCOL=dict(version=2,source_to_target_scale=4.,source_mass_kg=.03125,
    target_mass_kg=2.,source_position_radius_m=.0325,geometry_weight=1000.,
    target_initial_sites_m=[[-.17,-.035,-.02],[0.,.035,-.02],[.17,-.035,-.02]],
    source_patch_half_size_m=.0325,mu=.8,target_normal_limit_N=40.,
    force_application_point_body_m=[0.,.18,.02],target_force_box_scales_N=[2.56,2.56,8.],
    force_direction_world=[0.,0.,-1.],force_levels_N=[0.,-4.,4.,-8.,8.,-12.,12.],
    force_sign_convention='positive downward, negative upward; both tested before selecting a demonstration',
    pulse_start_s=4.,pulse_duration_s=1.,hold_duration_s=5.,physics_dt_s=.001,
    target_contact_geometry_adjustment=False,target_posture_and_forces_reoptimized=True,
    arm_control='impedance_feedforward',object_actuators=0,object_welds=0,
    pass_position_error_m=.008,pass_rotation_error_rad=.06,pass_contact_fraction=.99,
    source_normals_fixed=True,source_and_target_task_basis_fixed_before_trials=True,
    statistical_success_rate_claimed=False)


def source_refinement(robustness_enabled):
    cfg=PROTOCOL;scale=cfg['source_to_target_scale'];points=np.array(cfg['target_initial_sites_m'])/scale
    patches=[planar_patch(p,[0,0,-1],np.array([[1,0,0],[0,1,0]]).T,[cfg['source_patch_half_size_m']]*2) for p in points]
    point=np.array(cfg['force_application_point_body_m'])/scale;E=np.vstack([cross(point),np.eye(3)])
    task=TaskWrench(gravity_wrench(cfg['source_mass_kg'],[0,0,0],np.eye(3)),E,np.diag(np.array(cfg['target_force_box_scales_N'])/scale**3))
    caps=[Capability(mu=cfg['mu'],normal_max=cfg['target_normal_limit_N']/scale**3) for _ in points]
    result=refine_contacts(patches,task,caps,position_radius=cfg['source_position_radius_m'],normal_radius=0.,geometry_weight=cfg['geometry_weight'],robustness_enabled=robustness_enabled,settings=scenario_solver_settings())
    rotations=np.array([contact_basis([0,0,-1])]*3);before=fixed_wrench_margin(points,rotations,task,caps);after=fixed_wrench_margin(result.positions,result.rotations,task,caps)
    return result,dict(robustness_enabled=robustness_enabled,optimization_vertex_count=len(task.vertices) if robustness_enabled else 1,independent_common_polytope_vertex_count=len(task.vertices),source_rho_before=before.rho,source_rho_after=after.rho,source_independent_residual=after.primal_residual,source_positions_before_m=points.tolist(),source_positions_after_m=result.positions.tolist(),source_objective=result.objective,source_primal_residual=result.primal_residual,source_E=E.tolist(),source_D=task.D.tolist(),source_forces_transferred=False,network_inference_executed=False,synthetic_input=True)


class RobustHoldProblem(FullProblem):
    def task_load(self,qobj,*,source_object=None):
        if source_object is not None:raise ValueError('Source has its explicitly declared scale and task')
        obj=self.task_definition.object;T=obj.fk(qobj,'panel');J=obj.jacobian(qobj,'panel','world_aligned');offset=T[:3,:3]@np.array(PROTOCOL['force_application_point_body_m']);Jp=J[3:]-cross(offset)@J[:3]
        return GeneralizedTaskLoad(-obj.gravity(qobj),Jp.T,np.diag(PROTOCOL['target_force_box_scales_N']))


def make_plan(directory,robustness_enabled):
    out=Path(directory);out.mkdir(parents=True,exist_ok=True);refined,source=source_refinement(robustness_enabled);positions=refined.positions*PROTOCOL['source_to_target_scale'];normals=np.array([[0,0,-1.]]*3)
    task=FullTask.load('panel',out/'task');qo=np.array([0.,0.,.52,0,0,0,1.]);size=nearest_box(task.object,'panel',[0,0,0])
    if not np.isclose(sum(link.mass for link in task.object.tree.links.values()),PROTOCOL['target_mass_kg']):raise ValueError('Protocol mass differs from authoritative target URDF')
    if not np.isclose(PROTOCOL['source_mass_kg'],PROTOCOL['target_mass_kg']/task.scale**3):raise ValueError('Source mass scale differs from protocol')
    if np.any(abs(positions)>size/2+1e-9) or not np.allclose(positions[:,2],-size[2]/2):raise ValueError('Source transfer leaves the actual target support surface')
    features=tuple(Feature('support_'+str(i),'panel',p,n,p,np.eye(3),size,('P',)) for i,(p,n) in enumerate(zip(positions,normals)))
    arms=[]
    for x,y,yaw in [(-.75,-.25,0.),(0.,.65,-np.pi/2),(.75,-.25,np.pi)]:
        T=np.eye(4);T[:3,:3]=Rotation.from_euler('z',yaw).as_matrix();T[:3,3]=[x,y,0.];arms.append(Robot(task.arms[0].path,base_pose=T))
    keys=tuple(dict(id=name,time=t,object_q=qo.copy(),phase='hold') for name,t in [('settled',0.),('before_push',2.5),('hold_final',5.)])
    seeds=tuple(np.array([0.,-.4,0.,-1.8,0.,1.5,0.]) for _ in arms)
    task=replace(task,features=features,keyframes=keys,arms=tuple(arms),seeds=seeds,include_fixtures=False,posture_reference_rule='hardware_seed',posture_trust_radius_rad=None)
    p=RobustHoldProblem.create(task,out/'planning','PPP');q=[];interfaces=[]
    for i,(r,f,spec,frame,cap) in enumerate(zip(p.robots,features,p.specs,p.frames,p.capabilities)):
        target=qo[:3]+f.surface;fit=None
        for seed in [spec['q_initial'],[0.,-.8,0.,-2.2,0.,2.5,0.],[.3,.2,0.,-1.6,0.,2.2,0.]]:
            try:fit=point_pose_ik(r,frame,target,f.normal,seed);break
            except RuntimeError:continue
        if fit is None:raise RuntimeError('No local point IK for '+f.id)
        q.extend(fit);from .contacts import SurfacePatch
        interfaces.append(Interface(f.id,i,frame,'panel',SurfacePatch.fixed(f.surface,f.normal),cap,f.surface,contact_basis(f.normal)))
    q=np.r_[q,qo];prior=dict(positions=positions,normals=normals,record=source)
    key=fixed_geometry_keyframe(p,q,interfaces,prior,(),scenario_solver_settings(),q_reference=q)
    if key is None:raise RuntimeError('Controlled fixed-geometry target did not obtain a valid certificate')
    key.source=source;key.phase='hold';key.retreat_m=0.;key.interfaces=interfaces;key.posture_reference=q[:21];key=polish_keyframe(p,key)
    check=validate_keyframe(p,key)
    if not check['passed']:raise RuntimeError('Independent hold-keyframe mechanics rejected '+str(check))
    keyframes=tuple(deepcopy(key) for _ in keys);trajectories=[];contacts=[]
    for i,(left,right) in enumerate(zip(keyframes[:-1],keyframes[1:])):
        cs=interval_contacts(p,i,left,8);r,cs,independent=stationary_interval(p,left,right,cs,keys[i+1]['time']-keys[i]['time'],8);trajectories.append(r);contacts.append(cs)
    episode=FullEpisode(p,keyframes,trajectories,contacts);scene=build_scene(p,out/'planning-scene.xml')
    from .full_execution import reference_geometry
    geometry=reference_geometry(episode,scene)
    if not geometry['passed']:raise RuntimeError('Hold reference collision rejected '+str(geometry['violations'][:5]))
    identity=hashlib.sha256(json.dumps(PROTOCOL,sort_keys=True).encode()+str(robustness_enabled).encode()).hexdigest()
    for i,k in enumerate(keyframes):save_key(out/f'key{i}.npz',k,identity)
    for i,r in enumerate(trajectories):save_interval(out/f'interval{i}.npz',r,identity)
    source.update(target_rho=key.rho,target_positions_m=positions.tolist(),target_E=p.task_load(qo).E.tolist(),target_D=p.task_load(qo).D.tolist(),independent_target=check,reference_geometry=geometry)
    write_json(out/'plan.json',dict(protocol=PROTOCOL,source=source,full_paper_pipeline_executed=False,stationary_diagnostic=True))
    return episode,source


def hold_report(episode,trace,controller,scene,geometry):
    r=assess(episode,trace,controller,scene,geometry=geometry)
    r['task_metrics']=dict(task='suspended_panel_hold',fixture_present=False,final_position_error_m=r['object_tracking']['panel']['final_position_error_m'],final_rotation_error_rad=r['object_tracking']['panel']['final_rotation_error_rad'])
    r['task_passed']=bool(r['tracking_passed'] and r['contact_retention_passed'])
    r['execution_passed']=bool(r['task_passed'] and r['minimum_actuator_margin_Nm']>=-1e-8 and r['sampled_interface_wrench_capability']['passed'] and geometry['passed'] and not r['unexpected_loaded_contact_penetrations_m'])
    r['stationary_refinement_diagnostic']=True
    return r


def run_trial(directory,robustness_enabled,force_N,web=None):
    root=Path(directory);name='polytope' if robustness_enabled else 'nominal';episode,source=make_plan(root/name,robustness_enabled);folder=root/name/('force_'+str(int(force_N)));scene=build_scene(episode.problem,folder/'force-target.xml');pulses=() if force_N==0 else (WrenchPulse(next(iter(scene.object_roots.values())),PROTOCOL['pulse_start_s'],PROTOCOL['pulse_duration_s'],tuple(force_N*np.array(PROTOCOL['force_direction_world'])),tuple(PROTOCOL['force_application_point_body_m'])),)
    trace,controller,scene,geometry=run_full_episode(episode,folder/'native',disturbances=pulses,progress=lambda t,total:print(f'{name}/{force_N:g}N {t:.1f}/{total:.1f}s',flush=True) if int(t)%2==0 else None)
    r=hold_report(episode,trace,controller,scene,geometry);r.update(source=source,force_N=float(force_N),source_robustness_enabled=robustness_enabled,full_paper_pipeline_executed=False);write_json(folder/'result.json',r)
    if web is not None:
        from .full_view import export_full
        dataset=f'robust_refinement_{name}_{int(force_N)}N';export_full(episode,trace,r,scene,web,dataset_id=dataset,recording_metadata=dict(title=f'Source polytope {"ON" if robustness_enabled else "OFF"} · 외력 {force_N:g} N',scenario='source_robustness_study',full_task_recording=False,comparison_recording=True,study_kind='source_robustness',display=dict(description='3-Panda · 소형 접촉 refinement의 wrench polytope 제약 유무',scene_label='MATCHED SOURCE ROBUSTNESS ABLATION · P / P / P',geometry='동일 spherical tip 3개 · 측정된 native 관절/자유 패널',chart_note='동일 위치·시간의 수직 외력 · source geometry만 전달 · target geometry 고정')))
    return r
