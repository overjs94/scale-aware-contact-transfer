"""Task and finite-contact checks of raw native execution, separate from planning."""
import numpy as np
from scipy.spatial.transform import Rotation
from .full_trajectory import phase_mode
from .multi_object import RigidCollection


def frame_index(episode,trace,scene,frame):
    return trace.metadata['object_names'].index(scene.object_frames[frame])


def measured_contact_flags(episode,trace,scene,threshold=1e-3):
    flags=np.zeros((len(trace.time),len(episode.problem.robots)),bool);steps=np.rint(trace.contact_time/trace.metadata['timestep']).astype(int);loaded=np.linalg.norm(trace.contact_wrench[:,3:],axis=1)>threshold
    for i,(feature,spec) in enumerate(zip(episode.problem.task_definition.features,episode.problem.specs)):
        selected=loaded&np.any(np.char.startswith(trace.contact_pair,spec['id']+'_'),axis=1)&np.any(np.isin(trace.contact_pair,scene.allowed_by_group[feature.id]),axis=1);flags[np.clip(steps[selected],0,len(flags)-1),i]=True
    return flags


def planned_normal_reactions(episode,group_id,times):
    """Stored plan loads, independent of disabled feedforward or observations."""
    values=[]
    for t in times:
        i,k,_=episode.phase(t);r=episode.trajectories[i];values.append(r.impulses[group_id][k,2]/r.dt[k])
    return np.asarray(values)


def final_support_force(trace,object_geometries,counterpart,window=.25):
    """Mean world force on the named object from specified native counterparts."""
    h=trace.metadata['timestep'];steps=np.rint(trace.contact_time/h).astype(int);selected=trace.contact_time>=trace.time[-1]-window;total=np.zeros((len(trace.time),3))
    for side in [0,1]:
        mask=selected&np.isin(trace.contact_pair[:,side],object_geometries)&np.isin(trace.contact_pair[:,1-side],counterpart);np.add.at(total,steps[mask],(1 if side==1 else -1)*trace.contact_wrench[mask,3:])
    return total[trace.time>=trace.time[-1]-window].mean(axis=0)


def capability_samples(episode,trace,scene,sample_dt=.05):
    p=episode.problem;h=trace.metadata['timestep'];stride=max(1,round(sample_dt/h));step=np.rint(trace.contact_time/h).astype(int);starts=np.searchsorted(step,np.arange(len(trace.time)),side='left');ends=np.searchsorted(step,np.arange(len(trace.time)),side='right');records={}
    for arm,(f,spec,cap) in enumerate(zip(p.task_definition.features,p.specs,p.capabilities)):
        count=failed=0;maximum=0.;index=frame_index(episode,trace,scene,f.frame)
        for k in range(int(np.ceil((episode.preparation+.1)/h)),len(trace.time),stride):
            t=trace.time[k]
            if not episode.attached(arm,t):continue
            i,_,_=episode.phase(t)
            if phase_mode(episode.keyframes[i].phase,f)!='holding':continue
            pose=trace.object_pose[k,index];R=Rotation.from_quat(pose[3:]).as_matrix();Rc=R@episode.keyframes[i].rotations[arm];point=pose[:3]+R@episode.keyframes[i].positions[arm];w=np.zeros(6)
            for row in range(starts[k],ends[k]):
                pair=trace.contact_pair[row];tool=next((j for j,n in enumerate(pair) if n.startswith(spec['id']+'_')),None)
                if tool is None or pair[1-tool] not in scene.allowed_by_group[f.id]:continue
                sign=1 if tool==1 else -1;F=sign*trace.contact_wrench[row,3:];M=sign*trace.contact_wrench[row,:3];w+=np.r_[Rc.T@(M+np.cross(trace.contact_position[row]-point,F)),Rc.T@F]
            if cap.kind=='P':res=max(0.,-w[5],w[5]-cap.normal_max,np.linalg.norm(w[3:5])-cap.mu*w[5]);ok=res<=2e-5
            else:check=cap.check(w);res=check['primal_residual'];ok=check['feasible']
            count+=1;failed+=not ok;maximum=max(maximum,float(res))
        records[f.id]=dict(samples=count,failed_samples=failed,maximum_residual=maximum,passed=count>0 and failed==0)
    return dict(sample_dt_s=stride*h,raw_native_contacts=True,continuous_certificate=False,interfaces=records,passed=all(s['passed'] for s in records.values()))


def assess(episode,trace,controller,scene,*,geometry):
    p=episode.problem;task=p.task_definition;obj=task.object;active=trace.time>=episode.preparation;desired=np.array([episode.object_reference_local(np.clip(t-episode.preparation,0,episode.time[-1])) for t in trace.time]);errors={}
    frames=list(obj.names) if isinstance(obj,RigidCollection) else ['base','moving'] if task.id=='box' else [obj.tree.root]
    for frame in frames:
        target=np.array([np.r_[T[:3,3],Rotation.from_matrix(T[:3,:3]).as_quat()] for T in [obj.fk(q,frame) for q in desired]]);actual=trace.object_pose[:,frame_index(episode,trace,scene,frame)];pe=np.linalg.norm(actual[:,:3]-target[:,:3],axis=1);re=(Rotation.from_quat(target[:,3:]).inv()*Rotation.from_quat(actual[:,3:])).magnitude();errors[frame]=dict(maximum_position_error_m=float(pe[active].max()),final_position_error_m=float(pe[-1]),maximum_rotation_error_rad=float(re[active].max()),final_rotation_error_rad=float(re[-1]))
    effort=np.concatenate([r.effort for r in controller.robots]);actuated=effort>0;margin=float(np.min(effort[actuated]-abs(trace.torque[:,actuated])));contact_fractions={};geometric_fractions={};required_samples={};jaws={};dt=trace.metadata['timestep']
    for arm,(f,spec,cap) in enumerate(zip(task.features,p.specs,p.capabilities)):
        needed=np.array([t>=episode.preparation+.1 and episode.attached(arm,t) and phase_mode(episode.keyframes[episode.phase(t)[0]].phase,f)=='holding' for t in trace.time[:-1]]);present=np.zeros(len(trace.time),bool);pair=trace.contact_pair;selected=(np.linalg.norm(trace.contact_wrench[:,3:],axis=1)>1e-3)&np.any(np.char.startswith(pair,spec['id']+'_'),axis=1)&np.any(np.isin(pair,scene.allowed_by_group[f.id]),axis=1);steps=np.rint(trace.contact_time[selected]/dt).astype(int);present[np.clip(steps,0,len(present)-1)]=True;contact_fractions[f.id]=float(present[:-1][needed].mean()) if needed.any() else None
        geometric_fractions[f.id]=contact_fractions[f.id]
        if cap.kind=='P':
            # A zero-normal-force handoff is not a loaded support contact.
            # Use the stored optimized plan, even in the no-feedforward
            # ablation; never infer the requirement from measured forces.
            needed=needed&(planned_normal_reactions(episode,f.id,trace.time[:-1])>.1)
            contact_fractions[f.id]=float(present[:-1][needed].mean()) if needed.any() else None
        required_samples[f.id]=int(needed.sum())
        if cap.kind=='G':
            samples=np.array([entry['jaw_force'][arm] for entry in controller.logs]);values=samples[:len(needed)][needed];jaws[f.id]=dict(maximum_per_jaw_N=values.max(axis=0).tolist(),all_time_maximum_per_jaw_N=samples.max(axis=0).tolist(),force_limit_window='entire recorded execution including preparation and release',minimum_per_jaw_N=values.min(axis=0).tolist(),positive_reaction_fraction=(values>1e-3).mean(axis=0).tolist(),limit_N=cap.closing_force_per_finger,limit_margin_N=float(cap.closing_force_per_finger-samples.max()),passed=bool(samples.max()<=cap.closing_force_per_finger+1e-6 and min((values>1e-3).mean(axis=0))>=.99))
    measured=measured_contact_flags(episode,trace,scene);final_window=trace.time>=trace.time[-1]-.25;task_metrics={};task_pass=True
    if task.id=='box':
        theta=np.array(trace.metadata['articulated_state']['q'])[:,0];target=float(task.keyframes[-1]['object_q'][-1]);task_metrics=dict(final_hinge_angle_rad=float(theta[-1]),target_hinge_angle_rad=target,hinge_error_rad=float(abs(theta[-1]-target)),floating_box=True,passive_hinge=True);task_pass=abs(theta[-1]-target)<=.06
    elif task.id=='assembly':
        a=trace.object_pose[-1,frame_index(episode,trace,scene,'tabletop')];b=trace.object_pose[-1,frame_index(episode,trace,scene,'leg')];Ra=Rotation.from_quat(a[3:]).as_matrix();Rb=Rotation.from_quat(b[3:]).as_matrix();cfg=task.original.data['socket'];tip=b[:3]+Rb@np.array(cfg.get('peg_tip_object',[0,0,-.165]));socket=a[:3]+Ra@np.array(cfg['center_object']);delta=Ra.T@(tip-socket);leg_index=next(i for i,f in enumerate(task.features) if f.id=='leg_gripper');support=final_support_force(trace,scene.object_geometries['leg'],scene.object_geometries['tabletop']);weight=9.81*sum(link.mass for link in obj.bodies['leg'].tree.links.values());released=not measured[final_window,leg_index].any();task_metrics=dict(insertion_depth_m=float(-delta[2]),lateral_error_m=float(np.linalg.norm(delta[:2])),two_independent_free_bodies=True,leg_release_planned=episode.keyframes[-1].phase=='release_leg',leg_released=bool(released),leg_support_from_tabletop_N=support.tolist(),leg_weight_N=weight);task_pass=task_metrics['lateral_error_m']<=.002 and abs(task_metrics['insertion_depth_m']-cfg.get('target_depth_m',.025))<=.003 and released and support[2]>=.9*weight
    else:
        final=errors[obj.tree.root];jig=sorted({str(n) for pair in trace.contact_pair for n in pair if n.startswith('jig_')});support=final_support_force(trace,scene.object_geometries[obj.tree.root],jig);weight=9.81*sum(link.mass for link in obj.tree.links.values());released=not measured[final_window].any();supported=support[2]>=.95*weight;task_metrics=dict(placed_on_jig=bool(supported),jig_support_final_N=support.tolist(),object_weight_N=weight,all_tools_released=bool(released),robot_release_planned=episode.keyframes[-1].phase=='released',final_position_error_m=final['final_position_error_m'],final_rotation_error_rad=final['final_rotation_error_rad']);task_pass=final['final_position_error_m']<=.008 and final['final_rotation_error_rad']<=.06 and released and supported
    from .full_validation import Geometry
    oracle=Geometry(p,scene);unexpected={}
    for pair,gap,wrench in zip(trace.contact_pair,trace.contact_gap,trace.contact_wrench):
        if not oracle.intended(pair) and np.linalg.norm(wrench[3:])>.1:
            name=' / '.join(pair);unexpected[name]=max(unexpected.get(name,0.),float(max(0.,-gap)))
    capability=capability_samples(episode,trace,scene);tracking=all(e['maximum_position_error_m']<=.008 and e['maximum_rotation_error_rad']<=.06 for e in errors.values());retention=all(v is None or v>=.99 for v in contact_fractions.values());passed=tracking and retention and margin>=-1e-8 and all(v['passed'] for v in jaws.values()) and capability['passed'] and task_pass and geometry['passed'] and not unexpected
    return dict(execution_passed=bool(passed),engine=trace.metadata['engine'],duration_s=float(trace.time[-1]),object_tracking=errors,task_metrics=task_metrics,task_passed=bool(task_pass),tracking_passed=tracking,maintained_contact_fraction=contact_fractions,geometric_holding_contact_fraction=geometric_fractions,required_loaded_contact_samples=required_samples,contact_retention_definition='P: stored planned normal reaction >0.1 N; G: attached holding gripper. Zero-force handoff remains in geometric_holding_contact_fraction.',contact_retention_passed=retention,jaw_force=jaws,minimum_actuator_margin_Nm=margin,sampled_interface_wrench_capability=capability,unexpected_loaded_contact_penetrations_m=unexpected,reference_geometry=geometry,object_pose_prescribed=False,object_equality_constraints_used=0,internal_gripper_equalities=trace.metadata['internal_gripper_equalities'],controller=episode.control_mode,arm_measured_force_feedback=episode.control_mode=='hybrid_feedback',arm_force_selection=[0.]*6 if episode.control_mode=='impedance_feedforward' else [0.,0.,0.,0.,0.,1.],joint_nullspace_impedance=dict(stiffness=40.,damping=8.),arm_reaction_feedforward=episode.force_feedforward,measured_jaw_force_feedback=True,jaw_force_fraction=episode.jaw_force_fraction,jaw_force_target_N={f.id:episode.jaw_force_fraction*c.closing_force_per_finger for f,c in zip(task.features,p.capabilities) if c.kind=='G'},contacts_enabled=trace.metadata['contacts_enabled'],full_paper_pipeline_executed=False)
