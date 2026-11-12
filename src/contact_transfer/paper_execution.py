"""Execute accepted coupled plans with measured contact feedback, never object forcing."""
from dataclasses import dataclass
from pathlib import Path
import hashlib
import json
import numpy as np
from scipy.interpolate import CubicHermiteSpline
from scipy.spatial.transform import Rotation
from .robot import Robot
from .hybrid_control import HybridTorqueImpedance,HybridPositionAdmittance,HybridSelection,contact_coordinates
from .panel_fixture import build_panel_scene
from .contact_simulation import MujocoContactScene
from .lie import inverse


@dataclass
class CoupledEpisode:
    problem: object
    keyframes: tuple
    trajectories: tuple
    preparation: float=2.
    final_hold: float=1.
    contacts_by_interval: tuple | None=None
    control_mode: str='hybrid_feedback'
    jaw_force_fraction: float=.95

    def __post_init__(self):
        if not self.trajectories:raise ValueError('An optimized trajectory is required')
        if self.control_mode not in {'hybrid_feedback','hybrid_feedforward','impedance_feedforward'} or not 0<self.jaw_force_fraction<=1:raise ValueError('Invalid control comparison mode or jaw-force fraction')
        self.offsets=np.r_[0.,np.cumsum([r.dt.sum() for r in self.trajectories])]
        self.time=np.r_[0.,np.cumsum(np.concatenate([r.dt for r in self.trajectories]))]
        self.q=np.concatenate([r.q if i==0 else r.q[1:] for i,r in enumerate(self.trajectories)])
        self.v=np.concatenate([r.v if i==0 else r.v[1:] for i,r in enumerate(self.trajectories)])
        sys=self.problem.system
        self.spline=CubicHermiteSpline(self.time,self.q[:,:sys.object_q.start],self.v[:,:sys.object_v.start])
        dt=self.problem.case.data['simulation']['timestep'];self.duration=np.ceil((self.preparation+self.time[-1]+self.final_hold)/dt)*dt

    def phase(self,t):
        local=np.clip(t-self.preparation,0.,self.time[-1]);i=min(np.searchsorted(self.offsets,local,side='right')-1,len(self.trajectories)-1);i=max(i,0);r=self.trajectories[i];times=np.r_[0.,np.cumsum(r.dt)];k=min(max(np.searchsorted(times,local-self.offsets[i],side='right')-1,0),len(r.dt)-1)
        return i,k,float(local)

    def reactions(self,t):
        if t<self.preparation:return self.keyframes[0].reactions.mean(axis=0)
        i,k,_=self.phase(t);r=self.trajectories[i];out=[]
        for spec,cap in zip(self.problem.case.data['arms'],self.problem.capabilities):
            P=r.impulses[spec['group_id']][k];w=P/r.dt[k];out.append(np.r_[np.zeros(3),w] if cap.kind=='P' else w)
        return np.asarray(out)

    def attached(self,arm,t):
        if t<self.preparation:return True
        i,k,_=self.phase(t)
        if self.contacts_by_interval is None:return True
        name=self.problem.case.data['arms'][arm]['group_id'];c=next(c for c in self.contacts_by_interval[i] if c.name==name)
        return k in c.attached_steps if c.capability.kind=='G' else k not in c.released_steps

    def desired_object_pose(self,t):
        i,k,local=self.phase(t);sys=self.problem.system
        # The held object's reference follows the same grasp-consistent
        # retraction as the plan. Only the robots receive this reference.
        qr=self.spline(local)
        for j,cap in enumerate(self.problem.capabilities):
            if cap.has_moment and self.attached(j,t):
                T=self.problem.robots[j].fk(qr[sys.q_slices[j]],self.problem.frames[j]);p=self.keyframes[i].positions[j];Rc=self.keyframes[i].rotations[j]@np.diag([1.,-1.,-1.]);R=T[:3,:3]@Rc.T
                return np.r_[T[:3,3]-R@p,Rotation.from_matrix(R).as_quat()]
        # P-only episodes retain the optimized pose samples as a soft metric.
        q0=self.q[min(np.searchsorted(self.time,local,side='right')-1,len(self.q)-1),sys.object_q];return q0.copy()

    def identity(self):
        """Controller and native step may differ while task/plan stay matched."""
        from .paper_recording import model_fingerprint
        case=json.loads(json.dumps(self.problem.case.data));case['simulation'].pop('timestep',None)
        case_id=hashlib.sha256((json.dumps(case,sort_keys=True)+model_fingerprint(self.problem)).encode()).hexdigest()
        plan=hashlib.sha256()
        for result in self.trajectories:
            for a in [result.q,result.v,result.tau,result.dt]:plan.update(np.asarray(a,dtype='<f8').tobytes())
            for name in sorted(result.impulses):plan.update(name.encode());plan.update(np.asarray(result.impulses[name],dtype='<f8').tobytes())
            for name in sorted(result.apertures):plan.update(name.encode());plan.update(np.asarray(result.apertures[name],dtype='<f8').tobytes())
            plan.update(result.configuration_update.encode())
        return case_id,plan.hexdigest()


class CoupledContactController:
    """P: normal force/tangential motion; G: pose impedance + jaw-force loop.

    All arm references and desired reactions come from the accepted plan.
    Contact observations read native mj_contactForce; they never set object state.
    """
    def __init__(self,episode):
        self.episode=episode;self.problem=episode.problem;self.case=self.problem.case;self.dt=self.case.data['simulation']['timestep'];self.t=0.
        self.robots=[Robot(r.path,base_pose=r.base_pose) for r in self.problem.robots]
        self.offsets=np.cumsum([0]+[r.n for r in self.robots]);self.reduced_indices=[];self.servos=[];self.fingers=[];self.gripper_servos=[]
        for r,reduced,frame,cap,spec in zip(self.robots,self.problem.robots,self.problem.frames,self.problem.capabilities,self.case.data['arms']):
            self.reduced_indices.append([r.joint_names.index(n) for n in reduced.joint_names]);finger=[r.joint_names.index(n) for n in reduced.tree.locked];self.fingers.append(finger)
            selection=HybridSelection.normal() if cap.kind=='P' and episode.control_mode!='impedance_feedforward' else HybridSelection(np.zeros(6))
            cls=HybridPositionAdmittance if spec['controller']=='position' else HybridTorqueImpedance
            joint_k=np.zeros(r.n);joint_d=np.zeros(r.n);joint_k[self.reduced_indices[-1]]=40.;joint_d[self.reduced_indices[-1]]=8.
            kwargs={} if cls is HybridPositionAdmittance else dict(joint_k=joint_k,joint_d=joint_d,**(dict(force_k=0.,force_i=0.) if episode.control_mode!='hybrid_feedback' else {}))
            self.servos.append(cls(r,frame,selection,**kwargs))
            if spec.get('gripper_profile'):
                from .robotiq import RobotiqForceServo
                self.gripper_servos.append(RobotiqForceServo(spec['gripper_profile']))
            else:self.gripper_servos.append(None)
        self.measured=np.zeros((len(self.robots),6));self.filtered=np.zeros_like(self.measured);self.jaw_measured=[np.zeros(2 if g else len(f)) for g,f in zip(self.gripper_servos,self.fingers)];self.previous_origins=None;self.object_R=np.eye(3);self.object_p=np.zeros(3);self.logs=[]

    def reference(self,t):
        self.t=t;i,k,local=self.episode.phase(t);sys=self.problem.system
        red=[self.episode.spline(local,n) for n in [0,1,2]]
        if t<self.episode.preparation or t>=self.episode.preparation+self.episode.time[-1]:red[1][:]=0;red[2][:]=0
        out=[[],[],[]]
        for r,reduced,indices,qs in zip(self.robots,self.problem.robots,self.reduced_indices,sys.q_slices):
            q=np.array([reduced.tree.locked.get(n,0.) for n in r.joint_names]);v=np.zeros(r.n);a=v.copy();q[indices]=red[0][qs];v[indices]=red[1][qs];a[indices]=red[2][qs]
            for target,value in zip(out,[q,v,a]):target.extend(value)
        for arm,(a,b,fingers,cap) in enumerate(zip(self.offsets[:-1],self.offsets[1:],self.fingers,self.problem.capabilities)):
            name=self.case.data['arms'][arm]['group_id'];r=self.episode.trajectories[i]
            if fingers and name in r.apertures and not self.case.data['arms'][arm].get('gripper_profile'):
                times=np.r_[0.,np.cumsum(r.dt)];now=np.clip(local-self.episode.offsets[i],0.,times[-1]);gap=np.interp(now,times,r.apertures[name])
                for finger in fingers:out[0][a+finger]+=.5*(gap-cap.aperture_required)
        return tuple(np.asarray(v) for v in out)

    def observe(self,mj,model,data,t):
        body=mj.mj_name2id(model,mj.mjtObj.mjOBJ_BODY,'panel');contacts=[]
        for cidx in range(data.ncon):
            c=data.contact[cidx];value=np.zeros(6);mj.mj_contactForce(model,data,cidx,value);R=c.frame.reshape(3,3)
            contacts.append(dict(pair=[mj.mj_id2name(model,mj.mjtObj.mjOBJ_GEOM,int(g)) for g in c.geom],position=c.pos,wrench=np.r_[R.T@value[3:],R.T@value[:3]],normal_force=float(value[0])))
        addresses=[model.jnt_qposadr[mj.mj_name2id(model,mj.mjtObj.mjOBJ_JOINT,spec['id']+'_'+n)] for spec,r in zip(self.case.data['arms'],self.robots) for n in r.joint_names]
        pose=np.r_[data.xpos[body],data.xquat[body][[1,2,3,0]]]
        self.observe_contacts(data.qpos[addresses],pose,contacts)

    def observe_contacts(self,q,object_pose,contacts):
        """Engine-neutral observer: world [moment;force] ON pair's second geom."""
        self.object_R=Rotation.from_quat(object_pose[3:]).as_matrix();self.object_p=np.array(object_pose[:3]);sums=np.zeros_like(self.measured);origins=[];rotations=[];self.jaw_measured=[np.zeros(2 if g else len(f)) for g,f in zip(self.gripper_servos,self.fingers)]
        for i,(r,frame,spec,a,b,fingers) in enumerate(zip(self.robots,self.problem.frames,self.case.data['arms'],self.offsets[:-1],self.offsets[1:],self.fingers)):
            qi=q[a:b];T=r.fk(qi,frame);origins.append(T[:3,3]);rotations.append(T[:3,:3])
            for c in contacts:
                names=c['pair'];tool=next((j for j,n in enumerate(names) if n.startswith(spec['id']+'_')),None)
                if tool is None or names[1-tool]!='panel_collision_0':continue
                if not any(tag in names[tool] for tag in ['_tool_collision_','_panda_leftfinger_collision_','_panda_rightfinger_collision_','_left_pad_collision_','_right_pad_collision_']):continue
                value=np.asarray(c['wrench']);sign=1 if tool==1 else -1;F=sign*value[3:];M=sign*value[:3]+np.cross(np.asarray(c['position'])-T[:3,3],F);sums[i]+=np.r_[M,F]
                if self.gripper_servos[i]:
                    for j,link in enumerate(['left_pad','right_pad']):
                        if '_'+link+'_collision_' in names[tool]:self.jaw_measured[i][j]+=c['normal_force']
                else:
                    for j,finger in enumerate(fingers):
                        link='panda_leftfinger' if 'joint1' in r.joint_names[finger] else 'panda_rightfinger'
                        if link in names[tool]:self.jaw_measured[i][j]+=r.jacobian(qi,link,'world_aligned')[3:,finger]@F
        if self.previous_origins is not None:
            for i in range(len(origins)):self.filtered[i,:3]+=np.cross(self.previous_origins[i]-origins[i],self.filtered[i,3:])
        self.filtered+=self.dt/(.01+self.dt)*(sums-self.filtered);self.previous_origins=np.array(origins)
        for i,R in enumerate(rotations):self.measured[i]=np.r_[R.T@self.filtered[i,:3],self.filtered[i,3:]]

    def command(self,q,v,qd,vd,ad):
        i,k,_=self.episode.phase(self.t);reactions=self.episode.reactions(self.t);out=[];force_error=[];jaw=[];saturation=[]
        for j,(r,servo,spec,cap,frame,point,R,a,b,fingers) in enumerate(zip(self.robots,self.servos,self.case.data['arms'],self.problem.capabilities,self.problem.frames,self.episode.keyframes[i].positions,self.episode.keyframes[i].rotations,self.offsets[:-1],self.offsets[1:],self.fingers)):
            T=r.fk(q[a:b],frame);Rc=self.object_R@R;w=reactions[j];F=Rc@w[3:];M=Rc@w[:3]
            active=self.episode.attached(j,self.t)
            servo.selection=HybridSelection.normal() if cap.kind=='P' and active and self.episode.control_mode!='impedance_feedforward' else HybridSelection(np.zeros(6))
            if cap.kind=='G':M+=np.cross(self.object_p+self.object_R@point-T[:3,3],F)
            desired=np.r_[T[:3,:3].T@M,F]
            inputs=dict(reaction_desired=desired,reaction_measured=self.measured[j],contact_rotation_world=Rc,dt=self.dt)
            if isinstance(servo,HybridPositionAdmittance):tau=servo.command(q[a:b],qd[a:b],vd[a:b],**inputs)
            else:tau=servo.command(q[a:b],v[a:b],qd[a:b],vd[a:b],ad[a:b],**inputs)
            if self.gripper_servos[j]:
                profile=spec['gripper_profile'];positions=dict(zip(r.joint_names,q[a:b]));velocities=dict(zip(r.joint_names,v[a:b]));theta=self.gripper_servos[j].command(positions,velocities,self.jaw_measured[j],self.episode.jaw_force_fraction*cap.closing_force_per_finger,self.dt,active)
                for finger in fingers:tau[finger]=theta if r.joint_names[finger] in profile['drivers'] else 0.
            elif fingers:
                maximum_force=float(np.max(self.jaw_measured[j]));mean_velocity=float(np.mean(v[a+np.asarray(fingers)]));center_error=q[a+fingers[0]]-q[a+fingers[1]];center_velocity=v[a+fingers[0]]-v[a+fingers[1]]
                center_torque=-3000*center_error-10*center_velocity
                for z,finger in enumerate(fingers):
                    # Joint-positive is jaw opening; object reaction is positive.
                    if active:
                        target=self.episode.jaw_force_fraction*cap.closing_force_per_finger
                        # Width force control and centre impedance are distinct
                        # jaw coordinates; independent force loops let the grasp
                        # centre drift even when both normal forces are correct.
                        differential_load=.5*abs(self.jaw_measured[j][0]-self.jaw_measured[j][1])
                        feedback=differential_load+.5*(maximum_force-target) if self.episode.control_mode=='hybrid_feedback' else 0.
                        common=-target+feedback-10*mean_velocity
                        tau[finger]=r.gravity(q[a:b])[finger]+common+(center_torque if z==0 else -center_torque)
                    else:tau[finger]=r.gravity(q[a:b])[finger]+3000*(qd[a+finger]-q[a+finger])-10*v[a+finger]
                tau=np.clip(tau,-r.effort,r.effort)
            out.extend(tau);force_error.append(contact_coordinates(T[:3,:3],Rc)@(self.measured[j]-desired));jaw.append(self.jaw_measured[j].tolist());saturation.append(servo.last.get('saturated',False))
        self.logs.append(dict(time=self.t,force_error=np.asarray(force_error).tolist(),jaw_force=jaw,saturated=saturation))
        return np.asarray(out)


def run_coupled_episode(episode,output):
    from .paper_geometry import PanelGeometry
    from .paper_validation import validate_rigid_trajectory
    geometry=PanelGeometry(episode.problem)
    for i,r in enumerate(episode.trajectories):
        contacts=episode.contacts_by_interval[i] if episode.contacts_by_interval else episode.problem.contacts(episode.keyframes[i],len(r.dt))
        independent=validate_rigid_trajectory(episode.problem,r,contacts)
        if not independent['passed']:raise RuntimeError('Plan rejected by independent mechanics: '+json.dumps(independent))
        if not geometry.validate(r,object_interpolation='holding_grasp'):raise RuntimeError('Plan rejected by full sampled reference collision geometry: '+json.dumps(geometry.last_violations[:5]))
    out=Path(output);out.mkdir(parents=True,exist_ok=True);problem=episode.problem;controller=CoupledContactController(episode);fixtures=build_panel_scene(problem.case,out/'scene.xml')
    names=[s['id']+'_'+n for s,r in zip(problem.case.data['arms'],controller.robots) for n in r.joint_names];motors=[s['id']+'_motor_'+n if r.effort[j]>0 else None for s,r in zip(problem.case.data['arms'],controller.robots) for j,n in enumerate(r.joint_names)];modes=['passive' if r.effort[j]==0 else 'position' if n in s.get('gripper_profile',{}).get('drivers',{}) else s['controller'] for s,r in zip(problem.case.data['arms'],controller.robots) for j,n in enumerate(r.joint_names)]
    scene=MujocoContactScene(out/'scene.xml',names,motors,['panel'],actuator_modes=modes)
    case_id,plan_id=episode.identity()
    trace=scene.run(controller,controller.reference,episode.duration,initial_q=controller.reference(0)[0],case_id=case_id,plan_id=plan_id)
    trace.metadata.update(fixtures=fixtures,configuration_update=[r.configuration_update for r in episode.trajectories],controller=episode.control_mode,jaw_force_fraction=episode.jaw_force_fraction,object_pose_prescribed=False,equality_constraints_used=int(scene.model.neq),object_equality_constraints_used=0,internal_gripper_equalities=int(scene.model.neq));trace.save(out/'execution.npz')
    report=assess_coupled_execution(episode,trace,controller)
    (out/'execution-report.json').write_text(json.dumps(report,indent=2)+'\n');(out/'control.json').write_text(json.dumps(controller.logs,separators=(',',':')))
    return trace,report,controller


def assess_coupled_execution(episode,trace,controller):
    """Separate tracking/contact/effort evidence from a full manuscript pass."""
    desired=np.array([episode.desired_object_pose(t) for t in trace.time]);p_error=np.linalg.norm(trace.object_pose[:,0,:3]-desired[:,:3],axis=1);r_error=(Rotation.from_quat(desired[:,3:]).inv()*Rotation.from_quat(trace.object_pose[:,0,3:])).magnitude();active=trace.time>=episode.preparation
    effort=np.concatenate([r.effort for r in controller.robots]);margin=float(np.min(effort-abs(trace.torque)));contact_fractions={};jaw={}
    for arm,(spec,cap) in enumerate(zip(episode.problem.case.data['arms'],episode.problem.capabilities)):
        needed=np.array([episode.attached(arm,t) and t>=episode.preparation for t in trace.time[:-1]])
        present=np.zeros(len(trace.time),bool)
        selected=(np.linalg.norm(trace.contact_wrench[:,3:],axis=1)>1e-3)&np.any(np.char.startswith(trace.contact_pair,spec['id']+'_'),axis=1)&np.any(trace.contact_pair=='panel_collision_0',axis=1)
        steps=np.rint(trace.contact_time[selected]/trace.metadata['timestep']).astype(int);present[np.clip(steps,0,len(present)-1)]=True
        contact_fractions[spec['id']]=float(present[:-1][needed].mean()) if needed.any() else None
        if cap.kind=='G':
            samples=np.array([entry['jaw_force'][arm] for entry in controller.logs]);valid=needed[:len(samples)]
            jaw[spec['id']]=dict(maximum_per_finger_N=samples[valid].max(axis=0).tolist(),minimum_per_finger_N=samples[valid].min(axis=0).tolist(),declared_limit_N=cap.closing_force_per_finger,minimum_limit_margin_N=float(cap.closing_force_per_finger-samples[valid].max()),minimum_unilateral_margin_N=float(samples[valid].min()),positive_reaction_fraction=(samples[valid]>1e-3).mean(axis=0).tolist())
    force_errors=np.asarray([entry['force_error'] for entry in controller.logs]);force_active=active[:len(force_errors)]
    report=dict(engine=trace.metadata['engine'],maximum_position_error_m=float(p_error[active].max()),final_position_error_m=float(p_error[-1]),maximum_rotation_error_rad=float(r_error[active].max()),final_rotation_error_rad=float(r_error[-1]),duration_s=float(trace.time[-1]),object_pose_prescribed=False,equality_constraints_used=0,configuration_update=[r.configuration_update for r in episode.trajectories],controller=episode.control_mode,jaw_force_fraction=episode.jaw_force_fraction,measured_force_feedback=episode.control_mode=='hybrid_feedback',minimum_actuator_margin_Nm=margin,maintained_contact_fraction=contact_fractions,jaw_force=jaw,normal_force_RMSE_N=np.sqrt(np.mean(force_errors[force_active,:,5]**2,axis=0)).tolist(),tracking_and_contact_passed=bool(p_error[active].max()<=.008 and r_error[active].max()<=.06 and margin>=-1e-8 and all(v is None or v>=.99 for v in contact_fractions.values())),physical_finger_force_limits_passed=all(v['minimum_limit_margin_N']>=-1e-6 for v in jaw.values()),full_paper_pipeline_executed=False)
    report['unilateral_finger_reactions_passed']=all(v['minimum_unilateral_margin_N']>=-1e-6 and min(v['positive_reaction_fraction'])>=.99 for v in jaw.values())
    report.update(equality_constraints_used=trace.metadata.get('equality_constraints_used',0),object_equality_constraints_used=0,internal_gripper_equalities=trace.metadata.get('internal_gripper_equalities',0),gripper_model=episode.problem.case.data['arms'][0].get('gripper_model','panda_hand'),gripper_measured_force_feedback=any(s.get('gripper_profile') for s in episode.problem.case.data['arms']),reference_interpolation='holding_grasp',sampled_execution_reference_geometry_passed=True)
    capability=check_execution_wrenches(episode,trace)
    report['sampled_interface_wrench_capability']=capability
    report['execution_passed']=report['tracking_and_contact_passed'] and report['physical_finger_force_limits_passed'] and report['unilateral_finger_reactions_passed'] and capability['passed']
    return report


def check_execution_wrenches(episode,trace,sample_dt=.05):
    """Independent native-resultant check against each declared P/G model.

    This samples the *raw* solver contacts, not filtered feedback signals, and
    includes contact-origin moment shifts. It is not a continuous certificate.
    """
    h=trace.metadata['timestep'];records={};stride=max(1,round(sample_dt/h))
    step=np.rint(trace.contact_time/h).astype(int);start=np.searchsorted(step,np.arange(len(trace.time)),side='left');end=np.searchsorted(step,np.arange(len(trace.time)),side='right')
    for arm,(spec,cap) in enumerate(zip(episode.problem.case.data['arms'],episode.problem.capabilities)):
        count=0;failed=0;maximum=0.
        for k in range(int(np.ceil(episode.preparation/h)),len(trace.time),stride):
            t=trace.time[k]
            if not episode.attached(arm,t):continue
            i,_,_=episode.phase(t)
            # CRISP's impulse uses the configuration at the preceding step.
            # Pair it with that pose, not the newly integrated configuration.
            previous=trace.metadata['engine']=='crisp' and trace.metadata['contact_settings'].get('force_interval')=='preceding native step'
            pose=trace.object_pose[max(0,k-1) if previous else k,0];Ro=Rotation.from_quat(pose[3:]).as_matrix();Rc=Ro@episode.keyframes[i].rotations[arm];point=pose[:3]+Ro@episode.keyframes[i].positions[arm];w=np.zeros(6)
            for row in range(start[k],end[k]):
                pair=trace.contact_pair[row];tool=next((j for j,n in enumerate(pair) if n.startswith(spec['id']+'_')),None)
                if tool is None or pair[1-tool]!='panel_collision_0':continue
                sign=1 if tool==1 else -1;f=sign*trace.contact_wrench[row,3:];m=sign*trace.contact_wrench[row,:3];w+=np.r_[Rc.T@(m+np.cross(trace.contact_position[row]-point,f)),Rc.T@f]
            count+=1
            if cap.kind=='P':
                residual=max(0.,-w[5],w[5]-cap.normal_max,np.linalg.norm(w[3:5])-cap.mu*w[5]);feasible=residual<=2e-5
            else:
                check=cap.check(w);residual=check['primal_residual'];feasible=check['feasible']
            maximum=max(maximum,float(residual));failed+=not feasible
        records[spec['id']]=dict(samples=count,failed_samples=failed,maximum_residual=maximum,passed=count>0 and failed==0)
    return dict(sample_dt_s=stride*h,raw_native_contacts=True,continuous_certificate=False,interfaces=records,passed=all(r['passed'] for r in records.values()))
