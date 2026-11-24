"""Full-plan reference projection and native closed-loop execution, no object forcing."""
from pathlib import Path
import hashlib,json
import numpy as np
from scipy.interpolate import CubicHermiteSpline
from scipy.spatial.transform import Rotation
from .paper_execution import CoupledEpisode,CoupledContactController
from .full_trajectory import object_interpolate
from .full_scene import build_scene
from .full_validation import Geometry,validate_interval
from .contracted import numerical_pose_ik
from .contact_simulation import MujocoContactScene
from .perturbations import WrenchSchedule
from .multi_object import RigidCollection
from .configuration import task_configuration
from .reference import subdivide_reference_times,conditioned_joint_spline


class FullEpisode(CoupledEpisode):
    def __init__(self,problem,keyframes,trajectories,contacts_by_interval,*,reference_dt=None,force_feedforward=True,jaw_force_fraction=None):
        policy=task_configuration(problem.task_definition).execution;reference_dt=policy.reference_dt_s if reference_dt is None else reference_dt;fraction=policy.jaw_force_fraction if jaw_force_fraction is None else jaw_force_fraction
        super().__init__(problem,tuple(keyframes),tuple(trajectories),contacts_by_interval=tuple(contacts_by_interval),control_mode=policy.control_mode,jaw_force_fraction=fraction)
        self.force_feedforward=force_feedforward;self.nodal_spline=self.spline;self.reference_dt=reference_dt
        # Only interpolation is cached. Model/plan/code identity must match;
        # velocity bounds and native geometry remain fresh execution gates.
        cache=problem.directory/'dense-reference.npz';digest=hashlib.sha256()
        for name in ['full_execution.py','full_trajectory.py','contracted.py','reference.py']:digest.update(Path(__file__).with_name(name).read_bytes())
        for part in self.identity():digest.update(part.encode())
        digest.update(str(reference_dt).encode());identity=digest.hexdigest();loaded=False
        if cache.exists():
            with np.load(cache,allow_pickle=False) as a:
                if str(a['identity'])==identity:
                    times=a['times'].copy();Q=a['q'].copy();objects=a['object_q'].copy();largest=float(a['projection_max_rad']);loaded=True
        mesh,node_indices=subdivide_reference_times(self.time,reference_dt)
        if loaded and not np.array_equal(times,mesh):raise ValueError('Dense reference cache mesh differs from the optimized nodes')
        if not loaded:times=mesh;Q=[];objects=[];largest=0.
        for t in (() if loaded else times):
            i,k,_=self.phase(t+self.preparation);qo=self.object_reference_local(t);seed=self.nodal_spline(t);q=seed.copy()
            for j,(f,r,frame,c,qs) in enumerate(zip(problem.task_definition.features,problem.robots,problem.frames,contacts_by_interval[i],problem.system.q_slices)):
                if not self.attached(j,t+self.preparation):continue
                To=problem.task_definition.object.fk(qo,f.frame);R=self.keyframes[i].rotations[j];target=np.eye(4);target[:3,3]=To[:3,3]+To[:3,:3]@self.keyframes[i].positions[j]
                if c.capability.kind=='G':target[:3,:3]=To[:3,:3]@R@np.diag([1.,-1.,-1.])
                else:
                    current=r.fk(seed[qs],frame)[:3,:3];target[:3,:3]=Rotation.align_vectors(np.array([-To[:3,:3]@R[:,2]]),np.array([current[:,2]]))[0].as_matrix()@current
                q[qs]=numerical_pose_ik(r,frame,target,seed[qs])
            largest=max(largest,float(np.max(abs(q-seed))));Q.append(q);objects.append(qo)
        self.reference_times=times;self.reference_q=np.array(Q);self.reference_objects=np.array(objects)
        self.spline,velocity,self.reference_mesh_diagnostics=conditioned_joint_spline(times,self.reference_q,node_indices,self.v[:,:problem.system.object_v.start])
        self.reference_identity=identity
        self.reference_projection_max_rad=largest
        if largest>policy.reference_projection_limit_rad:raise RuntimeError(f'Reference IK projection {largest:.4g} rad exceeds {policy.reference_projection_limit_rad} rad; refine trajectory grid')
        for r,qs in zip(problem.robots,problem.system.q_slices):
            if np.max(abs(velocity[:,qs])-r.velocity)>1e-4:raise RuntimeError('Projected reference exceeds robot velocity limits')
        if not loaded:np.savez_compressed(cache,identity=identity,times=times,q=self.reference_q,object_q=self.reference_objects,projection_max_rad=largest)
        self.reference_cache_hit=loaded
    def object_reference_local(self,t):
        i,k,local=self.phase(t+self.preparation);r=self.trajectories[i];offset=self.offsets[i]+np.r_[0,np.cumsum(r.dt)][k];s=np.clip((local-offset)/r.dt[k],0,1);sys=self.problem.system
        return object_interpolate(self.problem.task_definition.object,r.q[k,sys.object_q],r.q[k+1,sys.object_q],r.v[k,sys.object_v],r.v[k+1,sys.object_v],r.dt[k],s)
    def desired_object_pose(self,t):
        qo=self.object_reference_local(np.clip(t-self.preparation,0,self.time[-1]));obj=self.problem.task_definition.object;T=obj.fk(qo,obj.tree.root);return np.r_[T[:3,3],Rotation.from_matrix(T[:3,:3]).as_quat()]
    def reactions(self,t):
        return super().reactions(t) if self.force_feedforward else np.zeros((len(self.problem.robots),6))
    def identity(self):
        p=self.problem;data=dict(task=p.task_definition.id,configuration=task_configuration(p.task_definition).to_dict(),types=p.types,assignment=p.assignment,specs=[{k:v for k,v in spec.items() if k!='allowed_object_geoms'} for spec in p.specs],source_refinement=p.refine_enabled);h=hashlib.sha256(json.dumps(data,sort_keys=True).encode())
        objects=p.task_definition.object.bodies.values() if isinstance(p.task_definition.object,RigidCollection) else [p.task_definition.object]
        for r in [*p.full_robots,*objects]:h.update(r.path.read_bytes());h.update(np.asarray(r.base_pose,dtype='<f8').tobytes())
        from .full_recording import jsonable
        h.update(json.dumps(p.task_definition.keyframes,sort_keys=True,default=jsonable).encode());h.update(json.dumps(p.task_definition.fixtures,sort_keys=True,default=jsonable).encode())
        plan=hashlib.sha256()
        for r in self.trajectories:
            for a in [r.q,r.v,r.tau,r.dt]:plan.update(np.asarray(a,dtype='<f8').tobytes())
            for n in sorted(r.impulses):plan.update(n.encode());plan.update(r.impulses[n].tobytes())
        for key in self.keyframes:
            for a in [key.positions,key.rotations]:plan.update(np.asarray(a,dtype='<f8').tobytes())
        return h.hexdigest(),plan.hexdigest()


class FullContactController(CoupledContactController):
    def __init__(self,episode,scene):
        self.scene=scene
        for s in episode.problem.specs:s['allowed_object_geoms']=scene.allowed_by_group[s['group_id']]
        super().__init__(episode)
        self.allocation_logs=[];self.allocation_scope_logs=[];self.last_active=None;self.allocated=None;self.next_allocation=0.
        self.jaw_targets=np.array([episode.jaw_force_fraction*c.closing_force_per_finger if c.kind=='G' else 0. for c in self.problem.hardware_capabilities])
        self.policy=task_configuration(episode.problem.task_definition).execution
        if self.policy.online_allocation:
            from .online_wrench import WrenchAllocator,operating_capabilities
            self.allocator=WrenchAllocator(operating_capabilities(self.problem.hardware_capabilities,self.policy.allocation_grip_fraction),settings=self.problem.conic_settings,characteristic_length=self.problem.task_definition.configuration.target.characteristic_length_m,project_unactuated=True)
            self.disturbance=np.zeros(self.problem.task_definition.object.nv);self.previous_object_v=None
            self.geom_frames={geom:frame for frame,geoms in scene.object_geometries.items() for geom in geoms}
    def observe(self,mj,model,data,t):
        contacts=[]
        for idx in range(data.ncon):
            c=data.contact[idx];value=np.zeros(6);mj.mj_contactForce(model,data,idx,value);R=c.frame.reshape(3,3);contacts.append(dict(pair=[mj.mj_id2name(model,mj.mjtObj.mjOBJ_GEOM,int(g)) for g in c.geom],position=c.pos,wrench=np.r_[R.T@value[3:],R.T@value[:3]],normal_force=float(value[0])))
        addresses=[model.jnt_qposadr[mj.mj_name2id(model,mj.mjtObj.mjOBJ_JOINT,spec['id']+'_'+n)] for spec,r in zip(self.case.data['arms'],self.robots) for n in r.joint_names];frames={}
        for frame,body in self.scene.object_frames.items():
            idx=mj.mj_name2id(model,mj.mjtObj.mjOBJ_BODY,body);frames[frame]=np.r_[data.xpos[idx],data.xquat[idx][[1,2,3,0]]]
        self.observe_contacts(data.qpos[addresses],next(iter(frames.values())),contacts,frame_poses=frames)
        if self.policy.online_allocation:self.observe_object(mj,model,data,frames,contacts)

    def observe_object(self,mj,model,data,frames,contacts):
        """State/contact sensors only: never read applied-force arrays or schedule.

        Native free-joint velocity is [linear_world; angular_body]. The public
        mechanics model uses [angular_body; linear_body]. All object contacts,
        including environmental and internal contacts, enter the observer.
        """
        from .full_validation import numeric_object_point,bias_object
        obj=self.problem.task_definition.object;qs=[];vs=[]
        components=obj.bodies if isinstance(obj,RigidCollection) else {'object':obj}
        for name,component in components.items():
            body=mj.mj_name2id(model,mj.mjtObj.mjOBJ_BODY,self.scene.object_roots[name]);joint=model.body_jntadr[body];dof=model.jnt_dofadr[joint]
            pose=np.r_[data.xpos[body],data.xquat[body][[1,2,3,0]]];R=Rotation.from_quat(pose[3:]).as_matrix()
            joints=[mj.mj_name2id(model,mj.mjtObj.mjOBJ_JOINT,self.scene.object_joints[n]) for n in component.joint_names]
            qs.extend(np.r_[pose,[data.qpos[model.jnt_qposadr[j]] for j in joints]])
            vs.extend(np.r_[data.qvel[dof+3:dof+6],R.T@data.qvel[dof:dof+3],[data.qvel[model.jnt_dofadr[j]] for j in joints]])
        q=np.array(qs);v=np.array(vs);total=np.zeros(obj.nv);environment=total.copy()
        for c in contacts:
            for side,geom in enumerate(c['pair']):
                frame=self.geom_frames.get(geom)
                if frame is None:continue
                T=obj.fk(q,frame);point=T[:3,:3].T@(c['position']-T[:3,3]);_,J=numeric_object_point(obj,q,frame,point)
                generalized=J.T@(np.asarray(c['wrench'])*(1 if side==1 else -1));total+=generalized
                if not c['pair'][1-side].startswith(tuple(s['id']+'_' for s in self.problem.specs)):environment+=generalized
        M=obj.mass(q);bias=bias_object(self.problem,q,v)
        if self.previous_object_v is not None:
            raw=M@((v-self.previous_object_v)/self.dt)+bias-total
            self.disturbance+=self.dt/(self.policy.disturbance_filter_s+self.dt)*(raw-self.disturbance)
        self.previous_object_v=v.copy();self.object_state=(q,v,M,bias,environment)

    def desired_reactions(self,q,v,qd,vd,ad):
        planned=super().desired_reactions(q,v,qd,vd,ad)
        if not self.policy.online_allocation or self.t<self.episode.preparation:return planned
        i,k,local=self.episode.phase(self.t)
        from .full_trajectory import phase_mode
        active=tuple(self.episode.attached(j,self.t) and phase_mode(self.episode.keyframes[i].phase,f)=='holding' for j,f in enumerate(self.problem.task_definition.features))
        if active!=self.last_active:self.next_allocation=self.t;self.last_active=active
        if not any(active):
            if np.max(abs(planned))>2e-5:raise RuntimeError('No holding interface, but the accepted plan requests robot reactions')
            self.allocated=planned
            if self.t+1e-10>=self.next_allocation:
                self.allocation_scope_logs.append(dict(time=self.t,status='not_required_no_holding_interface',qp_executed=False))
                self.next_allocation=self.t+self.policy.allocation_dt_s
            return planned
        if self.t+1e-10<self.next_allocation:return self.allocated if self.allocated is not None else planned
        self.next_allocation=self.t+self.policy.allocation_dt_s
        from .full_validation import numeric_contact
        from .trajectory import Contact
        obj=self.problem.task_definition.object;sys=self.problem.system;qo,vo,M,bias,environment=self.object_state
        qref=self.episode.object_reference_local(local);r=self.episode.trajectories[i];vr=r.v[k,sys.object_v];ar=(r.v[k+1,sys.object_v]-vr)/r.dt[k]
        if self.t>=self.episode.preparation+self.episode.time[-1]:vr=np.zeros_like(vr);ar=np.zeros_like(ar)
        from .online_wrench import reference_tangent
        error,vr,ar=reference_tangent(obj,qo,qref,vo,vr,ar)
        acceleration=ar+self.policy.object_acceleration_k*error+self.policy.object_acceleration_d*(vr-vo)
        rhs=self.disturbance+environment-bias-M@acceleration
        state=np.r_[*[q[a:b][indices] for a,b,indices in zip(self.offsets[:-1],self.offsets[1:],self.reduced_indices)],qo]
        maps=[];tmaps=[]
        for j,(f,frame,cap,point,R) in enumerate(zip(self.problem.task_definition.features,self.problem.frames,self.problem.capabilities,self.episode.keyframes[i].positions,self.episode.keyframes[i].rotations)):
            c=Contact(f.id,point,R[:,2],cap,robot_index=j,frame=frame,object_frame=f.frame,rotation_object=R)
            _,J,_,_=numeric_contact(self.problem,c,state);X=np.c_[np.zeros((sys.nv,3)),J.T] if cap.kind=='P' else J.T
            maps.append(-X[sys.object_v]);tmaps.append(np.asarray(sys.B).T@X)
        rb=np.concatenate([robot.inverse_dynamics(q[a:b][indices],v[a:b][indices],ad[a:b][indices]) for robot,a,b,indices in zip(self.problem.robots,self.offsets[:-1],self.offsets[1:],self.reduced_indices)])
        result=self.allocator.solve(maps,rhs,planned,tmaps,rb,sys.effort,active)
        if result['accepted']:self.allocated=result['reactions']
        else:self.allocated=planned  # Explicit fallback, not a certified allocation.
        if self.policy.adaptive_jaw_force and result['accepted']:
            for j,cap in enumerate(self.problem.hardware_capabilities):
                if cap.kind!='G':continue
                target=max(self.episode.jaw_force_fraction*cap.closing_force_per_finger,self.policy.jaw_reserve_factor*max(result['jaw_required'][j]))
                target=min(target,self.policy.jaw_target_max_fraction*cap.closing_force_per_finger)
                rate=self.policy.jaw_target_rate_N_s*self.policy.allocation_dt_s;self.jaw_targets[j]+=np.clip(target-self.jaw_targets[j],-rate,rate)
        self.allocation_logs.append(dict(time=self.t,accepted=result['accepted'],status=result['status'],residual=result['residual'],controlled_dynamics_rank=result['controlled_dynamics_rank'],object_dynamics_dimension=result['object_dynamics_dimension'],full_dynamics_equality_enforced=result['full_dynamics_equality_enforced'],unactuated_request=result['unactuated_request'].tolist(),wall_time_s=result['wall_time_s'],disturbance_estimate=self.disturbance.tolist(),rhs=rhs.tolist(),reactions=self.allocated.tolist(),jaw_required_N=result['jaw_required'].tolist(),jaw_target_N=self.jaw_targets.tolist(),active=active))
        return self.allocated

    def jaw_target(self,arm,cap):
        return self.jaw_targets[arm] if self.policy.adaptive_jaw_force else super().jaw_target(arm,cap)


def reference_geometry(episode,scene):
    p=episode.problem;geometry=Geometry(p,scene);states=np.c_[episode.reference_q,episode.reference_objects];opened=[tuple(f.id for i,(f,c) in enumerate(zip(p.task_definition.features,p.capabilities)) if c.kind=='G' and not episode.attached(i,t+episode.preparation)) for t in episode.reference_times];failures=geometry.discover(states,open_grippers=opened)
    return dict(passed=not failures,samples=len(states),continuous_certificate=False,reference_interpolation='object_SE3_and_conditioned_joint_Hermite_with_dense_IK',reference_id=episode.reference_identity,reference_mesh=episode.reference_mesh_diagnostics,released_gripper_reference='native URDF open configuration after stationary opening dwell',projection_limit_rad=task_configuration(p.task_definition).execution.reference_projection_limit_rad,projection_max_rad=episode.reference_projection_max_rad,compliant_pad_allowance_m=geometry.pad_compliance_allowance,unintended_clearance_m=geometry.clearance,violations=failures)


def run_full_episode(episode,output,*,contacts_enabled=True,disturbances=(),progress=None,allow_reduced_baseline=False):
    out=Path(output);out.mkdir(parents=True,exist_ok=True);p=episode.problem
    scene=build_scene(p,out/'scene.xml',timestep=p.case.data['simulation']['timestep'],contacts_enabled=contacts_enabled);geometry=reference_geometry(episode,scene)
    (out/'reference-geometry.json').write_text(json.dumps(geometry,indent=2)+'\n')
    if not geometry['passed']:raise RuntimeError('Projected reference geometry rejected; see reference-geometry.json')
    for r,contacts in zip(episode.trajectories,episode.contacts_by_interval):
        check=validate_interval(p,r,contacts)
        if not check['passed'] and not allow_reduced_baseline:raise RuntimeError('Native independent interval rejected '+str(check))
        r.full_geometry_checked=True
    controller=FullContactController(episode,scene);names=[];actuators=[];modes=[]
    for r,spec in zip(controller.robots,p.specs):
        for j,n in enumerate(r.joint_names):
            names.append(spec['id']+'_'+n);actuators.append(spec['id']+'_motor_'+n if r.effort[j]>0 else None);modes.append('passive' if r.effort[j]==0 else 'position' if n in spec.get('gripper_profile',{}).get('drivers',{}) else 'torque')
    objects=list(scene.object_roots.values());articulated={}
    if p.task_definition.id=='box':objects.append(scene.object_frames['moving']);articulated[scene.object_frames['moving']]=scene.object_joints['articulation']
    simulator=MujocoContactScene(scene.path,names,actuators,objects,actuator_modes=modes,articulated_object_joints=articulated);case_id,plan_id=episode.identity();schedule=WrenchSchedule(disturbances)
    trace=simulator.run(controller,controller.reference,episode.duration,initial_q=controller.reference(0)[0],case_id=case_id,plan_id=plan_id,external_forces=schedule,progress=progress);trace.metadata.update(object_pose_prescribed=False,object_equality_constraints_used=0,internal_gripper_equalities=int(simulator.model.neq),external_disturbances=schedule.to_list(),reference_interpolation=geometry['reference_interpolation'],contacts_enabled=contacts_enabled)
    trace.metadata['controller_configuration']=dict(mode=episode.control_mode,arm_reaction_feedforward=episode.force_feedforward,jaw_force_fraction=episode.jaw_force_fraction,jaw_force_target_N={f.id:episode.jaw_force_fraction*c.closing_force_per_finger for f,c in zip(p.task_definition.features,p.hardware_capabilities) if c.kind=='G'},jaw_targets_dynamic=controller.policy.adaptive_jaw_force,planning_gripper_fraction=p.task_definition.configuration.target.gripper_force_fraction,joint_nullspace_stiffness=task_configuration(p.task_definition).execution.joint_k,joint_nullspace_damping=task_configuration(p.task_definition).execution.joint_d)
    trace.metadata.update(reference_id=episode.reference_identity,reference_mesh=episode.reference_mesh_diagnostics)
    if controller.policy.online_allocation:
        from dataclasses import asdict
        trace.metadata['online_allocation']=dict(policy=asdict(controller.policy),sensor_inputs='native object pose/velocity and all object contact wrenches; no applied-force/schedule input',passed_solves=sum(x['accepted'] for x in controller.allocation_logs),total_solves=len(controller.allocation_logs),no_holding_interface_updates=len(controller.allocation_scope_logs),offline_plan_reoptimized=False,hardware_validation=False)
        (out/'allocation.json').write_text(json.dumps(controller.allocation_logs,separators=(',',':')))
        (out/'allocation-scope.json').write_text(json.dumps(controller.allocation_scope_logs,separators=(',',':')))
    trace.save(out/'execution.npz');(out/'control.json').write_text(json.dumps(controller.logs,separators=(',',':')))
    # Task/capability assessment is provided separately; a trace alone is not a pass.
    return trace,controller,scene,geometry
