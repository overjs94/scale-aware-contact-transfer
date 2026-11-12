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


class FullEpisode(CoupledEpisode):
    def __init__(self,problem,keyframes,trajectories,contacts_by_interval,*,reference_dt=.02,force_feedforward=True):
        super().__init__(problem,tuple(keyframes),tuple(trajectories),contacts_by_interval=tuple(contacts_by_interval),control_mode='impedance_feedforward',jaw_force_fraction=.7)
        self.force_feedforward=force_feedforward;self.nodal_spline=self.spline;self.reference_dt=reference_dt
        # Only interpolation is cached. Model/plan/code identity must match;
        # velocity bounds and native geometry remain fresh execution gates.
        cache=problem.directory/'dense-reference.npz';digest=hashlib.sha256()
        for name in ['full_execution.py','full_trajectory.py','contracted.py']:digest.update(Path(__file__).with_name(name).read_bytes())
        for part in self.identity():digest.update(part.encode())
        digest.update(str(reference_dt).encode());identity=digest.hexdigest();loaded=False
        if cache.exists():
            with np.load(cache,allow_pickle=False) as a:
                if str(a['identity'])==identity:
                    times=a['times'].copy();Q=a['q'].copy();objects=a['object_q'].copy();largest=float(a['projection_max_rad']);loaded=True
        if not loaded:times=np.unique(np.r_[self.time,np.arange(0,self.time[-1],reference_dt),self.time[-1]]);Q=[];objects=[];largest=0.
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
        self.reference_times=times;self.reference_q=np.array(Q);self.reference_objects=np.array(objects);velocity=np.gradient(self.reference_q,times,axis=0,edge_order=2);velocity[[0,-1]]=0
        self.reference_projection_max_rad=largest
        if largest>.02:raise RuntimeError(f'Reference IK projection {largest:.4g} rad exceeds 0.02 rad; refine trajectory grid')
        for r,qs in zip(problem.robots,problem.system.q_slices):
            if np.max(abs(velocity[:,qs])-r.velocity)>1e-4:raise RuntimeError('Projected reference exceeds robot velocity limits')
        self.spline=CubicHermiteSpline(times,self.reference_q,velocity)
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
        p=self.problem;data=dict(task=p.task_definition.id,types=p.types,assignment=p.assignment,specs=[{k:v for k,v in spec.items() if k!='allowed_object_geoms'} for spec in p.specs],source_refinement=p.refine_enabled);h=hashlib.sha256(json.dumps(data,sort_keys=True).encode())
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
    def observe(self,mj,model,data,t):
        contacts=[]
        for idx in range(data.ncon):
            c=data.contact[idx];value=np.zeros(6);mj.mj_contactForce(model,data,idx,value);R=c.frame.reshape(3,3);contacts.append(dict(pair=[mj.mj_id2name(model,mj.mjtObj.mjOBJ_GEOM,int(g)) for g in c.geom],position=c.pos,wrench=np.r_[R.T@value[3:],R.T@value[:3]],normal_force=float(value[0])))
        addresses=[model.jnt_qposadr[mj.mj_name2id(model,mj.mjtObj.mjOBJ_JOINT,spec['id']+'_'+n)] for spec,r in zip(self.case.data['arms'],self.robots) for n in r.joint_names];frames={}
        for frame,body in self.scene.object_frames.items():
            idx=mj.mj_name2id(model,mj.mjtObj.mjOBJ_BODY,body);frames[frame]=np.r_[data.xpos[idx],data.xquat[idx][[1,2,3,0]]]
        self.observe_contacts(data.qpos[addresses],next(iter(frames.values())),contacts,frame_poses=frames)


def reference_geometry(episode,scene):
    p=episode.problem;geometry=Geometry(p,scene);states=np.c_[episode.reference_q,episode.reference_objects];opened=[tuple(f.id for i,(f,c) in enumerate(zip(p.task_definition.features,p.capabilities)) if c.kind=='G' and not episode.attached(i,t+episode.preparation)) for t in episode.reference_times];failures=geometry.discover(states,open_grippers=opened)
    return dict(passed=not failures,samples=len(states),continuous_certificate=False,reference_interpolation='object_SE3_and_joint_Hermite_with_dense_robot_IK',released_gripper_reference='native URDF open configuration after stationary opening dwell',projection_limit_rad=.02,projection_max_rad=episode.reference_projection_max_rad,compliant_pad_allowance_m=geometry.pad_compliance_allowance,unintended_clearance_m=geometry.clearance,violations=failures)


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
    trace.save(out/'execution.npz');(out/'control.json').write_text(json.dumps(controller.logs,separators=(',',':')))
    # Task/capability assessment is provided separately; a trace alone is not a pass.
    return trace,controller,scene,geometry
