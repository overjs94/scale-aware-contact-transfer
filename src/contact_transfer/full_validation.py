"""Independent numerical mechanics and native geometry for full-task plans."""
from pathlib import Path
import casadi as ca
import numpy as np
from scipy.linalg import block_diag
from scipy.spatial.transform import Rotation
from .multi_object import RigidCollection,BodyContact
from .contacts import contact_basis


def cross(p):
    x,y,z=p;return np.array([[0,-z,y],[z,0,-x],[-y,x,0]])


def numeric_object_point(obj,q,frame,point):
    T=obj.fk(q,frame);J=obj.jacobian(q,frame,'world_aligned');offset=T[:3,:3]@point
    return T[:3,3]+offset,np.vstack([J[:3],J[3:]-cross(offset)@J[:3]])


def numeric_contact(problem,c,q):
    sys=problem.system;obj=problem.task_definition.object;qo=q[sys.object_q];point,Jo=numeric_object_point(obj,qo,c.object_frame,c.point_object);Tobj=obj.fk(qo,c.object_frame)
    if isinstance(c,BodyContact):
        other,Jb=numeric_object_point(obj,qo,c.other_frame,c.other_point);R=Tobj[:3,:3]@c.basis();gap=R[:,2]@(other-point);closest=other-gap*R[:,2];local=Tobj[:3,:3].T@(closest-Tobj[:3,3]);_,Ja=numeric_object_point(obj,qo,c.object_frame,local);J=np.zeros((3,sys.nv));J[:,sys.object_v]=R.T@(Jb[3:]-Ja[3:]);return gap,J,None,None
    if c.robot_index is None:
        J=np.zeros((3,sys.nv));J[:,sys.object_v]=c.basis().T@Jo[3:];return c.normal@point-c.plane_offset,J,None,None
    i=c.robot_index;robot=problem.robots[i];qr=q[sys.q_slices[i]];T=robot.fk(qr,c.frame);Jr=robot.jacobian(qr,c.frame,'world_aligned');R=Tobj[:3,:3]@c.basis();gap=R[:,2]@(T[:3,3]-point)
    if c.capability.kind=='P' and c.project_to_surface:
        closest=T[:3,3]-gap*R[:,2];local=Tobj[:3,:3].T@(closest-Tobj[:3,3]);_,Jo=numeric_object_point(obj,qo,c.object_frame,local)
    Jp=np.zeros((3,sys.nv));Jp[:,sys.v_slices[i]]=Jr[3:];Jp[:,sys.object_v]=-Jo[3:]
    if c.capability.kind=='P':return gap,R.T@Jp,T[:3,3]-point,None
    Jw=np.zeros((3,sys.nv));Jw[:,sys.v_slices[i]]=Jr[:3];Jw[:,sys.object_v]=-Jo[:3]
    return gap,np.vstack([R.T@Jw,R.T@Jp]),T[:3,3]-point,T[:3,:3].T@R@np.diag([1.,-1.,-1.])


def bias_object(problem,q,v):
    obj=problem.task_definition.object;b=obj.bias(q,v)
    if problem.task_definition.id=='box':
        cfg=problem.task_definition.original.data;b[-1]+=cfg['damping']*v[-1]+cfg['frictionloss']*np.tanh(v[-1]/.001)
    return b


def validate_keyframe(problem,key,environment=(),tolerance=2e-5):
    from .trajectory import Contact
    sys=problem.system;contacts=[]
    for i,(f,frame,cap,p,R) in enumerate(zip(problem.task_definition.features,problem.frames,problem.capabilities,key.positions,key.rotations)):contacts.append(Contact(f.id,p,R[:,2],cap,robot_index=i,frame=frame,object_frame=f.frame,rotation_object=R))
    residuals=dict(equilibrium=0.,capability=0.,torque=0.,closure=0.,rotation_closure=0.,rho_margin=0.)
    b=np.r_[*[r.gravity(key.q[qs]) for r,qs in zip(problem.robots,sys.q_slices)],bias_object(problem,key.q[sys.object_q],np.zeros(sys.object_v.stop-sys.object_v.start))]
    active={s.group_id for s in key.interfaces}
    for k,(forces,tau,vertex) in enumerate(zip(key.reactions,key.torques,problem.task_load(key.q[sys.object_q]).vertices)):
        total=np.array(sys.B)@tau
        for c,w in zip(contacts,forces):
            _,J,closure,rotation=numeric_contact(problem,c,key.q);total+=J.T@(w[3:] if c.capability.kind=='P' else w)
            if c.name in active:
                residuals['closure']=max(residuals['closure'],np.max(abs(closure)))
                if rotation is not None:residuals['rotation_closure']=max(residuals['rotation_closure'],Rotation.from_matrix(rotation).magnitude())
            if c.capability.kind=='P':residuals['capability']=max(residuals['capability'],-w[5],w[5]-c.capability.normal_max,np.linalg.norm(w[3:5])-c.capability.mu*w[5])
            elif not c.capability.check(w)['feasible']:residuals['capability']=max(residuals['capability'],c.capability.check(w)['primal_residual'])
        for c in environment:
            _,J,_,_=numeric_contact(problem,c,key.q);total+=J.T@key.environment_reactions[c.name][k]
        perturb=np.zeros(sys.nv);load=problem.task_load(key.q[sys.object_q]);perturb[sys.object_v]=key.rho*load.E@load.D@vertex;residuals['equilibrium']=max(residuals['equilibrium'],np.max(abs(b-perturb-total)));residuals['torque']=max(residuals['torque'],np.max(abs(tau)-sys.effort))
    from .generalized_conic import fixed_generalized_margin
    enabled={s.group_id for s in key.interfaces if s.force_enabled};maps=[];torque_maps=[];caps=[]
    for c in [*contacts,*environment]:
        if c.robot_index is not None and c.name not in enabled:continue
        gap,J,_,_=numeric_contact(problem,c,key.q)
        if c.robot_index is None and (gap>tolerance or not support_patch_contains(problem,c,key.q)):continue
        expanded=np.c_[np.zeros((sys.nv,3)),J.T] if c.capability.kind=='P' else J.T;maps.append(-expanded[sys.object_v]);torque_maps.append(np.array(sys.B).T@expanded);caps.append(c.capability)
    check=fixed_generalized_margin(maps,problem.task_load(key.q[sys.object_q]),caps,torque_gravity=np.array(sys.B).T@b,torque_maps=torque_maps,effort=sys.effort);maximum_status=check['status']
    if not check['certificate']:
        # A maximum-margin SOCP can stall on a degenerate exposed cone face.
        # Certify the NLP's *same* rho with a strictly convex witness problem;
        # neither shrink rho nor accept a failed solver status.
        check=fixed_generalized_margin(maps,problem.task_load(key.q[sys.object_q]),caps,torque_gravity=np.array(sys.B).T@b,torque_maps=torque_maps,effort=sys.effort,fixed_rho=key.rho)
    residuals['rho_margin']=max(0.,key.rho-check['rho']);residuals={k:float(max(0.,v)) for k,v in residuals.items()}
    return dict(passed=check['certificate'] and max(residuals.values())<=tolerance,residuals=residuals,frozen_conic=dict(rho=check['rho'],certificate=check['certificate'],residual=check['primal_residual'],status=check['status'],maximum_margin_status=maximum_status,fixed_rho_feasibility=check['fixed_rho_feasibility'],solver_attempts=check['solver_attempts']),independent='native Pinocchio, NumPy and frozen Clarabel')


def support_patch_contains(problem,c,q,tolerance=2e-5):
    obj=problem.task_definition.object;qo=q[problem.system.object_q]
    if isinstance(c,BodyContact) and c.patch_half_size is not None:
        a=obj.fk(qo,c.object_frame);b=obj.fk(qo,c.other_frame);delta=b[:3,3]+b[:3,:3]@c.other_point-a[:3,3]-a[:3,:3]@c.point_object;tangent=(a[:3,:3]@c.basis()).T@delta
        return np.all(abs(tangent[:2])<=c.patch_half_size+tolerance)
    if c.plane_bounds is not None:
        T=obj.fk(qo,c.object_frame);point=T[:3,3]+T[:3,:3]@c.point_object;return np.all(point[:2]>=c.plane_bounds[0]-tolerance) and np.all(point[:2]<=c.plane_bounds[1]+tolerance)
    return True


def validate_interval(problem,r,contacts,tolerance=2e-5):
    sys=problem.system;obj=problem.task_definition.object;res=dict(momentum=0.,integration=0.,complementarity=0.,dissipation=0.,capability=0.,closure=0.,rotation_closure=0.,grasp_velocity=0.,point_alignment=0.,point_tangent_bounds=0.,limits=0.,endpoint_velocity=0.,quaternion_norm=0.);defect=dict(maximum_translation_component_m=0.,maximum_rotation_component_rad=0.,maximum_translation_norm_m=0.,maximum_rotation_norm_rad=0.,bounds=r.configuration_defect_bounds)
    for k,h in enumerate(r.dt):
        q,qn=r.q[k:k+2];v,vn=r.v[k:k+2];M=block_diag(*[robot.mass(q[s]) for robot,s in zip(problem.robots,sys.q_slices)],obj.mass(q[sys.object_q]));b=np.r_[*[robot.bias(q[qs],v[vs]) for robot,qs,vs in zip(problem.robots,sys.q_slices,sys.v_slices)],bias_object(problem,q[sys.object_q],v[sys.object_v])];impulse=np.zeros(sys.nv)
        for c in contacts:
            P=r.impulses[c.name][k];_,J,_,_=numeric_contact(problem,c,q);gap,Jn,closure,rotation=numeric_contact(problem,c,qn);relative=Jn@vn;impulse+=J.T@P
            if c.capability.kind=='P':
                res['complementarity']=max(res['complementarity'],abs(P[2]*gap),-gap);res['dissipation']=max(res['dissipation'],abs(P[:2]@relative[:2]+c.capability.mu*P[2]*np.linalg.norm(relative[:2])));res['capability']=max(res['capability'],-P[2]/h,P[2]/h-c.capability.normal_max,(np.linalg.norm(P[:2])-c.capability.mu*P[2])/h)
                if c.robot_index is not None:
                    if k in c.maintained_steps or k in c.sticking_steps:
                        res['closure']=max(res['closure'],abs(gap))
                        if c.align_normal:
                            To=obj.fk(qn[sys.object_q],c.object_frame);Tee=problem.robots[c.robot_index].fk(qn[sys.q_slices[c.robot_index]],c.frame);Rw=To[:3,:3]@c.basis();res['point_alignment']=max(res['point_alignment'],np.max(abs(Rw[:,:2].T@Tee[:3,2])),Rw[:,2]@Tee[:3,2]+.5)
                    if k in c.sticking_steps:res['grasp_velocity']=max(res['grasp_velocity'],np.max(abs(relative[:2])))
                    if c.tangent_bounds is not None and (c.nodes is None or k in c.nodes) and k not in c.released_steps:
                        tangent=(obj.fk(qn[sys.object_q],c.object_frame)[:3,:3]@c.basis()).T@closure;res['point_tangent_bounds']=max(res['point_tangent_bounds'],np.max(c.tangent_bounds[0]-tangent[:2]),np.max(tangent[:2]-c.tangent_bounds[1]))
                    if (c.nodes is not None and k not in c.nodes) or k in c.released_steps:res['capability']=max(res['capability'],np.max(abs(P))/h)
                elif not support_patch_contains(problem,c,qn):
                    # A separated/outside support candidate cannot manufacture
                    # a load. Compare in the same impulse-gap units as CITO.
                    if isinstance(c,BodyContact):
                        a=obj.fk(qn[sys.object_q],c.object_frame);bb=obj.fk(qn[sys.object_q],c.other_frame);tangent=(a[:3,:3]@c.basis()).T@(bb[:3,3]+bb[:3,:3]@c.other_point-a[:3,3]-a[:3,:3]@c.point_object);outside=np.max(abs(tangent[:2])-c.patch_half_size)
                    else:
                        point,_=numeric_object_point(obj,qn[sys.object_q],c.object_frame,c.point_object);outside=max(np.max(c.plane_bounds[0]-point[:2]),np.max(point[:2]-c.plane_bounds[1]))
                    res['complementarity']=max(res['complementarity'],P[2]*outside)
            elif k in c.attached_steps:
                check=c.capability.check(P/h)
                if not check['feasible']:res['capability']=max(res['capability'],check['primal_residual'])
                res['closure']=max(res['closure'],np.max(abs(closure)));res['rotation_closure']=max(res['rotation_closure'],Rotation.from_matrix(rotation).magnitude());res['grasp_velocity']=max(res['grasp_velocity'],np.max(abs(relative)))
            else:res['capability']=max(res['capability'],np.max(abs(P))/h)
        res['momentum']=max(res['momentum'],np.max(abs(M@(vn-v)-h*(np.array(sys.B)@r.tau[k]-b)-impulse)))
        vint=(v+vn)/2 if r.configuration_update in {'independent_midpoint','grasp_consistent_midpoint'} else vn;pred=[]
        for robot,qs,vs in zip(problem.robots,sys.q_slices,sys.v_slices):pred.extend(robot.integrate(q[qs],h*vint[vs]));res['limits']=max(res['limits'],np.max(robot.lower-qn[qs]),np.max(qn[qs]-robot.upper),np.max(abs(vn[vs])-robot.velocity))
        pred.extend(obj.integrate(q[sys.object_q],h*vint[sys.object_v]));pred=np.array(pred)
        if r.configuration_update=='grasp_consistent_midpoint':
            roots=obj.q_slices if isinstance(obj,RigidCollection) else {obj.tree.root:slice(0,7)}
            for name,sl in roots.items():
                grasp=any(c.robot_index is not None and c.capability.has_moment and k in c.attached_steps and (not isinstance(obj,RigidCollection) or c.object_frame==name) for c in contacts);points=[c.point_object for c in contacts if c.robot_index is not None and c.capability.kind=='P' and k in c.sticking_steps and c.object_frame==name];rank=np.linalg.matrix_rank(np.array(points[1:])-points[0],tol=1e-9) if len(points)>=3 else 0
                if not (grasp or rank>=2):continue
                start=sys.object_q.start+sl.start;delta=qn[start:start+3]-pred[start:start+3];Rerr=Rotation.from_quat(pred[start+3:start+7]).inv()*Rotation.from_quat(qn[start+3:start+7]);vec=Rerr.as_rotvec();small=vec*(np.sin(np.linalg.norm(vec))/np.linalg.norm(vec)) if np.linalg.norm(vec)>1e-15 else vec;dp=float(np.max(abs(delta)));dr=float(np.max(abs(small)));defect['maximum_translation_component_m']=max(defect['maximum_translation_component_m'],dp);defect['maximum_rotation_component_rad']=max(defect['maximum_rotation_component_rad'],dr);defect['maximum_translation_norm_m']=max(defect['maximum_translation_norm_m'],float(np.linalg.norm(delta)));defect['maximum_rotation_norm_rad']=max(defect['maximum_rotation_norm_rad'],float(np.linalg.norm(vec)));res['integration']=max(res['integration'],dp-r.configuration_defect_bounds[0],dr-r.configuration_defect_bounds[1],np.linalg.norm(vec)-np.pi/2);pred[start:start+7]=qn[start:start+7]
        res['integration']=max(res['integration'],np.max(abs(qn-pred)));res['limits']=max(res['limits'],np.max(abs(r.tau[k])-sys.effort))
        for sl in getattr(obj,'quaternion_slices',(slice(3,7),)):res['quaternion_norm']=max(res['quaternion_norm'],abs(np.linalg.norm(qn[sys.object_q][sl])-1))
        if obj.n:res['limits']=max(res['limits'],np.max(obj.lower-qn[sys.object_q][7:]),np.max(qn[sys.object_q][7:]-obj.upper),np.max(abs(vn[sys.object_v][6:])-obj.velocity))
    res['endpoint_velocity']=np.max(abs(r.v[[0,-1]]));res={k:float(max(0.,v)) for k,v in res.items()};return dict(passed=max(res.values())<=tolerance,residuals=res,configuration_update=r.configuration_update,configuration_defect=defect,independent='native Pinocchio mass/bias/J and NumPy transforms')


class Geometry:
    def __init__(self,problem,scene,clearance=.001,tolerance=2e-5,pad_compliance_allowance=.001):
        import mujoco as mj
        self.mj=mj;self.problem=problem;self.scene=scene;self.model=mj.MjModel.from_xml_path(str(scene.path));self.data=mj.MjData(self.model);self.clearance=clearance;self.tolerance=tolerance;self.pad_compliance_allowance=pad_compliance_allowance;self.model.geom_margin[:]=clearance;self.model.pair_margin[:]=clearance;self.last_violations=[];self.addresses=[];self.locked=[]
        for reduced,full,spec in zip(problem.robots,problem.full_robots,problem.specs):
            adr=[self.model.jnt_qposadr[mj.mj_name2id(self.model,mj.mjtObj.mjOBJ_JOINT,spec['id']+'_'+n)] for n in reduced.joint_names];locks={self.model.jnt_qposadr[mj.mj_name2id(self.model,mj.mjtObj.mjOBJ_JOINT,spec['id']+'_'+n)]:v for n,v in reduced.tree.locked.items()};self.addresses.append(adr);self.locked.append(locks)
    def set_state(self,q):
        m,d,mj=self.model,self.data,self.mj;d.qpos[:]=m.qpos0;sys=self.problem.system
        for qs,adr,locked in zip(sys.q_slices,self.addresses,self.locked):
            d.qpos[adr]=q[qs]
            for i,v in locked.items():d.qpos[i]=v
        obj=self.problem.task_definition.object;qo=q[sys.object_q]
        for name,body in self.scene.object_roots.items():
            pose=qo[obj.q_slices[name]][:7] if isinstance(obj,RigidCollection) else qo[:7];bid=mj.mj_name2id(m,mj.mjtObj.mjOBJ_BODY,body);j=m.body_jntadr[bid];adr=m.jnt_qposadr[j];d.qpos[adr:adr+7]=pose[[0,1,2,6,3,4,5]]
        if not isinstance(obj,RigidCollection):
            for i,n in enumerate(obj.joint_names):j=mj.mj_name2id(m,mj.mjtObj.mjOBJ_JOINT,self.scene.object_joints[n]);d.qpos[m.jnt_qposadr[j]]=qo[7+i]
        mj.mj_forward(m,d)
    def intended(self,pair):
        for s in self.problem.specs:
            if any(n.startswith(s['id']+'_') and any(t in n for t in ['_tool_collision_','_left_pad_collision_','_right_pad_collision_']) for n in pair) and any(n in self.scene.allowed_by_group[s['group_id']] for n in pair):return True
        if any('_panda_link0_collision_' in n for n in pair) and any(n=='floor' or n.endswith('pedestal') for n in pair):return True
        object_geom=set(sum(self.scene.object_geometries.values(),[]))
        if any(n in object_geom for n in pair) and any(n.startswith('jig_') or n.startswith('top_cradle_') or n=='leg_cradle' or n=='floor' for n in pair):return True
        if self.problem.task_definition.id=='assembly' and all(n in object_geom for n in pair):return True
        return False
    def discover(self,states):
        failures=[]
        for k,q in enumerate(states):
            self.set_state(q)
            for c in self.data.contact:
                pair=tuple(self.mj.mj_id2name(self.model,self.mj.mjtObj.mjOBJ_GEOM,int(g)) for g in c.geom);allowed=self.intended(pair);pad=any('_left_pad_collision_' in n or '_right_pad_collision_' in n for n in pair);margin=-self.pad_compliance_allowance if allowed and pad else 0. if allowed else self.clearance
                if c.dist<margin-self.tolerance:failures.append(dict(node=k,pair=pair,gap_m=float(c.dist),required_m=margin))
        self.last_violations=failures;return failures
