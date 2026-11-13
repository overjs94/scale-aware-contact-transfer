"""Independent Pinocchio/NumPy checks of manuscript trajectory equations.

No CasADi dynamics or contact Jacobian is reused by this rigid-object validator.
The reported residuals concern the discrete trajectory, not continuous safety.
"""
import numpy as np
from scipy.linalg import block_diag
from scipy.spatial.transform import Rotation


def grasp_discretization_probe(robot,frame,q0,velocity,step_sizes):
    """Continuous rigid grasp, tested with native FK/J and finite SE(3) steps.

    Object frame equals tool frame. This prescribed continuous path need not
    satisfy both exact finite-step integration and instantaneous post-step Jv.
    """
    import pinocchio as pin
    q0=np.asarray(q0,float);velocity=np.asarray(velocity,float);T0=robot.fk(q0,frame);rows=[]
    for h in step_sizes:
        q1=robot.integrate(q0,h*velocity);T1=robot.fk(q1,frame);delta=np.linalg.inv(T0)@T1
        logarithm=pin.log6(pin.SE3(delta[:3,:3],delta[:3,3])).vector/h
        discrete=np.r_[logarithm[3:],logarithm[:3]];instantaneous=robot.jacobian(q1,frame,'body')@velocity;error=instantaneous-discrete
        rows.append(dict(dt_s=float(h),angular_velocity_residual_rad_s=float(np.linalg.norm(error[:3])),linear_velocity_residual_m_s=float(np.linalg.norm(error[3:])),max_residual=float(np.max(abs(error)))))
    return rows


def cross_matrix(p):
    x,y,z=p;return np.array([[0,-z,y],[z,0,-x],[-y,x,0]])


def numeric_contact(problem,contact,q):
    sys=problem.system;obj=problem.case.panel;qo=q[sys.object_q];To=obj.fk(qo,'panel');R=To[:3,:3];p=contact.point_object;point=To[:3,3]+R@p
    Jo=obj.jacobian(qo,'panel','world_aligned');Jpoint=Jo[3:]-cross_matrix(R@p)@Jo[:3];J=np.zeros((3,sys.nv));J[:,sys.object_v]=Jpoint
    if contact.robot_index is None:
        return float(contact.normal@point-contact.plane_offset),contact.basis().T@J,None,None
    i=contact.robot_index;r=problem.robots[i];qr=q[sys.q_slices[i]];T=r.fk(qr,contact.frame);Jr=r.jacobian(qr,contact.frame,'world_aligned');Rc=R@contact.basis();gap=float(Rc[:,2]@(T[:3,3]-point));closure=T[:3,3]-point
    if contact.capability.kind=='P' and contact.project_to_surface:
        closest=T[:3,3]-gap*Rc[:,2];Jpoint=Jo[3:]-cross_matrix(closest-To[:3,3])@Jo[:3];J[:,sys.object_v]=Jpoint
    J*=-1;J[:,sys.v_slices[i]]+=Jr[3:]
    if contact.capability.kind=='P':return gap,Rc.T@J,closure,None
    Jw=np.zeros((3,sys.nv));Jw[:,sys.object_v]=-Jo[:3];Jw[:,sys.v_slices[i]]+=Jr[:3]
    return gap,np.vstack([Rc.T@Jw,Rc.T@J]),closure,T[:3,:3].T@Rc@np.diag([1.,-1.,-1.])


def validate_rigid_trajectory(problem,trajectory,contacts):
    sys=problem.system;obj=problem.case.panel
    residual=dict(momentum=0.,joint_integration=0.,object_translation_integration=0.,object_rotation_integration=0.,normal_complementarity=0.,maximum_dissipation=0.,capability=0.,grasp_closure_m=0.,grasp_closure_rad=0.,grasp_velocity=0.,joint_limit=0.,velocity_limit=0.,torque_limit=0.,endpoint_velocity=0.)
    for l,h in enumerate(trajectory.dt):
        q,qnext=trajectory.q[l:l+2];v,vnext=trajectory.v[l:l+2];M=block_diag(*[r.mass(q[s]) for r,s in zip(problem.robots,sys.q_slices)],obj.mass(q[sys.object_q]));b=np.r_[*[r.bias(q[qs],v[vs]) for r,qs,vs in zip(problem.robots,sys.q_slices,sys.v_slices)],obj.bias(q[sys.object_q],v[sys.object_v])];impulse=np.zeros(sys.nv)
        for c in contacts:
            P=trajectory.impulses[c.name][l];_,J,_,_=numeric_contact(problem,c,q);gap,Jnext,closure,closure_R=numeric_contact(problem,c,qnext);relative=Jnext@vnext;impulse+=J.T@P
            if c.capability.kind=='P':
                residual['normal_complementarity']=max(residual['normal_complementarity'],abs(P[2]*gap),-gap)
                residual['maximum_dissipation']=max(residual['maximum_dissipation'],abs(P[:2]@relative[:2]+c.capability.mu*P[2]*np.linalg.norm(relative[:2])))
                residual['capability']=max(residual['capability'],-P[2]/h,P[2]/h-c.capability.normal_max,(np.linalg.norm(P[:2])-c.capability.mu*P[2])/h)
            elif l in c.attached_steps:
                witness=c.capability.check(P/h)
                if not witness['feasible']:residual['capability']=max(residual['capability'],witness['primal_residual'])
                residual['grasp_closure_m']=max(residual['grasp_closure_m'],np.max(abs(closure)))
                residual['grasp_velocity']=max(residual['grasp_velocity'],np.max(abs(relative)))
                residual['grasp_closure_rad']=max(residual['grasp_closure_rad'],Rotation.from_matrix(closure_R).magnitude())
        dyn=M@(vnext-v)-h*(np.asarray(sys.B)@trajectory.tau[l]-b)-impulse;residual['momentum']=max(residual['momentum'],float(np.max(abs(dyn))))
        for r,qs,vs in zip(problem.robots,sys.q_slices,sys.v_slices):
            vint=.5*(v[vs]+vnext[vs]) if trajectory.configuration_update=='independent_midpoint' else vnext[vs]
            residual['joint_integration']=max(residual['joint_integration'],np.max(abs(qnext[qs]-r.integrate(q[qs],h*vint))))
            residual['joint_limit']=max(residual['joint_limit'],np.max(r.lower-qnext[qs]),np.max(qnext[qs]-r.upper))
            residual['velocity_limit']=max(residual['velocity_limit'],np.max(abs(vnext[vs])-r.velocity))
        vint=.5*(v[sys.object_v]+vnext[sys.object_v]) if trajectory.configuration_update=='independent_midpoint' else vnext[sys.object_v]
        integrated=obj.integrate(q[sys.object_q],h*vint);actual=qnext[sys.object_q]
        residual['object_translation_integration']=max(residual['object_translation_integration'],np.max(abs(integrated[:3]-actual[:3])))
        residual['object_rotation_integration']=max(residual['object_rotation_integration'],(Rotation.from_quat(integrated[3:]).inv()*Rotation.from_quat(actual[3:])).magnitude())
        residual['torque_limit']=max(residual['torque_limit'],np.max(abs(trajectory.tau[l])-sys.effort))
    residual['endpoint_velocity']=float(np.max(abs(trajectory.v[[0,-1]])))
    residual={k:float(max(0,v)) for k,v in residual.items()}
    diagnostic={}
    if trajectory.configuration_update=='grasp_consistent':
        # The root update in this explicitly identified variant is the grasp
        # retraction, checked by closure/no-slip, rather than independent Euler.
        for key in ['object_translation_integration','object_rotation_integration']:diagnostic[key]=residual.pop(key)
        bounds=trajectory.configuration_defect_bounds
        if len(bounds)!=2:residual['configuration_defect_bound']=float('inf')
        else:residual['configuration_defect_bound']=max(0.,diagnostic['object_translation_integration']-bounds[0],diagnostic['object_rotation_integration']-np.sqrt(3)*bounds[1])
    return dict(residuals=residual,reference_discretization_defect=diagnostic,configuration_update=trajectory.configuration_update,passed=max(residual.values())<=2e-5,mechanics='independent native Pinocchio RNEA/M/J + NumPy contact transforms',continuous_collision_certificate=False)
