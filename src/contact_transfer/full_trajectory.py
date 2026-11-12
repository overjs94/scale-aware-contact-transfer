"""Restricted contact-implicit intervals for panel, box and two-body assembly."""
import numpy as np
import casadi as ca
from scipy.spatial.transform import Rotation
import pinocchio as pin
from .contacts import contact_basis
from .trajectory import Contact,TrajectoryResult,solve_cito
from .generalized_load import GeneralizedTaskLoad
from .generalized_conic import fixed_generalized_margin
from .multi_object import RigidCollection
from .contracted import numerical_pose_ik
from .full_scenario import point_pose_ik
from .full_scene import support_contacts
from .full_validation import numeric_contact,numeric_object_point,bias_object,support_patch_contains,validate_interval


def refine_kinematic_initialization(problem,initial,contacts,settings,*,minimize_band=False):
    """Solve only collocation/grasp kinematics before adding force witnesses.

    The default enforces a 5e-6 grasp-velocity band while staying near the
    smooth path. An optional diagnostic minimizes the band instead. Its
    output is only a seed: it neither certifies dynamics nor replaces CITO.
    """
    from .optimization import equality,solve
    from .lie import rotation_error,rotation_cost
    sys=problem.system;N=len(initial.dt);opti=ca.Opti();Q=opti.variable(sys.nq,N+1);V=opti.variable(sys.nv,N+1);eta=opti.variable() if minimize_band else ca.DM(5e-6)
    if minimize_band:opti.subject_to(opti.bounded(0,eta,.01));opti.set_initial(eta,.001)
    opti.set_initial(Q,initial.q.T);opti.set_initial(V,initial.v.T);equality(opti,Q[:,0]-initial.q[0]);equality(opti,V[:,0]);equality(opti,V[:,-1]);objective=1e5*eta if minimize_band else 0.
    from .lie import quat_rotation
    slices=[slice(sys.object_q.start+s.start,sys.object_q.start+s.stop) for s in getattr(problem.task_definition.object,'quaternion_slices',(slice(3,7),))];skipped={i for sl in slices for i in range(sl.start,sl.stop)};indices=[i for i in range(sys.nq) if i not in skipped];equality(opti,Q[indices,-1]-initial.q[-1,indices])
    for sl in slices:equality(opti,rotation_error(quat_rotation(Q[sl,-1]),quat_rotation(ca.DM(initial.q[-1,sl]))));opti.subject_to(ca.dot(Q[sl,-1],ca.DM(initial.q[-1,sl]))>=.5)
    for j in range(N+1):
        for r,qs,vs in zip(problem.robots,sys.q_slices,sys.v_slices):
            if j not in (0,N):opti.subject_to(opti.bounded(r.lower,Q[qs,j],r.upper));opti.subject_to(opti.bounded(-r.velocity,V[vs,j],r.velocity))
        if problem.task_definition.object.n and j not in (0,N):
            obj=problem.task_definition.object;opti.subject_to(opti.bounded(obj.lower,Q[sys.object_q.start+7:sys.object_q.stop,j],obj.upper));opti.subject_to(opti.bounded(-obj.velocity,V[sys.object_v.start+6:sys.object_v.stop,j],obj.velocity))
        objective+=ca.sumsqr(Q[:,j]-initial.q[j])+.001*ca.sumsqr(V[:,j])
    for k,h in enumerate(initial.dt):
        equality(opti,Q[:,k+1]-sys.integrate(Q[:,k],h*.5*(V[:,k]+V[:,k+1])))
        for c in contacts:
            gap,J,closure=c.geometry(sys,Q[:,k+1]);relative=J@V[:,k+1]
            if c.capability.kind=='G' and k in c.attached_steps:
                if k<N-1:
                    equality(opti,closure);T=sys.robots[c.robot_index].fk(Q[sys.q_slices[c.robot_index],k+1],c.frame);Rw=sys.object_pose(Q[:,k+1],c.object_frame)[:3,:3]@c.basis();desired=Rw@ca.diag(ca.DM([1.,-1.,-1.]));equality(opti,rotation_error(T[:3,:3],desired));opti.subject_to(rotation_cost(T[:3,:3],desired)<=1.);opti.subject_to(opti.bounded(-eta,relative,eta))
            elif c.capability.kind=='P':
                if k<N-1 and k not in c.maintained_steps and k not in c.sticking_steps:opti.subject_to(gap>=0)
                if k<N-1 and (k in c.maintained_steps or k in c.sticking_steps):
                    equality(opti,gap)
                    if c.align_normal:
                        T=sys.robots[c.robot_index].fk(Q[sys.q_slices[c.robot_index],k+1],c.frame);Rw=sys.object_pose(Q[:,k+1],c.object_frame)[:3,:3]@c.basis();equality(opti,Rw[:,:2].T@T[:3,2]);opti.subject_to(ca.dot(Rw[:,2],T[:3,2])<=-.5)
                if k<N-1 and c.tangent_bounds is not None and (c.nodes is None or k in c.nodes):
                    To=sys.object_pose(Q[:,k+1],c.object_frame);tangent=(To[:3,:3]@c.basis()).T@closure;opti.subject_to(opti.bounded(c.tangent_bounds[0],tangent[:2],c.tangent_bounds[1]))
                if k in c.sticking_steps and k<N-1:equality(opti,relative[:2])
    opti.minimize(objective);initializer_solved=True
    try:sol,res=solve(opti,settings);value=sol.value
    except RuntimeError:
        if minimize_band:raise
        # An initializer is not a planning certificate. A finite near-feasible
        # iterate may seed the mandatory fresh CITO even when this auxiliary
        # path-proximity objective did not terminate. Its failed status stays
        # explicit; no trial can pass from this iterate alone.
        value=opti.debug.value;g=np.asarray(value(opti.g)).ravel();lo=np.asarray(value(opti.lbg)).ravel();hi=np.asarray(value(opti.ubg)).ravel();res=float(max(np.max(lo-g,initial=0.),np.max(g-hi,initial=0.),0.))
        if not np.isfinite(g).all() or res>1e-4:raise
        initializer_solved=False
    result=TrajectoryResult(np.array(value(Q)).T,np.array(value(V)).T,initial.tau.copy(),initial.dt.copy(),{n:a.copy() for n,a in initial.impulses.items()},0.,res,0.,0.,0.,False,())
    required=max((np.max(abs(numeric_contact(problem,c,result.q[k+1])[1]@result.v[k+1])) for c in contacts if c.capability.kind=='G' for k in c.attached_steps),default=0.);report=dict(required_grasp_velocity_band=float(required),enforced_grasp_velocity_band=float(value(eta)),velocity_band_minimized=minimize_band,initializer_solver_status=opti.stats().get('return_status'),initializer_solved=initializer_solved,accepted_for_execution=False,primal_residual=res,steps=N,dynamics_certified=False)
    for k,h in enumerate(result.dt):
        tau,forces=nominal_distribution(problem,result.q[k],result.v[k],(result.v[k+1]-result.v[k])/h,contacts,k,N,next_configuration=result.q[k+1]);result.tau[k]=tau
        for c in contacts:result.impulses[c.name][k]=h*forces[c.name]
    return result,report


def hermite(x0,x1,v0,v1,h,s):
    return (2*s**3-3*s*s+1)*x0+(s**3-2*s*s+s)*h*v0+(-2*s**3+3*s*s)*x1+(s**3-s*s)*h*v1


def pose_interpolate(q0,q1,v0,v1,h,s):
    R0,R1=Rotation.from_quat(q0[3:7]),Rotation.from_quat(q1[3:7]);relative=R0.inv()*R1;phi=relative.as_rotvec();end_rate=pin.Jlog3(relative.as_matrix())@v1[:3];rot=R0*Rotation.from_rotvec(hermite(np.zeros(3),phi,v0[:3],end_rate,h,s));p=hermite(q0[:3],q1[:3],R0.apply(v0[3:6]),R1.apply(v1[3:6]),h,s)
    return np.r_[p,rot.as_quat(),hermite(q0[7:],q1[7:],v0[6:],v1[6:],h,s)]


def object_interpolate(obj,q0,q1,v0,v1,h,s):
    if isinstance(obj,RigidCollection):return np.concatenate([pose_interpolate(q0[qs],q1[qs],v0[obj.v_slices[n]],v1[obj.v_slices[n]],h,s) for n,qs in obj.q_slices.items()])
    return pose_interpolate(q0,q1,v0,v1,h,s)


def difference(obj,q0,q1,h):
    def single(a,b):
        R0=Rotation.from_quat(a[3:7]).as_matrix();R1=Rotation.from_quat(b[3:7]).as_matrix();T=np.eye(4);T[:3,:3]=R0.T@R1;T[:3,3]=R0.T@(b[:3]-a[:3]);x=pin.log6(pin.SE3(T[:3,:3],T[:3,3])).vector;return np.r_[x[3:],x[:3],b[7:]-a[7:]]/h
    return np.concatenate([single(q0[s],q1[s]) for s in obj.q_slices.values()]) if isinstance(obj,RigidCollection) else single(q0,q1)


def constant_twist_object(obj,q0,q1,alpha):
    def single(a,b):
        R0=Rotation.from_quat(a[3:7]).as_matrix();R1=Rotation.from_quat(b[3:7]).as_matrix();T0=pin.SE3(R0,a[:3]);T1=pin.SE3(R1,b[:3]);T=T0*pin.exp6(alpha*pin.log6(T0.inverse()*T1));return np.r_[T.translation,Rotation.from_matrix(T.rotation).as_quat(),(1-alpha)*a[7:]+alpha*b[7:]]
    return np.concatenate([single(q0[s],q1[s]) for s in obj.q_slices.values()]) if isinstance(obj,RigidCollection) else single(q0,q1)


def phase_mode(phase,feature):
    if phase=='released' or phase=='release_leg' and feature.id=='leg_gripper':return 'released'
    if phase=='unload' or phase=='unload_leg' and feature.id=='leg_gripper':return 'unloaded'
    return 'holding'


def interval_contacts(problem,k,left,steps):
    keys=problem.task_definition.keyframes;now,later=keys[k]['phase'],keys[k+1]['phase'];contacts=[]
    for i,(f,cap,frame,p,R) in enumerate(zip(problem.task_definition.features,problem.capabilities,problem.frames,left.positions,left.rotations)):
        active=phase_mode(now,f)=='holding';unload=phase_mode(later,f)=='unloaded';attached=tuple(range(steps)) if active and cap.kind=='G' else ();nodes=tuple(range(steps-1 if unload else steps)) if active else ()
        contacts.append(Contact(f.id,p,R[:,2],cap,robot_index=i,frame=frame,object_frame=f.frame,rotation_object=R,attached_steps=attached,nodes=nodes,maintained_steps=tuple(range(steps)) if active and cap.kind=='P' else (),sticking_steps=tuple(range(steps)) if active and cap.kind=='P' else (),released_steps=tuple(range(steps)) if not active and cap.kind=='P' else (),release_at_end=unload,project_to_surface=cap.kind=='P',align_normal=cap.kind=='P',tangent_bounds=(np.full(2,-.0002),np.full(2,.0002)) if active and cap.kind=='P' else None))
    # Restricted task candidates: jig/shoulder contact is introduced only in
    # the approach/seat intervals, not in a prescribed distant free-flight phase.
    for c in support_contacts(problem,now,steps):
        a=problem.task_definition.keyframes[k]['object_q'];b=problem.task_definition.keyframes[k+1]['object_q'];qa=np.r_[left.q[:problem.system.object_q.start],a];qb=np.r_[left.q[:problem.system.object_q.start],b]
        if min(numeric_contact(problem,c,qa)[0],numeric_contact(problem,c,qb)[0])<=.03:contacts.append(c)
    return tuple(contacts)


def nominal_distribution(problem,q,v,a,contacts,step,steps,*,next_configuration=None):
    """Frozen nominal dynamics warm start; inactive/separated candidates stay zero."""
    sys=problem.system;obj=problem.task_definition.object;maps=[];tmaps=[];caps=[];selected=[]
    robot_load=np.concatenate([r.inverse_dynamics(q[qs],v[vs],a[vs]) for r,qs,vs in zip(problem.robots,sys.q_slices,sys.v_slices)])
    required=-(obj.mass(q[sys.object_q])@a[sys.object_v]+bias_object(problem,q[sys.object_q],v[sys.object_v]))
    for c in contacts:
        gap,J,_,_=numeric_contact(problem,c,q)
        if c.robot_index is not None:
            active=(step in c.attached_steps and not (c.release_at_end and step==max(c.attached_steps))) if c.capability.kind=='G' else (c.nodes is None or step in c.nodes) and step not in c.released_steps
        else:
            contact_q=q if next_configuration is None else next_configuration;active=numeric_contact(problem,c,contact_q)[0]<=2e-5 and support_patch_contains(problem,c,contact_q)
        if not active:continue
        expanded=np.c_[np.zeros((sys.nv,3)),J.T] if c.capability.kind=='P' else J.T;maps.append(-expanded[sys.object_v]);tmaps.append(np.array(sys.B).T@expanded);caps.append(c.capability);selected.append(c)
    load=problem.task_load(q[sys.object_q]);task=GeneralizedTaskLoad(required,load.E[:,[-1]],np.eye(1));check=fixed_generalized_margin(maps,task,caps,torque_gravity=robot_load,torque_maps=tmaps,effort=sys.effort,fixed_rho=0.)
    if not check['certificate']:raise RuntimeError('No feasible frozen dynamic initialization: '+check['status'])
    values={c.name:np.zeros(3 if c.capability.kind=='P' else 6) for c in contacts};tau=robot_load.copy()
    for c,w,Jmap in zip(selected,check['reactions'].mean(axis=0),tmaps):values[c.name]=w[3:] if c.capability.kind=='P' else w;tau-=Jmap@w
    return tau,values


def initial_interval(problem,k,left,right,steps,duration,contacts):
    sys=problem.system;obj=problem.task_definition.object;Q=[];V=[];h=duration/steps;samples=np.linspace(0,1,steps+1);beta=30*samples**2*(1-samples)**2;alpha=np.r_[0,np.cumsum(.5*h*(beta[:-1]+beta[1:]))];normalization=alpha[-1];alpha/=normalization;beta/=normalization;object_twist=difference(obj,left.q[sys.object_q],right.q[sys.object_q],1.)
    for j in range(steps+1):
        s=alpha[j];qo=constant_twist_object(obj,left.q[sys.object_q],right.q[sys.object_q],s);vo=beta[j]*object_twist;row=[];velocities=[]
        for i,(f,r,frame,c) in enumerate(zip(problem.task_definition.features,problem.robots,problem.frames,contacts)):
            seed=(1-s)*left.q[sys.q_slices[i]]+s*right.q[sys.q_slices[i]]
            if phase_mode(problem.task_definition.keyframes[k]['phase'],f)=='holding':
                T=obj.fk(qo,f.frame);goal=np.eye(4);goal[:3,3]=T[:3,3]+T[:3,:3]@left.positions[i]
                local=left.rotations[i]@np.diag([1.,-1.,-1.]) if c.capability.kind=='G' else Rotation.align_vectors(np.array([-c.normal]),np.array([f.tool_rotation[:,2]]))[0].as_matrix()@f.tool_rotation;goal[:3,:3]=T[:3,:3]@local;seed=point_pose_ik(r,frame,goal[:3,3],T[:3,:3]@c.normal,seed) if c.capability.kind=='P' else numerical_pose_ik(r,frame,goal,seed)
            nominal=beta[j]*(right.q[sys.q_slices[i]]-left.q[sys.q_slices[i]])
            if phase_mode(problem.task_definition.keyframes[k]['phase'],f)=='holding':
                _,Jo=numeric_object_point(obj,qo,f.frame,left.positions[i]);Jr=r.jacobian(seed,frame,'world_aligned')
                if c.capability.kind=='P':
                    Rc=obj.fk(qo,f.frame)[:3,:3]@c.basis();project=np.zeros((5,6));project[:2,:3]=Rc[:,:2].T;project[2:,3:]=np.eye(3);Jr=project@Jr;Jo=project@Jo
                velocity=nominal+np.linalg.lstsq(Jr,Jo@vo-Jr@nominal,rcond=1e-9)[0]
            else:velocity=nominal
            row.extend(seed);velocities.extend(velocity)
        Q.append(np.r_[row,qo]);V.append(np.r_[velocities,vo])
    Q=np.array(Q);V=np.array(V);Q[0]=left.q;Q[-1]=right.q;V[[0,-1]]=0;U=[];impulses={c.name:[] for c in contacts}
    for j in range(steps):
        tau,forces=nominal_distribution(problem,Q[j],V[j],(V[j+1]-V[j])/h,contacts,j,steps,next_configuration=Q[j+1]);U.append(tau)
        for c in contacts:impulses[c.name].append(h*forces[c.name])
    return TrajectoryResult(Q,V,np.array(U),np.full(steps,h),{n:np.array(v) for n,v in impulses.items()},0.,0.,0.,0.,0.,False,())


def solve_interval(problem,k,left,right,settings,*,steps=16,configuration_update='independent_midpoint',configuration_defect_bounds=(2e-5,5e-5),collision_margins=None,initial_guess=None,force_coordinates='cartesian'):
    from .full_static import stationary_interval,released_arm_interval
    sys=problem.system;duration=problem.task_definition.keyframes[k+1]['time']-problem.task_definition.keyframes[k]['time'];contacts=interval_contacts(problem,k,left,steps)
    if np.max(abs(left.q-right.q))<1e-8:
        return stationary_interval(problem,left,right,contacts,duration,steps)
    if left.phase in {'released','release_leg'} and np.max(abs(left.q[sys.object_q]-right.q[sys.object_q]))<1e-8:
        return released_arm_interval(problem,left,right,contacts,duration,steps,settings)
    initial=initial_interval(problem,k,left,right,steps,duration,contacts)
    obj=problem.task_definition.object;frames=list(obj.names) if isinstance(obj,RigidCollection) else list(obj.tree.links) if problem.task_definition.id=='box' else [obj.tree.root];desired={f:[] for f in frames}
    # Failed iterates are optional numerical seeds, never accepted plans. A
    # fresh successful NLP and all independent gates below remain mandatory.
    failure_path=problem.directory/f'interval{k}-unaccepted.npz';kinematic_report=None;loaded_seed=False
    if initial_guess is not None:initial=initial_guess
    elif failure_path.exists():
        with np.load(failure_path,allow_pickle=False) as a:
            if a['q'].shape==initial.q.shape and a['v'].shape==initial.v.shape and np.isfinite(a['q']).all() and np.isfinite(a['v']).all():
                initial.q=a['q'].copy();initial.v=a['v'].copy();initial.tau=a['tau'].copy();initial.dt=np.clip(a['dt'].copy(),.9*duration/steps,1.1*duration/steps);initial.q[[0,-1]]=np.array([left.q,right.q]);initial.v[[0,-1]]=0.;loaded_seed=True
                for c in contacts:
                    name='impulse_'+c.name
                    if name in a and a[name].shape==initial.impulses[c.name].shape:initial.impulses[c.name]=a[name].copy()
    if initial_guess is None and not loaded_seed:
        from dataclasses import replace
        import json
        kin_settings=replace(settings,max_iterations=min(settings.max_iterations,400),verbose=False,diagnostics_path=str(problem.directory/f'interval{k}-kinematic-failure.json'));initial,kinematic_report=refine_kinematic_initialization(problem,initial,contacts,kin_settings);(problem.directory/f'interval{k}-kinematic.json').write_text(json.dumps(kinematic_report,indent=2)+'\n')
        if kinematic_report['required_grasp_velocity_band']>8e-6:raise RuntimeError(f'Kinematic initialization at {steps} nodes requires {kinematic_report["required_grasp_velocity_band"]:.3g} grasp velocity; refine the common task grid')
    for j in range(steps+1):
        s=j/steps;qo=object_interpolate(obj,left.q[sys.object_q],right.q[sys.object_q],np.zeros(obj.nv),np.zeros(obj.nv),duration,s)
        for f in frames:desired[f].append(obj.fk(qo,f))
    initial.configuration_update=configuration_update;initial.configuration_defect_bounds=configuration_defect_bounds if configuration_update=='grasp_consistent_midpoint' else ();feasible_initial=validate_interval(problem,initial,contacts)['passed'];gap_schedule=(1e-8,) if feasible_initial else (1e-3,1e-5,1e-8);friction_schedule=gap_schedule;velocity_schedule=(0.,) if feasible_initial else (1e-3,1e-5,0.)
    result=solve_cito(sys,left.q,right.q,contacts,steps=steps,duration=duration,dt_bounds=(duration/steps*.9,duration/steps*1.1),fixed_dt=False,initial=initial,desired_object_poses=desired,characteristic_length=.6,Kp=np.eye(3)*500,kR=50.,configuration_update=configuration_update,configuration_defect_bounds=configuration_defect_bounds,contact_force_coordinates=force_coordinates,friction_speed_model='huber_norm',friction_speed_smoothing=1e-6,eliminate_endpoint_redundancy=True,exact_gripper_velocity=configuration_update=='grasp_consistent_midpoint',exact_sticking_velocity=True,kinematic_tolerance=1e-5,Q_tau=np.diag(.01/sys.effort**2),Q_v=np.diag(np.r_[np.concatenate([.1/r.velocity**2 for r in problem.robots]),np.full(sys.object_v.stop-sys.object_v.start,.1)]),duration_weight=.1,witness_regularization=1e-6,relaxations=gap_schedule,friction_relaxations=friction_schedule,velocity_relaxations=velocity_schedule,collision_margins=collision_margins,failure_recording_path=problem.directory/f'interval{k}-unaccepted.npz',settings=settings)
    check=validate_interval(problem,result,contacts)
    if not check['passed']:raise RuntimeError('Independent interval validation rejected: '+str(check))
    if kinematic_report is not None:result.continuation['kinematic_initialization']=kinematic_report
    result.continuation['direct_final_feasible_initialization']=feasible_initial
    return result,contacts,check
