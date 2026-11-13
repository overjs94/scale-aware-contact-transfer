"""Condensed exact dynamics for declared stationary and released-arm phases.

These phases constrain stationary object/holding-arm coordinates in the planner,
not in MuJoCo. Every returned interval must pass the full native equations.
"""
from copy import deepcopy
from types import SimpleNamespace
import numpy as np
import casadi as ca
from .generalized_conic import fixed_generalized_margin
from .generalized_load import GeneralizedTaskLoad
from .paper_keyframe import KeyframeCertificate
from .trajectory import Contact, TrajectoryResult
from .full_validation import numeric_contact, support_patch_contains, validate_interval


def enabled(c, step):
    if c.nodes is not None and step not in c.nodes:
        return False
    if c.robot_index is None:
        return True
    if c.capability.kind == 'G':
        return step in c.attached_steps and not (c.release_at_end and step == max(c.attached_steps))
    return step not in c.released_steps


def allocation(problem, q, contacts, task, *, step=None, rho=None, rho_min=0., weighted=False):
    sys=problem.system; selected=[]; maps=[]; tmaps=[]
    for c in contacts:
        if step is not None and not enabled(c,step):continue
        gap,J,_,_=numeric_contact(problem,c,q)
        if c.robot_index is None and (abs(gap)>2e-5 or not support_patch_contains(problem,c,q)):continue
        X=np.c_[np.zeros((sys.nv,3)),J.T] if c.capability.kind=='P' else J.T
        selected.append(c);maps.append(-X[sys.object_v]);tmaps.append(np.asarray(sys.B).T@X)
    gravity=np.concatenate([r.gravity(q[s]) for r,s in zip(problem.robots,sys.q_slices)])
    check=fixed_generalized_margin(maps,task,[c.capability for c in selected],torque_gravity=gravity,torque_maps=tmaps,effort=sys.effort,fixed_rho=rho,rho_min=rho_min,torque_weight=np.diag(.01/sys.effort**2) if weighted else None)
    if not check['certificate'] and rho is None and np.isfinite(check['rho']) and check['rho']>=rho_min:
        maximum_status=check['status']; candidate=check['rho']
        check=fixed_generalized_margin(maps,task,[c.capability for c in selected],torque_gravity=gravity,torque_maps=tmaps,effort=sys.effort,fixed_rho=candidate,rho_min=rho_min)
        check['maximum_margin_status']=maximum_status
    if not check['certificate']:raise RuntimeError('Condensed allocation rejected: '+check['status'])
    forces={c.name:np.zeros((len(task.vertices),6)) for c in contacts};torques=np.tile(gravity,(len(task.vertices),1))
    for i,(c,T) in enumerate(zip(selected,tmaps)):
        forces[c.name]=check['reactions'][:,i];torques-=check['reactions'][:,i]@T.T
    witness=sum(np.mean([np.sum(row[i]**2) for row in check['coordinates']]) for i,c in enumerate(selected) if c.capability.kind=='G')
    return forces,torques,check,float(witness)


def fixed_posture_keyframe(problem,k,previous,environment,retreat):
    from .full_scenario import release_ik
    from .full_trajectory import phase_mode
    sys=problem.system;goal=problem.task_definition.keyframes[k];q=previous.q.copy();qo=goal['object_q']
    if np.max(abs(q[sys.object_q]-qo))>1e-8:raise ValueError('Stationary keyframe changes object pose')
    contacts=[];interfaces=[];prior=problem.source(goal)
    for i,(f,r,frame,cap,qs,p,R) in enumerate(zip(problem.task_definition.features,problem.robots,problem.frames,problem.capabilities,sys.q_slices,previous.positions,previous.rotations)):
        mode=phase_mode(goal['phase'],f)
        if mode=='released':
            if retreat:
                T=problem.task_definition.object.fk(qo,f.frame);target=r.fk(previous.q[qs],frame).copy();target[:3,3]=T[:3,3]+T[:3,:3]@(p+R[:,2]*retreat);q[qs]=release_ik(r,frame,target,previous.q[qs])
            continue
        interfaces.append(SimpleNamespace(group_id=f.id,force_enabled=mode=='holding'))
        if mode=='holding':contacts.append(Contact(f.id,p,R[:,2],cap,robot_index=i,frame=frame,object_frame=f.frame,rotation_object=R))
    forces,tau,check,_=allocation(problem,q,(*contacts,*environment),problem.task_load(qo),rho_min=.1)
    reaction=np.stack([forces.get(f.id,np.zeros((len(tau),6))) for f in problem.task_definition.features],axis=1)
    objective=-check['rho']
    for i,(f,r,spec,qs) in enumerate(zip(problem.task_definition.features,problem.robots,problem.specs,sys.q_slices)):
        if phase_mode(goal['phase'],f)=='released':continue
        reference=problem.branch_reference(k)[qs] if problem.task_definition.posture_reference_rule=='sequential_ik_branch' else previous.q[qs] if problem.task_definition.posture_reference_rule=='previous_keyframe' else spec['q_initial'];objective+=np.sum((previous.positions[i]-prior['positions'][i])**2)/.6**2+1-previous.rotations[i,:,2]@prior['normals'][i]+.01*np.sum(((q[qs]-reference)/np.maximum(r.upper-r.lower,1e-3))**2)
    result=KeyframeCertificate(q,previous.positions.copy(),previous.rotations.copy(),previous.apertures,check['rho'],reaction,tau,{c.name:forces[c.name][:,3:] for c in environment},float(objective),check['primal_residual'])
    result.source=prior['record'];result.phase=goal['phase'];result.retreat_m=retreat;result.interfaces=interfaces;result.fixed_posture_target=True;result.target_provenance=dict(method='declared_fixed_posture_robust_SOCP',posture_reference_rule=problem.task_definition.posture_reference_rule);result.posture_reference=problem.branch_reference(k)[:sys.object_q.start].copy() if problem.task_definition.posture_reference_rule=='sequential_ik_branch' else previous.q[:sys.object_q.start].copy() if problem.task_definition.posture_reference_rule=='previous_keyframe' else np.concatenate([s['q_initial'] for s in problem.specs])
    return result


def fixed_geometry_keyframe(problem,seed,interfaces,prior,environment,settings,*,q_reference=None,robot_configuration_bounds=None):
    """Exact separable realization when held material points are fixed.

    The unconstrained-torque SOCP bounds rho independently of arm posture.
    If posture-minimizing IK admits that same rho with all torque limits,
    both independent objective bounds are attained simultaneously. Otherwise
    return None and retain the coupled realization NLP.
    """
    from .optimization import equality,solve
    from .paper_keyframe import relative_interface
    from .lie import rotation_error,rotation_cost
    sys=problem.system;opti=ca.Opti();qr=opti.variable(sys.object_q.start);Q=ca.vertcat(qr,ca.DM(seed[sys.object_q]));opti.set_initial(qr,seed[:sys.object_q.start]);posture=0.;positions=[];rotations=[];geo=0.;contacts=[]
    for r,spec,qs in zip(problem.robots,problem.specs,sys.q_slices):
        reference=spec['q_initial'] if q_reference is None else q_reference[qs];lower=r.lower if robot_configuration_bounds is None else np.maximum(r.lower,robot_configuration_bounds[0][qs]);upper=r.upper if robot_configuration_bounds is None else np.minimum(r.upper,robot_configuration_bounds[1][qs]);opti.subject_to(opti.bounded(lower,qr[qs],upper));posture+=.01*ca.sumsqr((qr[qs]-reference)/np.maximum(r.upper-r.lower,1e-3))
    for interface in interfaces:
        if len(interface.patch.initial):return None
        p=np.asarray(interface.patch.position(interface.patch.initial)).ravel();R=np.asarray(interface.patch.rotation(interface.patch.initial));delta=p-interface.prior_position;normal=1-R[:,2]@interface.prior_rotation[:,2]
        if np.linalg.norm(delta)>.015+1e-9 or normal>.1+1e-9:raise ValueError('Fixed material geometry leaves target trust region')
        positions.append(p);rotations.append(R);geo+=delta@delta/.6**2+normal;T,pw,Rw,_=relative_interface(sys,Q,interface,ca.DM(p),ca.DM(R));equality(opti,T[:3,3]-pw)
        if interface.capability.kind=='P':equality(opti,Rw[:,:2].T@T[:3,2]);opti.subject_to(ca.dot(T[:3,2],Rw[:,2])<=-.5)
        else:equality(opti,rotation_error(T[:3,:3],Rw@ca.diag(ca.DM([1.,-1.,-1.]))));opti.subject_to(rotation_cost(T[:3,:3],Rw@ca.diag(ca.DM([1.,-1.,-1.])))<=1.)
        if interface.force_enabled:contacts.append(Contact(interface.group_id,p,R[:,2],interface.capability,robot_index=interface.robot_index,frame=interface.robot_frame,object_frame=interface.object_frame,rotation_object=R))
    opti.minimize(posture);sol,res=solve(opti,settings);q=np.r_[np.asarray(sol.value(qr)).ravel(),seed[sys.object_q]];maps=[];caps=[]
    for c in [*contacts,*environment]:
        gap,J,_,_=numeric_contact(problem,c,q)
        if c.robot_index is None and (abs(gap)>2e-5 or not support_patch_contains(problem,c,q)):continue
        X=np.c_[np.zeros((sys.nv,3)),J.T] if c.capability.kind=='P' else J.T;maps.append(-X[sys.object_v]);caps.append(c.capability)
    upper=fixed_generalized_margin(maps,problem.task_load(q[sys.object_q]),caps,rho_min=.1)
    if not upper['certificate']:return None
    try:forces,tau,check,_=allocation(problem,q,(*contacts,*environment),problem.task_load(q[sys.object_q]),rho=upper['rho'],rho_min=.1)
    except RuntimeError:return None
    count=len(tau);reaction=np.stack([forces.get(f.id,np.zeros((count,6))) for f in problem.task_definition.features],axis=1)
    result=KeyframeCertificate(q,np.array(positions),np.array(rotations),tuple(getattr(c,'aperture_required',None) for c in problem.capabilities),upper['rho'],reaction,tau,{c.name:forces[c.name][:,3:] for c in environment},float(sol.value(posture))+geo-upper['rho'],max(res,check['primal_residual']))
    result.separable_target_certificate=dict(posture_solver=opti.stats().get('return_status'),torque_free_rho_upper_bound=upper['rho'],same_rho_torque_feasibility=check['certificate'],posture_global_optimality_claimed=False);result.target_provenance=dict(method='fixed_geometry_separable_target',**result.separable_target_certificate)
    return result


def stationary_interval(problem,left,right,contacts,duration,steps):
    sys=problem.system;h=duration/steps;q=left.q;load=problem.task_load(q[sys.object_q]);task=GeneralizedTaskLoad(load.nominal,load.E[:,[-1]],np.eye(1));tau=[];impulses={c.name:[] for c in contacts};cost=.1*duration;Qt=np.diag(.01/sys.effort**2);statuses=[];cache={}
    for k in range(steps):
        signature=tuple(c.name for c in contacts if enabled(c,k))
        if signature not in cache:cache[signature]=allocation(problem,q,contacts,task,step=k,rho=0.,weighted=True)
        forces,u,check,witness=cache[signature];u=u.mean(axis=0);tau.append(u);cost+=h*(u@Qt@u+1e-6*witness);statuses.append(check['status'])
        for c in contacts:
            w=forces[c.name].mean(axis=0);impulses[c.name].append(h*(w[3:] if c.capability.kind=='P' else w))
    r=TrajectoryResult(np.tile(q,(steps+1,1)),np.zeros((steps+1,sys.nv)),np.array(tau),np.full(steps,h),{n:np.array(a) for n,a in impulses.items()},float(cost),0.,0.,0.,0.,False,())
    r.q[-1]=right.q;r.objective_mode='stationary_contact_conic_QP';r.configuration_update='independent_midpoint';r.continuation=dict(stationary_condensation=True,solver=statuses,object_coordinates_constrained_in_simulator=False)
    check=validate_interval(problem,r,contacts)
    if not check['passed']:raise RuntimeError('Stationary condensed native equations rejected: '+str(check))
    return r,contacts,check


def released_arm_interval(problem,left,right,contacts,duration,steps,settings):
    from .optimization import equality,solve
    sys=problem.system;moving=[i for i,s in enumerate(sys.q_slices) if np.max(abs(left.q[s]-right.q[s]))>1e-8]
    if not moving: return stationary_interval(problem,left,right,contacts,duration,steps)
    if any(any(enabled(c,k) for k in range(steps)) for c in contacts if c.robot_index in moving):raise ValueError('Condensed retreat requires released moving arms')
    if np.max(abs(left.q[sys.object_q]-right.q[sys.object_q]))>1e-8:raise ValueError('Condensed retreat requires stationary object')
    load=problem.task_load(left.q[sys.object_q]);task=GeneralizedTaskLoad(load.nominal,load.E[:,[-1]],np.eye(1));forces,hold,_,witness=allocation(problem,left.q,contacts,task,step=0,rho=0.,weighted=True);hold=hold.mean(axis=0)
    opti=ca.Opti();h=opti.variable();h0=duration/steps;opti.subject_to(opti.bounded(.9*h0,h,1.1*h0));opti.set_initial(h,h0);variables={};Qt=.01/sys.effort**2;objective=steps*h*(.1+1e-6*witness);uoffset=np.r_[0,np.cumsum([r.n for r in problem.robots])]
    for i,r in enumerate(problem.robots):
        us=slice(uoffset[i],uoffset[i+1]);qs=sys.q_slices[i]
        if i not in moving:objective+=steps*h*np.sum(Qt[us]*hold[us]**2);continue
        Q=opti.variable(r.n,steps+1);V=opti.variable(r.n,steps+1);U=opti.variable(r.n,steps);variables[i]=(Q,V,U);a=left.q[qs];b=right.q[qs];s=np.linspace(0,1,steps+1);beta=30*s*s*(1-s)**2/duration;alpha=np.r_[0,np.cumsum(h0*.5*(beta[:-1]+beta[1:]))];beta/=alpha[-1];alpha/=alpha[-1]
        opti.set_initial(Q,a[:,None]+(b-a)[:,None]*alpha);opti.set_initial(V,(b-a)[:,None]*beta);opti.set_initial(U,np.tile(hold[us,None],(1,steps)));equality(opti,Q[:,0]-a);equality(opti,Q[:,-1]-b);equality(opti,V[:,0]);equality(opti,V[:,-1]);sr=sys.robots[i]
        for j in range(steps+1):
            if j not in (0,steps):opti.subject_to(opti.bounded(r.lower,Q[:,j],r.upper))
            opti.subject_to(opti.bounded(-r.velocity,V[:,j],r.velocity))
        for k in range(steps):
            equality(opti,Q[:,k+1]-Q[:,k]-.5*h*(V[:,k]+V[:,k+1]));equality(opti,sr.mass(Q[:,k])@(V[:,k+1]-V[:,k])-h*(U[:,k]-sr.bias(Q[:,k],V[:,k])));opti.subject_to(opti.bounded(-r.effort,U[:,k],r.effort));objective+=h*(ca.dot(ca.DM(Qt[us]),U[:,k]**2)+ca.dot(ca.DM(.1/r.velocity**2),V[:,k+1]**2))
            if k<steps-1:
                full=ca.vertcat(*[Q[:,k+1] if l==i else ca.DM(left.q[sl]) for l,sl in enumerate(sys.q_slices)],ca.DM(left.q[sys.object_q]))
                for c in contacts:
                    if c.robot_index==i:
                        gap,_,_=c.geometry(sys,full);opti.subject_to(gap>=0)
    opti.minimize(objective);sol,res=solve(opti,settings);dt=float(sol.value(h));Qfull=np.tile(left.q,(steps+1,1));Vfull=np.zeros((steps+1,sys.nv));Ufull=np.tile(hold,(steps,1))
    for i,(Q,V,U) in variables.items():Qfull[:,sys.q_slices[i]]=np.asarray(sol.value(Q)).T;Vfull[:,sys.v_slices[i]]=np.asarray(sol.value(V)).T;Ufull[:,uoffset[i]:uoffset[i+1]]=np.asarray(sol.value(U)).T
    impulses={c.name:np.tile(dt*(forces[c.name].mean(axis=0)[3:] if c.capability.kind=='P' else forces[c.name].mean(axis=0)),(steps,1)) for c in contacts}
    r=TrajectoryResult(Qfull,Vfull,Ufull,np.full(steps,dt),impulses,float(sol.value(objective)),res,0.,0.,0.,False,());r.objective_mode='stationary_objects_release_NLP';r.configuration_update='independent_midpoint';r.continuation=dict(stationary_object_condensation=True,solver=opti.stats().get('return_status'),object_coordinates_constrained_in_simulator=False)
    check=validate_interval(problem,r,contacts)
    if not check['passed']:raise RuntimeError('Released-arm condensed native equations rejected: '+str(check))
    return r,contacts,check
