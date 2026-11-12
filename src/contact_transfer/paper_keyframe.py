"""Eq. (14), including shared robot bodies and explicit free-object extensions.

The rigid/fixed-base case reduces to the manuscript equations. An articulated
object or a floating mobile robot adds generalized coordinates and support
constraints; those extensions are recorded by the scenario adapter.
"""
from dataclasses import dataclass
import casadi as ca
import numpy as np
from .lie import rotation_cost,rotation_error
from .optimization import equality,solve

@dataclass
class Interface:
 group_id: str
 robot_index: int
 robot_frame: str
 object_frame: str | None
 patch: object
 capability: object
 prior_position: np.ndarray
 prior_rotation: np.ndarray

@dataclass
class KeyframeCertificate:
 q: np.ndarray
 positions: np.ndarray
 rotations: np.ndarray
 apertures: tuple
 rho: float
 reactions: np.ndarray
 torques: np.ndarray
 environment_reactions: dict
 objective: float
 primal_residual: float


def relative_interface(system,q,spec,point,rotation):
 i=spec.robot_index;r=system.robots[i];T=r.fk(q[system.q_slices[i]],spec.robot_frame);To=system.object_pose(q,spec.object_frame);p=To[:3,3]+To[:3,:3]@point;Rw=To[:3,:3]@rotation;Jr=r.jacobian(q[system.q_slices[i]],spec.robot_frame,'world_aligned');Jp=-system.point_jacobian_object(q,point,spec.object_frame);Jw=-system.angular_jacobian_object(q,spec.object_frame);Jp[:,system.v_slices[i]]+=Jr[3:,:];Jw[:,system.v_slices[i]]+=Jr[:3,:];return T,p,Rw,ca.vertcat(Rw.T@Jw,Rw.T@Jp)


def realize_system_keyframe(system,q_initial,object_configuration,interfaces,task,*,environment=(),q_reference=None,rho_min=0.,position_radius=.02,normal_radius=.1,characteristic_length=1.,normal_weight=1.,posture_weight=.01,robustness_weight=1.,geometry_constraints=None,base_constraints=None,settings=None):
 if not (interfaces or environment) or characteristic_length<=0:raise ValueError('Object/robot or environmental contacts and positive object length required')
 q_initial=np.asarray(q_initial,float).reshape(system.nq);q_reference=q_initial.copy() if q_reference is None else np.asarray(q_reference,float).reshape(system.nq);qo=np.asarray(object_configuration,float).reshape(system.object_q.stop-system.object_q.start);opti=ca.Opti();qr=opti.variable(system.object_q.start);Q=ca.vertcat(qr,ca.DM(qo));opti.set_initial(qr,q_initial[:system.object_q.start]);objective=0.
 for r,qs in zip(system.robots,system.q_slices):
  off=7 if r.floating_base else 0;qb=qr[qs.start+off:qs.stop];opti.subject_to(opti.bounded(r.lower,qb,r.upper));span=np.where(np.isfinite(r.upper-r.lower),r.upper-r.lower,2*np.pi);objective+=posture_weight*ca.sumsqr((qb-q_reference[qs.start+off:qs.stop])/np.maximum(span,1e-3))
  if r.floating_base:
   equality(opti,ca.sumsqr(qr[qs.start+3:qs.start+7])-1)
   if base_constraints is None:raise ValueError('Floating robot keyframes require explicit support/workspace constraints')
 if base_constraints is not None:base_constraints(opti,Q)
 warm=None
 if not environment and not any(r.floating_base for r in system.robots):
  from .generalized_conic import fixed_generalized_margin
  maps=[];torque_maps=[]
  for spec in interfaces:
   _,_,_,J0=relative_interface(system,ca.DM(q_initial),spec,spec.patch.position(spec.patch.initial),spec.patch.rotation(spec.patch.initial));J0=np.asarray(ca.DM(J0));maps.append(-J0.T[system.object_v,:]);torque_maps.append(np.asarray(system.B).T@J0.T)
  b0=np.asarray(ca.DM(system.bias(ca.DM(q_initial),ca.DM.zeros(system.nv)))).ravel()
  trial=fixed_generalized_margin(maps,task,[spec.capability for spec in interfaces],torque_gravity=np.asarray(system.B).T@b0,torque_maps=torque_maps,effort=system.effort,rho_min=rho_min)
  if trial['certificate']:warm=trial
 geom=[];apertures=[]
 for spec in interfaces:
  _,p,R=spec.patch.instantiate(opti);T,pw,Rw,J=relative_interface(system,Q,spec,p,R);equality(opti,T[:3,3]-pw)
  if spec.capability.kind=='P':
   equality(opti,Rw[:,:2].T@T[:3,2]);opti.subject_to(ca.dot(T[:3,2],Rw[:,2])<=-.5)
  elif spec.capability.has_moment:
   desired=Rw@ca.diag(ca.DM([1,-1,-1]));equality(opti,rotation_error(T[:3,:3],desired));opti.subject_to(rotation_cost(T[:3,:3],desired)<=1.)
  p0,R0=ca.DM(spec.prior_position),ca.DM(spec.prior_rotation)
  if len(spec.patch.initial):
   opti.subject_to(ca.sumsqr(p-p0)<=position_radius**2);opti.subject_to(1-ca.dot(R[:,2],R0[:,2])<=normal_radius)
  else:
   if float(ca.norm_2(p-p0))>position_radius+1e-9 or float(1-ca.dot(R[:,2],R0[:,2]))>normal_radius+1e-9:raise ValueError('Fixed grasp feature leaves transferred-prior trust region')
  objective+=ca.sumsqr(p-p0)/characteristic_length**2+normal_weight*(1-ca.dot(R[:,2],R0[:,2]));geom.append((p,R,J))
  apertures.append(spec.capability.aperture(opti) if hasattr(spec.capability,'aperture_required') else None)
 if geometry_constraints is not None:geometry_constraints(opti,Q,apertures)
 for contact in environment:
  gap,_,_=contact.geometry(system,Q)
  if not ca.symvar(gap):
   if float(ca.evalf(gap))< -1e-10:raise ValueError('Static object penetrates its environment plane')
  else:opti.subject_to(gap>=0)
 rho=opti.variable();opti.subject_to(rho>=rho_min);opti.set_initial(rho,rho_min if warm is None else warm['rho']);objective-=robustness_weight*rho;reactions=[];torques=[];environment_reactions={c.name:[] for c in environment}
 nominal=system.bias(Q,ca.DM.zeros(system.nv));nominal0=np.asarray(system.bias(ca.DM(np.r_[q_initial[:system.object_q.start],qo]),ca.DM.zeros(system.nv))).ravel()[system.object_v]
 if not np.allclose(np.asarray(task.nominal),-nominal0,atol=1e-8):raise ValueError('Task nominal must match gravity/known-load model; do not count an environment load twice')
 for vertex_index,vertex in enumerate(task.vertices):
  tau=opti.variable(system.nu);opti.subject_to(opti.bounded(-system.effort,tau,system.effort));opti.set_initial(tau,0);total=system.B@tau;lam=[]
  for contact_index,(spec,(p,R,J),a) in enumerate(zip(interfaces,geom,apertures)):
   seed=None if warm is None else warm['coordinates'][vertex_index][contact_index] if hasattr(spec.capability,'aperture_required') else warm['reactions'][vertex_index,contact_index]
   w=spec.capability.reaction(opti,aperture=a,initial=seed);lam.append(w);total+=J.T@w
  if warm is not None:
   tau0=np.asarray(system.B).T@b0-sum(T@w for T,w in zip(torque_maps,warm['reactions'][vertex_index]));opti.set_initial(tau,tau0)
  for contact in environment:
   gap,J,_=contact.geometry(system,Q);f=contact.capability.reaction(opti)[3:];total+=J.T@f;opti.subject_to(f[2]*gap<=1e-8);environment_reactions[contact.name].append(f)
  perturb=ca.MX.zeros(system.nv);perturb[system.object_v]=rho*ca.DM(task.E)@ca.DM(task.D)@ca.DM(vertex);equality(opti,nominal-perturb-total);reactions.append(lam);torques.append(tau)
 opti.minimize(objective);sol,res=solve(opti,settings)
 return KeyframeCertificate(np.r_[np.asarray(sol.value(qr)).ravel(),qo],np.array([np.asarray(sol.value(p)).ravel() for p,R,J in geom]),np.array([np.asarray(sol.value(R)) for p,R,J in geom]),tuple(None if a is None else float(sol.value(a)) for a in apertures),float(sol.value(rho)),np.array([[np.asarray(sol.value(w)).ravel() for w in row] for row in reactions]),np.array([np.asarray(sol.value(t)).ravel() for t in torques]),{name:np.array([np.asarray(sol.value(w)).ravel() for w in ws]) for name,ws in environment_reactions.items()},float(sol.value(objective)),res)
