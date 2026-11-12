"""Contact-frame hybrid force/motion feedback, joint impedance and position admittance.

These are execution controllers, separate from manuscript Eq. (7), (14), (19).
All measured/desired reactions are OBJECT-ON-ROBOT about the tool origin,
[moment_tool; force_world]. Normals/force axes are explicit task data.
"""
from dataclasses import dataclass
import numpy as np
from scipy.spatial.transform import Rotation
from .control import position_servo_step


def vector(value,n):
 a=np.broadcast_to(np.asarray(value,float),(n,)).copy()
 if not np.isfinite(a).all():raise ValueError('Finite gains/limits required')
 return a

@dataclass
class HybridSelection:
 force: np.ndarray
 def __post_init__(self):
  F=np.asarray(self.force,float)
  if F.shape==(6,):F=np.diag(F)
  if F.shape!=(6,6) or not np.isfinite(F).all() or not np.allclose(F,F.T) or not np.allclose(F@F,F):raise ValueError('Force selection must be an orthogonal projector')
  self.force=F;self.motion=np.eye(6)-F
 @classmethod
 def normal(cls):return cls(np.array([0,0,0,0,0,1.]))


def contact_coordinates(tool_rotation,contact_rotation):
 Rt,Rc=np.asarray(tool_rotation,float),np.asarray(contact_rotation,float)
 for R in [Rt,Rc]:
  if R.shape!=(3,3) or not np.allclose(R.T@R,np.eye(3)) or not np.isclose(np.linalg.det(R),1):raise ValueError('Proper rotations required')
 H=np.zeros((6,6));H[:3,:3]=Rc.T@Rt;H[3:,3:]=Rc.T;return H


def torque_nullspace(M,J,damping=1e-8):
 """Dynamically consistent N: J M^-1 N.T=0 for nonsingular task inertia."""
 if damping<=0:raise ValueError('Positive regularization required')
 M=np.asarray(M,float);J=np.asarray(J,float)
 X=np.linalg.solve(M,J.T);Jbar=X@np.linalg.solve(J@X+damping*np.eye(len(J)),np.eye(len(J)))
 return np.eye(M.shape[0])-Jbar@J


class HybridTorqueImpedance:
 """Torque arm: model/feedforward + selected Cartesian feedback + nullspace impedance.

 Force feedback uses f_desired-f_measured=measured_reaction-desired_reaction.
 Joint posture impedance is projected out of the entire contact task so it
 does not fight the force-controlled normal. Conditional integration prevents
 force-integrator windup under actuator clipping. No passivity guarantee is
 claimed for arbitrary moving frames, saturation or estimated force signals.
 """
 def __init__(self,robot,frame,selection=None,*,motion_k=(40,40,40,3000,3000,3000),motion_d=(2,2,2,40,40,40),force_k=.25,force_i=1.,joint_k=40.,joint_d=8.,feedback_limit=(2,2,2,20,20,20),integral_limit=(.2,.2,.2,5,5,5)):
  if robot.floating_base:raise ValueError('Floating-base torque control needs a constrained whole-body controller')
  self.robot=robot;self.frame=frame;self.selection=selection or HybridSelection.normal();self.motion_k=vector(motion_k,6);self.motion_d=vector(motion_d,6);self.force_k=vector(force_k,6);self.force_i=vector(force_i,6);self.joint_k=vector(joint_k,robot.nv);self.joint_d=vector(joint_d,robot.nv);self.feedback_limit=vector(feedback_limit,6);self.integral_limit=vector(integral_limit,6)
  if any(np.any(v<0) for v in [self.motion_k,self.motion_d,self.force_k,self.force_i,self.joint_k,self.joint_d,self.feedback_limit,self.integral_limit]):raise ValueError('Controller gains/limits must be nonnegative')
  self.reset()
 def reset(self):self.integral=np.zeros(6);self.last={}
 def command(self,q,v,qd,vd,ad,*,reaction_desired,reaction_measured,contact_rotation_world,dt):
  if dt<=0 or not np.isfinite(dt):raise ValueError('Positive finite timestep required')
  r=self.robot;q,v,qd,vd,ad=[np.asarray(a,float).reshape(r.nv) for a in [q,v,qd,vd,ad]];ld,lm=[np.asarray(a,float).reshape(6) for a in [reaction_desired,reaction_measured]]
  if not all(np.isfinite(a).all() for a in [q,v,qd,vd,ad,ld,lm]):raise ValueError('Finite control inputs required')
  T,Td=r.fk(q,self.frame),r.fk(qd,self.frame);J,Jd=r.jacobian(q,self.frame),r.jacobian(qd,self.frame);H=contact_coordinates(T[:3,:3],contact_rotation_world);Hd=contact_coordinates(Td[:3,:3],contact_rotation_world);Jc=H@J
  error=np.r_[H[:3,:3]@Rotation.from_matrix(T[:3,:3].T@Td[:3,:3]).as_rotvec(),np.asarray(contact_rotation_world).T@(Td[:3,3]-T[:3,3])];velocity=Hd@Jd@vd-Jc@v;ef=self.selection.force@(H@(lm-ld));previous=self.integral.copy();self.integral=np.clip(self.integral+dt*ef,-self.integral_limit,self.integral_limit)
  force_feedback=self.selection.force@np.clip(self.force_k*ef+self.force_i*self.integral,-self.feedback_limit,self.feedback_limit);motion_feedback=self.selection.motion@(self.motion_k*error+self.motion_d*velocity);N=torque_nullspace(r.mass(q),Jc);posture=N.T@(self.joint_k*(qd-q)+self.joint_d*(vd-v));raw=r.inverse_dynamics(q,v,ad)-J.T@ld+Jc.T@(motion_feedback+force_feedback)+posture;command=np.clip(raw,-r.effort,r.effort);saturated=bool(np.any(abs(command-raw)>1e-9))
  if saturated:self.integral=previous
  self.last=dict(force_error_contact=ef.copy(),motion_error_contact=self.selection.motion@error,motion_feedback_contact=motion_feedback,force_feedback_contact=force_feedback,joint_impedance_torque=posture,unclipped_torque=raw,saturated=saturated,task_leakage=float(np.linalg.norm(Jc@np.linalg.solve(r.mass(q),posture))))
  return command


class HybridPositionAdmittance:
 """Position arm: selected force error -> contact-frame velocity -> bounded IK.

 The manufacturer's inner position servo is not replaced or inferred here.
 Tangential/orientation motion uses a pose servo; selected force axes use
 virtual inertia/damping and measured force feedback, without a competing
 pose stiffness. State is stored in WORLD coordinates and re-expressed at
 every moving contact frame. Tool rotations/twists retain mixed conventions.
 """
 def __init__(self,robot,frame,selection=None,*,mass=(.2,.2,.2,2,2,2),damping=(4,4,4,40,40,40),motion_gain=(4,4,4,6,6,6),velocity_limit=(.3,.3,.3,.1,.1,.1)):
  if robot.floating_base:raise ValueError('Position admittance requires an arm adapter')
  self.robot=robot;self.frame=frame;self.selection=selection or HybridSelection.normal();self.mass=vector(mass,6);self.damping=vector(damping,6);self.motion_gain=vector(motion_gain,6);self.velocity_limit=vector(velocity_limit,6)
  if np.any(self.mass<=0) or np.any(self.damping<0) or np.any(self.motion_gain<0) or np.any(self.velocity_limit<=0):raise ValueError('Invalid admittance parameters')
  self.reset()
 def reset(self):self.velocity_world=np.zeros(6);self.command_configuration=None;self.last={}
 def command(self,q,qd,vd,*,reaction_desired,reaction_measured,contact_rotation_world,dt):
  if dt<=0 or not np.isfinite(dt):raise ValueError('Positive finite timestep required')
  from .pmi import passive_midpoint_step
  r=self.robot;q,qd,vd=[np.asarray(a,float).reshape(r.nv) for a in [q,qd,vd]];ld,lm=[np.asarray(a,float).reshape(6) for a in [reaction_desired,reaction_measured]];Rc=np.asarray(contact_rotation_world,float);T,Td=r.fk(q,self.frame),r.fk(qd,self.frame);H=contact_coordinates(T[:3,:3],Rc);Hd=contact_coordinates(Td[:3,:3],Rc);error=np.r_[H[:3,:3]@Rotation.from_matrix(T[:3,:3].T@Td[:3,:3]).as_rotvec(),Rc.T@(Td[:3,3]-T[:3,3])];ef=self.selection.force@(H@(lm-ld));R6=np.zeros((6,6));R6[:3,:3]=R6[3:,3:]=Rc.T;old=self.selection.force@(R6@self.velocity_world);_,next_velocity,balance=passive_midpoint_step(np.zeros(6),old,ef,dt,mass=self.mass,damping=self.damping);normal=self.selection.force@balance['midpoint_velocity'];desired=self.selection.motion@(Hd@r.jacobian(qd,self.frame)@vd+self.motion_gain*error)+normal;desired=np.clip(desired,-self.velocity_limit,self.velocity_limit);self.velocity_world=R6.T@(self.selection.force@next_velocity);
  if self.command_configuration is None:self.command_configuration=q.copy()
  Tcmd=r.fk(self.command_configuration,self.frame);Hcmd=contact_coordinates(Tcmd[:3,:3],Rc);qcmd,vactual=position_servo_step(r,self.command_configuration,self.frame,Hcmd.T@desired,dt);self.command_configuration=qcmd.copy()
  self.last=dict(force_error_contact=ef.copy(),admittance_velocity_contact=normal,requested_twist_contact=desired,achieved_twist_contact=H@r.jacobian(q,self.frame)@vactual,virtual_PMI_energy_balance=balance['balance_residual'])
  return qcmd
