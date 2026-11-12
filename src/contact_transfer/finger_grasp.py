"""Manuscript Eq. (11): finger friction resultants and coupled closing-force bounds."""
from dataclasses import dataclass,field
import casadi as ca
import numpy as np
from .contacts import contact_basis
from .lie import skew

@dataclass(frozen=True)
class FingerGrasp:
 """Two opposing rectangular finger patches, four force sites on each patch.

 Points are about the grasp/TCP origin; interface_from_grasp shifts and rotates
 their resultants to the named interface origin. Aperture is the physical gap
 between the opposing patches, not a raw vendor finger joint coordinate.
 Parameters describe an explicit simulation interface, not hardware calibration.
 """
 aperture_required: float
 half_width: float
 half_height: float
 mu: float
 closing_force_per_finger: float
 aperture_min: float=0.
 aperture_max: float=.08
 interface_from_grasp: object=field(default_factory=lambda:np.eye(4))
 force_component_limits: object=None
 finger_linear_limits: object=None
 preferred_force_coordinates: str='cartesian'
 kind: str=field(default='G',init=False)
 has_moment: bool=field(default=True,init=False)
 def __post_init__(self):
  T=np.asarray(self.interface_from_grasp,float);values=[self.aperture_required,self.half_width,self.half_height,self.mu,self.closing_force_per_finger,self.aperture_min,self.aperture_max]
  if not np.isfinite(values).all() or self.half_width<=0 or self.half_height<=0 or self.mu<=0 or self.closing_force_per_finger<=0 or not 0<=self.aperture_min<=self.aperture_required<=self.aperture_max:raise ValueError('Invalid finger geometry, friction, aperture or force bound')
  if T.shape!=(4,4) or not np.allclose(T[3],[0,0,0,1]) or not np.allclose(T[:3,:3].T@T[:3,:3],np.eye(3)) or not np.isclose(np.linalg.det(T[:3,:3]),1):raise ValueError('Proper grasp/interface transform required')
  if self.force_component_limits is not None and (np.asarray(self.force_component_limits).shape!=(3,) or np.any(np.asarray(self.force_component_limits)<=0) or not np.isfinite(self.force_component_limits).all()):raise ValueError('Positive grasp-frame force component limits required')
  if self.finger_linear_limits is not None:
   A,b=map(np.asarray,self.finger_linear_limits)
   if A.ndim!=2 or A.shape[1]!=24 or b.shape!=(len(A),) or not np.isfinite(A).all() or not np.isfinite(b).all():raise ValueError('Finite linear constraints on the 24 finger reactions required')
  if self.preferred_force_coordinates not in {'cartesian','scaled_tangent'}:raise ValueError('Unknown preferred finger-force coordinates')
 def balanced_preload(self,fingers,scale=1.,fraction=.1):
  u=np.asarray(fingers,float).reshape(8,3).copy();spare=scale*self.closing_force_per_finger-max(u[:4,2].sum(),u[4:,2].sum());amount=fraction*max(0.,spare);direction=np.zeros(24);direction[2::3]=.25
  if self.finger_linear_limits is not None:
   A,b=map(np.asarray,self.finger_linear_limits);gain=A@direction;valid=gain>1e-12
   if valid.any():amount=min(amount,fraction*max(0.,float(np.min((scale*b-A@u.ravel())[valid]/gain[valid]))))
  return u+amount*direction.reshape(8,3)
 def maps(self,aperture=None):
  a=self.aperture_required if aperture is None else aperture;T=ca.DM(self.interface_from_grasp);out=[]
  for sign in [-1,1]:
   R=ca.DM(np.asarray(self.interface_from_grasp)[:3,:3]@contact_basis([0,sign,0]))
   for x,z in [(-self.half_width,-self.half_height),(-self.half_width,self.half_height),(self.half_width,-self.half_height),(self.half_width,self.half_height)]:
    p=T[:3,3]+T[:3,:3]@ca.vertcat(x,sign*a/2,z);out.append(ca.vertcat(skew(p)@R,R))
  return out
 def aperture(self,opti,holding=True):
  a=opti.variable();opti.subject_to(opti.bounded(self.aperture_min,a,self.aperture_max));opti.set_initial(a,self.aperture_required)
  if holding:opti.subject_to(a==self.aperture_required)
  return a
 def reaction(self,opti,scale=1.,initial=None,aperture=None):
  return self.reaction_with_witness(opti,scale,initial,aperture,force_coordinates=self.preferred_force_coordinates)[0]
 def reaction_with_witness(self,opti,scale=1.,initial=None,aperture=None,force_coordinates='cartesian'):
  forces=[];warm=None
  try:initial_scale=float(scale)
  except (TypeError,RuntimeError):initial_scale=float(opti.debug.value(scale,opti.initial()))
  if initial is not None and np.size(initial)==24:warm=np.asarray(initial,float).reshape(8,3)
  elif initial is not None and np.size(initial)==6:
   check=self.check(np.asarray(initial,float)/initial_scale)
   if check['feasible']:
    warm=self.balanced_preload(initial_scale*check['finger_forces'],scale=initial_scale)
  if force_coordinates not in {'cartesian','scaled_tangent'}:raise ValueError('Unknown finger-force coordinates')
  for index in range(8):
   seed=warm[index] if warm is not None else np.array([0.,0.,initial_scale*self.closing_force_per_finger/8])
   if force_coordinates=='scaled_tangent':
    normal=opti.variable();ratio=opti.variable(2);opti.subject_to(normal>=0);opti.subject_to(ca.sumsqr(ratio)<=1);f=ca.vertcat(self.mu*normal*ratio,normal);opti.set_initial(normal,seed[2]);opti.set_initial(ratio,seed[:2]/(self.mu*seed[2]) if seed[2]>1e-12 else np.zeros(2))
   else:
    f=opti.variable(3);opti.subject_to(f[2]>=0);opti.subject_to(ca.sumsqr(f[:2])<=(self.mu*f[2])**2);opti.set_initial(f,seed)
   forces.append(f)
  for finger in [0,1]:opti.subject_to(sum(f[2] for f in forces[4*finger:4*finger+4])<=scale*self.closing_force_per_finger)
  wrench=sum(B@f for B,f in zip(self.maps(aperture),forces))
  if self.force_component_limits is not None:
   force=ca.DM(np.asarray(self.interface_from_grasp)[:3,:3]).T@wrench[3:];limit=scale*ca.DM(self.force_component_limits);opti.subject_to(opti.bounded(-limit,force,limit))
  if self.finger_linear_limits is not None:
   A,b=self.finger_linear_limits;opti.subject_to(ca.DM(A)@ca.vertcat(*forces)<=scale*ca.DM(b))
  return wrench,ca.vertcat(*forces)
 def conic_matrices(self,aperture=None):
  """Aeq*u=w, Alinear*u<=blinear, and eight SOC blocks C*u∈Q3."""
  B=np.hstack([np.asarray(ca.DM(x)) for x in self.maps(aperture)]);A=[];b=[]
  for finger in [0,1]:
   row=np.zeros(24);row[np.arange(4*finger,4*finger+4)*3+2]=1;A.append(row);b.append(self.closing_force_per_finger)
  for point in range(8):
   row=np.zeros(24);row[point*3+2]=-1;A.append(row);b.append(0.)
  if self.force_component_limits is not None:
   force=np.asarray(self.interface_from_grasp)[:3,:3].T@B[3:]
   for row,limit in zip(force,self.force_component_limits):A.extend([row,-row]);b.extend([limit,limit])
  if self.finger_linear_limits is not None:
   extra,limits=self.finger_linear_limits;A.extend(np.asarray(extra));b.extend(np.asarray(limits))
  cones=[]
  for point in range(8):
   C=np.zeros((3,24));C[0,point*3+2]=self.mu;C[1,point*3]=1;C[2,point*3+1]=1;cones.append(C)
  return B,np.asarray(A),np.asarray(b),cones
 def check(self,wrench,*,tolerance=1e-7):
  import clarabel
  import scipy.sparse as sp
  B,L,b,C=self.conic_matrices();A=sp.csc_matrix(np.vstack([B,L,*[-c for c in C]]));rhs=np.r_[wrench,b,np.zeros(24)];settings=clarabel.DefaultSettings();settings.verbose=False;settings.tol_feas=1e-10;settings.tol_gap_abs=1e-10;settings.tol_gap_rel=1e-10
  solver=clarabel.DefaultSolver(sp.eye(24,format='csc')*1e-6,np.zeros(24),A,rhs,[clarabel.ZeroConeT(6),clarabel.NonnegativeConeT(len(b)),*[clarabel.SecondOrderConeT(3) for _ in C]],settings);s=solver.solve();u=np.asarray(s.x);res=max(float(np.max(abs(B@u-wrench))),float(np.max(L@u-b)),max(float(np.linalg.norm((c@u)[1:])-(c@u)[0]) for c in C));return dict(feasible=str(s.status) in {'Solved','AlmostSolved'} and res<tolerance,primal_residual=res,finger_forces=u.reshape(8,3),status=str(s.status))
