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
 kind: str=field(default='G',init=False)
 has_moment: bool=field(default=True,init=False)
 def __post_init__(self):
  T=np.asarray(self.interface_from_grasp,float);values=[self.aperture_required,self.half_width,self.half_height,self.mu,self.closing_force_per_finger,self.aperture_min,self.aperture_max]
  if not np.isfinite(values).all() or self.half_width<=0 or self.half_height<=0 or self.mu<=0 or self.closing_force_per_finger<=0 or not 0<=self.aperture_min<=self.aperture_required<=self.aperture_max:raise ValueError('Invalid finger geometry, friction, aperture or force bound')
  if T.shape!=(4,4) or not np.allclose(T[3],[0,0,0,1]) or not np.allclose(T[:3,:3].T@T[:3,:3],np.eye(3)) or not np.isclose(np.linalg.det(T[:3,:3]),1):raise ValueError('Proper grasp/interface transform required')
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
  forces=[];warm=None
  try:initial_scale=float(scale)
  except (TypeError,RuntimeError):initial_scale=float(opti.debug.value(scale,opti.initial()))
  if initial is not None and np.size(initial)==24:warm=np.asarray(initial,float).reshape(8,3)
  elif initial is not None and np.size(initial)==6:
   check=self.check(np.asarray(initial,float)/initial_scale)
   if check['feasible']:warm=initial_scale*check['finger_forces']
  for index in range(8):
   f=opti.variable(3);opti.subject_to(f[2]>=0);opti.subject_to(ca.sumsqr(f[:2])<=(self.mu*f[2])**2);
   opti.set_initial(f,warm[index] if warm is not None else [0,0,initial_scale*self.closing_force_per_finger/8]);forces.append(f)
  for finger in [0,1]:opti.subject_to(sum(f[2] for f in forces[4*finger:4*finger+4])<=scale*self.closing_force_per_finger)
  return sum(B@f for B,f in zip(self.maps(aperture),forces))
 def conic_matrices(self,aperture=None):
  """Aeq*u=w, Alinear*u<=blinear, and eight SOC blocks C*u∈Q3."""
  B=np.hstack([np.asarray(ca.DM(x)) for x in self.maps(aperture)]);A=[];b=[]
  for finger in [0,1]:
   row=np.zeros(24);row[np.arange(4*finger,4*finger+4)*3+2]=1;A.append(row);b.append(self.closing_force_per_finger)
  for point in range(8):
   row=np.zeros(24);row[point*3+2]=-1;A.append(row);b.append(0.)
  cones=[]
  for point in range(8):
   C=np.zeros((3,24));C[0,point*3+2]=self.mu;C[1,point*3]=1;C[2,point*3+1]=1;cones.append(C)
  return B,np.asarray(A),np.asarray(b),cones
 def check(self,wrench,*,tolerance=1e-7):
  import clarabel
  import scipy.sparse as sp
  B,L,b,C=self.conic_matrices();A=sp.csc_matrix(np.vstack([B,L,*[-c for c in C]]));rhs=np.r_[wrench,b,np.zeros(24)];settings=clarabel.DefaultSettings();settings.verbose=False;settings.tol_feas=1e-10;settings.tol_gap_abs=1e-10;settings.tol_gap_rel=1e-10
  solver=clarabel.DefaultSolver(sp.eye(24,format='csc')*1e-6,np.zeros(24),A,rhs,[clarabel.ZeroConeT(6),clarabel.NonnegativeConeT(len(b)),*[clarabel.SecondOrderConeT(3) for _ in C]],settings);s=solver.solve();u=np.asarray(s.x);res=max(float(np.max(abs(B@u-wrench))),float(np.max(L@u-b)),max(float(np.linalg.norm((c@u)[1:])-(c@u)[0]) for c in C));return dict(feasible=str(s.status) in {'Solved','AlmostSolved'} and res<tolerance,primal_residual=res,finger_forces=u.reshape(8,3),status=str(s.status))
