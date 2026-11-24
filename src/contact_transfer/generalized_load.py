"""Explicit multi-body extension of the manuscript's fixed E,D wrench tasks."""
from dataclasses import dataclass
from itertools import product
import casadi as ca
import numpy as np
from .lie import skew

@dataclass(frozen=True)
class GeneralizedTaskLoad:
 nominal: np.ndarray
 E: np.ndarray
 D: np.ndarray
 def __post_init__(self):
  w,E,D=[np.asarray(a,float) for a in [self.nominal,self.E,self.D]]
  if w.ndim!=1 or E.ndim!=2 or E.shape[0]!=len(w) or not 1<=E.shape[1]<=len(w) or np.linalg.matrix_rank(E)!=E.shape[1] or D.shape!=(E.shape[1],E.shape[1]) or not np.allclose(D,np.diag(np.diag(D))) or np.any(np.diag(D)<=0) or not all(np.isfinite(a).all() for a in [w,E,D]):raise ValueError('Fixed, independent generalized task directions and positive scales required')
 @property
 def vertices(self):return np.asarray(list(product((-1.,1.),repeat=np.shape(self.E)[1])))
 def required(self,rho,vertex):return ca.DM(self.nominal)+rho*ca.DM(self.E)@ca.DM(self.D)@ca.DM(vertex)


def link_reaction_map(object_model,configuration,link):
 """Object-on-agent interface wrench -> generalized reaction at the object.

 Positive result is the object-on-agent load sum; the object's physical load is
 its negative. Therefore its nominal equilibrium target is -object.gravity().
 """
 q=ca.DM(configuration);T=object_model.fk(q,link);J=object_model.jacobian(q,link,'world_aligned')
 def apply(point,rotation,wrench):
  offset=T[:3,:3]@point;Jp=ca.vertcat(J[:3,:],J[3:,:]-skew(offset)@J[:3,:]);Rw=T[:3,:3]@rotation;world=ca.vertcat(Rw@wrench[:3],Rw@wrench[3:]);return Jp.T@world
 return apply
