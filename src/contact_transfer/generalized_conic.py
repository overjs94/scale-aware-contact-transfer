"""Independent frozen-geometry SOCP for explicit interface/finger load models."""
import numpy as np
import scipy.sparse as sp
from .contacts import Capability


def encoding(cap):
 """lambda=B*y; L*y<=b; C*y+c belongs to SOC(3)."""
 if hasattr(cap,'conic_matrices'):
  B,L,b,C=cap.conic_matrices();return B,L,b,[(c,np.zeros(3)) for c in C]
 if cap.kind in {'P','S'}:
  B=np.vstack((np.zeros((3,3)),np.eye(3)));L=np.array([[0,0,1],[0,0,-1.]]);adh=cap.adhesion_max if cap.kind=='S' else 0.;b=np.array([cap.normal_max,adh]);C=np.array([[0,0,cap.mu],[1,0,0],[0,1,0.]]);return B,L,b,[(C,np.array([cap.mu*adh,0,0]))]
 B=np.asarray(cap.H);n=B.shape[1];return B,np.r_[np.eye(n),-np.eye(n)],np.tile(cap.bounds,2),[]


def fixed_generalized_margin(maps,task,capabilities,*,torque_gravity=None,torque_maps=None,effort=None,rho_min=0.,tolerance=2e-6):
 import clarabel
 enc=[encoding(c) for c in capabilities];offsets=np.cumsum([0]+[B.shape[1] for B,L,b,C in enc]);nlocal=offsets[-1];nv=len(task.nominal);vertices=task.vertices;ntotal=1+len(vertices)*nlocal;eq=[];rhs=[];linear=[];linear_b=[];socs=[];soc_b=[];Lrho=sp.lil_matrix((1,ntotal));Lrho[0,0]=-1;linear.append(Lrho.tocsc());linear_b.append(np.array([-rho_min]))
 if len(maps)!=len(enc):raise ValueError('One generalized map per interface required')
 for k,vertex in enumerate(vertices):
  start=1+k*nlocal;A=sp.lil_matrix((nv,ntotal));A[:,0]=-np.asarray(task.E)@np.asarray(task.D)@vertex
  for i,(B,L,b,C) in enumerate(enc):
   sl=slice(start+offsets[i],start+offsets[i+1]);A[:,sl]=np.asarray(maps[i])@B
   if len(b):
    mat=sp.lil_matrix((len(b),ntotal));mat[:,sl]=L;linear.append(mat.tocsc());linear_b.append(b)
   for cone,constant in C:
    mat=sp.lil_matrix((3,ntotal));mat[:,sl]=-cone;socs.append(mat.tocsc());soc_b.append(constant)
  eq.append(A.tocsc());rhs.append(np.asarray(task.nominal))
  if torque_gravity is not None:
   g=np.asarray(torque_gravity);limit=np.asarray(effort);T=sp.lil_matrix((len(g),ntotal))
   for i,(B,L,b,C) in enumerate(enc):T[:,start+offsets[i]:start+offsets[i+1]]=np.asarray(torque_maps[i])@B
   # tau=g-T*y, hence -T*y<=limit-g and T*y<=limit+g.
   linear.extend([-T.tocsc(),T.tocsc()]);linear_b.extend([limit-g,limit+g])
 E=sp.vstack(eq,format='csc');L=sp.vstack(linear,format='csc');S=sp.vstack(socs,format='csc') if socs else sp.csc_matrix((0,ntotal));A=sp.vstack([E,L,S],format='csc');b=np.r_[np.concatenate(rhs),np.concatenate(linear_b),np.concatenate(soc_b) if soc_b else []];q=np.zeros(ntotal);q[0]=-1;settings=clarabel.DefaultSettings();settings.verbose=False;settings.tol_feas=1e-9;settings.tol_gap_abs=1e-9;settings.tol_gap_rel=1e-9;settings.max_iter=150
 solver=clarabel.DefaultSolver(sp.csc_matrix((ntotal,ntotal)),q,A,b,[clarabel.ZeroConeT(E.shape[0]),clarabel.NonnegativeConeT(L.shape[0]),*[clarabel.SecondOrderConeT(3) for _ in socs]],settings);sol=solver.solve();x=np.asarray(sol.x);slack=b-A@x;res=max(float(np.max(abs(slack[:E.shape[0]]))),float(max(0,-np.min(slack[E.shape[0]:E.shape[0]+L.shape[0]]))));at=E.shape[0]+L.shape[0]
 for _ in socs:res=max(res,float(np.linalg.norm(slack[at+1:at+3])-slack[at]));at+=3
 reactions=[];coordinates=[]
 for k in range(len(vertices)):
  row=[];local=[];start=1+k*nlocal
  for i,(B,L,b,C) in enumerate(enc):
   u=x[start+offsets[i]:start+offsets[i+1]];row.append(B@u);local.append(u)
  reactions.append(row);coordinates.append(tuple(local))
 return dict(rho=float(x[0]),primal_residual=res,status=str(sol.status),certificate=str(sol.status) in {'Solved','AlmostSolved'} and res<tolerance,reactions=np.asarray(reactions),coordinates=tuple(coordinates))
