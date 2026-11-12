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


def fixed_generalized_margin(maps,task,capabilities,*,torque_gravity=None,torque_maps=None,effort=None,rho_min=0.,tolerance=2e-6,fixed_rho=None,torque_weight=None,witness_weight=1e-6):
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
 if fixed_rho is not None:
  row=sp.lil_matrix((1,ntotal));row[0,0]=1.;eq.append(row.tocsc());rhs.append(np.array([float(fixed_rho)]))
 E=sp.vstack(eq,format='csc');L=sp.vstack(linear,format='csc');S=sp.vstack(socs,format='csc') if socs else sp.csc_matrix((0,ntotal));A=sp.vstack([E,L,S],format='csc');b=np.r_[np.concatenate(rhs),np.concatenate(linear_b),np.concatenate(soc_b) if soc_b else []];q=np.zeros(ntotal);q[0]=-1 if fixed_rho is None else 0;settings=clarabel.DefaultSettings();settings.verbose=False;settings.tol_feas=1e-9;settings.tol_gap_abs=1e-9;settings.tol_gap_rel=1e-9;settings.max_iter=150
 cones=[clarabel.ZeroConeT(E.shape[0]),clarabel.NonnegativeConeT(L.shape[0]),*[clarabel.SecondOrderConeT(3) for _ in socs]];attempts=[]
 for optimality_tolerance,regularization in ([(1e-9,0.),(1e-8,0.),(1e-8,1e-10)] if fixed_rho is None else [(1e-9,2*witness_weight),(1e-8,2*witness_weight)]):
  settings.tol_feas=optimality_tolerance;settings.tol_gap_abs=optimality_tolerance;settings.tol_gap_rel=optimality_tolerance;penalty=np.concatenate([np.full(entry[0].shape[1],regularization if torque_weight is None or cap.kind=='G' else 0.) for cap,entry in zip(capabilities,enc)]);P=sp.diags(np.r_[0.,np.tile(penalty,len(vertices))],format='csc');linear_cost=q.copy()
  if torque_weight is not None:
   Tlocal=np.hstack([np.asarray(torque_maps[i])@entry[0] for i,entry in enumerate(enc)]);Qt=np.asarray(torque_weight);block=2*Tlocal.T@Qt@Tlocal;P=P+sp.block_diag([sp.csc_matrix((1,1)),*[sp.csc_matrix(block) for _ in vertices]],format='csc');linear_cost[1:]=np.tile(-2*Tlocal.T@Qt@np.asarray(torque_gravity),len(vertices))
  solver=clarabel.DefaultSolver(P,linear_cost,A,b,cones,settings);sol=solver.solve();attempts.append(dict(status=str(sol.status),solver_tolerance=optimality_tolerance,witness_regularization=regularization))
  if str(sol.status) in {'Solved','AlmostSolved','PrimalInfeasible','DualInfeasible'}:break
 x=np.asarray(sol.x);slack=b-A@x;res=max(float(np.max(abs(slack[:E.shape[0]]))),float(max(0,-np.min(slack[E.shape[0]:E.shape[0]+L.shape[0]]))));at=E.shape[0]+L.shape[0]
 for _ in socs:res=max(res,float(np.linalg.norm(slack[at+1:at+3])-slack[at]));at+=3
 reactions=[];coordinates=[]
 for k in range(len(vertices)):
  row=[];local=[];start=1+k*nlocal
  for i,(B,L,b,C) in enumerate(enc):
   u=x[start+offsets[i]:start+offsets[i+1]];row.append(B@u);local.append(u)
  reactions.append(row);coordinates.append(tuple(local))
 return dict(rho=float(x[0]),primal_residual=res,status=str(sol.status),certificate=str(sol.status) in {'Solved','AlmostSolved'} and res<tolerance,reactions=np.asarray(reactions),coordinates=tuple(coordinates),solver_attempts=attempts,fixed_rho_feasibility=fixed_rho is not None)
