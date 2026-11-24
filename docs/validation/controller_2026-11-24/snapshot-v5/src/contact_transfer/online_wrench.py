"""Measured-state contact allocation, independent of task names and load schedules.

The QP uses the same exact P/G conic encoding as the offline validator. A failed
or inaccurate solve is never applied. This is an execution extension, not an
additional claim about the manuscript's static robustness certificate.
"""
from dataclasses import replace
from time import perf_counter

import numpy as np
import scipy.sparse as sp
from scipy.spatial.transform import Rotation

from .conic_backends import solve_conic, ConicSettings
from .generalized_conic import encoding


def reference_tangent(obj,q,qref,v,vref,aref):
    """Reference error, velocity and acceleration in the ACTUAL body basis.

    Derivatives include the moving-body transport terms. Internal joint
    coordinates retain the authoritative model's scalar tangent convention.
    """
    error=np.zeros(obj.nv);velocity=np.array(vref,copy=True);acceleration=np.array(aref,copy=True)
    slices=[(obj.q_slices[n],obj.v_slices[n]) for n in obj.names] if hasattr(obj,'names') else [(slice(0,obj.nq),slice(0,obj.nv))]
    for qs,vs in slices:
        qa,qb=q[qs],qref[qs];R=Rotation.from_quat(qa[3:7]).as_matrix();Rd=Rotation.from_quat(qb[3:7]).as_matrix();Q=R.T@Rd
        va,vd,ad=v[vs],vref[vs],aref[vs]
        error[vs]=np.r_[Rotation.from_matrix(Q).as_rotvec(),R.T@(qb[:3]-qa[:3]),qb[7:]-qa[7:]]
        velocity[vs]=np.r_[Q@vd[:3],Q@vd[3:6],vd[6:]]
        acceleration[vs]=np.r_[Q@ad[:3]-np.cross(va[:3],Q@vd[:3]),Q@(ad[3:6]+np.cross(vd[:3],vd[3:6]))-np.cross(va[:3],Q@vd[3:6]),ad[6:]]
    return error,velocity,acceleration


def operating_capabilities(capabilities, fraction):
    if not np.isfinite(fraction) or not 0 < fraction <= 1:
        raise ValueError('Operating fraction must be in (0,1]')
    return tuple(replace(c, closing_force_per_finger=fraction*c.closing_force_per_finger)
                 if c.kind == 'G' else c for c in capabilities)


class WrenchAllocator:
    """min ||lambda-lambda_plan||_W^2 + eps ||finger forces||^2.

    Subject to A lambda = rhs, exact contact cones, jaw operating limits and
    tau = bias_robot - T lambda within the native joint effort bounds.
    """
    def __init__(self, capabilities, *, settings=None, characteristic_length=.6, project_unactuated=False):
        if not np.isfinite(characteristic_length) or characteristic_length<=0:
            raise ValueError('Positive finite characteristic length required')
        self.capabilities=tuple(capabilities)
        self.enc=tuple(encoding(c) for c in self.capabilities)
        self.offsets=np.cumsum([0]+[e[0].shape[1] for e in self.enc])
        self.B=sp.block_diag([e[0] for e in self.enc],format='csc')
        self.L=sp.block_diag([e[1] for e in self.enc],format='csc')
        self.bounds=np.concatenate([e[2] for e in self.enc])
        self.S=sp.block_diag([np.vstack([-C for C,_ in e[3]]) if e[3] else np.zeros((0,e[0].shape[1])) for e in self.enc],format='csc')
        self.constants=np.concatenate([c for e in self.enc for _,c in e[3]])
        self.weights=np.tile(np.r_[np.full(3,1/characteristic_length**2),np.ones(3)],len(self.enc))
        self.P=2*(self.B.T@sp.diags(self.weights)@self.B+sp.eye(self.B.shape[1])*1e-5)
        self.settings=settings or ConicSettings()
        self.project_unactuated=project_unactuated

    def solve(self, maps, rhs, planned, torque_maps, robot_bias, effort, active):
        if len(maps)!=len(self.enc) or len(torque_maps)!=len(self.enc) or len(active)!=len(self.enc):
            raise ValueError('One map, torque map and activity flag per contact required')
        if np.shape(planned)!=(len(self.enc),6) or any(np.shape(m)!=(len(rhs),6) for m in maps) or any(np.shape(t)!=(len(robot_bias),6) for t in torque_maps):
            raise ValueError('Inconsistent generalized wrench dimensions')
        if not all(np.isfinite(x).all() for x in [*maps,rhs,planned,*torque_maps,robot_bias,effort]) or np.shape(effort)!=np.shape(robot_bias) or np.min(effort)<0:
            raise ValueError('Finite mechanics inputs and nonnegative effort bounds required')
        started=perf_counter();Aobj=np.hstack(maps)@self.B;T=np.hstack(torque_maps)@self.B
        controlled=np.eye(len(rhs));unactuated_request=np.zeros(len(rhs))
        if self.project_unactuated:
            columns=np.concatenate([np.arange(self.offsets[i],self.offsets[i+1]) for i,a in enumerate(active) if a]) if any(active) else np.zeros(0,int)
            image=np.asarray(Aobj)[:,columns]
            U,singular,_=np.linalg.svd(image,full_matrices=False)
            rank=int(np.sum(singular>max(image.shape,default=1)*np.max(singular,initial=0.)*1e-10))
            if rank<len(rhs):
                controlled=U[:,:rank].T
                unactuated_request=np.asarray(rhs)-controlled.T@(controlled@rhs)
        inactive=[self.B[6*i:6*i+6] for i,a in enumerate(active) if not a]
        E=sp.vstack([sp.csc_matrix(controlled@Aobj),*inactive],format='csc')
        beq=np.r_[controlled@rhs,np.zeros(6*len(inactive))]
        L=sp.vstack([self.L,-T,T],format='csc');bl=np.r_[self.bounds,effort-robot_bias,effort+robot_bias]
        A=sp.vstack([E,L,self.S],format='csc');b=np.r_[beq,bl,self.constants]
        objective=-2*np.asarray(self.B.T@(self.weights*np.asarray(planned).ravel())).ravel()
        x,status,timing=solve_conic(self.P,objective,A,b,equality_rows=E.shape[0],linear_rows=L.shape[0],soc_dimensions=[3]*(self.S.shape[0]//3),settings=self.settings,tolerance=1e-8)
        slack=b-A@x;res=max(float(np.max(abs(slack[:len(beq)]),initial=0)),float(np.max(-slack[len(beq):len(beq)+len(bl)],initial=0)))
        cones=slack[len(beq)+len(bl):].reshape(-1,3)
        if len(cones):res=max(res,float(np.max(np.linalg.norm(cones[:,1:],axis=1)-cones[:,0],initial=0)))
        accepted=status in {'Solved','AlmostSolved'} and res<2e-5
        jaw=[]
        for i,c in enumerate(self.capabilities):
            u=x[self.offsets[i]:self.offsets[i+1]]
            jaw.append(u.reshape(2,4,3)[:,:,2].sum(axis=1) if c.kind=='G' else np.zeros(2))
        return dict(accepted=bool(accepted),status=status,residual=res,reactions=np.asarray(self.B@x).reshape(-1,6),jaw_required=np.array(jaw),controlled_dynamics_rank=controlled.shape[0],object_dynamics_dimension=len(rhs),unactuated_request=unactuated_request,full_dynamics_equality_enforced=controlled.shape[0]==len(rhs),wall_time_s=perf_counter()-started,solver=timing)
