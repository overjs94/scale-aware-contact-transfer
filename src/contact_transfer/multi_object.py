"""Independent free rigid bodies in one generalized manipulation system.

Each component is an authoritative one-link URDF Robot(floating_base=True).
There are no hidden joints, welds or object actuators. Internal contacts enter
the global momentum equation as equal-and-opposite Jacobian impulses.
"""
from types import SimpleNamespace
import casadi as ca
import numpy as np
from scipy.linalg import block_diag
from .lie import integrate_floating,skew
from .trajectory import Contact


class RigidCollection:
    def __init__(self,bodies,*,symbolic=False):
        self.bodies=dict(bodies);self.names=tuple(self.bodies);self.symbolic_mode=symbolic
        if not self.names or any(not r.floating_base or r.n for r in self.bodies.values()):raise ValueError('Collection components must be independent free one-body URDFs')
        self.floating_base=True;self.n=0;self.nq=7*len(self.names);self.nv=6*len(self.names)
        self.lower=self.upper=self.velocity=self.effort=np.zeros(0)
        self.tree=SimpleNamespace(root=self.names[0],links={n:None for n in self.names})
        self.q_slices={n:slice(7*i,7*i+7) for i,n in enumerate(self.names)};self.v_slices={n:slice(6*i,6*i+6) for i,n in enumerate(self.names)}
        self.quaternion_slices=tuple(slice(7*i+3,7*i+7) for i in range(len(self.names)))
    def symbolic(self):
        return RigidCollection({n:r.symbolic() for n,r in self.bodies.items()},symbolic=True)
    def fk(self,q,frame=None):
        n=frame or self.names[0]
        if n not in self.bodies:raise ValueError('Unknown rigid body '+str(n))
        r=self.bodies[n];return r.fk(q[self.q_slices[n]],r.tree.root)
    def jacobian(self,q,frame=None,representation='mixed'):
        n=frame or self.names[0];r=self.bodies[n];local=r.jacobian(q[self.q_slices[n]],r.tree.root,representation)
        J=ca.MX.zeros(6,self.nv) if isinstance(local,ca.MX) else ca.SX.zeros(6,self.nv) if isinstance(local,ca.SX) else ca.DM.zeros(6,self.nv) if self.symbolic_mode else np.zeros((6,self.nv));J[:,self.v_slices[n]]=local;return J
    def mass(self,q):
        values=[r.mass(q[self.q_slices[n]]) for n,r in self.bodies.items()]
        return ca.diagcat(*values) if self.symbolic_mode else block_diag(*values)
    def bias(self,q,v):
        values=[r.bias(q[self.q_slices[n]],v[self.v_slices[n]]) for n,r in self.bodies.items()]
        return ca.vertcat(*values) if self.symbolic_mode else np.concatenate(values)
    def gravity(self,q):
        return self.bias(q,ca.DM.zeros(self.nv) if self.symbolic_mode else np.zeros(self.nv))
    def integrate_configuration(self,q,delta):
        if self.symbolic_mode:return ca.vertcat(*[integrate_floating(q[self.q_slices[n]],delta[self.v_slices[n]]) for n in self.names])
        return np.concatenate([r.integrate(q[self.q_slices[n]],delta[self.v_slices[n]]) for n,r in self.bodies.items()])
    def integrate(self,q,delta):return self.integrate_configuration(q,delta)


class BodyContact(Contact):
    """Point on body B against a finite-feature candidate on body A.

    Normal is outward from A, in A coordinates. Positive gap is separation.
    Its impulse pushes B along +normal and A oppositely; no outside support is
    manufactured when the assembly's internal contact acquires load.
    """
    def __init__(self,*args,other_frame,other_point,patch_half_size=None,**kwargs):
        super().__init__(*args,**kwargs);self.other_frame=other_frame;self.other_point=np.asarray(other_point,float).reshape(3)
        self.patch_half_size=None if patch_half_size is None else np.broadcast_to(np.asarray(patch_half_size,float),(2,)).copy()
        if self.patch_half_size is not None and np.any(self.patch_half_size<=0):raise ValueError('Positive finite support patch required')
        if self.robot_index is not None or self.object_frame is None:raise ValueError('Internal body contact needs two explicit object frames')
    def support_coordinates(self,system,q):
        a=system.point_world(q,self.point_object,self.object_frame);b=system.point_world(q,self.other_point,self.other_frame);R=system.object_pose(q,self.object_frame)[:3,:3]@ca.DM(self.basis());return (R.T@(b-a))[:2]
    def geometry(self,system,q):
        a=system.point_world(q,self.point_object,self.object_frame);b=system.point_world(q,self.other_point,self.other_frame);R=system.object_pose(q,self.object_frame)[:3,:3]@ca.DM(self.basis())
        gap=ca.dot(R[:,2],b-a);Ta=system.object_pose(q,self.object_frame);closest=b-gap*R[:,2];local=Ta[:3,:3].T@(closest-Ta[:3,3])
        Ja=system.point_jacobian_object(q,local,self.object_frame);Jb=system.point_jacobian_object(q,self.other_point,self.other_frame)
        return gap,R.T@(Jb-Ja),None
