"""Assemble URDF robots and a free rigid object without closing kinematic loops."""

from dataclasses import dataclass

import casadi as ca
import numpy as np

from .lie import ad, integrate_floating, quat_rotation, skew, spatial_inertia, transform


@dataclass
class RigidObject:
    mass: float
    com: np.ndarray
    inertia_com: np.ndarray

    def __post_init__(self):
        if self.mass <= 0 or np.linalg.eigvalsh(self.inertia_com)[0] <= 0:
            raise ValueError("object requires positive mass and COM inertia")


class MechanicalSystem:
    """q=[robot configurations; object p,xyzw], v=[robot tangents; object w_b,v_b].

    Floating robot base coordinates remain unactuated. External environment
    loads are supplied explicitly through external_load(q,v) or contact impulses.
    """
    def __init__(self, robots, object: RigidObject, *, gravity=(0, 0, -9.81),
                 external_load=None, object_passive_bias=None):
        self.robots, self.object = list(robots), object
        self.q_slices, self.v_slices = [], []
        nq, nv = 0, 0
        for r in robots:
            self.q_slices.append(slice(nq, nq+r.nq))
            self.v_slices.append(slice(nv, nv+r.nv))
            nq, nv = nq+r.nq, nv+r.nv
        self.articulated_object = not isinstance(object, RigidObject)
        if self.articulated_object and not object.floating_base:
            raise ValueError('A manipulated URDF object must have a free root')
        oq, ov = (object.nq, object.nv) if self.articulated_object else (7, 6)
        self.object_q, self.object_v = slice(nq, nq+oq), slice(nv, nv+ov)
        self.nq, self.nv, self.nu = nq+oq, nv+ov, sum(r.n for r in robots)
        B = np.zeros((self.nv, self.nu))
        off = 0
        for r, vs in zip(robots, self.v_slices):
            start = vs.start+(6 if r.floating_base else 0)
            B[start:start+r.n, off:off+r.n] = np.eye(r.n)
            off += r.n
        self.B = ca.DM(B)
        self.effort = np.concatenate([r.effort for r in robots]) if robots else np.zeros(0)
        self.gravity = ca.DM(gravity)
        self.external_load = external_load
        self.object_passive_bias = object_passive_bias
        self.Iobj = None if self.articulated_object else spatial_inertia(object.mass, object.com, object.inertia_com)

    def object_pose(self, q, frame=None):
        qo = q[self.object_q]
        if self.articulated_object:
            return self.object.fk(qo, frame or self.object.tree.root)
        if frame not in (None, 'object'):
            raise ValueError('RigidObject has only its root frame')
        return transform(quat_rotation(qo[3:7]), qo[:3])

    def mass(self, q):
        Mo = self.object.mass(q[self.object_q]) if self.articulated_object else self.Iobj
        return ca.diagcat(*(r.mass(q[s]) for r, s in zip(self.robots, self.q_slices)), Mo)

    def bias(self, q, v):
        V = v[self.object_v]
        if self.articulated_object:
            bo = self.object.bias(q[self.object_q], V)
        else:
            To = self.object_pose(q)
            fgrav = self.object.mass*To[:3, :3].T@self.gravity
            wgrav = ca.vertcat(skew(self.object.com)@fgrav, fgrav)
            bo = -ad(V).T@self.Iobj@V-wgrav
        if self.object_passive_bias is not None:
            bo += self.object_passive_bias(q[self.object_q], V)
        b = ca.vertcat(*(r.bias(q[qs], v[vs]) for r, qs, vs in
                        zip(self.robots, self.q_slices, self.v_slices)), bo)
        return b if self.external_load is None else b-self.external_load(q, v)

    def integrate(self, q, delta):
        configs = []
        for r, qs, vs in zip(self.robots, self.q_slices, self.v_slices):
            configs.append(integrate_floating(q[qs], delta[vs]) if r.floating_base
                           else q[qs]+delta[vs])
        configs.append(integrate_floating(q[self.object_q], delta[self.object_v]))
        return ca.vertcat(*configs)

    def point_jacobian_object(self, q, point_object, frame=None):
        R = self.object_pose(q, frame)[:3, :3]
        J = ca.MX.zeros(3, self.nv) if isinstance(q, ca.MX) else ca.SX.zeros(3, self.nv)
        if self.articulated_object:
            Jo = self.object.jacobian(q[self.object_q], frame or self.object.tree.root, 'world_aligned')
            p = point_object if isinstance(point_object, (ca.SX, ca.MX, ca.DM)) else ca.DM(point_object)
            J[:, self.object_v] = Jo[3:, :]-skew(R@p)@Jo[:3, :]
        else:
            J[:, self.object_v] = R@ca.horzcat(-skew(point_object), ca.DM.eye(3))
        return J

    def angular_jacobian_object(self, q, frame=None):
        J = ca.MX.zeros(3, self.nv) if isinstance(q, ca.MX) else ca.SX.zeros(3, self.nv)
        if self.articulated_object:
            Jo = self.object.jacobian(q[self.object_q], frame or self.object.tree.root, 'world_aligned')
            J[:, self.object_v] = Jo[:3, :]
        else:
            J[:, self.object_v.start:self.object_v.start+3] = self.object_pose(q)[:3, :3]
        return J

    def point_world(self, q, point_object, frame=None):
        T = self.object_pose(q, frame)
        p = point_object if isinstance(point_object, (ca.SX, ca.MX, ca.DM)) else ca.DM(point_object)
        return T[:3, 3]+T[:3, :3]@p
