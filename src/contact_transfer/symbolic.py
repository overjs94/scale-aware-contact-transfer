"""Differentiable URDF tree: Lie recursive Newton--Euler and generated functions.

Pinocchio's PyPI wheel need not provide CasADi bindings. This implementation
builds CasADi graphs from the same URDF and is independently checked against
Pinocchio, including floating bases, fixed inertias, and mimic joints.
"""

from pathlib import Path
import hashlib
import json
import re

import casadi as ca
import numpy as np

from .lie import (ad, adjoint, axis_rotation, inverse, quat_rotation, skew,
                  spatial_inertia, transform)
from .urdf import URDFTree


class SymbolicRobot:
    def __init__(self, urdf, *, floating_base=False, locked_joints=None,
                 base_pose=None, gravity=(0, 0, -9.81)):
        self.tree = URDFTree(urdf, locked_joints)
        self.floating_base = bool(floating_base)
        self.n = self.tree.n
        self.nq, self.nv = self.n+(7 if floating_base else 0), self.n+(6 if floating_base else 0)
        self.base_pose = np.eye(4) if base_pose is None else np.asarray(base_pose, float)
        self.gravity_world = np.asarray(gravity, float)
        self.joint_names = self.tree.joint_names
        self.lower, self.upper = self.tree.lower, self.tree.upper
        self.effort, self.velocity = self.tree.effort, self.tree.velocity
        q, v, a = ca.SX.sym("q", self.nq), ca.SX.sym("v", self.nv), ca.SX.sym("a", self.nv)
        self.q, self.v, self.a = q, v, a
        qj = q[7:] if floating_base else q
        off = 6 if floating_base else 0
        vj, aj = v[off:], a[off:]
        T0 = transform(quat_rotation(q[3:7]), q[:3]) if floating_base else ca.DM(self.base_pose)
        poses = {self.tree.root: T0}
        velocities = {self.tree.root: v[:6] if floating_base else ca.SX.zeros(6)}
        accelerations = {self.tree.root: (
            a[:6] if floating_base else ca.SX.zeros(6)) -
            ca.vertcat(ca.SX.zeros(3), T0[:3, :3].T @ ca.DM(gravity))}
        Xs, forces, axes, ancestors = {}, {}, {}, {self.tree.root: []}
        for j in self.tree.joints:
            gain, offset = self.tree.mapping[j.name]
            angle = ca.dot(ca.DM(gain), qj) + offset
            T_origin = transform(ca.DM(j.R), ca.DM(j.p))
            if j.kind in {"revolute", "continuous"}:
                T_motion = transform(axis_rotation(j.axis, angle), ca.DM.zeros(3))
                S = ca.vertcat(j.axis, ca.DM.zeros(3))
            elif j.kind == "prismatic":
                T_motion = transform(ca.DM.eye(3), ca.DM(j.axis)*angle)
                S = ca.vertcat(ca.DM.zeros(3), j.axis)
            else:
                T_motion, S = ca.DM.eye(4), ca.DM.zeros(6)
            Trel = T_origin @ T_motion
            poses[j.child] = poses[j.parent] @ Trel
            ancestors[j.child] = ancestors[j.parent]+[j]
            axes[j.name] = poses[j.parent][:3, :3] @ ca.DM(j.R) @ ca.DM(j.axis)
            X = adjoint(inverse(Trel))
            Xs[j.name] = X
            Vj = S * ca.dot(ca.DM(gain), vj)
            velocities[j.child] = X @ velocities[j.parent] + Vj
            accelerations[j.child] = (
                X @ accelerations[j.parent] + S * ca.dot(ca.DM(gain), aj)
                + ad(velocities[j.child]) @ Vj)
        for name, link in self.tree.links.items():
            I = spatial_inertia(link.mass, link.com, link.inertia)
            V = velocities[name]
            forces[name] = I @ accelerations[name] - ad(V).T @ I @ V
        tau_j = ca.SX.zeros(self.n)
        for j in reversed(self.tree.joints):
            S = (ca.vertcat(j.axis, ca.DM.zeros(3)) if j.kind in {"revolute", "continuous"}
                 else ca.vertcat(ca.DM.zeros(3), j.axis) if j.kind == "prismatic"
                 else ca.DM.zeros(6))
            tau_j += ca.DM(self.tree.mapping[j.name][0]) * ca.dot(S, forces[j.child])
            forces[j.parent] += Xs[j.name].T @ forces[j.child]
        tau = ca.vertcat(forces[self.tree.root], tau_j) if floating_base else tau_j
        M = ca.jacobian(tau, a)
        h = ca.substitute(tau, a, ca.SX.zeros(self.nv))
        G = ca.substitute(h, v, ca.SX.zeros(self.nv))
        c = h-G
        # Fixed-base: Christoffel factorization. Floating-base: valid quadratic
        # bias factorization, without claiming a coordinate Christoffel property.
        C = .5 * ca.jacobian(c, v)
        self.mass = ca.Function("mass", [q], [M])
        self.gravity = ca.Function("gravity", [q], [G])
        self.bias = ca.Function("bias", [q, v], [h])
        self.coriolis = ca.Function("coriolis", [q, v], [C])
        self.inverse_dynamics = ca.Function("inverse_dynamics", [q, v, a], [tau])
        self._fk, self._jac = {}, {}
        for name, T in poses.items():
            Jp = ca.jacobian(T[:3, 3], qj) if not floating_base else ca.jacobian(T[:3, 3], q)[:, 7:]
            Jw = ca.SX.zeros(3, self.n)
            for j in ancestors[name]:
                if j.kind in {"revolute", "continuous"}:
                    Jw += axes[j.name] @ ca.DM(self.tree.mapping[j.name][0]).T
            if floating_base:
                R0 = T0[:3, :3]
                Jp = ca.horzcat(-skew(T[:3, 3]-T0[:3, 3])@R0, R0, Jp)
                Jw = ca.horzcat(R0, ca.DM.zeros(3, 3), Jw)
            clean = re.sub(r"[^a-zA-Z0-9_]", "_", name)
            self._fk[name] = ca.Function("fk_"+clean, [q], [T])
            matrices = {
                "mixed": ca.vertcat(T[:3, :3].T@Jw, Jp),
                "body": ca.vertcat(T[:3, :3].T@Jw, T[:3, :3].T@Jp),
                "world_aligned": ca.vertcat(Jw, Jp),
            }
            for mode, J in matrices.items():
                self._jac[name, mode] = ca.Function("J_"+clean+"_"+mode, [q], [J])

    def fk(self, q, frame):
        return self._fk[frame](q)

    def jacobian(self, q, frame, convention="mixed"):
        return self._jac[frame, convention](q)

    def jacobian_dot(self, q, v, frame, convention="mixed"):
        J = self._jac[frame, convention](self.q)
        qdot = self.configuration_derivative(self.q, self.v)
        dJ = ca.jtimes(J, self.q, qdot)
        return ca.Function("jacobian_dot", [self.q, self.v], [dJ])(q, v)

    def configuration_derivative(self, q, v):
        if not self.floating_base:
            return v
        xyz, w, omega = q[3:6], q[6], v[:3]
        dq_xyz = .5 * (w*omega + ca.cross(xyz, omega))
        dq_w = -.5 * ca.dot(xyz, omega)
        return ca.vertcat(quat_rotation(q[3:7]) @ v[3:6], dq_xyz, dq_w, v[6:])

    def export(self, directory, *, frames=None):
        directory = Path(directory)
        directory.mkdir(parents=True, exist_ok=True)
        functions = [self.mass, self.coriolis, self.gravity, self.bias, self.inverse_dynamics]
        for frame in (frames or self._fk):
            functions.append(self._fk[frame])
            functions.extend(self._jac[frame, mode] for mode in ("mixed", "body", "world_aligned"))
        for f in functions:
            f.save(str(directory / (f.name()+".casadi")))
        metadata = {
            "urdf_sha256": hashlib.sha256(self.tree.xml.encode()).hexdigest(),
            "floating_base": self.floating_base, "joint_names": self.joint_names,
            "locked_joints": self.tree.locked, "nq": self.nq, "nv": self.nv,
            "velocity_order": "omega_base_body,v_base_body,joints" if self.floating_base else "joints",
            "jacobian_order": "omega_frame_body,pdot_world",
            "wrench_order": "moment_frame_body,force_world",
            "base_pose": self.base_pose.tolist(), "gravity_world": self.gravity_world.tolist(),
            "casadi_version": ca.__version__,
        }
        (directory/"model.json").write_text(json.dumps(metadata, indent=2)+"\n")
        return metadata
