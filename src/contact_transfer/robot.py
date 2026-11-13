"""Fast numerical Pinocchio model with angular-first/mixed API adapters."""

from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pinocchio as pin

from .urdf import URDFTree


class Robot:
    """URDF numerical model.

    Fixed-base q/v contain independent joints in tree order. Floating-base
    q=[p_world, quaternion_xyzw, joints], v=[omega_base_body, v_base_body, joints].
    Frame Jacobians return [omega_frame_body; pdot_world], at the frame origin.
    Mimic/locked joints are mapped through an exact constant tangent matrix B.
    """

    def __init__(self, urdf, *, floating_base=False, locked_joints=None,
                 base_pose=None, gravity=(0, 0, -9.81)):
        self.tree = URDFTree(urdf, locked_joints)
        self.path = Path(urdf).resolve()
        self.floating_base = bool(floating_base)
        self.n = self.tree.n
        self.nq = self.n + (7 if floating_base else 0)
        self.nv = self.n + (6 if floating_base else 0)
        self.base_pose = np.eye(4) if base_pose is None else np.asarray(base_pose, float)
        if self.base_pose.shape != (4, 4) or not np.allclose(self.base_pose[3], [0, 0, 0, 1]):
            raise ValueError("base_pose must be a homogeneous transform")
        Rbase = self.base_pose[:3, :3]
        if not np.allclose(Rbase.T@Rbase, np.eye(3), atol=1e-8) or not np.isclose(np.linalg.det(Rbase), 1.):
            raise ValueError("base rotation must belong to SO(3)")
        if floating_base and not np.allclose(self.base_pose, np.eye(4)):
            raise ValueError("For a floating base, specify its world pose in q.")
        self.gravity_world = np.asarray(gravity, float).reshape(3)
        # Expand mimic explicitly so this works across Pinocchio versions.
        xml_root = ET.fromstring(self.tree.xml)
        for j in xml_root.findall("joint"):
            for m in j.findall("mimic"):
                j.remove(m)
        xml = ET.tostring(xml_root, encoding="unicode")
        self.model = (pin.buildModelFromXML(xml, pin.JointModelFreeFlyer())
                      if floating_base else pin.buildModelFromXML(xml))
        self.model.gravity.linear = (
            self.gravity_world if floating_base else
            self.base_pose[:3, :3].T @ self.gravity_world)
        self.data = self.model.createData()
        self.B = np.zeros((self.model.nv, self.nv))
        if floating_base:
            self.B[:3, 3:6] = np.eye(3)
            self.B[3:6, :3] = np.eye(3)
        off = 6 if floating_base else 0
        for j in self.tree.joints:
            if j.kind != "fixed":
                pj = self.model.joints[self.model.getJointId(j.name)]
                if pj.nv != 1:
                    raise ValueError(f"Unsupported multi-DoF URDF joint: {j.name}")
                self.B[pj.idx_v, off:] = self.tree.mapping[j.name][0]
        self.joint_names = self.tree.joint_names
        self.lower, self.upper = self.tree.lower.copy(), self.tree.upper.copy()
        self.effort, self.velocity = self.tree.effort.copy(), self.tree.velocity.copy()

    def neutral(self):
        joints = self.tree.neutral()
        return np.r_[0., 0., 0., 0., 0., 0., 1., joints] if self.floating_base else joints

    def _qpin(self, q):
        q = np.asarray(q, float).reshape(self.nq)
        if not np.isfinite(q).all():
            raise ValueError("q must be finite")
        out = pin.neutral(self.model)
        if self.floating_base:
            if not np.isclose(np.linalg.norm(q[3:7]), 1., atol=1e-7):
                raise ValueError("floating-base quaternion must be normalized")
            out[:7] = q[:7]
        qj = q[7:] if self.floating_base else q
        for j in self.tree.joints:
            if j.kind == "fixed":
                continue
            pj = self.model.joints[self.model.getJointId(j.name)]
            a, b = self.tree.mapping[j.name]
            angle = float(a @ qj + b)
            if pj.nq == 2:
                out[pj.idx_q:pj.idx_q+2] = [np.cos(angle), np.sin(angle)]
            else:
                out[pj.idx_q] = angle
        return out

    def _vpin(self, v):
        return self.B @ np.asarray(v, float).reshape(self.nv)

    def mass(self, q):
        return self.B.T @ pin.crba(self.model, self.data, self._qpin(q)) @ self.B

    def coriolis(self, q, v):
        return self.B.T @ pin.computeCoriolisMatrix(
            self.model, self.data, self._qpin(q), self._vpin(v)) @ self.B

    def gravity(self, q):
        return self.B.T @ pin.computeGeneralizedGravity(self.model, self.data, self._qpin(q))

    def bias(self, q, v):
        return self.B.T @ pin.nonLinearEffects(
            self.model, self.data, self._qpin(q), self._vpin(v))

    def inverse_dynamics(self, q, v, a):
        return self.B.T @ pin.rnea(self.model, self.data, self._qpin(q),
                                  self._vpin(v), self._vpin(a))

    def forward_dynamics(self, q, v, tau):
        # Reduced mass solve also handles mimic/locked joints exactly.
        return np.linalg.solve(self.mass(q), np.asarray(tau)-self.bias(q, v))

    def _frame(self, name):
        if not self.model.existFrame(name):
            raise ValueError(f"unknown frame: {name}")
        return self.model.getFrameId(name)

    def fk(self, q, frame):
        pin.framesForwardKinematics(self.model, self.data, self._qpin(q))
        T = self.data.oMf[self._frame(frame)].homogeneous.copy()
        return T if self.floating_base else self.base_pose @ T

    def jacobian(self, q, frame, convention="mixed"):
        qp = self._qpin(q)
        J = np.array(pin.computeFrameJacobian(self.model, self.data, qp,
                    self._frame(frame), pin.LOCAL_WORLD_ALIGNED)).reshape(6,self.model.nv) @ self.B
        if not self.floating_base:
            J = np.vstack((self.base_pose[:3, :3] @ J[:3],
                           self.base_pose[:3, :3] @ J[3:]))
        R = self.fk(q, frame)[:3, :3]
        if convention == "mixed":
            return np.vstack((R.T @ J[3:], J[:3]))
        if convention == "body":
            return np.vstack((R.T @ J[3:], R.T @ J[:3]))
        if convention == "world_aligned":
            return np.vstack((J[3:], J[:3]))
        raise ValueError("convention must be mixed, body or world_aligned")

    def integrate(self, q, delta):
        qp = pin.integrate(self.model, self._qpin(q), self._vpin(delta))
        joint_values = []
        for name in self.joint_names:
            j = self.model.joints[self.model.getJointId(name)]
            joint_values.append(np.arctan2(qp[j.idx_q+1], qp[j.idx_q])
                                if j.nq == 2 else qp[j.idx_q])
        return np.r_[qp[:7], joint_values] if self.floating_base else np.array(joint_values)

    def symbolic(self):
        from .symbolic import SymbolicRobot
        return SymbolicRobot(self.path, floating_base=self.floating_base,
                             locked_joints=self.tree.locked, base_pose=self.base_pose,
                             gravity=self.gravity_world)
