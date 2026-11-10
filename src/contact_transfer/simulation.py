"""MuJoCo model export and headless controller evaluation.

Bundled URDFs are dynamics-only. Small visual markers below have collision
disabled; they never substitute for full-geometry contact validation.
"""

from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation

from .lie import axis_rotation


def _text(v):
    return " ".join(f"{x:.16g}" for x in np.asarray(v).reshape(-1))


def _quat(R):
    q = Rotation.from_matrix(R).as_quat()
    return np.r_[q[3], q[:3]]


def export_mjcf(robot, *, timestep=.002, markers=True):
    if any(j.mimic is not None and np.linalg.norm(robot.tree.mapping[j.name][0]) > 0
           for j in robot.tree.joints):
        raise ValueError("Lock mimic joints for this dynamics-only MuJoCo exporter.")
    root = ET.Element("mujoco", model=robot.tree.name)
    ET.SubElement(root, "compiler", angle="radian", inertiafromgeom="false", fusestatic="false")
    ET.SubElement(root, "option", timestep=str(timestep), gravity=_text(robot.gravity_world),
                  integrator="implicitfast")
    world = ET.SubElement(root, "worldbody")
    base = ET.SubElement(world, "body", name=robot.tree.root,
                         pos=_text(robot.base_pose[:3, 3]), quat=_text(_quat(robot.base_pose[:3, :3])))
    if robot.floating_base:
        ET.SubElement(base, "freejoint", name="floating_base")
    bodies = {robot.tree.root: base}
    for j in robot.tree.joints:
        gain, angle = robot.tree.mapping[j.name]
        R, p = j.R.copy(), j.p.copy()
        if not np.any(gain):
            if j.kind in {"revolute", "continuous"}:
                R = R@np.asarray(axis_rotation(j.axis, angle))
            elif j.kind == "prismatic":
                p += j.R@j.axis*angle
        body = ET.SubElement(bodies[j.parent], "body", name=j.child,
                             pos=_text(p), quat=_text(_quat(R)))
        bodies[j.child] = body
        if np.any(gain):
            attrs = {"name": j.name, "type": "slide" if j.kind == "prismatic" else "hinge",
                     "axis": _text(j.axis), "damping": "0", "armature": "0"}
            if np.isfinite(j.lower) and np.isfinite(j.upper):
                attrs.update(limited="true", range=_text([j.lower, j.upper]))
            else:
                attrs["limited"] = "false"
            ET.SubElement(body, "joint", **attrs)
    for name, link in robot.tree.links.items():
        if link.mass > 0:
            I = link.inertia
            ET.SubElement(bodies[name], "inertial", pos=_text(link.com), mass=str(link.mass),
                          fullinertia=_text([I[0, 0], I[1, 1], I[2, 2], I[0, 1], I[0, 2], I[1, 2]]))
        if markers and link.mass > 0:
            ET.SubElement(bodies[name], "geom", type="sphere", size=".025", pos=_text(link.com),
                          rgba=".2 .5 .8 1", contype="0", conaffinity="0", mass="0")
    actuators = ET.SubElement(root, "actuator")
    for name, effort in zip(robot.joint_names, robot.effort):
        attrs = {"name": "motor_"+name, "joint": name, "gear": "1"}
        if np.isfinite(effort):
            attrs.update(ctrllimited="true", ctrlrange=_text([-effort, effort]))
        ET.SubElement(actuators, "motor", **attrs)
    return ET.tostring(root, encoding="unicode")


class MujocoRobot:
    def __init__(self, robot, *, timestep=.002):
        import mujoco
        self.mj, self.robot = mujoco, robot
        self.xml = export_mjcf(robot, timestep=timestep)
        self.model = mujoco.MjModel.from_xml_string(self.xml)
        self.data = mujoco.MjData(self.model)
        self.joint_ids = [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, n)
                          for n in robot.joint_names]
        self.actuator_ids = [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, "motor_"+n)
                            for n in robot.joint_names]

    def reset(self, q, v=None):
        q, v = np.asarray(q), np.zeros(self.robot.nv) if v is None else np.asarray(v)
        self.mj.mj_resetData(self.model, self.data)
        if self.robot.floating_base:
            self.data.qpos[:3] = q[:3]
            self.data.qpos[3:7] = np.r_[q[6], q[3:6]]
            R = Rotation.from_quat(q[3:7]).as_matrix()
            self.data.qvel[:3], self.data.qvel[3:6] = R@v[3:6], v[:3]
        qo, vo = (7, 6) if self.robot.floating_base else (0, 0)
        for i, jid in enumerate(self.joint_ids):
            self.data.qpos[self.model.jnt_qposadr[jid]] = q[qo+i]
            self.data.qvel[self.model.jnt_dofadr[jid]] = v[vo+i]
        self.mj.mj_forward(self.model, self.data)

    def state(self):
        qj = np.array([self.data.qpos[self.model.jnt_qposadr[j]] for j in self.joint_ids])
        vj = np.array([self.data.qvel[self.model.jnt_dofadr[j]] for j in self.joint_ids])
        if not self.robot.floating_base:
            return qj, vj
        quat = np.r_[self.data.qpos[4:7], self.data.qpos[3]]
        R = Rotation.from_quat(quat).as_matrix()
        return (np.r_[self.data.qpos[:3], quat, qj],
                np.r_[self.data.qvel[3:6], R.T@self.data.qvel[:3], vj])

    def step(self, torque):
        self.data.ctrl[self.actuator_ids] = np.asarray(torque)
        self.mj.mj_step(self.model, self.data)
        return self.state()

    def run(self, controller, reference, steps):
        """reference(t)->(q_d,v_d,a_d); returns time,q,v,tau arrays."""
        records = []
        for _ in range(steps):
            q, v = self.state()
            desired = reference(float(self.data.time))
            tau = controller.command(q, v, *desired)
            records.append((float(self.data.time), q.copy(), v.copy(), tau.copy()))
            self.step(tau)
        return {"time": np.array([x[0] for x in records]),
                "q": np.array([x[1] for x in records]),
                "v": np.array([x[2] for x in records]),
                "tau": np.array([x[3] for x in records])}
