"""Finite adhesive-point suction with acquisition, loss and venting.

This is a pressure-limited compliant seal approximation, not fluid mechanics.
Parallel-jaw grasping and spherical point contacts use MuJoCo contact dynamics.
"""

import numpy as np


class VacuumSeal:
    def __init__(self, tool_body, object_body, point_object, normal_object, capability,
                 parameters, enabled):
        if capability.kind != "S":
            raise ValueError("vacuum requires an S capability")
        self.tool_body, self.object_body = tool_body, object_body
        self.point = np.asarray(point_object, float).reshape(3)
        self.normal = np.asarray(normal_object, float).reshape(3)
        if not np.isfinite(np.r_[self.point, self.normal]).all() or not np.isclose(np.linalg.norm(self.normal), 1):
            raise ValueError("vacuum contact requires finite position and unit normal")
        self.cap, self.parameters, self.enabled = capability, dict(parameters), enabled
        keys = ("stiffness", "damping", "acquire_gap", "break_gap", "seal_radius", "max_angle_rad", "pressure_pa", "area_efficiency")
        if any(not np.isfinite(parameters[k]) or parameters[k] <= 0 for k in keys) or parameters["break_gap"] <= parameters["acquire_gap"] or parameters["area_efficiency"] > 1:
            raise ValueError("invalid vacuum seal parameters")
        self.adhesion = min(capability.adhesion_max, parameters["pressure_pa"]*np.pi*parameters["seal_radius"]**2*parameters["area_efficiency"])
        self.sealed = False
        self.history, self.events = [], []

    def __call__(self, mj, model, data, time):
        tool = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, self.tool_body)
        obj = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, self.object_body)
        if min(tool, obj) < 0:
            raise ValueError("unknown suction tool/object body")
        R = data.xmat[obj].reshape(3, 3)
        point = data.xpos[obj]+R@self.point
        n = R@self.normal
        delta = data.xpos[tool]-point
        gap = float(delta@n)
        lateral = float(np.linalg.norm(delta-gap*n))
        angle = float(np.arccos(np.clip(-data.xmat[tool].reshape(3, 3)[:, 2]@n, -1, 1)))
        p = self.parameters
        previous = self.sealed
        enabled = bool(self.enabled(time))
        if not enabled:
            self.sealed = False
        elif self.sealed:
            self.sealed = (abs(gap) <= p["break_gap"] and lateral <= p["seal_radius"] and angle <= p["max_angle_rad"])
        else:
            self.sealed = (abs(gap) <= p["acquire_gap"] and lateral <= p["seal_radius"]/2 and angle <= p["max_angle_rad"])
        if previous != self.sealed:
            self.events.append({"time": time, "event": "acquired" if self.sealed else "lost" if enabled else "vented"})
        force = np.zeros(3)
        if self.sealed:
            Jtool, Jobj = np.zeros((3, model.nv)), np.zeros((3, model.nv))
            mj.mj_jac(model, data, Jtool, None, data.xpos[tool], tool)
            mj.mj_jac(model, data, Jobj, None, point, obj)
            raw = p["stiffness"]*delta+p["damping"]*((Jtool-Jobj)@data.qvel)
            fn = np.clip(raw@n, -self.cap.normal_max, self.adhesion)
            ft = raw-(raw@n)*n
            limit = self.cap.mu*max(0., self.adhesion-fn)
            ft *= min(1., limit/max(np.linalg.norm(ft), 1e-15))
            force = fn*n+ft
            # Equal/opposite forces at one interface point conserve total
            # force and moment. No object pose or velocity is prescribed.
            mj.mj_applyFT(model, data, force, np.zeros(3), point, obj, data.qfrc_applied)
            mj.mj_applyFT(model, data, -force, np.zeros(3), point, tool, data.qfrc_applied)
        self.history.append((time, enabled, self.sealed, gap, lateral, angle, *force, *point))

    def save(self, path):
        values = np.asarray(self.history, float)
        np.savez_compressed(path, time=values[:, 0], enabled=values[:, 1].astype(bool),
                            sealed=values[:, 2].astype(bool), gap=values[:, 3], lateral=values[:, 4],
                            angle=values[:, 5], force_on_object_world=values[:, 6:9], point_world=values[:, 9:12])
