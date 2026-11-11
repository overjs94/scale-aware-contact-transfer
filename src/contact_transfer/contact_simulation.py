"""Closed-loop replay in a supplied collision-enabled MuJoCo scene."""

import hashlib
from pathlib import Path

import numpy as np

from .execution import ExecutionTrace, TRACE_CONVENTIONS


def trajectory_reference(result, q_indices, v_indices):
    """Cubic Hermite joint reference from CITO nodes; scalar joints only.

    Indices explicitly select robot joints, excluding free-base/object poses.
    Interpolation is a control reference, not a dynamics/clearance certificate.
    """
    from scipy.interpolate import CubicHermiteSpline
    h = np.asarray(result.dt, float).reshape(-1)
    qidx, vidx = np.asarray(q_indices, int), np.asarray(v_indices, int)
    if (not len(h) or not np.isfinite(h).all() or np.any(h <= 0)
            or qidx.ndim != 1 or vidx.shape != qidx.shape or not len(qidx)
            or len(result.q) != len(h)+1 or len(result.v) != len(h)+1):
        raise ValueError("invalid trajectory nodes or scalar-joint indices")
    time = np.r_[0., np.cumsum(h)]
    spline = CubicHermiteSpline(time, np.asarray(result.q)[:, qidx], np.asarray(result.v)[:, vidx])

    def reference(t):
        if not np.isfinite(t) or t < -1e-10 or t > time[-1]+1e-10:
            raise ValueError("reference time outside planned interval")
        t = np.clip(t, 0, time[-1])
        return spline(t), spline(t, 1), spline(t, 2)

    return reference


class MujocoContactScene:
    """Named scalar joints, direct unit-gear torque motors, dynamic objects.

    Load the calibrated robot/gripper/object/fixture scene from MJCF, including
    collision meshes and exclusions. The dynamics-only exporter is unchanged.
    """
    def __init__(self, scene, joint_names, actuator_names, object_names):
        import mujoco
        self.mj = mujoco
        self.scene = Path(scene).resolve()
        self.model = mujoco.MjModel.from_xml_path(str(self.scene))
        self.data = mujoco.MjData(self.model)
        self.joint_names, self.actuator_names, self.object_names = map(list, (joint_names, actuator_names, object_names))
        if len(self.joint_names) != len(self.actuator_names) or not self.object_names:
            raise ValueError("one actuator per joint and at least one dynamic object required")
        if any(len(names) != len(set(names)) for names in (self.joint_names, self.actuator_names, self.object_names)):
            raise ValueError("scene names must be unique")

        def ids(kind, names):
            result = [mujoco.mj_name2id(self.model, kind, name) for name in names]
            if any(i < 0 for i in result):
                raise ValueError(f"unknown scene names: {names}")
            return np.array(result, dtype=int)

        self.joints = ids(mujoco.mjtObj.mjOBJ_JOINT, self.joint_names)
        self.actuators = ids(mujoco.mjtObj.mjOBJ_ACTUATOR, self.actuator_names)
        self.objects = ids(mujoco.mjtObj.mjOBJ_BODY, self.object_names)
        for joint, actuator in zip(self.joints, self.actuators):
            if int(self.model.jnt_type[joint]) not in (int(mujoco.mjtJoint.mjJNT_HINGE), int(mujoco.mjtJoint.mjJNT_SLIDE)):
                raise ValueError("controlled joints must be scalar")
            if (self.model.actuator_trntype[actuator] != mujoco.mjtTrn.mjTRN_JOINT
                    or self.model.actuator_trnid[actuator, 0] != joint
                    or self.model.actuator_dyntype[actuator] != mujoco.mjtDyn.mjDYN_NONE
                    or self.model.actuator_gaintype[actuator] != mujoco.mjtGain.mjGAIN_FIXED
                    or self.model.actuator_biastype[actuator] != mujoco.mjtBias.mjBIAS_NONE
                    or not np.allclose(self.model.actuator_gear[actuator], [1, 0, 0, 0, 0, 0])
                    or not np.isclose(self.model.actuator_gainprm[actuator, 0], 1)):
                raise ValueError("replay requires direct unit-gear torque motors")
        if set(self.actuators) != set(range(self.model.nu)):
            raise ValueError("all scene actuators must be explicitly controlled")
        for body in self.objects:
            adr = self.model.body_jntadr[body]
            if adr < 0 or self.model.jnt_type[adr] != mujoco.mjtJoint.mjJNT_FREE:
                raise ValueError("objects must have a free joint; replay never prescribes their pose")
        enabled = (self.model.geom_contype != 0) | (self.model.geom_conaffinity != 0)
        if np.count_nonzero(enabled) < 2 and self.model.npair == 0:
            raise ValueError("scene needs collision-enabled geometry")
        if int(self.model.opt.disableflags) & int(mujoco.mjtDisableBit.mjDSBL_CONTACT):
            raise ValueError("scene disables contact dynamics")
        for body in self.objects:
            descendants = {int(body)}
            for child in range(int(body)+1, self.model.nbody):
                if int(self.model.body_parentid[child]) in descendants:
                    descendants.add(child)
            geoms = np.flatnonzero(np.isin(self.model.geom_bodyid, list(descendants)))
            explicit = np.isin(self.model.pair_geom1, geoms) | np.isin(self.model.pair_geom2, geoms)
            if not np.any(enabled[geoms]) and not np.any(explicit):
                raise ValueError("each object needs collision-enabled geometry")
        self.qadr = self.model.jnt_qposadr[self.joints]
        self.vadr = self.model.jnt_dofadr[self.joints]

    def run(self, controller, reference, duration, *, case_id, plan_id, attachments=()):
        """reference(t)->(q_d,v_d,a_d); controller.command returns joint torques.

        attachments contains (time, equality_name, active). Named equality
        constraints are scene-defined grasp models. They model ideal holding,
        not finger closure or verified grasp retention. Start at the scene's
        initial state, never at interpolated optimized object poses.
        """
        mj, model, data = self.mj, self.model, self.data
        dt = float(model.opt.timestep)
        if not np.isfinite(duration) or duration <= 0:
            raise ValueError("duration must be positive and finite")
        steps = round(duration/dt)
        if not np.isfinite(duration) or duration <= 0 or not np.isclose(steps*dt, duration, rtol=0, atol=1e-10):
            raise ValueError("duration must be a positive integer multiple of scene timestep")
        attachments = tuple(attachments)
        events = []
        for t, name, active in attachments:
            eid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_EQUALITY, name)
            if (eid < 0 or not np.isfinite(t) or not 0 <= t < duration
                    or not np.isclose(round(t/dt)*dt, t, rtol=0, atol=1e-10)):
                raise ValueError("attachment event needs a known equality and an in-range step time")
            events.append((round(t/dt), eid, bool(active)))
        mj.mj_resetData(model, data)
        mj.mj_forward(model, data)
        times, qs, refs, poses, torques, contacts = [], [], [], [], [], []
        for k in range(steps+1):
            for event_step, eid, active in events:
                if event_step == k:
                    data.eq_active[eid] = active
            mj.mj_forward(model, data)
            t = k*dt
            desired = tuple(np.asarray(x, float).reshape(len(self.joints)) for x in reference(t))
            if len(desired) != 3 or not all(np.isfinite(x).all() for x in desired):
                raise ValueError("reference must return finite q/v/a")
            q, v = data.qpos[self.qadr].copy(), data.qvel[self.vadr].copy()
            times.append(t); qs.append(q); refs.append(desired[0])
            poses.append(np.c_[data.xpos[self.objects], data.xquat[self.objects][:, [1, 2, 3, 0]]])
            for i in range(data.ncon):
                c = data.contact[i]
                local = np.zeros(6)
                mj.mj_contactForce(model, data, i, local)
                frame = c.frame.reshape(3, 3)
                pair = [mj.mj_id2name(model, mj.mjtObj.mjOBJ_GEOM, int(g)) or f"geom_{g}" for g in c.geom]
                contacts.append((t, pair, c.pos.copy(), frame[0].copy(), float(c.dist),
                                 np.r_[frame.T@local[3:], frame.T@local[:3]]))
            if k == steps:
                break
            tau = np.asarray(controller.command(q, v, *desired), float).reshape(len(self.joints))
            if not np.isfinite(tau).all():
                raise RuntimeError("controller returned nonfinite torque")
            for value, actuator in zip(tau, self.actuators):
                if model.actuator_ctrllimited[actuator] and not model.actuator_ctrlrange[actuator, 0] <= value <= model.actuator_ctrlrange[actuator, 1]:
                    raise RuntimeError("command exceeds scene actuator control range")
                if model.actuator_forcelimited[actuator] and not model.actuator_forcerange[actuator, 0] <= value <= model.actuator_forcerange[actuator, 1]:
                    raise RuntimeError("command exceeds scene actuator force range")
            torques.append(tau.copy())
            data.ctrl[self.actuators] = tau
            mj.mj_step(model, data)
            if not np.isfinite(data.qpos).all() or not np.isfinite(data.qvel).all():
                raise RuntimeError("nonfinite simulation state")
            if any(w.number for w in data.warning):
                raise RuntimeError("MuJoCo reported a simulation warning")
        # The compiled model fingerprint includes included files and mesh data.
        # Keep the source hash too, but do not treat it as an asset bundle hash.
        model_bytes = np.empty(mj.mj_sizeModel(model), dtype=np.uint8)
        mj.mj_saveModel(model, buffer=model_bytes)
        metadata = {**TRACE_CONVENTIONS, "engine": "mujoco", "engine_version": mj.__version__,
                    "case_id": case_id, "plan_id": plan_id, "timestep": dt,
                    "joint_names": self.joint_names, "object_names": self.object_names,
                    "scene_sha256": hashlib.sha256(self.scene.read_bytes()).hexdigest(),
                    "compiled_model_sha256": hashlib.sha256(model_bytes.tobytes()).hexdigest(),
                    "contact_settings": {"integrator": int(model.opt.integrator),
                        "solver": int(model.opt.solver), "iterations": int(model.opt.iterations),
                        "tolerance": float(model.opt.tolerance), "cone": int(model.opt.cone),
                        "disableflags": int(model.opt.disableflags), "enableflags": int(model.opt.enableflags),
                        "gravity": model.opt.gravity.tolist(), "geom_friction": model.geom_friction.tolist(),
                        "geom_solref": model.geom_solref.tolist(), "geom_solimp": model.geom_solimp.tolist()},
                    "attachments": [list(e) for e in attachments],
                    "full_geometry_certified": False}
        log = ExecutionTrace(metadata, np.array(times), np.array(qs), np.array(refs),
                             np.array(torques).reshape(steps, len(self.joints)), np.array(poses))
        if contacts:
            for name, col in (("contact_time", 0), ("contact_pair", 1), ("contact_position", 2),
                              ("contact_normal", 3), ("contact_gap", 4), ("contact_wrench", 5)):
                setattr(log, name, np.array([row[col] for row in contacts]))
        return log.validate()
