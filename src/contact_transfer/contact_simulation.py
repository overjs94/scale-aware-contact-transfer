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
    """Controlled scalar joints and free or explicitly named passive objects.

    Load a robot/gripper/object/fixture scene from MJCF, including
    collision meshes and exclusions. The dynamics-only exporter is unchanged.
    """
    def __init__(self, scene, joint_names, actuator_names, object_names, *, actuator_modes=None, articulated_object_joints=None):
        import mujoco
        self.mj = mujoco
        self.scene = Path(scene).resolve()
        self.model = mujoco.MjModel.from_xml_path(str(self.scene))
        self.data = mujoco.MjData(self.model)
        self.joint_names, self.actuator_names, self.object_names = map(list, (joint_names, actuator_names, object_names))
        self.actuator_modes = list(['torque']*len(self.joint_names) if actuator_modes is None else actuator_modes)
        if len(self.actuator_modes)!=len(self.joint_names) or any(m not in {'torque','position'} for m in self.actuator_modes):
            raise ValueError('one torque/position actuator mode per joint required')
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
        for joint, actuator, mode in zip(self.joints, self.actuators, self.actuator_modes):
            if int(self.model.jnt_type[joint]) not in (int(mujoco.mjtJoint.mjJNT_HINGE), int(mujoco.mjtJoint.mjJNT_SLIDE)):
                raise ValueError("controlled joints must be scalar")
            if (self.model.actuator_trntype[actuator] != mujoco.mjtTrn.mjTRN_JOINT
                    or self.model.actuator_trnid[actuator, 0] != joint
                    or self.model.actuator_dyntype[actuator] != mujoco.mjtDyn.mjDYN_NONE
                    or self.model.actuator_gaintype[actuator] != mujoco.mjtGain.mjGAIN_FIXED
                    or not np.allclose(self.model.actuator_gear[actuator], [1, 0, 0, 0, 0, 0])):
                raise ValueError("replay requires direct unit-gear joint actuators")
            if mode=='torque':
                if self.model.actuator_biastype[actuator]!=mujoco.mjtBias.mjBIAS_NONE or not np.isclose(self.model.actuator_gainprm[actuator,0],1):
                    raise ValueError('torque mode requires unit fixed gain and no bias')
            else:
                kp=self.model.actuator_gainprm[actuator,0];bias=self.model.actuator_biasprm[actuator]
                if not np.isfinite([kp,*bias[:3]]).all() or kp<=0 or self.model.actuator_biastype[actuator]!=mujoco.mjtBias.mjBIAS_AFFINE or not np.isclose(bias[0],0) or not np.isclose(bias[1],-kp) or bias[2]>0:
                    raise ValueError('position mode requires a positive PD position actuator')
        if set(self.actuators) != set(range(self.model.nu)):
            raise ValueError("all scene actuators must be explicitly controlled")
        articulated_object_joints=dict(articulated_object_joints or {})
        if set(articulated_object_joints)-set(self.object_names):raise ValueError('Articulated mapping must name logged objects')
        self.observed_names=[];self.observed_ids=[]
        for name,body in zip(self.object_names,self.objects):
            adr = self.model.body_jntadr[body]
            if adr>=0 and self.model.jnt_type[adr]==mujoco.mjtJoint.mjJNT_FREE:continue
            if name not in articulated_object_joints:raise ValueError('objects must have a free joint or an explicit passive articulated joint')
            joint=int(ids(mujoco.mjtObj.mjOBJ_JOINT,[articulated_object_joints[name]])[0])
            if self.model.jnt_bodyid[joint]!=body or joint in self.joints or int(self.model.jnt_type[joint]) not in [int(mujoco.mjtJoint.mjJNT_HINGE),int(mujoco.mjtJoint.mjJNT_SLIDE)]:raise ValueError('Articulated object joint must be passive, scalar and on its logged body')
            if np.any(self.model.actuator_trnid[:,0]==joint):raise ValueError('Articulated object joint must not have an actuator')
            self.observed_names.append(articulated_object_joints[name]);self.observed_ids.append(joint)
        self.observed_ids=np.array(self.observed_ids,int)
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

    def run(self, controller, reference, duration, *, case_id, plan_id, attachments=(),
            initial_q=None, initial_articulated_q=None, external_forces=None, progress=None):
        """reference(t)->(q_d,v_d,a_d); controller.command returns torque (Nm) or position (rad), per joint mode.

        attachments contains (time, equality_name, active). Named equality
        constraints are scene-defined grasp models. They model ideal holding,
        not finger closure or verified grasp retention. Start at the scene's
        initial state, never at interpolated optimized object poses.

        initial_q and initial_articulated_q set controlled/passive scalar joints
        once after reset, before stepping; they are never object references. Optional
        external_forces(mj, model, data, t) may add generalized/body forces after
        the runner clears applied-force arrays each step; it must not prescribe
        object state. Stateful force models are owned/reset by the caller.
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
        if initial_q is not None:
            initial_q = np.asarray(initial_q, float).reshape(len(self.joints))
            if not np.isfinite(initial_q).all():
                raise ValueError("initial_q must be finite")
            data.qpos[self.qadr] = initial_q
        for name,value in dict(initial_articulated_q or {}).items():
            if name not in self.observed_names or not np.isfinite(value):raise ValueError('Initial articulated state must name an observed passive joint')
            jid=int(self.observed_ids[self.observed_names.index(name)])
            if model.jnt_limited[jid] and not model.jnt_range[jid,0]<=value<=model.jnt_range[jid,1]:raise ValueError('Initial articulated coordinate outside joint limits')
            data.qpos[model.jnt_qposadr[jid]]=value
        mj.mj_forward(model, data)
        times, qs, refs, poses, torques, contacts = [], [], [], [], [], []
        observed_q,observed_v=[],[]
        for k in range(steps+1):
            for event_step, eid, active in events:
                if event_step == k:
                    data.eq_active[eid] = active
            mj.mj_forward(model, data)
            t = k*dt
            if progress is not None and (k % max(1, round(1/dt)) == 0 or k == steps):
                progress(t, duration)
            data.qfrc_applied[:] = 0
            data.xfrc_applied[:] = 0
            if external_forces is not None:
                external_forces(mj, model, data, t)
                mj.mj_forward(model, data)
            desired = tuple(np.asarray(x, float).reshape(len(self.joints)) for x in reference(t))
            if len(desired) != 3 or not all(np.isfinite(x).all() for x in desired):
                raise ValueError("reference must return finite q/v/a")
            q, v = data.qpos[self.qadr].copy(), data.qvel[self.vadr].copy()
            times.append(t); qs.append(q); refs.append(desired[0])
            if len(self.observed_ids):
                observed_q.append(data.qpos[model.jnt_qposadr[self.observed_ids]].tolist());observed_v.append(data.qvel[model.jnt_dofadr[self.observed_ids]].tolist())
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
            if hasattr(controller, 'observe'):
                # Read-only force/state observation precedes the next command.
                # No reference or observer is allowed to prescribe object poses.
                controller.observe(mj, model, data, t)
            tau = np.asarray(controller.command(q, v, *desired), float).reshape(len(self.joints))
            if not np.isfinite(tau).all():
                raise RuntimeError("controller returned nonfinite command")
            for value, actuator, mode in zip(tau, self.actuators, self.actuator_modes):
                if model.actuator_ctrllimited[actuator] and not model.actuator_ctrlrange[actuator, 0] <= value <= model.actuator_ctrlrange[actuator, 1]:
                    raise RuntimeError("command exceeds scene actuator control range")
                if mode=='torque' and model.actuator_forcelimited[actuator] and not model.actuator_forcerange[actuator, 0] <= value <= model.actuator_forcerange[actuator, 1]:
                    raise RuntimeError("command exceeds scene actuator force range")
            data.ctrl[self.actuators] = tau
            mj.mj_step(model, data)
            applied=tau.copy()
            for j,mode in enumerate(self.actuator_modes):
                if mode=='position':applied[j]=data.actuator_force[self.actuators[j]]
            torques.append(applied)
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
                        "geom_solref": model.geom_solref.tolist(), "pair_friction":model.pair_friction.tolist(), "pair_solref":model.pair_solref.tolist(), "pair_solimp":model.pair_solimp.tolist(), "pair_dim":model.pair_dim.tolist(), "geom_solimp": model.geom_solimp.tolist()},
                    "attachments": [list(e) for e in attachments],
                    "actuator_modes": self.actuator_modes,
                    "full_geometry_certified": False}
        if len(self.observed_ids):metadata['articulated_state']=dict(joint_names=self.observed_names,q=observed_q,v=observed_v)
        log = ExecutionTrace(metadata, np.array(times), np.array(qs), np.array(refs),
                             np.array(torques).reshape(steps, len(self.joints)), np.array(poses))
        if contacts:
            for name, col in (("contact_time", 0), ("contact_pair", 1), ("contact_position", 2),
                              ("contact_normal", 3), ("contact_gap", 4), ("contact_wrench", 5)):
                setattr(log, name, np.array([row[col] for row in contacts]))
        return log.validate()
