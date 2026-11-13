"""Three-keyframe/three-arm experiment from assumed network contact groups."""

from dataclasses import dataclass
from importlib.resources import files
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation, Slerp
from scipy.optimize import least_squares

from .contacts import Capability, SurfacePatch, TaskWrench, gravity_wrench
from .contacts import contact_basis
from .conic import fixed_wrench_margin
from .realization import Arm, realize_keyframe
from .robot import Robot
from .system import MechanicalSystem, RigidObject
from .trajectory import Contact, TrajectoryResult, solve_cito


def pose_matrix(pose):
    pose = np.asarray(pose, float)
    if pose.shape != (7,) or not np.isfinite(pose).all() or not np.isclose(np.linalg.norm(pose[3:]), 1, atol=1e-8):
        raise ValueError("object pose must be finite world xyz + unit xyzw")
    T = np.eye(4); T[:3, :3] = Rotation.from_quat(pose[3:]).as_matrix(); T[:3, 3] = pose[:3]
    return T


@dataclass
class ContractedCase:
    path: Path
    data: dict
    robots: tuple
    capabilities: tuple

    @classmethod
    def load(cls, path=None, *, contacts_path=None):
        path = Path(path or str(files("contact_transfer").joinpath("scenarios/three_arm/case.json"))).resolve()
        d = json.loads(path.read_text())
        if contacts_path is not None:
            prediction = json.loads(Path(contacts_path).read_text())
            if prediction.get("schema_version") != 1 or prediction.get("units") != "SI" or prediction.get("contact_frame") != "object":
                raise ValueError("supplied contact predictions must declare schema 1, SI and object coordinates")
            d["contracted_contacts"] = prediction["contracted_contacts"]
            d["prediction_source"] = prediction.get("prediction_source", "supplied_contracted_contacts")
        if (d.get("schema_version") != 1 or d.get("units") != "SI"
                or d.get("contact_frame") != "object" or d.get("quaternion_order") != "xyzw"):
            raise ValueError("unsupported contracted contact schema or coordinates")
        if len(d["arms"]) != 3 or len(d["keyframes"]) != 3 or len(d["contracted_contacts"]) != 3:
            raise ValueError("this experiment requires three arms, three keyframes and three groups")
        family = d.get("model_family", "cartesian")
        if family not in {"cartesian", "franka_panda"}:
            raise ValueError("unsupported experiment model family")
        if family == "franka_panda":
            if d["optimization"].get("planner") != "fixed_contact_path":
                raise ValueError("Panda baseline requires the fixed_contact_path planner; full Panda CITO is not implemented")
            count = d["optimization"].get("path_samples_per_interval", 16)
            if not isinstance(count, int) or not 4 <= count <= 128:
                raise ValueError("fixed path needs 4 to 128 samples per interval")
        ids = [a["id"] for a in d["arms"]]
        groups = [c["group_id"] for c in d["contracted_contacts"]]
        assignments = [c["arm_id"] for c in d["contracted_contacts"]]
        if len(set(ids)) != 3 or len(set(groups)) != 3 or set(assignments) != set(ids):
            raise ValueError("contact groups must have unique IDs and an injective arm assignment")
        d["contracted_contacts"] = sorted(d["contracted_contacts"], key=lambda c: ids.index(c["arm_id"]))
        if family == "franka_panda":
            for arm, contact in zip(d["arms"], d["contracted_contacts"]):
                axis = Rotation.from_quat(arm["tool_quaternion_xyzw"]).as_matrix()[:, 2]
                if not np.isclose(axis@np.asarray(contact["normal_object"]), -1., atol=1e-6):
                    raise ValueError("contact normal is incompatible with the preset tool orientation")
        times = np.array([k["time"] for k in d["keyframes"]], float)
        if not np.isfinite(times).all() or times[0] != 0 or np.any(np.diff(times) <= 0):
            raise ValueError("keyframe times must increase from zero")
        for k in d["keyframes"]:
            pose_matrix(k["object_pose"])
        for c in d["contracted_contacts"]:
            p, n = np.asarray(c["point_object"], float), np.asarray(c["normal_object"], float)
            if p.shape != (3,) or n.shape != (3,) or not np.isfinite(np.r_[p, n]).all() or not np.isclose(np.linalg.norm(n), 1):
                raise ValueError("contact positions and unit outward normals must be finite")
            half = np.asarray(d["object"]["size"])/2
            if np.any(abs(p) > half+1e-8) or not np.any(abs(abs(p)-half) < 1e-8):
                raise ValueError("contracted contact must lie on the panel surface")
            face = np.flatnonzero(abs(abs(p)-half) < 1e-8)
            if not any(np.isclose(n[i], np.sign(p[i]), atol=1e-8) for i in face):
                raise ValueError("normal must point out of the panel surface")
        robots, caps = [], []
        for a in d["arms"]:
            if Path(a["urdf"]).is_absolute():
                raise ValueError("scenario URDF paths must be relative to the case directory")
            urdf = (path.parent/a["urdf"]).resolve()
            if not urdf.is_relative_to(path.parent):
                raise ValueError("scenario URDF must be inside the case directory")
            T = np.eye(4); T[:3, :3] = Rotation.from_euler("xyz", a["base_rpy"]).as_matrix(); T[:3, 3] = a["base_position"]
            robot = Robot(urdf, base_pose=T, locked_joints=a.get("locked_joints"))
            if d.get("model_family", "cartesian") == "cartesian" and (robot.n != 3 or any(j.kind not in {"fixed", "prismatic"} for j in robot.tree.joints)):
                raise ValueError("initial experiment uses three-axis URDF Cartesian arms")
            robots.append(robot)
            spec = a["capability"]
            if spec["kind"] == "G":
                caps.append(Capability.force_gripper(spec["force_bounds"]))
            else:
                caps.append(Capability(**spec))
        if sorted(c.kind for c in caps) != ["G", "P", "S"]:
            raise ValueError("this testbed requires one spherical point, one parallel-jaw and one suction tool")
        vacuum = d["simulation"]["vacuum"]
        physical_limit = vacuum["pressure_pa"]*np.pi*vacuum["seal_radius"]**2*vacuum["area_efficiency"]
        if next(c.adhesion_max for c in caps if c.kind == "S") > physical_limit:
            raise ValueError("suction capability exceeds the pressure-area limit")
        steps = d["optimization"]["steps_per_interval"]
        if not isinstance(steps, int) or steps < 2:
            raise ValueError("at least two intervals per keyframe transition required")
        return cls(path, d, tuple(robots), tuple(caps))


@dataclass
class ContractedPlan:
    case: ContractedCase
    system: MechanicalSystem
    keyframes: tuple
    transitions: tuple
    trajectory: TrajectoryResult


def plan_contracted(case: ContractedCase, *, settings=None):
    """Fixed predicted contact groups; optimize posture/reactions, then CITO.

    No network inference, regrasp search or full shared-variable sequence NLP is
    claimed. All three groups stay at fixed object-frame locations throughout.
    """
    d = case.data
    if d.get("optimization", {}).get("planner") == "fixed_contact_path":
        from .fixed_path import plan_fixed_contact_path
        return plan_fixed_contact_path(case, settings=settings)
    models = [r.symbolic() for r in case.robots]
    obj = d["object"]
    system = MechanicalSystem(models, RigidObject(obj["mass"], np.array(obj["com"]), np.diag(obj["inertia_diagonal"])))
    cfg = d["optimization"]
    patches = [SurfacePatch.fixed(c["point_object"], c["normal_object"]) for c in d["contracted_contacts"]]
    keyframes = []
    for k in d["keyframes"]:
        T = pose_matrix(k["object_pose"])
        arms = []
        for i, (numeric, model, spec, contact) in enumerate(zip(case.robots, models, d["arms"], d["contracted_contacts"])):
            q0 = numeric.neutral()
            target = T[:3, 3]+T[:3, :3]@np.asarray(contact["point_object"])
            fk = numeric.fk(q0, spec["tool_frame"])
            # Exact Cartesian initialization from the URDF Jacobian, without a
            # robot-specific System_Param or dimension table.
            delta = np.linalg.solve(numeric.jacobian(q0, spec["tool_frame"])[3:], target-fk[:3, 3])
            arms.append(Arm(model, spec["tool_frame"], q0+delta))
        task = TaskWrench(gravity_wrench(obj["mass"], obj["com"], T[:3, :3]), np.eye(6), np.diag(cfg["disturbance_scale"]))
        keyframes.append(realize_keyframe(arms, patches, case.capabilities, range(3), T, task,
                                          rho_min=cfg["rho_min"], geometry_weight=0, settings=settings))
    trajectories = []
    steps = cfg["steps_per_interval"]
    for index in range(2):
        left, right = keyframes[index:index+2]
        q0 = np.r_[*left.q, d["keyframes"][index]["object_pose"]]
        q1 = np.r_[*right.q, d["keyframes"][index+1]["object_pose"]]
        contacts = []
        for i, (spec, prediction, cap) in enumerate(zip(d["arms"], d["contracted_contacts"], case.capabilities)):
            contacts.append(Contact(prediction["group_id"], prediction["point_object"], prediction["normal_object"], cap,
                                    robot_index=i, frame=spec["tool_frame"], group_id=prediction["group_id"],
                                    attached_steps=tuple(range(steps)) if cap.kind != "P" else (),
                                    maintained_steps=tuple(range(steps)) if cap.kind == "P" else (),
                                    sticking_steps=tuple(range(steps)) if cap.kind == "P" else ()))
        duration = d["keyframes"][index+1]["time"]-d["keyframes"][index]["time"]
        endpoints = np.array([d["keyframes"][index]["object_pose"], d["keyframes"][index+1]["object_pose"]])
        rotations = Slerp([0, 1], Rotation.from_quat(endpoints[:, 3:]))
        desired = [pose_matrix(np.r_[(1-f)*endpoints[0, :3]+f*endpoints[1, :3], rotations(f).as_quat()]) for f in np.linspace(0, 1, steps+1)]
        initial = _cartesian_initial(case, system, q0, q1, contacts, steps, duration)
        trajectories.append(solve_cito(system, q0, q1, contacts, steps=steps, duration=duration,
                                      desired_object_poses=desired, settings=settings, initial=initial))
    a, b = trajectories
    combined = TrajectoryResult(np.r_[a.q, b.q[1:]], np.r_[a.v, b.v[1:]], np.r_[a.tau, b.tau], np.r_[a.dt, b.dt],
                                {name: np.r_[a.impulses[name], b.impulses[name]] for name in a.impulses},
                                a.objective+b.objective, max(a.primal_residual, b.primal_residual),
                                max(a.complementarity_residual, b.complementarity_residual), max(a.friction_residual, b.friction_residual),
                                min(a.minimum_gap, b.minimum_gap), False, a.relaxations,
                                max(a.gripper_velocity_residual, b.gripper_velocity_residual))
    return ContractedPlan(case, system, tuple(keyframes), tuple(trajectories), combined)


def _cartesian_initial(case, system, start, goal, contacts, steps, duration):
    """A momentum-balanced warm start for this translation-only testbed."""
    if not np.allclose(start[system.object_q][3:], goal[system.object_q][3:]):
        raise ValueError("Cartesian testbed currently supports translated keyframes with fixed object orientation")
    h = duration/steps
    weights = np.r_[0., np.sin(np.pi*np.arange(1, steps)/(steps)), 0.]
    progress = np.cumsum(weights)/weights.sum()
    Q = start+(goal-start)*progress[:, None]
    Robj = pose_matrix(start[system.object_q])[:3, :3]
    V = np.zeros((steps+1, system.nv))
    for k in range(1, steps+1):
        for qs, vs in zip(system.q_slices, system.v_slices):
            V[k, vs] = (Q[k, qs]-Q[k-1, qs])/h
        V[k, system.object_v.start+3:system.object_v.stop] = Robj.T@(Q[k, system.object_q.start:system.object_q.start+3]-Q[k-1, system.object_q.start:system.object_q.start+3])/h
    U, impulses = [], {c.name: [] for c in contacts}
    rotations = [contact_basis(c.normal) for c in contacts]
    for k in range(steps):
        acceleration = (V[k+1]-V[k])/h
        required = -(np.asarray(system.Iobj, float)@acceleration[system.object_v]
                     +np.asarray(system.bias(Q[k], V[k]), float).reshape(-1)[system.object_v])
        task = TaskWrench(required, np.eye(6)[:, 5:], np.eye(1))
        margin = fixed_wrench_margin([c.point_object for c in contacts], rotations, task, case.capabilities)
        lam = margin.reactions.mean(axis=0)
        torque = []
        for j, (robot, spec, cap, c) in enumerate(zip(case.robots, case.data["arms"], case.capabilities, contacts)):
            qs, vs = system.q_slices[j], system.v_slices[j]
            Rwc = Robj@rotations[j]
            reaction = robot.jacobian(Q[k, qs], spec["tool_frame"])[3:].T@(Rwc@lam[j, 3:])
            torque.append(robot.mass(Q[k, qs])@acceleration[vs]+robot.bias(Q[k, qs], V[k, vs])-reaction)
            impulses[c.name].append(h*(lam[j, 3:] if cap.kind == "P" else lam[j]))
        U.append(np.concatenate(torque))
    return TrajectoryResult(Q, V, np.array(U), np.full(steps, h), {k: np.array(v) for k, v in impulses.items()},
                            0., 0., 0., 0., 0., False, ())

def numerical_pose_ik(robot, frame, target, seed):
    seed = np.asarray(seed, float)
    def residual(q):
        T = robot.fk(q, frame)
        return np.r_[10*(T[:3, 3]-target[:3, 3]), Rotation.from_matrix(target[:3, :3].T@T[:3, :3]).as_rotvec(), 1e-5*(q-seed)]
    fit = least_squares(residual, np.clip(seed, robot.lower+1e-5, robot.upper-1e-5),
                        bounds=(robot.lower+1e-6, robot.upper-1e-6), max_nfev=160, ftol=1e-12, xtol=1e-12, gtol=1e-12)
    if np.linalg.norm(residual(fit.x)[:6]) > 2e-5:
        raise RuntimeError(f"IK failed at {frame}: position/orientation target is unreachable with this seed")
    return fit.x
