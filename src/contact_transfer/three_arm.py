"""Executable contracted-contact optimization and physical three-tool replay."""

import hashlib
import json
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation

from .contracted import ContractedCase, plan_contracted, numerical_pose_ik
from .contacts import contact_basis
from .contact_simulation import MujocoContactScene, trajectory_reference
from .robot import Robot
from .simulation import _text
from .tool_physics import VacuumSeal
from .urdf_scene import append_urdf_robot


def build_three_arm_scene(plan, destination):
    d, cfg = plan.case.data, plan.case.data["simulation"]
    root = ET.Element("mujoco", model="three_arm_contracted_contact")
    ET.SubElement(root, "compiler", angle="radian", inertiafromgeom="false", fusestatic="false")
    ET.SubElement(root, "option", timestep=str(cfg["timestep"]), gravity="0 0 -9.81", integrator="implicitfast",
                  cone="elliptic", solver="Newton", iterations="100", tolerance="1e-10")
    world, actuators, contact = ET.SubElement(root, "worldbody"), ET.SubElement(root, "actuator"), ET.SubElement(root, "contact")
    assets = ET.SubElement(root, "asset")
    franka = d.get("model_family") == "franka_panda"
    mesh_files = None
    if franka:
        mesh_cache = Path(d.get("mesh_cache", str(Path.cwd()/".cache/panda_meshes"))).resolve()
        mesh_files = {p.name: p for p in mesh_cache.glob("*.stl")}
        if len(mesh_files) < 8:
            raise ValueError("Panda collision meshes are missing. Run python tools/fetch_panda_meshes.py first.")
    full = []
    for original, spec, cap in zip(plan.case.robots, d["arms"], plan.case.capabilities):
        robot = Robot(original.path, base_pose=original.base_pose)
        full.append(robot)
        prefix = spec["id"]+"_"
        collisions = append_urdf_robot(world, actuators, robot, prefix, mesh_files=mesh_files, assets=assets, robot_collisions=franka)
        if franka:
            srdf = ET.parse(plan.case.path.parent/"panda.srdf")
            for pair in srdf.findall("disable_collisions"):
                a, b = pair.get("link1"), pair.get("link2")
                if a in robot.tree.links and b in robot.tree.links:
                    ET.SubElement(contact, "exclude", body1=prefix+("urdf_"+a if a == robot.tree.root else a),
                                  body2=prefix+("urdf_"+b if b == robot.tree.root else b))
        for geom in collisions:
            if franka and not any(geom.startswith(prefix+name+"_collision_") for name in spec["tool_collision_links"]):
                continue
            mu = cap.mu if cap.kind != "G" else .9
            ET.SubElement(contact, "pair", geom1=geom, geom2="panel_collision", condim="3",
                          friction=_text([mu, mu, 0, 0, 0]), solref=".006 1", solimp=".95 .99 .001")
    obj, start = d["object"], d["keyframes"][0]["object_pose"]
    panel = ET.SubElement(world, "body", name="panel", pos=_text(start[:3]), quat=_text([start[6], *start[3:6]]))
    ET.SubElement(panel, "freejoint", name="panel_free")
    ET.SubElement(panel, "inertial", pos=_text(obj["com"]), mass=str(obj["mass"]), diaginertia=_text(obj["inertia_diagonal"]))
    ET.SubElement(panel, "geom", name="panel_collision", type="box", size=_text(np.array(obj["size"])/2), mass="0",
                  contype="1", conaffinity="3", rgba=".85 .65 .35 1", condim="3", friction=".8 0 0", solref=".006 1")
    top = cfg["fixture_top"]
    thickness = cfg["fixture_thickness"]
    ET.SubElement(world, "geom", name="receiver", type="box", size=_text([*cfg["fixture_half_size"], thickness/2]),
                  pos=_text([0., 0., top-thickness/2]), contype="1", conaffinity="3", rgba=".25 .3 .35 1")
    ET.SubElement(world, "geom", name="floor", type="plane", size="2 2 .1", contype="1", conaffinity="3", rgba=".15 .18 .22 1")
    ET.indent(root)
    destination = Path(destination)
    destination.write_bytes(ET.tostring(root, encoding="utf-8", xml_declaration=True))
    return tuple(full)


class ThreeArmController:
    def __init__(self, plan, full_robots):
        self.plan, self.robots = plan, full_robots
        self.cfg = plan.case.data["simulation"]
        self.node_time = np.r_[0., np.cumsum(plan.trajectory.dt)]
        self.prepare = self.cfg["preparation_seconds"]
        self.release = float(round(self.prepare+self.node_time[-1]+self.cfg["hold_seconds"], 12))
        self.duration = float(round(self.release+(self.cfg["release_seconds"] if self.cfg.get("release_tools", True) else 0), 12))
        if not self.cfg.get("release_tools", True):
            self.release = self.duration+1.
        self.refs = [trajectory_reference(plan.trajectory, np.arange(s.start, s.stop), np.arange(v.start, v.stop))
                     for s, v in zip(plan.system.q_slices, plan.system.v_slices)]
        self.time = 0.
        self.retracted = []
        if plan.case.data.get("model_family") == "franka_panda" and self.cfg.get("release_tools", True):
            for robot, spec, qs in zip(plan.case.robots, plan.case.data["arms"], plan.system.q_slices):
                last = plan.trajectory.q[-1, qs]
                target = robot.fk(last, spec["tool_frame"]).copy()
                target[:3, 3] += spec["retract_world"]
                self.retracted.append(numerical_pose_ik(robot, spec["tool_frame"], target, last))

    @staticmethod
    def blend(t):
        t = np.clip(t, 0., 1.)
        return t*t*(3-2*t)

    def reference(self, time):
        self.time = time
        t = np.clip(time-self.prepare, 0., self.node_time[-1])
        positions, velocities, accelerations = [], [], []
        for i, (robot, reduced, spec, ref) in enumerate(zip(self.robots, self.plan.case.robots, self.plan.case.data["arms"], self.refs)):
            qp, vp, ap = ref(t)
            if time < self.prepare or time > self.prepare+self.node_time[-1]:
                vp, ap = np.zeros_like(vp), np.zeros_like(ap)
            if time >= self.release:
                if self.retracted:
                    qp = qp+(self.retracted[i]-qp)*self.blend((time-self.release)/.5)
                else:
                    direction = np.asarray(spec["retract_world"])
                    J = reduced.jacobian(qp, spec["tool_frame"])[3:]
                    qp = qp+np.linalg.solve(J, direction)*self.blend((time-self.release)/.35)
            q, v, a = np.zeros(robot.nq), np.zeros(robot.nv), np.zeros(robot.nv)
            for j, name in enumerate(robot.joint_names):
                if name in reduced.joint_names:
                    k = reduced.joint_names.index(name); q[j], v[j], a[j] = qp[k], vp[k], ap[k]
                else:
                    closing = self.blend(time/max(self.prepare*.6, 1e-6))
                    opening = self.blend((time-self.release)/.15)
                    q[j] = self.cfg["finger_open"]+(self.cfg["finger_closed_command"]-self.cfg["finger_open"])*closing*(1-opening)
            positions.append(q); velocities.append(v); accelerations.append(a)
        return np.concatenate(positions), np.concatenate(velocities), np.concatenate(accelerations)

    def command(self, q, v, desired, velocity, acceleration):
        index = np.clip(np.searchsorted(self.node_time, self.time-self.prepare, side="right")-1, 0, len(self.plan.trajectory.tau)-1)
        outputs, offset = [], 0
        for i, (robot, reduced) in enumerate(zip(self.robots, self.plan.case.robots)):
            sl = slice(offset, offset+robot.n); offset += robot.n
            if self.cfg.get("controller_type") == "computed_torque":
                feedback = self.cfg["arm_kp"]*(desired[sl]-q[sl])+self.cfg["arm_kd"]*(velocity[sl]-v[sl])
                tau = robot.inverse_dynamics(q[sl], v[sl], acceleration[sl]+feedback)
                if self.time < self.release:
                    prediction = self.plan.case.data["contracted_contacts"][i]
                    impulse = self.plan.trajectory.impulses[prediction["group_id"]][index]
                    force = impulse[-3:]/self.plan.trajectory.dt[index]
                    Robj = Rotation.from_quat(self.plan.trajectory.q[index, self.plan.system.object_q][3:]).as_matrix()
                    Rwc = Robj@contact_basis(prediction["normal_object"])
                    tau -= robot.jacobian(q[sl], self.plan.case.data["arms"][i]["tool_frame"])[3:].T@(Rwc@force)
                for j, name in enumerate(robot.joint_names):
                    if name not in reduced.joint_names:
                        tau[j] = robot.gravity(q[sl])[j]+self.cfg["finger_kp"]*(desired[sl][j]-q[sl][j])+self.cfg["finger_kd"]*(velocity[sl][j]-v[sl][j])
                outputs.append(np.clip(tau, -robot.effort, robot.effort))
                continue
            tau = robot.inverse_dynamics(q[sl], v[sl], np.zeros(robot.n))
            for j, name in enumerate(robot.joint_names):
                if name in reduced.joint_names:
                    k = reduced.joint_names.index(name)
                    if self.time < self.release:
                        tau[j] = self.plan.trajectory.tau[index, self.plan.system.v_slices[i].start+k]
                    tau[j] += self.cfg["arm_kp"]*(desired[sl][j]-q[sl][j])+self.cfg["arm_kd"]*(velocity[sl][j]-v[sl][j])
                else:
                    tau[j] += self.cfg["finger_kp"]*(desired[sl][j]-q[sl][j])+self.cfg["finger_kd"]*(velocity[sl][j]-v[sl][j])
            outputs.append(np.clip(tau, -robot.effort, robot.effort))
        return np.concatenate(outputs)


def run_three_arm(output="outputs/three_arm", *, case_path=None, contacts_path=None, settings=None, plan=None, progress=None):
    output = Path(output); output.mkdir(parents=True, exist_ok=True)
    if plan is None:
        case = ContractedCase.load(case_path, contacts_path=contacts_path)
        if settings is None and case.data.get("model_family") == "franka_panda":
            from .optimization import SolverSettings
            settings = SolverSettings(max_iterations=600, options={"expand": False, "ipopt.hessian_approximation": "limited-memory"})
        plan = plan_contracted(case, settings=settings)
    d = plan.case.data
    case_bytes = (json.dumps(d, indent=2, allow_nan=False)+"\n").encode()
    (output/"case.json").write_bytes(case_bytes)
    bundle = hashlib.sha256(case_bytes)
    for robot, spec in zip(plan.case.robots, d["arms"]):
        destination = output/spec["urdf"]; destination.parent.mkdir(parents=True, exist_ok=True)
        if destination.resolve() != robot.path.resolve():
            shutil.copyfile(robot.path, destination)
        bundle.update(spec["urdf"].encode()); bundle.update(robot.path.read_bytes())
    for name in ("panda.srdf", "provenance.json", "LICENSE.upstream"):
        source = plan.case.path.parent/name
        if source.exists() and source.resolve() != (output/name).resolve():
            shutil.copyfile(source, output/name)
    full = build_three_arm_scene(plan, output/"scene.xml")
    if progress: progress("simulation")
    controller = ThreeArmController(plan, full)
    joint_names = [spec["id"]+"_"+name for spec, robot in zip(d["arms"], full) for name in robot.joint_names]
    motor_names = [spec["id"]+"_motor_"+name for spec, robot in zip(d["arms"], full) for name in robot.joint_names]
    scene = MujocoContactScene(output/"scene.xml", joint_names, motor_names, ["panel"])
    suction = next(i for i, c in enumerate(plan.case.capabilities) if c.kind == "S")
    spec, prediction = d["arms"][suction], d["contracted_contacts"][suction]
    seal = VacuumSeal(spec["id"]+"_"+spec["tool_frame"], "panel", prediction["point_object"], prediction["normal_object"],
                      plan.case.capabilities[suction], d["simulation"]["vacuum"], lambda t: t < controller.release)
    case_id = bundle.hexdigest()
    plan_id = hashlib.sha256(plan.trajectory.q.tobytes()+plan.trajectory.v.tobytes()+plan.trajectory.tau.tobytes()+plan.trajectory.dt.tobytes()).hexdigest()
    trace = scene.run(controller, controller.reference, controller.duration, case_id=case_id, plan_id=plan_id,
                      initial_q=controller.reference(0.)[0], external_forces=seal)
    trace.metadata.update(external_force_model="pressure_limited_compliant_adhesive_point", seal_events=seal.events,
                          prediction_source=d["prediction_source"], tool_types=[c.kind for c in plan.case.capabilities],
                          release_time=float(controller.release), model_scope=d.get("model_family", "synthetic_URDF_Cartesian_tool_testbed"))
    trace.metadata.update(vacuum_parameters=dict(d["simulation"]["vacuum"]), vacuum_adhesion_limit_N=seal.adhesion)
    trace.save(output/"execution.npz"); seal.save(output/"vacuum.npz")
    np.savez_compressed(output/"plan.npz", q=plan.trajectory.q, v=plan.trajectory.v, tau=plan.trajectory.tau,
                        dt=plan.trajectory.dt, **{"impulse_"+k: v for k, v in plan.trajectory.impulses.items()})
    errors = []
    for k in d["keyframes"]:
        index = int(np.argmin(abs(trace.time-(controller.prepare+k["time"]))))
        desired, actual = np.array(k["object_pose"]), trace.object_pose[index, 0]
        errors.append({"id": k["id"], "time": float(trace.time[index]),
                       "translation_m": float(np.linalg.norm(actual[:3]-desired[:3])),
                       "rotation_rad": float((Rotation.from_quat(desired[3:]).inv()*Rotation.from_quat(actual[3:])).magnitude())})
    before_release = np.asarray(seal.history)[:, 0] < controller.release
    local_wrenches, force_checks = measured_tool_wrenches(plan, trace, seal, controller.prepare, min(controller.release, controller.duration))
    unexpected = {}
    if d.get("model_family") == "franka_panda":
        for pair, gap in zip(trace.contact_pair, trace.contact_gap):
            other = next((g for g in pair if g != "panel_collision"), "")
            expected = "panel_collision" in pair and any(other.startswith(a["id"]+"_"+link+"_collision_") for a in d["arms"] for link in a["tool_collision_links"])
            if not expected and gap < -.0002:
                key = " / ".join(pair)
                unexpected[key] = min(unexpected.get(key, 0.), float(gap))
    np.savez_compressed(output/"tool_wrenches.npz", time=trace.time, local_reactions=local_wrenches,
                        group_ids=np.array([c["group_id"] for c in d["contracted_contacts"]]))
    report = {"model_family": d.get("model_family", "cartesian"), "planner": d["optimization"].get("planner", "cito"),
              "prediction_source": d["prediction_source"], "network_inference_executed": False, "arms": 3, "keyframes": 3,
              "solver_backend": "ipopt" if settings is None else settings.backend,
              "unexpected_penetrations_m": unexpected, "sampled_collision_check_passed": not unexpected,
              "complementarity_applicable": d["optimization"].get("planner") != "fixed_contact_path",
              "mujoco_version": trace.metadata["engine_version"], "timestep": trace.metadata["timestep"],
              "tool_types": [c.kind for c in plan.case.capabilities], "keyframe_rho": [k.rho for k in plan.keyframes],
              "primal_residual": plan.trajectory.primal_residual, "complementarity_residual": plan.trajectory.complementarity_residual,
              "friction_residual": plan.trajectory.friction_residual, "gripper_velocity_residual": plan.trajectory.gripper_velocity_residual,
              "keyframe_errors": errors, "tracking_passed": all(e["translation_m"] < d["simulation"]["tracking_tolerance_m"] and e["rotation_rad"] < d["simulation"]["tracking_tolerance_rad"] for e in errors),
              "seal_events": seal.events, "seal_retained_before_vent": bool(np.asarray(seal.history)[before_release, 2].all()),
              "vacuum_adhesion_limit_N": seal.adhesion, "contact_samples": len(trace.contact_time),
              "tool_force_checks": force_checks,
              "force_envelopes_passed": all(c["maximum_violation_N"] < .05 for c in force_checks),
              "final_object_position": trace.object_pose[-1, 0, :3].tolist(), "full_geometry_certified": False,
              "equality_constraints_used": int(scene.model.neq),
              "limitations": ["assumed network predictions; no learned model inference",
                              "IK-prescribed joint path with optimized forces/torques; not full CITO" if d.get("model_family") == "franka_panda" else "translation-only Cartesian arm prototype",
                              "assumed synthetic gripper capability; no hardware calibration", "force-only planning does not certify measured finite-gripper moments",
                              "pressure-limited point seal, not cup/fluid FEM", "preparation is scripted; final pose is held" if not d["simulation"].get("release_tools", True) else "preparation and post-task release are scripted, not optimized placement"]}
    (output/"report.json").write_text(json.dumps(report, indent=2, allow_nan=False)+"\n")
    return plan, trace, seal, report


def measured_tool_wrenches(plan, trace, seal, start, stop):
    """Object-on-tool reactions about each predicted interface, in local axes.

    Sum physical contacts and vacuum traction. The planned force-only models
    do not certify the measured resultant moments of finite tool geometry.
    """
    d = plan.case.data
    world = np.zeros((len(trace.time), 3, 6))
    Robj = Rotation.from_quat(trace.object_pose[:, 0, 3:]).as_matrix()
    references = np.array([trace.object_pose[:, 0, :3]+np.einsum("nij,j->ni", Robj, c["point_object"])
                           for c in d["contracted_contacts"]]).transpose(1, 0, 2)
    dt = trace.metadata["timestep"]
    for t, pair, position, wrench in zip(trace.contact_time, trace.contact_pair, trace.contact_position, trace.contact_wrench):
        if "panel_collision" not in pair:
            continue
        other = pair[0] if pair[1] == "panel_collision" else pair[1]
        sign = -1 if pair[1] == "panel_collision" else 1
        k = int(round(t/dt))
        for i, spec in enumerate(d["arms"]):
            if other.startswith(spec["id"]+"_"):
                w = sign*wrench
                world[k, i] += np.r_[w[:3]+np.cross(position-references[k, i], w[3:]), w[3:]]
    vacuum = next(i for i, c in enumerate(plan.case.capabilities) if c.kind == "S")
    history = np.array(seal.history)
    world[:, vacuum, 3:] -= history[:, 6:9]
    world[:, vacuum, :3] += np.cross(history[:, 9:12]-references[:, vacuum], -history[:, 6:9])
    local = np.zeros_like(world)
    checks = []
    active = (trace.time >= start) & (trace.time < stop)
    for i, (contact, cap) in enumerate(zip(d["contracted_contacts"], plan.case.capabilities)):
        R = Robj@contact_basis(contact["normal_object"])
        local[:, i, :3] = np.einsum("nij,ni->nj", R, world[:, i, :3])
        local[:, i, 3:] = np.einsum("nij,ni->nj", R, world[:, i, 3:])
        f = local[active, i, 3:]
        if cap.kind == "G":
            violation = np.max(abs(f)-np.asarray(cap.bounds)[-3:])
        else:
            adhesion = cap.adhesion_max if cap.kind == "S" else 0.
            violation = np.max(np.c_[-f[:, 2]-adhesion, f[:, 2]-cap.normal_max,
                                     np.linalg.norm(f[:, :2], axis=1)-cap.mu*(f[:, 2]+adhesion)])
        checks.append({"group_id": contact["group_id"], "kind": cap.kind, "window_seconds": [float(start), float(stop)],
                       "maximum_violation_N": float(max(0., violation)), "force_min_N": f.min(0).tolist(), "force_max_N": f.max(0).tolist(),
                       "peak_resultant_moment_Nm": float(np.linalg.norm(local[active, i, :3], axis=1).max()),
                       "moment_capacity_certified": False})
    return local, checks
