"""Articulated-arm fixed-contact path with optimized dynamic reaction allocation.

IK supplies the joint path; IPOPT optimizes the reactions and derived torques.
This reduced planner is explicitly distinct from full contact-implicit q search.
"""

from dataclasses import replace
import casadi as ca
import numpy as np
from scipy.spatial.transform import Rotation

from .contacts import TaskWrench, contact_basis, gravity_wrench, map_reaction
from .conic import fixed_wrench_margin
from .optimization import SolverSettings, equality, solve
from .realization import Arm, RealizationResult
from .system import MechanicalSystem, RigidObject
from .trajectory import TrajectoryResult


def plan_fixed_contact_path(case, settings=None):
    from .contracted import ContractedPlan, numerical_pose_ik, pose_matrix
    d, robots = case.data, case.robots
    obj, cfg = d["object"], d["optimization"]
    system = MechanicalSystem(robots, RigidObject(obj["mass"], np.array(obj["com"]), np.diag(obj["inertia_diagonal"])))
    count = cfg.get("path_samples_per_interval", 16)
    times, poses, object_v, object_a = [], [], [], []
    for interval in range(2):
        first, last = d["keyframes"][interval:interval+2]
        a, b = np.array(first["object_pose"]), np.array(last["object_pose"])
        if not np.allclose(a[3:], b[3:]): raise ValueError("Fixed-contact baseline currently uses fixed object orientation")
        duration = last["time"]-first["time"]
        for j in range(count+1):
            if interval and j == 0: continue
            s = j/count
            f, fd, fdd = 10*s**3-15*s**4+6*s**5, (30*s*s-60*s**3+30*s**4)/duration, (60*s-180*s*s+120*s**3)/duration**2
            times.append(first["time"]+s*duration)
            poses.append(np.r_[a[:3]+f*(b[:3]-a[:3]), a[3:]])
            object_v.append(fd*(b[:3]-a[:3])); object_a.append(fdd*(b[:3]-a[:3]))
    times, poses, object_v, object_a = map(np.array, (times, poses, object_v, object_a))
    Q = np.zeros((len(times), system.nq)); Q[:, system.object_q] = poses
    for i, (robot, spec, prediction, qs) in enumerate(zip(robots, d["arms"], d["contracted_contacts"], system.q_slices)):
        seed = np.array(spec["q_initial"])
        for k, pose in enumerate(poses):
            T = pose_matrix(pose)
            target = np.eye(4); target[:3, :3] = T[:3, :3]@Rotation.from_quat(spec["tool_quaternion_xyzw"]).as_matrix()
            target[:3, 3] = T[:3, 3]+T[:3, :3]@np.array(prediction["point_object"])
            try:
                seed = numerical_pose_ik(robot, spec["tool_frame"], target, seed)
            except RuntimeError as error:
                raise RuntimeError(f"{spec['id']} path node {k}: {error}") from error
            Q[k, qs] = seed
    V = np.zeros((len(times), system.nv))
    for qs, vs in zip(system.q_slices, system.v_slices): V[:, vs] = np.gradient(Q[:, qs], times, axis=0, edge_order=2)
    V[[0, count, -1], :system.object_v.start] = 0.
    Robj = pose_matrix(poses[0])[:3, :3]
    V[:, system.object_v.start+3:] = object_v@Robj
    A = np.gradient(V, times, axis=0, edge_order=2)
    A[:, system.object_v.start+3:] = object_a@Robj
    for robot, vs in zip(robots, system.v_slices):
        if np.any(abs(V[:, vs]) > robot.velocity+1e-8): raise RuntimeError("IK path exceeds URDF joint velocity limits; increase keyframe duration")
    positions = [c["point_object"] for c in d["contracted_contacts"]]
    rotations = [contact_basis(c["normal_object"]) for c in d["contracted_contacts"]]
    keyframes = []
    for node in (0, count, len(times)-1):
        arms = [Arm(r, s["tool_frame"], Q[node, qs]) for r, s, qs in zip(robots, d["arms"], system.q_slices)]
        task = TaskWrench(gravity_wrench(obj["mass"], obj["com"], Robj), np.eye(6), np.diag(cfg["disturbance_scale"]))
        margin = fixed_wrench_margin(positions, rotations, task, case.capabilities, arms=arms, q=[Q[node, qs] for qs in system.q_slices], assignment=range(3), object_pose=pose_matrix(poses[node]),rho_min=cfg["rho_min"])
        keyframes.append(RealizationResult(tuple(Q[node, qs].copy() for qs in system.q_slices), np.array(positions), np.array(rotations), margin.rho,
                                          margin.reactions, margin.torques, (0, 1, 2), tuple(c.kind for c in case.capabilities), -margin.rho, margin.primal_residual))
    opti = ca.Opti(); all_lam, all_tau, objective = [], [], 0.
    for k, h in enumerate(np.diff(times)):
        lam = [cap.reaction(opti, initial=keyframes[0].reactions.mean(axis=0)[i]) for i, cap in enumerate(case.capabilities)]
        required = gravity_wrench(obj["mass"], obj["com"], Robj)-np.r_[np.cross(obj["com"], obj["mass"]*(Robj.T@object_a[k])), obj["mass"]*(Robj.T@object_a[k])]
        total = sum(map_reaction(p, R, f) for p, R, f in zip(positions, rotations, lam))
        equality(opti, total-required)
        torques=[]
        for r, spec, qs, vs, Rc, reaction in zip(robots, d["arms"], system.q_slices, system.v_slices, rotations, lam):
            tau = ca.DM(r.inverse_dynamics(Q[k, qs], V[k, vs], A[k, vs]))-ca.DM(r.jacobian(Q[k, qs], spec["tool_frame"])[3:].T@Robj@Rc)@reaction[3:]
            opti.subject_to(opti.bounded(-r.effort,tau,r.effort)); torques.append(tau)
            objective += h*(1e-4*ca.sumsqr(tau)+1e-3*ca.sumsqr(reaction))
        all_tau.append(ca.vertcat(*torques)); all_lam.append(lam)
    opti.minimize(objective)
    selected = settings or SolverSettings()
    selected = replace(selected, options={k:v for k,v in selected.options.items() if k != "ipopt.hessian_approximation"})
    sol, residual = solve(opti, selected)
    dt = np.diff(times)
    impulses = {c["group_id"]:np.array([np.asarray(sol.value(lam[i])).reshape(6)[3:] if cap.kind=="P" else np.asarray(sol.value(lam[i])).reshape(6) for lam in all_lam])*dt[:, None]
                for i,(c,cap) in enumerate(zip(d["contracted_contacts"],case.capabilities))}
    tau = np.array([np.asarray(sol.value(t)).reshape(-1) for t in all_tau])
    gap, velocity = 0., 0.
    for k in range(len(times)):
        for robot, spec, c, qs, vs in zip(robots,d["arms"],d["contracted_contacts"],system.q_slices,system.v_slices):
            p = robot.fk(Q[k,qs],spec["tool_frame"])[:3,3]
            target=poses[k,:3]+Robj@np.array(c["point_object"])
            gap=max(gap,float(np.linalg.norm(p-target)))
            velocity=max(velocity,float(np.linalg.norm(robot.jacobian(Q[k,qs],spec["tool_frame"])[3:]@V[k,vs]-object_v[k])))
    trajectory=TrajectoryResult(Q,V,tau,dt,impulses,float(sol.value(objective)),residual,0.,0.,-gap,False,(),velocity)
    transitions=[]
    for a,b in ((0,count),(count,len(dt))):
        transitions.append(TrajectoryResult(Q[a:b+1],V[a:b+1],tau[a:b],dt[a:b],{n:x[a:b] for n,x in impulses.items()},0.,residual,0.,0.,-gap,False,(),velocity))
    return ContractedPlan(case,system,tuple(keyframes),tuple(transitions),trajectory)
