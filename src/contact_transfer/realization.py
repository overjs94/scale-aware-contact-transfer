"""Sec. III-C: URDF arm/contact realization with vertex torque certificates."""

from dataclasses import dataclass

import casadi as ca
import numpy as np

from .contacts import TaskWrench, map_reaction
from .lie import rotation_cost
from .optimization import equality, solve


@dataclass
class Arm:
    model: object
    frame: str
    q_initial: np.ndarray
    q_reference: object = None
    tool_outward_axis: tuple = (0., 0., 1.)

    def __post_init__(self):
        if self.model.floating_base:
            raise ValueError("Keyframe Arm requires a fixed-base model; floating whole-body constraints use CITO callbacks.")
        if not np.isclose(np.linalg.norm(self.tool_outward_axis), 1):
            raise ValueError("tool_outward_axis must be a unit vector")
        self.q_initial = np.asarray(self.q_initial, float).reshape(self.model.nq)
        self.q_reference = self.q_initial if self.q_reference is None else np.asarray(self.q_reference)


@dataclass
class RealizationResult:
    q: tuple
    positions: np.ndarray
    rotations: np.ndarray
    rho: float
    reactions: np.ndarray
    torques: np.ndarray
    assignment: tuple
    types: tuple
    objective: float
    primal_residual: float


def realize_keyframe(arms, patches, capabilities, assignment, object_pose,
                     task: TaskWrench, *, rho_min=0., characteristic_length=1.,
                     position_radius=.05, normal_radius=.2, geometry_weight=1.,
                     normal_weight=1.,
                     posture_weight=.01, robustness_weight=1., collision_margins=None,
                     settings=None):
    m = len(patches)
    if len(capabilities) != m or len(assignment) != m or len(set(assignment)) != m:
        raise ValueError("one capability and one distinct arm per contact are required")
    if any(i < 0 or i >= len(arms) for i in assignment):
        raise ValueError("invalid arm assignment")
    if characteristic_length <= 0:
        raise ValueError("characteristic length must be positive")
    Tobj = ca.DM(object_pose)
    opti = ca.Opti()
    qs = []
    objective = 0
    for arm in arms:
        q = opti.variable(arm.model.nq)
        opti.subject_to(opti.bounded(arm.model.lower, q, arm.model.upper))
        opti.set_initial(q, arm.q_initial)
        qs.append(q)
        span = np.where(np.isfinite(arm.model.upper-arm.model.lower),
                        arm.model.upper-arm.model.lower, 2*np.pi)
        objective += posture_weight*ca.sumsqr((q-arm.q_reference)/np.maximum(span, 1e-3))
    geometry = [p.instantiate(opti) for p in patches]
    for i, (patch, (_, p, R)) in enumerate(zip(patches, geometry)):
        arm = arms[assignment[i]]
        Tee = arm.model.fk(qs[assignment[i]], arm.frame)
        world_position = Tobj[:3, 3]+Tobj[:3, :3]@p
        opti.subject_to(Tee[:3, 3] == world_position)
        tool_n = Tee[:3, :3]@ca.DM(arm.tool_outward_axis)
        object_n = Tobj[:3, :3]@R[:, 2]
        # Toleranced normal alignment avoids a redundant 3-equality SO(3) constraint.
        opti.subject_to(1+ca.dot(tool_n, object_n) <= 1e-6)
        p0, R0 = patch.position(patch.initial), patch.rotation(patch.initial)
        if len(patch.initial):
            opti.subject_to(ca.sumsqr(p-p0) <= position_radius**2)
            opti.subject_to(1-ca.dot(R[:, 2], R0[:, 2]) <= normal_radius)
        objective += geometry_weight*(ca.sumsqr(p-p0)/characteristic_length**2
                                     +normal_weight*(1-ca.dot(R[:, 2], R0[:, 2])))
        # Moment capability must be accompanied by matching orientation closure.
        cap = capabilities[i]
        if cap.has_moment:
            desired = Tobj[:3, :3]@R@ca.diag(ca.DM([1, -1, -1]))
            opti.subject_to(rotation_cost(Tee[:3, :3], desired) <= 1e-7)
    if collision_margins is not None:
        opti.subject_to(collision_margins(ca.vertcat(*qs)) >= 0)
    rho = opti.variable()
    opti.subject_to(rho >= rho_min)
    opti.set_initial(rho, rho_min)
    objective -= robustness_weight*rho
    all_lam, all_tau = [], []
    for vertex in task.vertices:
        lam = [cap.reaction(opti) for cap in capabilities]
        total = sum(map_reaction(g[1], g[2], f) for g, f in zip(geometry, lam))
        equality(opti, total-task.required(rho, vertex))
        torques = []
        for j, arm in enumerate(arms):
            tau = arm.model.gravity(qs[j])
            for i in range(m):
                if assignment[i] != j:
                    continue
                Tee = arm.model.fk(qs[j], arm.frame)
                Rwc = Tobj[:3, :3]@geometry[i][2]
                dual = ca.vertcat(Tee[:3, :3].T@Rwc@lam[i][:3], Rwc@lam[i][3:])
                tau -= arm.model.jacobian(qs[j], arm.frame).T@dual
            opti.subject_to(opti.bounded(-arm.model.effort, tau, arm.model.effort))
            torques.append(tau)
        all_lam.append(lam)
        all_tau.append(ca.vertcat(*torques))
    opti.minimize(objective)
    sol, residual = solve(opti, settings)
    return RealizationResult(
        tuple(np.asarray(sol.value(q)).reshape(-1) for q in qs),
        np.array([np.asarray(sol.value(g[1])).reshape(3) for g in geometry]),
        np.array([np.asarray(sol.value(g[2])) for g in geometry]),
        float(sol.value(rho)),
        np.array([[np.asarray(sol.value(f)).reshape(6) for f in v] for v in all_lam]),
        np.array([np.asarray(sol.value(t)).reshape(-1) for t in all_tau]),
        tuple(assignment), tuple(c.kind for c in capabilities),
        float(sol.value(objective)), residual)
