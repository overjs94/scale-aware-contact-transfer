"""URDF-based constrained geometric IK for contact-realization initialization."""

from dataclasses import dataclass

import casadi as ca
import numpy as np

from .lie import rotation_cost
from .optimization import solve


@dataclass
class IKResult:
    q: np.ndarray
    position_error: float
    rotation_cost: float
    primal_residual: float


def solve_ik(model, frame, target_pose, q_initial, *, posture_weight=1e-6,
             collision_margins=None, settings=None):
    if model.floating_base:
        raise ValueError("Choose a base pose or supply whole-body support constraints.")
    opti = ca.Opti()
    q = opti.variable(model.nq)
    opti.subject_to(opti.bounded(model.lower, q, model.upper))
    opti.set_initial(q, q_initial)
    T, Td = model.fk(q, frame), ca.DM(target_pose)
    dp = T[:3, 3]-Td[:3, 3]
    er = rotation_cost(T[:3, :3], Td[:3, :3])
    opti.minimize(ca.sumsqr(dp)+er+posture_weight*ca.sumsqr(q-q_initial))
    if collision_margins is not None:
        opti.subject_to(collision_margins(q) >= 0)
    sol, residual = solve(opti, settings)
    return IKResult(np.asarray(sol.value(q)).reshape(-1),
                    float(np.linalg.norm(sol.value(dp))), float(sol.value(er)), residual)
