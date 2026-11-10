"""Sec. III-B: box-vertex task-wrench-robust source contact refinement."""

from dataclasses import dataclass

import casadi as ca
import numpy as np

from .contacts import Capability, TaskWrench, map_reaction
from .optimization import equality, solve


@dataclass
class RefinementResult:
    positions: np.ndarray
    rotations: np.ndarray
    rho: float
    reactions: np.ndarray
    objective: float
    primal_residual: float


def refine_contacts(patches, task: TaskWrench, capabilities=None, *,
                    position_radius=.03, normal_radius=.1, rho_min=0.,
                    geometry_weight=0., settings=None):
    m = len(patches)
    caps = capabilities or [Capability() for _ in patches]
    if not m or len(caps) != m:
        raise ValueError("supply one capability per nonempty contact set")
    if position_radius < 0 or normal_radius < 0 or rho_min < 0 or geometry_weight < 0:
        raise ValueError("radii, margin and weights must be nonnegative")
    opti = ca.Opti()
    geometry = [p.instantiate(opti) for p in patches]
    rho = opti.variable()
    opti.subject_to(rho >= rho_min)
    opti.set_initial(rho, rho_min)
    geometry_cost = 0
    for patch, (_, p, R) in zip(patches, geometry):
        p0, R0 = patch.position(patch.initial), patch.rotation(patch.initial)
        # Fixed patches are constants: avoid adding constant Opti constraints.
        if len(patch.initial):
            opti.subject_to(ca.sumsqr(p-p0) <= position_radius**2)
            opti.subject_to(1-ca.dot(R[:, 2], R0[:, 2]) <= normal_radius)
            geometry_cost += ca.sumsqr(p-p0)
    reactions = []
    for vertex in task.vertices:
        lam = [cap.reaction(opti) for cap in caps]
        total = sum(map_reaction(g[1], g[2], f) for g, f in zip(geometry, lam))
        equality(opti, total-task.required(rho, vertex))
        reactions.append(lam)
    objective = -rho + geometry_weight*geometry_cost
    opti.minimize(objective)
    solution, residual = solve(opti, settings)
    return RefinementResult(
        np.array([np.asarray(solution.value(g[1])).reshape(3) for g in geometry]),
        np.array([np.asarray(solution.value(g[2])) for g in geometry]),
        float(solution.value(rho)),
        np.array([[np.asarray(solution.value(f)).reshape(6) for f in v] for v in reactions]),
        float(solution.value(objective)), residual)
