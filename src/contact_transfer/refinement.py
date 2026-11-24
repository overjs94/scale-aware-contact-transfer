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
                    geometry_weight=0., reaction_maps=None, settings=None,
                    robustness_enabled=True, coordinate_groups=None,
                    equilibrium_scale=None, normalize_trust_region=False,
                    initial_witness=None, point_force_coordinates='cartesian'):
    m = len(patches)
    if point_force_coordinates not in {'cartesian','scaled_tangent'}:raise ValueError('Unknown source point force coordinates')
    caps = capabilities or [Capability() for _ in patches]
    if not m or len(caps) != m:
        raise ValueError("supply one capability per nonempty contact set")
    if reaction_maps is not None and len(reaction_maps) != m:
        raise ValueError('One object/link reaction map per contact required')
    if position_radius < 0 or normal_radius < 0 or rho_min < 0 or geometry_weight < 0:
        raise ValueError("radii, margin and weights must be nonnegative")
    if not robustness_enabled and (rho_min>0 or geometry_weight<=0):
        raise ValueError('Nominal-only ablation needs rho_min=0 and a positive common geometry weight')
    balance_scale=np.ones(len(task.nominal)) if equilibrium_scale is None else np.asarray(equilibrium_scale,float)
    if balance_scale.shape!=np.shape(task.nominal) or not np.isfinite(balance_scale).all() or np.min(balance_scale)<=0:
        raise ValueError('Positive finite per-row equilibrium scales required')
    opti = ca.Opti()
    # Optional physical sites can share a group's planar translation. Every
    # site still has its own unilateral force; no moment capability is invented.
    # The ordinary one-patch/one-coordinate Eq. (7) path is unchanged.
    labels = list(range(m)) if coordinate_groups is None else list(coordinate_groups)
    if len(labels) != m:
        raise ValueError('One coordinate group per physical contact site required')
    geometry, shared, representatives, frozen_groups = [], {}, set(), set()
    frozen=position_radius==0 and normal_radius==0
    for patch, label in zip(patches, labels):
        if label not in shared:
            probe=ca.SX.sym('freeze_dependency',len(patch.initial))
            if frozen and not ca.depends_on(patch.rotation(probe),probe):
                u=ca.DM(patch.initial);instance=(u,patch.position(u),patch.rotation(u))
                frozen_groups.add(label)
            else:instance = patch.instantiate(opti)
            shared[label] = (patch, instance[0])
        else:
            first, u = shared[label]
            if any(not np.array_equal(getattr(patch,n),getattr(first,n)) for n in ('initial','lower','upper')):
                raise ValueError('Shared source charts require identical coordinates and bounds')
            probe=ca.SX.sym('shared_source_chart',len(patch.initial))
            delta=patch.position(probe)-first.position(probe)
            if ca.depends_on(ca.simplify(delta),probe) or ca.depends_on(patch.rotation(probe),probe) or ca.depends_on(first.rotation(probe),probe):
                raise ValueError('Shared sites must differ by fixed translations and have fixed normals')
            instance=(u,patch.position(u),patch.rotation(u))
        geometry.append(instance)
    rho = opti.variable() if robustness_enabled else ca.DM(0.)
    if robustness_enabled:
        opti.subject_to(rho >= rho_min)
        opti.set_initial(rho, rho_min if initial_witness is None else initial_witness['rho'])
    geometry_cost = 0
    for patch, (_, p, R), label in zip(patches, geometry, labels):
        p0, R0 = patch.position(patch.initial), patch.rotation(patch.initial)
        # Fixed patches are constants: avoid adding constant Opti constraints.
        if len(patch.initial) and label not in representatives and label not in frozen_groups:
            probe=ca.SX.sym('normal_dependency',len(patch.initial))
            if ca.depends_on(patch.position(probe),probe):
                if normalize_trust_region and position_radius>0:opti.subject_to(ca.sumsqr((p-p0)/position_radius)<=1.)
                else:opti.subject_to(ca.sumsqr(p-p0) <= position_radius**2)
            if ca.depends_on(patch.rotation(probe)[:,2],probe):
                opti.subject_to(1-ca.dot(R[:, 2], R0[:, 2]) <= normal_radius)
            geometry_cost += ca.sumsqr(p-p0)
        representatives.add(label)
    reactions = []
    vertices=task.vertices if robustness_enabled else [np.zeros(task.E.shape[1])]
    if initial_witness is not None:
        values=np.asarray(initial_witness['reactions'])
        if values.shape!=(len(vertices),m,6) or not np.isfinite(values).all() or not np.isfinite(initial_witness['rho']):raise ValueError('Source warm witness dimensions must match the actual task vertices/sites')
    for k,vertex in enumerate(vertices):
        lam=[]
        for i,cap in enumerate(caps):
            seed=None if initial_witness is None else values[k,i]
            if point_force_coordinates=='scaled_tangent' and cap.kind=='P':
                fn=opti.variable();opti.subject_to(opti.bounded(0.,fn,cap.normal_max))
                opti.set_initial(fn,min(cap.normal_max/2,1.) if seed is None else np.clip(seed[5],0,cap.normal_max))
                if cap.mu:
                    ratio=opti.variable(2);opti.subject_to(ca.sumsqr(ratio)<=1.)
                    opti.set_initial(ratio,np.zeros(2) if seed is None or seed[5]<1e-12 else seed[3:5]/(cap.mu*seed[5]))
                    force=ca.vertcat(cap.mu*fn*ratio,fn)
                else:force=ca.vertcat(ca.DM.zeros(2),fn)
                lam.append(ca.vertcat(ca.DM.zeros(3),force))
            else:lam.append(cap.reaction(opti,initial=seed))
        total = sum((reaction_maps[i](g[1], g[2], f) if reaction_maps is not None
                     else map_reaction(g[1], g[2], f)) for i, (g, f) in enumerate(zip(geometry, lam)))
        equality(opti, (total-task.required(rho, vertex))/ca.DM(balance_scale))
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
