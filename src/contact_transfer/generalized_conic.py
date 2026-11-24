"""Frozen-geometry SOCP/QP with block sparse assembly and independent residuals."""
from time import perf_counter

import numpy as np
import scipy.sparse as sp

from .conic_backends import ConicSettings, solve_conic


def encoding(cap):
    """lambda=B*y; L*y<=b; C*y+c belongs to SOC(3)."""
    if hasattr(cap, 'conic_matrices'):
        B, L, b, C = cap.conic_matrices()
        return B, L, b, [(c, np.zeros(3)) for c in C]
    if cap.kind in {'P', 'S'}:
        B = np.vstack((np.zeros((3, 3)), np.eye(3)))
        L = np.array([[0, 0, 1], [0, 0, -1.]])
        adhesion = cap.adhesion_max if cap.kind == 'S' else 0.
        C = np.array([[0, 0, cap.mu], [1, 0, 0], [0, 1, 0.]])
        return B, L, np.array([cap.normal_max, adhesion]), [(C, np.array([cap.mu*adhesion, 0, 0]))]
    B = np.asarray(cap.H)
    n = B.shape[1]
    return B, np.r_[np.eye(n), -np.eye(n)], np.tile(cap.bounds, 2), []


def fixed_generalized_margin(maps, task, capabilities, *, torque_gravity=None,
                             torque_maps=None, effort=None, rho_min=0., tolerance=2e-6,
                             fixed_rho=None, torque_weight=None, witness_weight=1e-6,
                             solver_settings=None):
    started = perf_counter()
    settings = solver_settings or ConicSettings()
    caps = tuple(capabilities)
    enc = [encoding(c) for c in caps]
    if len(maps) != len(enc):
        raise ValueError('One generalized map per interface required')
    if any(np.asarray(M).shape!=(len(task.nominal),6) or not np.isfinite(M).all() for M in maps):
        raise ValueError('Generalized maps need finite (nv,6) matrices')
    if not np.isfinite([rho_min, tolerance, witness_weight]).all() or rho_min < 0 or tolerance <= 0 or witness_weight < 0:
        raise ValueError('Invalid margin, residual tolerance or witness weight')
    if fixed_rho is not None and (not np.isfinite(fixed_rho) or fixed_rho < 0):
        raise ValueError('fixed_rho must be finite and nonnegative')
    offsets = np.cumsum([0] + [B.shape[1] for B, _, _, _ in enc])
    nlocal, nv, vertices = int(offsets[-1]), len(task.nominal), task.vertices
    count = len(vertices)
    ntotal = 1 + count*nlocal
    eye = sp.eye(count, format='csc')
    wrench = np.hstack([np.asarray(M)@B for M, (B, _, _, _) in zip(maps, enc)]) if enc else np.zeros((nv, 0))
    E = sp.hstack([sp.csc_matrix(-(vertices@np.asarray(task.D).T@np.asarray(task.E).T).reshape(-1, 1)),
                   sp.kron(eye, sp.csc_matrix(wrench), format='csc')], format='csc')
    rhs = np.tile(task.nominal, count)
    if fixed_rho is not None:
        E = sp.vstack([E, sp.csc_matrix(([1.], ([0], [0])), shape=(1, ntotal))], format='csc')
        rhs = np.r_[rhs, fixed_rho]
    local_L = sp.block_diag([sp.csc_matrix(L) for _, L, _, _ in enc], format='csc') if enc else sp.csc_matrix((0, 0))
    local_b = np.concatenate([b for _, _, b, _ in enc]) if enc else np.zeros(0)
    Tlocal = None
    if torque_gravity is not None:
        g, limit = np.asarray(torque_gravity), np.asarray(effort)
        if torque_maps is None or len(torque_maps) != len(enc):
            raise ValueError('One torque map per interface required')
        Tlocal = np.hstack([np.asarray(T)@entry[0] for T, entry in zip(torque_maps, enc)]) if enc else np.zeros((len(g), 0))
        local_L = sp.vstack([local_L, sp.csc_matrix(-Tlocal), sp.csc_matrix(Tlocal)], format='csc')
        local_b = np.r_[local_b, limit-g, limit+g]
    L = sp.hstack([sp.csc_matrix((count*len(local_b), 1)), sp.kron(eye, local_L, format='csc')], format='csc')
    L = sp.vstack([sp.csc_matrix(([-1.], ([0], [0])), shape=(1, ntotal)), L], format='csc')
    linear_b = np.r_[-rho_min, np.tile(local_b, count)]
    local_S = sp.block_diag([
        sp.vstack([sp.csc_matrix(-C) for C, _ in cones], format='csc')
        if cones else sp.csc_matrix((0, B.shape[1])) for B, _, _, cones in enc], format='csc') if enc else sp.csc_matrix((0, 0))
    constants = np.concatenate([c for _, _, _, cones in enc for _, c in cones]) if local_S.shape[0] else np.zeros(0)
    S = sp.hstack([sp.csc_matrix((count*len(constants), 1)), sp.kron(eye, local_S, format='csc')], format='csc')
    A = sp.vstack([E, L, S], format='csc')
    b = np.r_[rhs, linear_b, np.tile(constants, count)]
    q = np.zeros(ntotal)
    q[0] = -1 if fixed_rho is None else 0
    Qtorque = sp.csc_matrix((ntotal, ntotal))
    if torque_weight is not None:
        if Tlocal is None:
            raise ValueError('Torque objective requires torque maps and gravity')
        Qt = np.asarray(torque_weight)
        if Qt.shape!=(Tlocal.shape[0],Tlocal.shape[0]) or not np.isfinite(Qt).all() or not np.allclose(Qt,Qt.T) or np.min(np.linalg.eigvalsh(Qt))<-1e-12:
            raise ValueError('Torque weight must be finite symmetric positive semidefinite')
        Qtorque = sp.block_diag([sp.csc_matrix((1, 1)), sp.kron(eye, 2*Tlocal.T@Qt@Tlocal)], format='csc')
        q[1:] = np.tile(-2*Tlocal.T@Qt@np.asarray(torque_gravity), count)
    soc_count = S.shape[0]//3
    assembled = perf_counter()
    attempts = []
    schedule = [(1e-9, 0.), (1e-8, 0.), (1e-8, 1e-10)] if fixed_rho is None else [(1e-9, 2*witness_weight), (1e-8, 2*witness_weight)]
    for optimality_tolerance, regularization in schedule:
        penalty = np.concatenate([np.full(entry[0].shape[1], regularization if torque_weight is None or cap.kind == 'G' else 0.)
                                  for cap, entry in zip(caps, enc)]) if enc else np.zeros(0)
        P = sp.diags(np.r_[0., np.tile(penalty, count)], format='csc') + Qtorque
        x, status, timing = solve_conic(P, q, A, b, equality_rows=E.shape[0], linear_rows=L.shape[0],
                                         soc_dimensions=[3]*soc_count, settings=settings, tolerance=optimality_tolerance)
        attempts.append(dict(status=status, witness_regularization=regularization, **timing))
        if status in {'Solved', 'AlmostSolved', 'PrimalInfeasible', 'DualInfeasible'}:
            break
    solved = perf_counter()
    slack = b-A@x
    residual = max(float(np.max(abs(slack[:E.shape[0]]), initial=0.)),
                   float(max(0., -np.min(slack[E.shape[0]:E.shape[0]+L.shape[0]], initial=0.))))
    if soc_count:
        cones = slack[E.shape[0]+L.shape[0]:].reshape(-1, 3)
        residual = max(residual, float(np.max(np.linalg.norm(cones[:, 1:], axis=1)-cones[:, 0], initial=0.)))
    coordinates = tuple(tuple(x[1+k*nlocal+offsets[i]:1+k*nlocal+offsets[i+1]] for i in range(len(enc))) for k in range(count))
    reactions = np.array([[entry[0]@u for entry, u in zip(enc, row)] for row in coordinates])
    return dict(rho=float(x[0]), primal_residual=residual, status=status,
                certificate=status in {'Solved', 'AlmostSolved'} and residual < tolerance,
                reactions=reactions, coordinates=coordinates, solver_attempts=attempts,
                backend=settings.backend, fixed_rho_feasibility=fixed_rho is not None,
                timings=dict(assembly_s=assembled-started, solve_s=solved-assembled,
                             validation_s=perf_counter()-solved, total_s=perf_counter()-started))
