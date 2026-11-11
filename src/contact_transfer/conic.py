"""Independent SOCP wrench margin for frozen III-B/III-C geometry.

Uses Clarabel directly; no NLP linearization or friction pyramid. Reactions
remain object-on-agent, [moment; force], in each contact's local axes.
"""

from dataclasses import dataclass
from time import perf_counter

import numpy as np
from scipy import sparse

from .contacts import map_reaction


@dataclass
class WrenchMarginResult:
    rho: float
    reactions: np.ndarray
    torques: np.ndarray
    primal_residual: float
    status: str
    solve_seconds: float


def fixed_wrench_margin(positions, rotations, task, capabilities, *, arms=(),
                        q=(), assignment=(), object_pose=None, rho_min=0.,
                        tolerance=1e-8, acceptance_tolerance=2e-6):
    """Maximize the box margin, holding contact geometry and arm poses fixed.

    Optional arms/q/assignment impose the same gravity-minus-J.T-reaction
    torque bounds as realize_keyframe. Geometry/closure/collisions are not
    certified here. Supply already realized poses, not posture guesses.
    """
    try:
        import clarabel
    except ImportError as error:
        raise ImportError('Install the conic extra: pip install ".[conic]"') from error
    p, R = np.asarray(positions, float), np.asarray(rotations, float)
    caps, arms = tuple(capabilities), tuple(arms)
    m = len(caps)
    if not m or p.shape != (m, 3) or R.shape != (m, 3, 3):
        raise ValueError("one position and rotation per nonempty contact set required")
    if (not np.isfinite(p).all() or not np.isfinite(R).all()
            or not np.allclose(R.transpose(0, 2, 1)@R, np.eye(3), atol=1e-8)
            or not np.allclose(np.linalg.det(R), 1, atol=1e-8)):
        raise ValueError("contact frames must be finite proper rotations")
    if (not np.isfinite([rho_min, tolerance, acceptance_tolerance]).all()
            or rho_min < 0 or tolerance <= 0 or acceptance_tolerance <= 0):
        raise ValueError("invalid margin or tolerances")
    if arms:
        if (len(q) != len(arms) or len(assignment) != m
                or len(set(assignment)) != m
                or any(j < 0 or j >= len(arms) for j in assignment)):
            raise ValueError("fixed q for each arm and an injective assignment required")
        T = np.asarray(np.eye(4) if object_pose is None else object_pose, float)
        if (T.shape != (4, 4) or not np.isfinite(T).all()
                or not np.allclose(T[3], [0, 0, 0, 1])
                or not np.allclose(T[:3, :3].T@T[:3, :3], np.eye(3), atol=1e-8)
                or not np.isclose(np.linalg.det(T[:3, :3]), 1, atol=1e-8)):
            raise ValueError("object_pose must be a finite rigid transform")
    elif len(q) or len(assignment):
        raise ValueError("q/assignment require arms")
    Hs = [np.vstack((np.zeros((3, 3)), np.eye(3))) if c.kind == "P"
          else np.asarray(c.H, float) for c in caps]
    for c in caps:
        if c.kind == "P" and not np.isfinite([c.mu, c.normal_max]).all():
            raise ValueError("point-contact bounds must be finite")
    vertices = task.vertices
    offsets = np.cumsum([0]+[H.shape[1] for H in Hs])
    width, n = offsets[-1], 1+len(vertices)*offsets[-1]
    matrices, rhs, cones = [], [], []

    def rows(A, b, cone):
        matrices.append(np.atleast_2d(A))
        rhs.append(np.asarray(b, float).reshape(-1))
        cones.append(cone)

    def upper(A, b):
        if np.atleast_2d(A).shape[0]:
            rows(A, b, clarabel.NonnegativeConeT(np.atleast_2d(A).shape[0]))

    row = np.zeros((1, n)); row[0, 0] = -1
    upper(row, [-rho_min])
    mappings = [np.asarray(map_reaction(pi, Ri, Hi), float)
                for pi, Ri, Hi in zip(p, R, Hs)]
    torque_blocks, gravity = [], []
    for j, arm in enumerate(arms):
        qj = np.asarray(q[j], float).reshape(arm.model.nq)
        if not np.isfinite(qj).all() or np.any(qj < arm.model.lower) or np.any(qj > arm.model.upper):
            raise ValueError("fixed posture must be finite and within joint limits")
        Tee = np.asarray(arm.model.fk(qj, arm.frame), float)
        J = np.asarray(arm.model.jacobian(qj, arm.frame), float)
        G = np.asarray(arm.model.gravity(qj), float).reshape(-1)
        blocks = []
        for i, Hi in enumerate(Hs):
            dual = np.zeros((6, 6))
            dual[:3, :3] = Tee[:3, :3].T@T[:3, :3]@R[i]
            dual[3:, 3:] = T[:3, :3]@R[i]
            blocks.append(-J.T@dual@Hi if assignment[i] == j
                          else np.zeros((len(G), Hi.shape[1])))
        torque_blocks.append(np.hstack(blocks))
        gravity.append(G)
    for k, vertex in enumerate(vertices):
        start = 1+k*width
        balance = np.zeros((6, n))
        balance[:, 0] = -np.asarray(task.E)@np.asarray(task.D)@vertex
        balance[:, start:start+width] = np.hstack(mappings)
        rows(balance, task.nominal, clarabel.ZeroConeT(6))
        for i, (cap, Hi) in enumerate(zip(caps, Hs)):
            sl = slice(start+offsets[i], start+offsets[i+1])
            select = np.zeros((Hi.shape[1], n)); select[:, sl] = np.eye(Hi.shape[1])
            if cap.kind == "P":
                upper(-select[2:], [0.])
                upper(select[2:], [cap.normal_max])
                # b-Ax = [mu*f_n, f_t1, f_t2] in the Lorentz cone.
                rows(-np.vstack((cap.mu*select[2], select[:2])), np.zeros(3),
                     clarabel.SecondOrderConeT(3))
            else:
                upper(select, cap.bounds)
                upper(-select, cap.bounds)
        for arm, block, G in zip(arms, torque_blocks, gravity):
            A = np.zeros((len(G), n)); A[:, start:start+width] = block
            finite = np.isfinite(arm.model.effort)
            upper(A[finite], np.asarray(arm.model.effort)[finite]-G[finite])
            upper(-A[finite], np.asarray(arm.model.effort)[finite]+G[finite])
    settings = clarabel.DefaultSettings()
    settings.verbose = False
    settings.tol_gap_abs = settings.tol_gap_rel = settings.tol_feas = tolerance
    cost = np.zeros(n); cost[0] = -1
    solver = clarabel.DefaultSolver(sparse.csc_matrix((n, n)), cost,
                                   sparse.csc_matrix(np.vstack(matrices)),
                                   np.concatenate(rhs), cones, settings)
    before = perf_counter()
    solution = solver.solve()
    elapsed = perf_counter()-before
    status = str(solution.status)
    if status != "Solved":
        raise RuntimeError(f"SOCP rejected: {status}")
    x = np.asarray(solution.x)
    reactions, torques = [], []
    residual = max(0., rho_min-x[0])
    for k, vertex in enumerate(vertices):
        z = x[1+k*width:1+(k+1)*width]
        lam = [Hi@z[offsets[i]:offsets[i+1]] for i, Hi in enumerate(Hs)]
        balance = sum(W@z[offsets[i]:offsets[i+1]] for i, W in enumerate(mappings))
        required = np.asarray(task.nominal)+x[0]*np.asarray(task.E)@np.asarray(task.D)@vertex
        residual = max(residual, np.max(abs(balance-required)))
        for i, (cap, f) in enumerate(zip(caps, lam)):
            zi = z[offsets[i]:offsets[i+1]]
            violations = ([-f[5], f[5]-cap.normal_max, np.linalg.norm(f[3:5])-cap.mu*f[5]]
                          if cap.kind == "P" else abs(zi)-np.asarray(cap.bounds))
            residual = max(residual, np.max(violations))
        tau = [G+B@z for G, B in zip(gravity, torque_blocks)]
        for arm, value in zip(arms, tau):
            residual = max(residual, np.max(abs(value)-arm.model.effort))
        reactions.append(lam)
        torques.append(np.concatenate(tau) if tau else np.zeros(0))
    if not np.isfinite(x).all() or not np.isfinite(residual) or residual > acceptance_tolerance:
        raise RuntimeError(f"SOCP primal residual {residual:.3g} exceeds acceptance tolerance")
    return WrenchMarginResult(float(x[0]), np.array(reactions), np.array(torques),
                              float(residual), status, elapsed)
