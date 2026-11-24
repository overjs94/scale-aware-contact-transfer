"""Optional conic libraries sharing one canonical problem and acceptance gate."""
from dataclasses import dataclass
import importlib.util
from time import perf_counter

import numpy as np
from scipy import sparse

from .optimization import SolverUnavailableError

CONIC_BACKENDS = ('clarabel', 'scs')


@dataclass(frozen=True)
class ConicSettings:
    backend: str = 'clarabel'
    tolerance: float = 1e-9
    max_iterations: int | None = None

    def __post_init__(self):
        if self.backend not in CONIC_BACKENDS:
            raise ValueError(f'Conic backend must be one of {CONIC_BACKENDS}')
        if not np.isfinite(self.tolerance) or self.tolerance <= 0:
            raise ValueError('Positive finite conic solver tolerance required')
        if self.max_iterations is not None and self.max_iterations <= 0:
            raise ValueError('Positive conic iteration budget required')


def conic_available(backend):
    return backend in CONIC_BACKENDS and importlib.util.find_spec(backend) is not None


def solve_conic(P, q, A, b, *, equality_rows, linear_rows, soc_dimensions,
                settings=None, tolerance=1e-9):
    settings = settings or ConicSettings()
    if not conic_available(settings.backend):
        raise SolverUnavailableError(f'Selected conic backend {settings.backend} is unavailable; install its optional extra')
    tolerance = max(tolerance, settings.tolerance)
    started = perf_counter()
    if settings.backend == 'clarabel':
        import clarabel
        options = clarabel.DefaultSettings()
        options.verbose = False
        options.tol_feas = options.tol_gap_abs = options.tol_gap_rel = tolerance
        options.max_iter = settings.max_iterations or 150
        cones = [clarabel.ZeroConeT(equality_rows), clarabel.NonnegativeConeT(linear_rows),
                 *[clarabel.SecondOrderConeT(n) for n in soc_dimensions]]
        result = clarabel.DefaultSolver(sparse.triu(P, format='csc'), q, A, b, cones, options).solve()
        x, status, iterations = result.x, str(result.status), result.iterations
    else:
        import scs
        result = scs.solve(dict(P=sparse.triu(P, format='csc'), c=q, A=A, b=b),
                           dict(z=equality_rows, l=linear_rows, q=list(soc_dimensions)),
                           verbose=False, eps_abs=tolerance, eps_rel=tolerance,
                           max_iters=settings.max_iterations or 100000)
        native = result['info']['status'].lower()
        status = {'solved': 'Solved', 'solved (inaccurate - reached max_iters)': 'AlmostSolved',
                  'infeasible': 'PrimalInfeasible', 'unbounded': 'DualInfeasible'}.get(native, native)
        x, iterations = result['x'], result['info']['iter']
        if status in {'PrimalInfeasible', 'DualInfeasible'}:
            x = np.zeros(A.shape[1])  # Never accepted as a primal witness.
    x = np.asarray(x, float)
    if x.shape != q.shape or not np.isfinite(x).all():
        raise RuntimeError(f'Conic solver {settings.backend} returned no finite primal iterate: {status}')
    return x, status, dict(backend=settings.backend, solver_tolerance=tolerance,
                           iterations=int(iterations), wall_time_s=perf_counter()-started)
