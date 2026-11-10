"""Shared NLP configuration and independent primal-residual acceptance."""

from dataclasses import dataclass

import casadi as ca
import numpy as np


@dataclass
class SolverSettings:
    max_iterations: int = 1500
    tolerance: float = 1e-8
    acceptance_tolerance: float = 2e-6
    verbose: bool = False


def equality(opti, expression):
    """Add nonconstant equality rows and reject inconsistent constant rows."""
    expression = ca.vec(expression)
    # MX function calls and dense zero blocks can hide identically zero rows.
    # SX expansion discovers them without a tolerance-based numerical rank test.
    symbols = ca.symvar(expression)
    f = ca.Function("equality_simplification", symbols, [expression]).expand()
    sx = [ca.SX.sym(s.name(), s.size1(), s.size2()) for s in symbols]
    simplified = f.call(sx)[0]
    for i in range(expression.numel()):
        row = simplified[i]
        if any(ca.depends_on(row, s) for s in sx):
            opti.subject_to(expression[i] == 0)
        elif abs(float(ca.evalf(row))) > 1e-10:
            raise ValueError("inconsistent constant equality")


def solve(opti, settings=None):
    settings = settings or SolverSettings()
    opti.solver("ipopt", {
        "expand": True, "print_time": settings.verbose,
        "ipopt.print_level": 5 if settings.verbose else 0,
        "ipopt.sb": "yes", "ipopt.max_iter": settings.max_iterations,
        "ipopt.tol": settings.tolerance,
        "ipopt.acceptable_tol": settings.tolerance*10,
        "ipopt.bound_relax_factor": 0,
    })
    try:
        solution = opti.solve()
    except RuntimeError as error:
        status = opti.stats().get("return_status", "unknown")
        raise RuntimeError(f"NLP rejected: {status}") from error
    g = np.asarray(solution.value(opti.g)).reshape(-1)
    lo = np.asarray(solution.value(opti.lbg)).reshape(-1)
    hi = np.asarray(solution.value(opti.ubg)).reshape(-1)
    residual = float(max(np.max(lo-g, initial=0), np.max(g-hi, initial=0), 0))
    if not np.isfinite(g).all() or residual > settings.acceptance_tolerance:
        raise RuntimeError(f"primal residual {residual:.3g} exceeds acceptance tolerance")
    return solution, residual
