"""Shared NLP configuration and independent primal-residual acceptance."""

from dataclasses import dataclass, field

import casadi as ca
import numpy as np


@dataclass
class SolverSettings:
    max_iterations: int = 1500
    tolerance: float = 1e-8
    acceptance_tolerance: float = 2e-6
    verbose: bool = False
    backend: str = "ipopt"
    options: dict = field(default_factory=dict)
    diagnostics_path: str | None = None

    def __post_init__(self):
        if self.backend not in {"ipopt", "snopt"}:
            raise ValueError("backend must be ipopt or snopt")


def equality(opti, expression):
    """Add nonconstant equality rows and reject inconsistent constant rows."""
    expression = ca.vec(expression)
    if isinstance(expression, ca.DM):
        if not np.isfinite(np.asarray(expression)).all() or np.any(abs(np.asarray(expression)) > 1e-10):
            raise ValueError("inconsistent constant equality")
        return
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
    options = {
        "expand": True, "print_time": settings.verbose,
        "ipopt.print_level": 5 if settings.verbose else 0,
        "ipopt.sb": "yes", "ipopt.max_iter": settings.max_iterations,
        "ipopt.tol": settings.tolerance,
        "ipopt.acceptable_tol": settings.tolerance*10,
        "ipopt.bound_relax_factor": 0,
    } if settings.backend == "ipopt" else {
        "expand": True, "print_time": settings.verbose,
        "snopt": {"Major_iterations_limit": settings.max_iterations,
                  "Major_feasibility_tolerance": settings.tolerance,
                  "Major_optimality_tolerance": settings.tolerance},
    }
    # Merge backend dictionaries so a print option does not erase the shared
    # feasibility/optimality budget. Do not mutate caller-owned settings.
    for key, value in settings.options.items():
        if key in options and isinstance(options[key], dict) and isinstance(value, dict):
            options[key] = {**options[key], **value}
        else:
            options[key] = value
    opti.solver(settings.backend, options)
    try:
        solution = opti.solve()
    except RuntimeError as error:
        try:
            status = opti.stats().get("return_status", "unknown")
        except RuntimeError:
            status = "solver initialization failed"
        if settings.diagnostics_path is not None:
            import json
            from pathlib import Path
            path=Path(settings.diagnostics_path);path.parent.mkdir(parents=True,exist_ok=True)
            try:
                values=np.asarray(opti.debug.value(opti.g)).ravel();lower=np.asarray(opti.debug.value(opti.lbg)).ravel();upper=np.asarray(opti.debug.value(opti.ubg)).ravel();violation=np.maximum.reduce([lower-values,values-upper,np.zeros(len(values))]);indices=np.argsort(violation)[-20:][::-1]
                finite=lambda v:float(v) if np.isfinite(v) else None
                rows=[dict(index=int(i),violation=finite(violation[i]),value=finite(values[i]),lower=finite(lower[i]),upper=finite(upper[i]),source=opti.debug.g_describe(int(i))) for i in indices]
                path.write_text(json.dumps(dict(status=status,accepted=False,constraints=rows),indent=2)+'\n')
            except RuntimeError:path.write_text(json.dumps(dict(status=status,accepted=False,diagnostics='unavailable'))+'\n')
        suffix = "; SNOPT requires a working CasADi plugin and solver installation/license" if settings.backend == "snopt" else ""
        raise RuntimeError(f"NLP rejected [{settings.backend}]: {status}{suffix}") from error
    g = np.asarray(solution.value(opti.g)).reshape(-1)
    lo = np.asarray(solution.value(opti.lbg)).reshape(-1)
    hi = np.asarray(solution.value(opti.ubg)).reshape(-1)
    residual = float(max(np.max(lo-g, initial=0), np.max(g-hi, initial=0), 0))
    if not np.isfinite(g).all() or residual > settings.acceptance_tolerance:
        raise RuntimeError(f"primal residual {residual:.3g} exceeds acceptance tolerance")
    return solution, residual
