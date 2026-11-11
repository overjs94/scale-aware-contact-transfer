"""Same restricted stationary CITO, fresh starts and acceptance for each solver.

This is a solver smoke benchmark, not a furniture assembly experiment.
"""

import argparse
from dataclasses import asdict
import json
from pathlib import Path
from time import perf_counter

import casadi as ca
import numpy as np

from contact_transfer.contacts import Capability
from contact_transfer.optimization import SolverSettings
from contact_transfer.system import MechanicalSystem, RigidObject
from contact_transfer.trajectory import Contact, solve_cito


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--backends", nargs="+", choices=["ipopt", "snopt"], default=["ipopt"])
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument("--output", default="outputs/solver_benchmark.json")
    args = parser.parse_args()
    if args.repeats < 1:
        parser.error("repeats must be positive")
    report = {"case": "restricted_stationary_support", "casadi_version": ca.__version__,
              "steps": 8, "duration": .4, "relaxations": [1e-3, 1e-5, 1e-8],
              "contact_acceptance": 2e-5, "full_geometry_checked": False,
              "initialization": "fresh default initialization for every solve", "backends": {}}
    for backend in args.backends:
        settings = SolverSettings(backend=backend)
        entry = {"settings": asdict(settings), "trials": []}
        report["backends"][backend] = entry
        if not ca.has_nlpsol(backend):
            entry.update(status="unavailable", accepted_rate=None, accepted_median_seconds=None)
            continue
        for trial in range(args.repeats):
            system = MechanicalSystem([], RigidObject(1., np.zeros(3), np.eye(3)*.02))
            q = np.array([0., 0, .5, 0, 0, 0, 1])
            contact = Contact("ground", np.zeros(3), [0, 0, 1],
                              Capability(mu=.5, normal_max=30), plane_offset=.5)
            start = perf_counter()
            record = {"trial": trial}
            try:
                result = solve_cito(system, q, q, [contact], steps=8, duration=.4,
                                    relaxations=tuple(report["relaxations"]),
                                    acceptance=report["contact_acceptance"], settings=settings)
                residuals = {key: float(getattr(result, key)) for key in
                             ("primal_residual", "complementarity_residual", "friction_residual",
                              "minimum_gap", "gripper_velocity_residual")}
                accepted = (np.isfinite(list(residuals.values())).all()
                            and residuals["primal_residual"] <= settings.acceptance_tolerance
                            and max(residuals["complementarity_residual"], abs(residuals["friction_residual"]),
                                    residuals["gripper_velocity_residual"], -residuals["minimum_gap"])
                            <= report["contact_acceptance"])
                record.update(accepted=bool(accepted), residuals=residuals)
            except (RuntimeError, ValueError) as error:
                record.update(accepted=False, error=str(error))
            record["seconds"] = perf_counter()-start
            entry["trials"].append(record)
        times = [r["seconds"] for r in entry["trials"] if r["accepted"]]
        entry.update(status="completed", accepted_rate=len(times)/args.repeats,
                     accepted_median_seconds=float(np.median(times)) if times else None)
    path = Path(args.output)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(report, indent=2, allow_nan=False)+"\n")
    print(json.dumps(report, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
