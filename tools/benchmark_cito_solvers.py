"""Isolated solver qualification on identical stationary CITO problems.

Every trial has a fresh process. Native crashes and timeouts remain results.
This benchmark is not a furniture assembly or full-geometry experiment.
"""
import argparse
from dataclasses import asdict
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
from time import perf_counter

import casadi as ca
import numpy as np

from contact_transfer.contacts import Capability
from contact_transfer.optimization import SolverSettings, NLP_BACKENDS, EXPERIMENTAL_NLP_BACKENDS
from contact_transfer.system import MechanicalSystem, RigidObject
from contact_transfer.trajectory import Contact, solve_cito


def worker(backend, output):
    settings = SolverSettings(backend=backend, allow_experimental=backend in EXPERIMENTAL_NLP_BACKENDS)
    if not ca.has_nlpsol(backend):
        Path(output).write_text(json.dumps(dict(available=False, accepted=False)))
        return
    system = MechanicalSystem([], RigidObject(1., np.zeros(3), np.eye(3)*.02))
    q = np.array([0., 0, .5, 0, 0, 0, 1])
    contact = Contact("ground", np.zeros(3), [0, 0, 1], Capability(mu=.5, normal_max=30), plane_offset=.5)
    started = perf_counter()
    record = dict(available=True, accepted=False)
    try:
        result = solve_cito(system, q, q, [contact], steps=8, duration=.4,
                            relaxations=(1e-3, 1e-5, 1e-8), acceptance=2e-5, settings=settings)
        residuals = {key: float(getattr(result, key)) for key in
                     ("primal_residual", "complementarity_residual", "friction_residual",
                      "minimum_gap", "gripper_velocity_residual")}
        accepted = (np.isfinite(list(residuals.values())).all()
                    and residuals["primal_residual"] <= settings.acceptance_tolerance
                    and max(residuals["complementarity_residual"], abs(residuals["friction_residual"]),
                            residuals["gripper_velocity_residual"], -residuals["minimum_gap"]) <= 2e-5)
        record.update(accepted=bool(accepted), residuals=residuals)
    except (RuntimeError, ValueError) as error:
        record["error"] = str(error)
    record["seconds"] = perf_counter()-started
    Path(output).write_text(json.dumps(record, allow_nan=False))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--backends", nargs="+", choices=NLP_BACKENDS+EXPERIMENTAL_NLP_BACKENDS, default=["ipopt"])
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument("--timeout", type=float, default=60.)
    parser.add_argument("--output", default="outputs/solver_benchmark.json")
    parser.add_argument("--worker-backend", choices=NLP_BACKENDS+EXPERIMENTAL_NLP_BACKENDS, help=argparse.SUPPRESS)
    parser.add_argument("--worker-output", help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.worker_backend:
        worker(args.worker_backend, args.worker_output)
        return
    if args.repeats < 1 or args.timeout <= 0:
        parser.error("repeats and timeout must be positive")
    report = dict(case="restricted_stationary_support", casadi_version=ca.__version__,
                  steps=8, duration=.4, relaxations=[1e-3, 1e-5, 1e-8], contact_acceptance=2e-5,
                  full_geometry_checked=False, initialization="fresh process and initialization for every solve",
                  subprocess_timeout_s=args.timeout, backends={})
    destination = Path(args.output)
    destination.parent.mkdir(parents=True, exist_ok=True)
    for backend in args.backends:
        settings = SolverSettings(backend=backend, allow_experimental=backend in EXPERIMENTAL_NLP_BACKENDS)
        entry = dict(settings=asdict(settings), trials=[])
        report["backends"][backend] = entry
        for trial in range(args.repeats):
            with tempfile.TemporaryDirectory(prefix="cito-solver-trial-") as directory:
                result_path = Path(directory)/"result.json"
                command = [sys.executable, str(Path(__file__).resolve()), "--worker-backend", backend,
                           "--worker-output", str(result_path)]
                try:
                    process = subprocess.run(command, capture_output=True, text=True, timeout=args.timeout,
                                             env=dict(os.environ, OPENBLAS_NUM_THREADS="1", OMP_NUM_THREADS="1", MKL_NUM_THREADS="1"))
                except subprocess.TimeoutExpired:
                    record = dict(accepted=False, error="subprocess timeout", timeout=True)
                else:
                    if process.returncode != 0 or not result_path.exists():
                        record = dict(accepted=False, error="native crash or worker failure", exit_code=process.returncode,
                                      diagnostic=(process.stderr+process.stdout)[-2000:])
                    else:
                        record = json.loads(result_path.read_text())
                record["trial"] = trial
                entry["trials"].append(record)
                if record.get("available") is False:
                    break
        times = [r["seconds"] for r in entry["trials"] if r["accepted"]]
        unavailable = all(r.get("available") is False for r in entry["trials"])
        entry.update(status="unavailable" if unavailable else "completed", accepted_rate=None if unavailable else len(times)/len(entry["trials"]),
                     accepted_median_seconds=float(np.median(times)) if times else None)
        destination.write_text(json.dumps(report, indent=2, allow_nan=False)+"\n")
    print(json.dumps(report, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
