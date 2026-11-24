"""Reproduce the 2026-11-23 review probes without changing task/model files.

These are small frozen-contact solver measurements, not full robot trials.
The cache probe records a known omission; it does not repair or invalidate caches.
"""
import argparse
import cProfile
from importlib.metadata import version
import io
import json
from pathlib import Path
import platform
import pstats
import statistics
import subprocess
import tempfile
from time import perf_counter
from types import SimpleNamespace

import numpy as np

from contact_transfer.contacts import Capability, SurfacePatch, TaskWrench, contact_basis, map_reaction
from contact_transfer.conic import fixed_wrench_margin
from contact_transfer.full_recording import fingerprint
from contact_transfer.full_scenario import FullProblem, FullTask
from contact_transfer.generalized_conic import fixed_generalized_margin
from contact_transfer.generalized_load import GeneralizedTaskLoad
from contact_transfer.optimization import SolverSettings
from contact_transfer.refinement import refine_contacts


def cache_probe():
    with tempfile.TemporaryDirectory(prefix='contact-framework-audit-') as directory:
        root = Path(directory)
        task = FullTask.load('panel', root / 'task')
        source = task.original.path.parent / task.original.data['source_file']
        data = json.loads(source.read_text())
        copied = root / task.original.data['source_file']
        copied.parent.mkdir(parents=True, exist_ok=True)
        for value in task.original.data.values():
            if isinstance(value,str) and value.endswith(('.json','.urdf','.srdf')):
                target=root/value;target.parent.mkdir(parents=True,exist_ok=True)
                target.write_bytes((task.original.path.parent/value).read_bytes())
        task.original = SimpleNamespace(path=root / 'case.json', data=task.original.data)
        problem = FullProblem.create(task, root / 'problem', ('P', 'P', 'P'))
        before_hash = fingerprint(problem, SolverSettings(), 8)
        before = problem.source(task.keyframes[0])['record']
        old_limit = data['refinement']['normal_max_N']
        data['refinement']['normal_max_N'] = .01
        copied.write_text(json.dumps(data))
        after_hash = fingerprint(problem, SolverSettings(), 8)
        try:
            after = problem.source(task.keyframes[0])['record']
        except RuntimeError as error:
            fresh_after = dict(accepted=False, reason=str(error))
        else:
            fresh_after = dict(accepted=True, rho=after['source_rho_after'])
        return dict(
            changed_input='source JSON refinement.normal_max_N', before_N=old_limit,
            after_N=.01, hash_unchanged=before_hash == after_hash,
            before_fingerprint=before_hash, after_fingerprint=after_hash,
            fresh_source_before=dict(accepted=before['source_certificate'], rho=before['source_rho_after']),
            fresh_source_after=fresh_after,
            scope='Only temporary source JSON changed; no production cache was loaded or written.')


def timed(call, repeats):
    call()  # Warm-up excluded, including IPOPT plugin initialization.
    values, residuals, durations = [], [], []
    for _ in range(repeats):
        start = perf_counter()
        rho, residual = call()
        durations.append(perf_counter() - start)
        values.append(float(rho))
        residuals.append(float(residual))
    return dict(rho=values, residuals=residuals, wall_seconds=durations,
                median_seconds=statistics.median(durations))


def solver_probe(repeats):
    points = np.array([[-.2, -.15, 0.], [.2, -.15, 0.], [0., .2, 0.]])
    normals = np.tile([0., 0., -1.], (3, 1))
    rotations = np.array([contact_basis(n) for n in normals])
    patches = [SurfacePatch.fixed(p, n) for p, n in zip(points, normals)]
    caps = [Capability(mu=.8, normal_max=30.) for _ in points]
    nominal = np.array([0., 0., 0., 0., 0., -9.81])
    task = TaskWrench(nominal, np.eye(6), np.diag([.1] * 3 + [1.] * 3))
    generalized = GeneralizedTaskLoad(task.nominal, task.E, task.D)
    maps = [np.asarray(map_reaction(p, R, np.eye(6))) for p, R in zip(points, rotations)]

    def nlp():
        r = refine_contacts(patches, task, caps)
        return r.rho, r.primal_residual

    def conic():
        r = fixed_wrench_margin(points, rotations, task, caps)
        return r.rho, r.primal_residual

    def sparse_conic():
        r = fixed_generalized_margin(maps, generalized, caps)
        if not r['certificate']:
            raise RuntimeError(str(r['status']))
        return r['rho'], r['primal_residual']

    measurements = {name: timed(call, repeats) for name, call in
                    [('ipopt_fixed_geometry', nlp), ('clarabel_rigid', conic),
                     ('clarabel_generalized', sparse_conic)]}
    profiles = {}
    for name, call in [('clarabel_rigid', conic), ('clarabel_generalized', sparse_conic)]:
        profile = cProfile.Profile()
        profile.runcall(call)
        output = io.StringIO()
        pstats.Stats(profile, stream=output).strip_dirs().sort_stats('cumulative').print_stats(12)
        profiles[name] = output.getvalue()
    reference = measurements['ipopt_fixed_geometry']['rho'][0]
    return dict(
        case='three fixed unilateral P contacts; 6D box, 64 vertices; no robot or geometry search',
        timing='warm end-to-end calls, including graph/matrix assembly, solver setup and solve',
        repeats=repeats, measurements=measurements, profiles=profiles,
        maximum_rho_difference=max(abs(rho - reference) for m in measurements.values() for rho in m['rho']),
        geometry_or_native_execution_validated=False)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repeats', type=int, default=5)
    parser.add_argument('--output', type=Path, default=Path('outputs/review/framework-probes.json'))
    args = parser.parse_args()
    if args.repeats < 1:
        parser.error('--repeats must be positive')
    result = dict(
        reviewed_commit=subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip(),
        python=platform.python_version(),
        versions={package: version(package) for package in ['casadi', 'pin', 'mujoco', 'numpy', 'scipy', 'clarabel']},
        cache=cache_probe(), solvers=solver_probe(args.repeats))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    print(json.dumps(dict(output=str(args.output), cache=result['cache'],
                         medians={k: v['median_seconds'] for k, v in result['solvers']['measurements'].items()},
                         maximum_rho_difference=result['solvers']['maximum_rho_difference']), indent=2))


if __name__ == '__main__':
    main()
