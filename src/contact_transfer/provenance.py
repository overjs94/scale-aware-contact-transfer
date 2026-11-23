"""Content-addressed, inspectable identities for planning inputs and stages.

Identity is intentionally conservative: a package code change invalidates a
planning cache. This is preferable to silently missing an indirect dependency.
Execution and independent validation still run after a cache hit.
"""
from dataclasses import asdict, is_dataclass
import hashlib
from importlib.metadata import PackageNotFoundError, version
import json
from pathlib import Path

import numpy as np

CACHE_SCHEMA = 2


def jsonable(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, Path):
        return str(value)
    if is_dataclass(value):
        return asdict(value)
    raise TypeError(type(value).__name__)


def content_identity(value, *, namespace='stage'):
    payload = json.dumps(value, sort_keys=True, default=jsonable, allow_nan=False,
                         separators=(',', ':')).encode()
    return f'ct-{namespace}-v{CACHE_SCHEMA}:' + hashlib.sha256(payload).hexdigest()


def file_digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def model_manifest(robot):
    return dict(urdf_sha256=file_digest(robot.path), base_pose=robot.base_pose,
                floating_base=robot.floating_base, locked_joints=robot.tree.locked,
                gravity=robot.gravity_world, effort=robot.effort,
                velocity=robot.velocity, lower=robot.lower, upper=robot.upper)


def planning_manifest(problem, settings, steps):
    task = problem.task_definition
    original = task.original
    inputs = {}
    # Capture case contents as well as live data: adapters can override data in
    # memory, and source/fixture files can change without changing the anchors.
    if Path(original.path).is_file():
        inputs['case'] = file_digest(original.path)
    for key, value in original.data.items():
        if isinstance(value, str) and value.endswith(('.json', '.urdf', '.srdf')):
            inputs[key] = file_digest(Path(original.path).parent / value)
    objects = task.object.bodies.values() if hasattr(task.object, 'bodies') else [task.object]
    package = Path(__file__).parent
    libraries = {}
    for name in ('numpy', 'scipy', 'casadi', 'pin', 'clarabel', 'scs', 'mujoco'):
        try:
            libraries[name] = version(name)
        except PackageNotFoundError:
            libraries[name] = None
    return dict(
        schema=CACHE_SCHEMA, task=task.id, case_data=original.data, input_files=inputs,
        scale=task.scale, fixtures=task.fixtures, include_fixtures=task.include_fixtures,
        gripper_force_coordinates=task.gripper_force_coordinates,
        posture_reference_rule=task.posture_reference_rule,
        posture_trust_radius_rad=task.posture_trust_radius_rad,
        configuration=getattr(task, 'configuration', None),
        source_settings=getattr(problem,'source_settings',None),
        conic_settings=getattr(problem,'conic_settings',None),
        types=problem.types, assignment=problem.assignment,
        refine_source=problem.refine_enabled, features=[vars(f) for f in task.features],
        keyframes=task.keyframes, specs=[{k:v for k,v in spec.items() if k!='allowed_object_geoms'} for spec in problem.specs], steps=steps,
        settings={k: v for k, v in vars(settings).items()
                  if k not in {'diagnostics_path', 'verbose'}},
        models=[model_manifest(r) for r in [*problem.full_robots, *objects]],
        formulation={p.name: file_digest(p) for p in sorted(package.glob('*.py'))},
        native_asset_manifest=file_digest(package/'models'/'native_assets.json'),
        libraries=libraries)


def endpoint_manifest(key):
    if key is None:
        return None
    return {name: getattr(key, name, None) for name in
            ('q', 'positions', 'rotations', 'apertures', 'rho', 'phase',
             'retreat_m', 'posture_reference')}


def stage_identity(base, stage, index, *, previous=None, left=None, right=None):
    """Bind dependent stages to their actual endpoints, not only task inputs."""
    return content_identity(dict(base=base, stage=stage, index=index,
                                 previous=endpoint_manifest(previous),
                                 left=endpoint_manifest(left), right=endpoint_manifest(right)))
