"""Validated task policies, separate from mechanical models and solver options."""
from copy import deepcopy
from dataclasses import asdict, dataclass, field, fields
import json
from pathlib import Path

import numpy as np


@dataclass(frozen=True)
class SourcePolicy:
    representation: str = 'single_point'
    site_file: str | None = None
    mu: float = .8
    normal_max_N: float = 2.
    patch_half_size_m: float = .001
    position_radius_m: float = .0005
    normal_radius: float = 0.
    geometry_weight: float = 0.
    failure_policy: str = 'error'


@dataclass(frozen=True)
class TargetPolicy:
    robustness_enabled: bool = True
    gripper_geometry: str = 'fixed_surface'
    point_geometry: str = 'fixed_after_first'
    normal_geometry: str = 'fixed_surface'
    position_radius_m: float = .015
    first_point_patch_half_size_m: float = .012
    normal_radius: float = .1
    characteristic_length_m: float = .6
    posture_weight: float = .01
    rho_min: float = .1


@dataclass(frozen=True)
class TransitionPolicy:
    stationary_solver: str = 'condensed'
    configuration_update: str = 'independent_midpoint'
    support_activation_distance_m: float = .03
    point_tangent_half_size_m: float = .0002
    support_gap_hermite_bounds: bool = False
    terminal_support_transitions: tuple = ()
    opening_dwell_s: float = .8


@dataclass(frozen=True)
class ExecutionPolicy:
    control_mode: str = 'impedance_feedforward'
    jaw_force_fraction: float = .7
    reference_dt_s: float = .02
    reference_projection_limit_rad: float = .02
    physics_dt_s: float = .001
    joint_k: float = 40.
    joint_d: float = 8.
    motion_k: tuple = (40.,40.,40.,3000.,3000.,3000.)
    motion_d: tuple = (2.,2.,2.,40.,40.,40.)
    force_k: float = .25
    force_i: float = 1.
    force_d: tuple = (2.,2.,2.,40.,40.,40.)
    admittance_mass: tuple = (.2,.2,.2,2.,2.,2.)
    admittance_damping: tuple = (4.,4.,4.,40.,40.,40.)


@dataclass(frozen=True)
class DisturbancePolicy:
    validation_fraction: float = .5
    kind: str = 'rigid_wrench'
    source_scales: tuple = (.001, .001, .001, .05, .05, .05)
    target_scales: tuple = (.15, .15, .15, 1., 1., 1.)
    body_weights: tuple = ()
    base_frame: str = 'base'


@dataclass(frozen=True)
class TaskConfiguration:
    formulation_mode: str = 'declared_extension'
    source: SourcePolicy = field(default_factory=SourcePolicy)
    target: TargetPolicy = field(default_factory=TargetPolicy)
    transition: TransitionPolicy = field(default_factory=TransitionPolicy)
    execution: ExecutionPolicy = field(default_factory=ExecutionPolicy)
    disturbance: DisturbancePolicy = field(default_factory=DisturbancePolicy)
    feature_phase_modes: dict = field(default_factory=dict)
    default_types: tuple = ()
    schema_version: int = 1

    def __post_init__(self):
        s, t, e, d = self.source, self.transition, self.execution, self.disturbance
        target=self.target
        if self.formulation_mode not in {'declared_extension','manuscript_literal'}:
            raise ValueError('Unsupported formulation_mode')
        if target.gripper_geometry!='fixed_surface' or target.point_geometry!='fixed_after_first' or target.normal_geometry!='fixed_surface':
            raise ValueError('This task adapter implements fixed G surfaces/normals and held P material sites; another policy requires a regrasp adapter')
        if t.stationary_solver not in {'condensed','cito'}:
            raise ValueError('Unsupported stationary_solver')
        if not isinstance(target.robustness_enabled,bool) or (not target.robustness_enabled and target.rho_min!=0):
            raise ValueError('Nominal target ablation requires robustness_enabled=false and rho_min=0')
        if self.schema_version != 1:
            raise ValueError('Unsupported framework configuration schema')
        positive = [s.mu, s.normal_max_N, s.patch_half_size_m,
                    t.support_activation_distance_m, t.point_tangent_half_size_m,
                    t.opening_dwell_s, e.reference_dt_s, e.reference_projection_limit_rad,e.physics_dt_s,target.position_radius_m,target.first_point_patch_half_size_m,target.characteristic_length_m]
        nonnegative = [s.position_radius_m, s.normal_radius, s.geometry_weight,e.joint_k,e.joint_d,e.force_k,e.force_i,target.normal_radius,target.posture_weight,target.rho_min]
        if not np.isfinite(positive + nonnegative).all() or min(positive) <= 0 or min(nonnegative) < 0:
            raise ValueError('Invalid task policy bounds')
        if s.failure_policy not in {'error', 'geometric_prior'}:
            raise ValueError('Source failure_policy must be error or geometric_prior')
        if s.representation not in {'single_point','functional_group_sites'}:
            raise ValueError('Unsupported source representation')
        if s.representation=='functional_group_sites' and (not isinstance(s.site_file,str) or not s.site_file):
            raise ValueError('Functional source groups require an explicit site_file')
        if t.configuration_update not in {'independent_midpoint', 'contact_coordinate_midpoint', 'independent_se3'}:
            raise ValueError('Unsupported configuration_update')
        if e.control_mode not in {'impedance_feedforward', 'hybrid_feedback', 'hybrid_feedforward'}:
            raise ValueError('Unsupported control_mode')
        if not 0 < e.jaw_force_fraction <= 1:
            raise ValueError('jaw_force_fraction must be in (0, 1]')
        for vector in (e.motion_k,e.motion_d,e.force_d,e.admittance_mass,e.admittance_damping):
            if len(vector)!=6 or not np.isfinite(vector).all() or min(vector)<0:raise ValueError('Controller vectors need six finite nonnegative entries')
        if min(e.admittance_mass)<=0:raise ValueError('Admittance mass must be positive')
        if d.kind not in {'rigid_wrench', 'body_weighted_wrench', 'base_force_joint_torque'}:
            raise ValueError('Unsupported disturbance image')
        if not np.isfinite(d.validation_fraction) or not 0<d.validation_fraction<=1:
            raise ValueError('Validation disturbance must lie within the optimized margin')
        dimension = 4 if d.kind == 'base_force_joint_torque' else 6
        if any(len(v) != dimension or not np.isfinite(v).all() or min(v) <= 0
               for v in (d.source_scales, d.target_scales)):
            raise ValueError('Disturbance scales need the declared dimension and positive values')
        if d.kind == 'body_weighted_wrench' and (not d.body_weights or not np.isfinite(d.body_weights).all()):
            raise ValueError('Weighted-body disturbance needs finite body_weights')
        if any(mode not in {'holding', 'unloaded', 'released'}
               for modes in self.feature_phase_modes.values() for mode in modes.values()):
            raise ValueError('Unknown feature phase mode')
        if any(len(pair) != 2 for pair in t.terminal_support_transitions):
            raise ValueError('Terminal support transitions need from/to phase pairs')
        if any(kind not in {'P','G'} for kind in self.default_types):raise ValueError('Full-task default types must be P/G')

    def to_dict(self):
        return asdict(self)


def _merge(base, override):
    result = deepcopy(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = _merge(result[key], value)
        else:
            result[key] = deepcopy(value)
    return result


def configuration_from_dict(data):
    data = deepcopy(data)
    if 'default_types' in data:data['default_types']=tuple(data['default_types'])
    unknown = set(data) - {f.name for f in fields(TaskConfiguration)}
    if unknown:
        raise ValueError(f'Unknown framework configuration keys: {sorted(unknown)}')
    for name, cls in [('source', SourcePolicy),('target',TargetPolicy), ('transition', TransitionPolicy),
                      ('execution', ExecutionPolicy), ('disturbance', DisturbancePolicy)]:
        if name in data:
            section=data[name]
            for key,value in section.items():
                if isinstance(value,list):section[key]=tuple(tuple(v) if isinstance(v,list) else v for v in value)
            data[name] = cls(**section)
    return TaskConfiguration(**data)


def load_task_configuration(task_id, override=None):
    path = Path(__file__).with_name('scenarios') / 'framework_profiles.json'
    profiles = json.loads(path.read_text())
    data = profiles['tasks'].get(task_id)
    if data is None:
        raise ValueError(f'No configured task adapter: {task_id}')
    data = _merge(profiles['defaults'], data)
    if override is not None:
        data = _merge(data, json.loads(Path(override).read_text()) if isinstance(override, (str, Path)) else override)
    return configuration_from_dict(data)


def task_configuration(task):
    """Synthetic test systems may intentionally have no task adapter."""
    return getattr(task, 'configuration', None) or TaskConfiguration()


def contact_phase_mode(phase, feature):
    return getattr(feature, 'phase_modes', {}).get(
        phase, 'released' if phase == 'released' else 'unloaded' if phase == 'unload' else 'holding')
