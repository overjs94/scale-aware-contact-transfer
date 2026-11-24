from dataclasses import replace

import numpy as np
import pytest

from contact_transfer.configuration import (configuration_from_dict, contact_phase_mode,
                                           load_task_configuration)
from contact_transfer.full_scenario import FullTask, FullProblem
from contact_transfer.pipeline import source_prior
from contact_transfer.contacts import Capability, SurfacePatch, TaskWrench


@pytest.mark.parametrize('task,integrator,fraction', [
    ('panel', 'contact_coordinate_midpoint', .45),
    ('box', 'contact_coordinate_midpoint', .6),
    ('assembly', 'independent_midpoint', .7)])
def test_declared_profiles_preserve_existing_experiment(task, integrator, fraction):
    cfg = load_task_configuration(task)
    assert cfg.transition.configuration_update == integrator
    assert cfg.execution.jaw_force_fraction == fraction
    assert configuration_from_dict(cfg.to_dict()) == cfg


def test_overrides_do_not_mutate_defaults_and_reject_typos():
    changed = load_task_configuration('panel', {'execution': {'jaw_force_fraction': .5}})
    assert changed.execution.jaw_force_fraction == .5
    assert load_task_configuration('panel').execution.jaw_force_fraction == .45
    with pytest.raises(ValueError, match='Unknown'):
        load_task_configuration('panel', {'executoin': {}})
    with pytest.raises(ValueError):
        load_task_configuration('panel', {'execution': {'jaw_force_fraction': 2}})


def test_feature_roles_survive_task_rename_and_custom_feature_names(tmp_path):
    task = FullTask.load('assembly', tmp_path / 'task')
    leg = next(f for f in task.features if f.id == 'leg_gripper')
    leg.id = 'custom_tool'
    assert contact_phase_mode('release_leg', leg) == 'released'
    other = next(f for f in task.features if f.id == 'point_support')
    assert contact_phase_mode('release_leg', other) == 'holding'


@pytest.mark.parametrize('task_id', ['panel', 'box', 'assembly'])
def test_generalized_task_load_has_declared_image_without_id_dispatch(task_id, tmp_path):
    task = FullTask.load(task_id, tmp_path / 'task')
    p = FullProblem.create(task, tmp_path / 'problem', 'PPP')
    q = task.keyframes[0]['object_q']
    before = p.task_load(q)
    task.id = 'user_defined_task'
    after = p.task_load(q)
    np.testing.assert_array_equal(before.E, after.E)
    np.testing.assert_array_equal(before.D, after.D)


def test_source_failure_policy_is_explicit():
    task = TaskWrench(np.array([0., 0, 0, 0, 0, -9.81]), np.eye(6), np.eye(6))
    patch = SurfacePatch.fixed([0, 0, 0], [0, 0, -1])
    with pytest.raises(RuntimeError):
        source_prior([patch], task, [Capability(normal_max=.01)], failure_policy='error')
    result = source_prior([patch], task, [Capability(normal_max=.01)], failure_policy='geometric_prior')
    assert not result.source_certificate


def test_assembly_source_reports_structural_failure_without_mislabeling_solver_success(tmp_path):
    task=FullTask.load('assembly',tmp_path/'task');p=FullProblem.create(task,tmp_path/'problem','PPP')
    record=p.source(task.keyframes[0])['record']
    assert record['source_status']=='structurally_infeasible'
    assert record['source_failure_kind']=='structural_infeasibility'
    assert not record['source_certificate'] and not record['new_source_solver_executed']
    assert {f['maximum_rank'] for f in record['source_diagnostics']['span_failures']}=={3,5}


def test_file_source_refinement_hyperparameters_are_effective_and_reported(tmp_path):
    task=FullTask.load('panel',tmp_path/'task',configuration={'source':{'position_radius_m':.0002,'geometry_weight':0.}})
    p=FullProblem.create(task,tmp_path/'problem','PPP');record=p.source(task.keyframes[0])['record']
    assert record['effective_source_refinement']['position_radius_m']==.0002
    assert record['effective_source_refinement']['geometry_weight']==0.
    before=np.array(record['source_positions_before_m']);after=np.array(record['source_positions_after_m'])
    assert np.max(np.linalg.norm(after-before,axis=1))<=.0002+1e-7


def test_shared_hybrid_override_is_loaded_for_all_task_adapters():
    from importlib.resources import files
    override=files('contact_transfer').joinpath('scenarios/panda_hybrid_control.json')
    for task in ('panel','box','assembly'):
        cfg=load_task_configuration(task,override)
        assert cfg.execution.control_mode=='hybrid_feedback'
        assert cfg.execution.force_d[-1]==160.
    assert load_task_configuration('panel').execution.control_mode=='impedance_feedforward'
