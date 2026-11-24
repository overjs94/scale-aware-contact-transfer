"""Changed source inputs and local endpoints must never hit a stale cache."""
import json
from types import SimpleNamespace

import numpy as np
import pytest

from contact_transfer.full_recording import fingerprint
from contact_transfer.full_scenario import FullTask, FullProblem
from contact_transfer.optimization import SolverSettings
from contact_transfer.provenance import stage_identity


@pytest.fixture
def point_problem(tmp_path):
    task = FullTask.load('panel', tmp_path / 'task')
    source = task.original.path.parent / task.original.data['source_file']
    copied = tmp_path / task.original.data['source_file']
    copied.parent.mkdir(parents=True, exist_ok=True)
    for value in task.original.data.values():
        if isinstance(value,str) and value.endswith(('.json','.urdf','.srdf')):
            target=tmp_path/value;target.parent.mkdir(parents=True,exist_ok=True)
            target.write_bytes((task.original.path.parent/value).read_bytes())
    task.original = SimpleNamespace(path=tmp_path / 'case.json', data=task.original.data.copy())
    return FullProblem.create(task, tmp_path / 'problem', 'PPP'), copied


@pytest.mark.parametrize('section,name,value', [
    ('refinement', 'normal_max_N', .01), ('refinement', 'mu', .4),
    ('refinement', 'radius_m', .0001), ('object', 'mass', .1)])
def test_source_changes_invalidate_without_moving_anchors(point_problem, section, name, value):
    problem, source = point_problem
    before = fingerprint(problem, SolverSettings(), 8)
    data = json.loads(source.read_text())
    data[section][name] = value
    source.write_text(json.dumps(data))
    assert fingerprint(problem, SolverSettings(), 8) != before


def test_case_fixture_and_diagnostics_identity(point_problem):
    problem, _ = point_problem
    settings = SolverSettings()
    before = fingerprint(problem, settings, 8)
    settings.diagnostics_path = 'different-output.json'
    settings.verbose = True
    assert fingerprint(problem, settings, 8) == before
    problem.task_definition.fixtures = ({'size': [1, 2, 3]},)
    assert fingerprint(problem, settings, 8) != before


def test_missing_declared_source_fails_closed(point_problem):
    problem, source = point_problem
    source.unlink()
    with pytest.raises(FileNotFoundError):
        fingerprint(problem, SolverSettings(), 8)


def test_actual_endpoints_and_previous_contacts_are_part_of_stage_identity():
    left = SimpleNamespace(q=np.zeros(3), positions=np.zeros((1, 3)), phase='hold')
    right = SimpleNamespace(q=np.ones(3), positions=np.zeros((1, 3)), phase='hold')
    before = stage_identity('input', 'interval', 0, left=left, right=right)
    right.q[0] += .01
    assert stage_identity('input', 'interval', 0, left=left, right=right) != before
    before = stage_identity('input', 'keyframe', 1, previous=left)
    left.positions[0, 0] += .001
    assert stage_identity('input', 'keyframe', 1, previous=left) != before


def test_source_reuse_isolated_from_output_mutation_and_invalidated_by_input(point_problem):
    problem,source=point_problem;key=problem.task_definition.keyframes[0]
    first=problem.source(key)
    assert first['record']['source_certificate'] and not first['record']['source_cache_hit']
    first['positions'][0,0]+=1.
    second=problem.source(key)
    assert second['record']['source_cache_hit'] and not second['record']['new_source_solver_executed']
    assert abs(second['positions'][0,0]-first['positions'][0,0])>.5
    data=json.loads(source.read_text());data['refinement']['normal_max_N']=.01;source.write_text(json.dumps(data))
    with pytest.raises(RuntimeError,match='PrimalInfeasible'):
        problem.source(key)


def test_live_planner_rejects_inputs_changed_after_problem_construction(point_problem,tmp_path):
    from contact_transfer.full_pipeline import FullPlanner
    p,source=point_problem;planner=FullPlanner(p.task_definition,tmp_path/'planner')
    planner.problems[p.types,p.assignment]=p
    planner.identities[p.types,p.assignment]=fingerprint(p,planner.settings,planner.steps)
    source.write_text(source.read_text()+'\n')
    with pytest.raises(ValueError,match='Planning inputs changed'):
        planner.keyframe(0,p.types,p.assignment,None)


def test_inspectable_manifest_serializes_nested_configuration(point_problem,tmp_path):
    from contact_transfer.provenance import planning_manifest
    from contact_transfer.full_recording import write_json
    p,_=point_problem;destination=tmp_path/'manifest.json'
    write_json(destination,planning_manifest(p,SolverSettings(),8))
    record=json.loads(destination.read_text())
    assert record['schema']==2
    assert record['configuration']['target']['posture_weight']==.01
    assert record['conic_settings']['backend']=='clarabel'


def test_optimizer_archive_requires_matching_full_manifest_including_timing(point_problem,tmp_path):
    from contact_transfer.provenance import planning_manifest
    from contact_transfer.full_recording import write_json,archive_manifest_matches
    p,_=point_problem;folder=tmp_path/'archive';identity=fingerprint(p,SolverSettings(),8)
    assert not archive_manifest_matches(folder,identity)
    write_json(folder/'planning-manifest.json',planning_manifest(p,SolverSettings(),8))
    assert archive_manifest_matches(folder,identity)
    manifest=json.loads((folder/'planning-manifest.json').read_text())
    manifest['keyframes'][1]['time']+=.1
    write_json(folder/'planning-manifest.json',manifest)
    assert not archive_manifest_matches(folder,identity)
