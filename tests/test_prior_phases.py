import numpy as np
import pytest

from contact_transfer.contacts import Capability, SurfacePatch, TaskWrench
from contact_transfer.pipeline import source_prior, scale_prior
from contact_transfer.system import MechanicalSystem, RigidObject
from contact_transfer.trajectory import Contact, solve_cito
from contact_transfer.optimization import SolverSettings


def test_source_prior_zero_margin_is_distinct_from_failure():
    patch = SurfacePatch.fixed([0, 0, 0], [0, 0, -1])
    task = TaskWrench(np.zeros(6), np.array([[0., 0, 0, 0, 0, 1]]).T, np.eye(1))
    prior = source_prior([patch], task, group_ids=["support"])
    assert prior.source_certificate and prior.source_status == "feasible_zero_margin"
    failed = source_prior([patch], task, group_ids=["support"], rho_min=.1)
    assert not failed.source_certificate and failed.source_rho is None
    transferred = scale_prior(failed, 3)
    assert transferred["group_ids"] == ("support",)
    np.testing.assert_array_equal(transferred["positions"], [[0, 0, 0]])


def test_required_point_maintenance_and_sticking():
    system = MechanicalSystem([], RigidObject(1., np.zeros(3), np.eye(3)*.02))
    q = np.array([0., 0, .5, 0, 0, 0, 1])
    contact = Contact("required", [0, 0, 0], [0, 0, 1],
                      Capability(normal_max=30), plane_offset=.5,
                      maintained_steps=tuple(range(8)), sticking_steps=tuple(range(8)))
    result = solve_cito(system, q, q, [contact], steps=8, duration=.4)
    assert abs(result.minimum_gap) < 2e-6
    assert result.gripper_velocity_residual < 2e-6


def test_conflicting_contact_phases_rejected():
    with pytest.raises(ValueError):
        Contact("bad", [0, 0, 0], [0, 0, 1], maintained_steps=(1,), released_steps=(1,))
    with pytest.raises(ValueError):
        SolverSettings(backend="unknown")
