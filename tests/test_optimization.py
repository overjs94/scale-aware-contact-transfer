import numpy as np
import pytest

from contact_transfer import Robot
from contact_transfer.contacts import Capability, SurfacePatch, TaskWrench
from contact_transfer.refinement import refine_contacts
from contact_transfer.realization import Arm, realize_keyframe


def vertical_task():
    return TaskWrench(np.array([0., 0, 0, 0, 0, -9.81]),
                      np.array([[0., 0, 0, 0, 0, 1]]).T, np.eye(1))


def test_source_margin_matches_closed_form():
    result = refine_contacts([SurfacePatch.fixed([0, 0, 0], [0, 0, -1])],
                             vertical_task(), [Capability(normal_max=20)])
    assert result.rho == pytest.approx(9.81, abs=1e-5)
    assert result.primal_residual < 1e-6


def test_target_realization_urdf_torque_balance(slider_urdf):
    model = Robot(slider_urdf).symbolic()
    T = np.eye(4)
    T[2, 3] = .5
    result = realize_keyframe([Arm(model, "link2", [0, 0, .5])],
                             [SurfacePatch.fixed([0, 0, 0], [0, 0, -1])],
                             [Capability(normal_max=20)], [0], T, vertical_task())
    np.testing.assert_allclose(result.q[0], [0, 0, .5], atol=1e-6)
    assert result.rho == pytest.approx(9.81, abs=2e-5)
    assert result.primal_residual < 1e-6
    # Robot object-on-agent reaction acts downward, increasing required torque.
    np.testing.assert_allclose(sorted(result.torques[:, 2]), [9.81, 29.43], atol=1e-4)


def test_unsupported_disturbance_does_not_get_a_false_certificate():
    task = TaskWrench(np.array([0., 0, 0, 0, 0, -9.81]),
                      np.array([[1., 0, 0, 0, 0, 0]]).T, np.eye(1))
    with pytest.raises((RuntimeError, ValueError)):
        refine_contacts([SurfacePatch.fixed([0, 0, 0], [0, 0, -1])],
                        task, [Capability(normal_max=20)], rho_min=.1)
