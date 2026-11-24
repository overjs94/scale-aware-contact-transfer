import numpy as np
import pytest

pytest.importorskip("clarabel")

from contact_transfer import Robot
from contact_transfer.conic import fixed_wrench_margin
from contact_transfer.contacts import Capability, SurfacePatch, TaskWrench, contact_basis
from contact_transfer.realization import Arm, realize_keyframe
from contact_transfer.refinement import refine_contacts


def test_fixed_margin_matches_nlp_and_closed_form():
    task = TaskWrench(np.array([0., 0, 0, 0, 0, -9.81]), np.eye(6)[:, 5:], np.eye(1))
    patches = [SurfacePatch.fixed([0, 0, 0], [0, 0, -1])]
    caps = [Capability(normal_max=20)]
    nlp = refine_contacts(patches, task, caps)
    conic = fixed_wrench_margin(nlp.positions, nlp.rotations, task, caps)
    assert conic.rho == pytest.approx(9.81, abs=1e-6)
    assert conic.rho == pytest.approx(nlp.rho, abs=1e-5)
    assert conic.primal_residual < 1e-6


def test_true_circular_friction_cone():
    # Two tangent directions at once: radius is mu*N/sqrt(2), not mu*N.
    E = np.zeros((6, 2)); E[3:5] = np.eye(2)
    task = TaskWrench(np.array([0., 0, 0, 0, 0, 10]), E, np.eye(2))
    result = fixed_wrench_margin([[0, 0, 0]], [np.eye(3)], task, [Capability(mu=.5, normal_max=20)])
    assert result.rho == pytest.approx(5/np.sqrt(2), abs=1e-6)


def test_fixed_target_torque_bound_matches_nlp(slider_urdf):
    arm = Arm(Robot(slider_urdf).symbolic(), "link2", np.array([0, 0, .5]))
    # The vertical robot joint supports 9.81N of robot gravity plus the
    # downward contact load; a 25N limit permits rho <= 5.38N.
    arm.model.effort[2] = 25.
    task = TaskWrench(np.array([0., 0, 0, 0, 0, -9.81]), np.eye(6)[:, 5:], np.eye(1))
    caps = [Capability(normal_max=50)]
    T = np.eye(4); T[2, 3] = .5
    result = realize_keyframe([arm], [SurfacePatch.fixed([0, 0, 0], [0, 0, -1])], caps, [0], T, task)
    conic = fixed_wrench_margin(result.positions, result.rotations, task, caps,
                               arms=[arm], q=result.q, assignment=[0], object_pose=T)
    assert conic.rho == pytest.approx(25-9.81-9.81, abs=2e-5)
    assert conic.rho == pytest.approx(result.rho, abs=2e-5)
    np.testing.assert_allclose(conic.torques, result.torques, atol=1e-4)


def test_rotated_off_origin_gripper_and_zero_margin():
    p = np.array([.2, -.3, .1]); R = contact_basis([1, 0, 0])
    from contact_transfer.contacts import map_reaction
    H = np.eye(6)
    W = np.asarray(map_reaction(p, R, H))
    task = TaskWrench(np.zeros(6), W[:, :2], np.eye(2))
    cap = Capability("G", H=H, bounds=np.array([2., 3, 4, 5, 6, 7]))
    result = fixed_wrench_margin([p], [R], task, [cap])
    assert result.rho == pytest.approx(2., abs=1e-6)
    unsupported = TaskWrench(np.zeros(6), np.eye(6)[:, :1], np.eye(1))
    assert abs(fixed_wrench_margin([[0, 0, 0]], [np.eye(3)], unsupported, [Capability()]).rho) < 1e-6
    with pytest.raises(RuntimeError, match="SOCP rejected"):
        fixed_wrench_margin([[0, 0, 0]], [np.eye(3)], unsupported, [Capability()], rho_min=.1)


def test_reject_invalid_rotation():
    task = TaskWrench(np.zeros(6), np.eye(6)[:, :1], np.eye(1))
    with pytest.raises(ValueError, match="proper rotations"):
        fixed_wrench_margin([[0, 0, 0]], [-np.eye(3)], task, [Capability()])
