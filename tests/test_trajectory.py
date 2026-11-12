import numpy as np
import casadi as ca
import pytest

from contact_transfer.contacts import Capability
from contact_transfer.system import MechanicalSystem, RigidObject
from contact_transfer.trajectory import Contact, solve_cito
from contact_transfer import Robot
from contact_transfer.embodiments import load, catalog


@pytest.mark.parametrize('friction_speed_model',['auxiliary_cone','direct_norm'])
def test_cito_support_impulse_and_unrelaxed_residuals(friction_speed_model):
    body = RigidObject(1., np.zeros(3), np.eye(3)*.02)
    system = MechanicalSystem([], body)
    q = np.array([0., 0, .5, 0, 0, 0, 1])
    contact = Contact("ground", np.zeros(3), [0, 0, 1],
                      Capability(mu=.5, normal_max=30), plane_offset=.5)
    result = solve_cito(system, q, q, [contact], steps=8, duration=.4,friction_speed_model=friction_speed_model)
    assert result.complementarity_residual < 2e-5
    assert result.friction_residual < 2e-5
    assert result.minimum_gap >= -2e-6
    assert not result.full_geometry_checked
    # Integrating the momentum balance requires exactly the gravity impulse.
    np.testing.assert_allclose(result.impulses["ground"][:, 2].sum(), 9.81*.4, atol=2e-5)


@pytest.mark.parametrize('eliminate_endpoint_redundancy',[False,True])
def test_dual_arm_attached_object_translation(slider_urdf,eliminate_endpoint_redundancy):
    robot = Robot(slider_urdf).symbolic()
    system = MechanicalSystem([robot, robot], RigidObject(1., np.zeros(3), np.eye(3)*.02))
    q = np.r_[-.25, 0, .5, .25, 0, .5, 0, 0, .5, 0, 0, 0, 1]
    goal = q.copy()
    goal[[2, 5, 8]] += .03
    cap = Capability.force_gripper([50., 50., 50.])
    contacts = [Contact("left", [-.25, 0, 0], [0, 0, -1], cap,
                        robot_index=0, frame="link2", attached_steps=tuple(range(8))),
                Contact("right", [.25, 0, 0], [0, 0, -1], cap,
                        robot_index=1, frame="link2", attached_steps=tuple(range(8)))]
    result = solve_cito(system, q, goal, contacts, steps=8, duration=.8,eliminate_endpoint_redundancy=eliminate_endpoint_redundancy)
    np.testing.assert_allclose(result.q[-1], goal, atol=1e-6)
    assert result.primal_residual < 2e-6
    assert result.gripper_velocity_residual < 2e-6


def test_floating_robot_environment_contact_velocity():
    numeric = load("unitree_g1")
    robot = numeric.symbolic()
    system = MechanicalSystem([robot], RigidObject(1., np.zeros(3), np.eye(3)*.02))
    frame = catalog()["unitree_g1"]["frame"]
    contact = Contact("robot_plane", [.02, 0, 0], [0, 0, 1],
                      environment_robot_index=0, frame=frame)
    qs = ca.SX.sym("q", system.nq)
    gap, J, _ = contact.geometry(system, qs)
    fn = ca.Function("contact_geometry", [qs], [gap, J])
    q = np.r_[numeric.neutral(), 0., 0, 1, 0, 0, 0, 1]
    v = np.random.default_rng(2).normal(0, .1, system.nv)
    h = 1e-6
    plus, minus = system.integrate(q, h*v), system.integrate(q, -h*v)
    finite_difference = (float(fn(plus)[0])-float(fn(minus)[0]))/(2*h)
    np.testing.assert_allclose(np.array(fn(q)[1])[2]@v, finite_difference, atol=1e-7)
