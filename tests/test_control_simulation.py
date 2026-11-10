import numpy as np
import pytest

from contact_transfer import Robot
from contact_transfer.control import ComputedTorque, position_servo_step
from contact_transfer.embodiments import load
from contact_transfer.simulation import MujocoRobot


@pytest.mark.parametrize("name", ["franka_panda", "rb3", "rb5", "kuka_iiwa7", "ur5e", "kinova_gen3", "xarm7"])
def test_mujoco_computed_torque_tracking(name):
    pytest.importorskip("mujoco")
    r = load(name)
    sim = MujocoRobot(r)
    q0 = np.maximum(r.lower+.1, np.minimum(r.upper-.1, r.neutral()))
    sim.reset(q0)
    target = np.clip(q0+.02, r.lower+.05, r.upper-.05)
    controller = ComputedTorque(r, kp=100., kd=20.)
    log = sim.run(controller, lambda _: (target, np.zeros(r.nv), np.zeros(r.nv)), 700)
    assert np.isfinite(log["q"]).all()
    assert np.linalg.norm(sim.state()[0]-target) < 2e-4
    assert np.max(np.abs(log["tau"])/r.effort) <= 1+1e-12


def test_position_servo(slider_urdf):
    r = Robot(slider_urdf)
    command, velocity = position_servo_step(r, [0, 0, 0], "link2",
                                            [0, 0, 0, .1, -.2, .3], .01)
    np.testing.assert_allclose(command, np.array([.001, -.002, .003]), atol=1e-7)
    assert (abs(velocity) <= r.velocity).all()
