from types import SimpleNamespace
import numpy as np

from contact_transfer.embodiments import load
from contact_transfer.ik import solve_ik
from contact_transfer.sequence import choose_sequence


def test_sequence_compares_all_feasible_types_and_checks_transitions():
    visited = []
    def keyframe(k, types, assignment, previous):
        visited.append((k, types))
        return SimpleNamespace(objective=4. if types[0] == "P" else 1.)
    def transition(k, left, right, types, assignments):
        return SimpleNamespace(objective=0.)
    result = choose_sequence([("P", "G")], [((0,), (0,))], keyframe, transition)
    assert result.best.types == ("G",)
    assert len(result.candidates) == 2 and len(visited) == 4
    def reject_gripper(k, left, right, types, assignments):
        if types[0] == "G":
            raise RuntimeError("transition infeasible")
        return SimpleNamespace(objective=0.)
    result = choose_sequence([("P", "G")], [((0,), (0,))], keyframe, reject_gripper)
    assert result.best.types == ("P",) and len(result.failures) == 1


def test_urdf_ik_initialization_reaches_known_pose():
    robot = load("franka_panda")
    q0 = np.array([0., -.4, 0., -1.8, 0., 1.4, .7])
    target = robot.fk(q0+np.array([.01, -.02, .01, .01, -.01, .02, -.01]), "panda_link8")
    result = solve_ik(robot.symbolic(), "panda_link8", target, q0)
    assert result.position_error < 2e-5 and result.rotation_cost < 1e-8
