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
        return SimpleNamespace(objective=0., full_geometry_checked=True)
    result = choose_sequence([("P", "G")], [((0,), (0,))], keyframe, transition)
    assert result.best.types == ("G",)
    assert len(result.candidates) == 2 and len(visited) == 4
    def reject_gripper(k, left, right, types, assignments):
        if types[0] == "G":
            raise RuntimeError("transition infeasible")
        return SimpleNamespace(objective=0., full_geometry_checked=True)
    result = choose_sequence([("P", "G")], [((0,), (0,))], keyframe, reject_gripper)
    assert result.best.types == ("P",) and len(result.failures) == 1


def test_urdf_ik_initialization_reaches_known_pose():
    robot = load("franka_panda")
    q0 = np.array([0., -.4, 0., -1.8, 0., 1.4, .7])
    target = robot.fk(q0+np.array([.01, -.02, .01, .01, -.01, .02, -.01]), "panda_link8")
    result = solve_ik(robot.symbolic(), "panda_link8", target, q0)
    assert result.position_error < 2e-5 and result.rotation_cost < 1e-8


def test_sequence_prunes_only_dominated_realized_endpoint_cost_bounds():
    visited=[]
    def keyframe(k,types,assignment,previous):return SimpleNamespace(objective=-5. if types[0]=='G' else -1.)
    def transition(k,left,right,types,assignments):
        visited.append(types);return SimpleNamespace(objective=.5,full_geometry_checked=True)
    pruned=choose_sequence([('G','P')],[((0,),(0,))],keyframe,transition,transition_cost_lower_bound=0.)
    exhaustive=choose_sequence([('G','P')],[((0,),(0,))],keyframe,transition)
    assert pruned.best.types==exhaustive.best.types==('G',)
    assert pruned.best.objective==exhaustive.best.objective
    assert len(pruned.pruned)==1 and not pruned.failures
    assert pruned.pruned[0]['types']==('P',)
    assert visited==[('G',),('G',),('P',)]
