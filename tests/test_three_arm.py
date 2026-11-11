from copy import deepcopy
import json
import shutil

import numpy as np
import pytest

from contact_transfer.contacts import Capability, SurfacePatch, TaskWrench
from contact_transfer.conic import fixed_wrench_margin
from contact_transfer.contracted import ContractedCase
from contact_transfer.refinement import refine_contacts
from contact_transfer.robot import Robot
from contact_transfer.simulation import MujocoRobot
from contact_transfer.three_arm import run_three_arm
from contact_transfer.tool_physics import VacuumSeal


def test_supplied_network_groups_keep_ids_when_reordered(tmp_path):
    base = ContractedCase.load()
    prediction = {"schema_version": 1, "units": "SI", "contact_frame": "object",
                  "contracted_contacts": list(reversed(base.data["contracted_contacts"]))}
    path = tmp_path/"network.json"; path.write_text(json.dumps(prediction))
    case = ContractedCase.load(contacts_path=path)
    assert [c["group_id"] for c in case.data["contracted_contacts"]] == [c["group_id"] for c in base.data["contracted_contacts"]]
    assert case.data["prediction_source"] == "supplied_contracted_contacts"
    prediction["contact_frame"] = "world"; path.write_text(json.dumps(prediction))
    with pytest.raises(ValueError, match="object coordinates"):
        ContractedCase.load(contacts_path=path)


def test_suction_adhesion_margin_matches_nlp_and_pressure_bound():
    pytest.importorskip("clarabel")
    cap = Capability("S", mu=.8, normal_max=5, adhesion_max=15)
    task = TaskWrench(np.array([0., 0, 0, 0, 0, -9.81]), np.eye(6)[:, 5:], np.eye(1))
    result = refine_contacts([SurfacePatch.fixed([0, 0, 0], [0, 0, 1])], task, [cap])
    conic = fixed_wrench_margin(result.positions, result.rotations, task, [cap])
    assert result.rho == pytest.approx(5.19, abs=2e-5)
    assert conic.rho == pytest.approx(5.19, abs=1e-6)
    # At the tensile limit no shear is available under the shifted cone.
    shear = TaskWrench(np.array([0., 0, 0, 0, 0, -15]), np.eye(6)[:, 3:4], np.eye(1))
    assert abs(fixed_wrench_margin([[0, 0, 0]], [np.eye(3)], shear, [cap]).rho) < 1e-6


@pytest.mark.parametrize("change", ["duplicate", "normal", "pressure"])
def test_invalid_contracted_prediction_is_rejected(tmp_path, change):
    original = ContractedCase.load()
    for asset in original.path.parent.glob("*.urdf"):
        shutil.copy(asset, tmp_path/asset.name)
    data = deepcopy(original.data)
    if change == "duplicate":
        data["contracted_contacts"][1]["group_id"] = data["contracted_contacts"][0]["group_id"]
    elif change == "normal":
        data["contracted_contacts"][0]["normal_object"] = [0, 0, 1]
    else:
        data["simulation"]["vacuum"]["pressure_pa"] = 100
    path = tmp_path/"case.json"; path.write_text(json.dumps(data))
    with pytest.raises(ValueError):
        ContractedCase.load(path)


@pytest.mark.parametrize("index", range(3))
def test_authored_tool_mechanics_against_pinocchio_and_mujoco(index):
    pytest.importorskip("mujoco")
    case = ContractedCase.load()
    full = Robot(case.robots[index].path, base_pose=case.robots[index].base_pose)
    q = np.full(full.n, .03)
    symbolic = full.symbolic()
    np.testing.assert_allclose(np.asarray(symbolic.mass(q)), full.mass(q), atol=1e-9)
    np.testing.assert_allclose(np.asarray(symbolic.gravity(q)).reshape(-1), full.gravity(q), atol=1e-9)
    sim = MujocoRobot(full); sim.reset(q)
    np.testing.assert_allclose(sim.data.qfrc_bias, full.gravity(q), atol=1e-9)


@pytest.fixture(scope="module")
def execution(tmp_path_factory):
    pytest.importorskip("mujoco"); pytest.importorskip("clarabel")
    return run_three_arm(tmp_path_factory.mktemp("three_arm"))


def test_three_keyframes_two_transitions_and_physical_release(execution):
    plan, trace, seal, report = execution
    assert len(plan.keyframes) == 3 and len(plan.transitions) == 2
    assert plan.trajectory.primal_residual < 2e-6
    assert plan.trajectory.gripper_velocity_residual < 2e-6
    assert report["tracking_passed"] and report["seal_retained_before_vent"] and report["force_envelopes_passed"]
    assert max(e["translation_m"] for e in report["keyframe_errors"]) < .004
    assert [e["event"] for e in seal.events] == ["acquired", "vented"]
    assert trace.object_pose[-1, 0, 2] < trace.object_pose[int(2.7/.001), 0, 2]-.15
    assert any("left_finger" in x for x in trace.contact_pair.reshape(-1))
    assert any("point_tool" in x for x in trace.contact_pair.reshape(-1))
    assert not report["full_geometry_certified"]
    assert report["equality_constraints_used"] == 0


def test_vacuum_clipping_loss_and_equal_opposite_power():
    mj = pytest.importorskip("mujoco")
    case = ContractedCase.load()
    cap = next(c for c in case.capabilities if c.kind == "S")
    model = mj.MjModel.from_xml_string('''<mujoco><option gravity="0 0 0"/><worldbody>
    <body name="cup" quat="0 1 0 0"><joint name="z" type="slide" axis="0 0 1"/><geom type="sphere" size=".01" mass="1" contype="0" conaffinity="0"/></body>
    <body name="panel"><freejoint/><geom type="sphere" size=".01" mass="1" contype="0" conaffinity="0"/></body></worldbody></mujoco>''')
    data = mj.MjData(model)
    seal = VacuumSeal("cup", "panel", [0, 0, 0], [0, 0, 1], cap, case.data["simulation"]["vacuum"], lambda _: True)
    mj.mj_forward(model, data); seal(mj, model, data, 0.)
    assert seal.sealed
    obj = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, "panel")
    adr = model.jnt_qposadr[model.body_jntadr[obj]]
    data.qpos[adr+2] = -.004
    data.qfrc_applied[:] = 0
    mj.mj_forward(model, data); seal(mj, model, data, .001)
    force = np.array(seal.history[-1][6:9])
    np.testing.assert_allclose(force, [0, 0, 15], atol=1e-8)
    # Tool slide points down; action and reaction have equal generalized signs.
    assert data.qfrc_applied[0] == pytest.approx(15.)
    assert data.qfrc_applied[3] == pytest.approx(15.)
    jt, jo = np.zeros((3, model.nv)), np.zeros((3, model.nv))
    mj.mj_jac(model, data, jt, None, data.xpos[obj], mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, "cup"))
    mj.mj_jac(model, data, jo, None, data.xpos[obj], obj)
    velocity = np.arange(model.nv)*.03
    assert data.qfrc_applied@velocity == pytest.approx(force@((jo-jt)@velocity))
    data.qpos[adr+2] = -.01; data.qfrc_applied[:] = 0
    mj.mj_forward(model, data); seal(mj, model, data, .002)
    assert not seal.sealed and seal.events[-1]["event"] == "lost"
    np.testing.assert_allclose(data.qfrc_applied, 0)
