from types import SimpleNamespace

import numpy as np
import pytest

from contact_transfer.contact_simulation import MujocoContactScene, trajectory_reference
from contact_transfer.execution import ExecutionTrace, compare_executions


@pytest.fixture
def scene(tmp_path):
    pytest.importorskip("mujoco")
    path = tmp_path/"contact.xml"
    path.write_text('''<mujoco><option timestep=".002" gravity="0 0 -9.81"/>
    <worldbody><geom name="floor" type="plane" size="2 2 .1"/>
    <body name="arm" pos="1 0 .2"><joint name="slide" type="slide" axis="1 0 0"/>
    <geom name="arm_geom" type="sphere" size=".05" mass="1"/></body>
    <body name="panel" pos="0 0 .3"><freejoint/><geom name="panel_geom" type="box"
    size=".1 .1 .05" mass="1"/></body></worldbody>
    <equality><weld name="hold" body1="panel" active="false"/></equality>
    <actuator><motor name="motor" joint="slide" ctrllimited="true" ctrlrange="-10 10"/></actuator>
    </mujoco>''')
    return path


class PD:
    def command(self, q, v, qr, vr, ar):
        return np.clip(100*(qr-q)+20*(vr-v)+ar, -10, 10)


def stationary(t):
    return np.zeros(1), np.zeros(1), np.zeros(1)


def test_dynamic_object_contact_force_and_trace_roundtrip(scene, tmp_path):
    sim = MujocoContactScene(scene, ["slide"], ["motor"], ["panel"])
    log = sim.run(PD(), stationary, 1., case_id="drop", plan_id="stationary")
    assert log.object_pose[0, 0, 2] == pytest.approx(.3)
    assert log.object_pose[50, 0, 2] < .28  # free fall, no prescribed object pose
    assert log.object_pose[-1, 0, 2] == pytest.approx(.05, abs=.001)
    panel = log.contact_pair[:, 1] == "panel_geom"
    tail = panel & (log.contact_time > .8)
    times = np.unique(log.contact_time[tail])
    # Contacts act on the second geom. Angular-first world force must point up.
    force = np.array([log.contact_wrench[tail & (log.contact_time == t), 5].sum() for t in times])
    np.testing.assert_allclose(force, 9.81, atol=.03)
    assert np.all(log.contact_normal[panel, 2] > .99)
    assert np.isfinite(log.contact_gap).all()
    path = tmp_path/"mujoco.npz"; log.save(path)
    loaded = ExecutionTrace.load(path)
    assert compare_executions(log, loaded)["final_translation_difference_m"] == [0.]
    loaded.metadata["plan_id"] = "different"
    with pytest.raises(ValueError, match="plan_id"):
        compare_executions(log, loaded)


def test_hold_release_and_closed_loop_joint_reference(scene):
    sim = MujocoContactScene(scene, ["slide"], ["motor"], ["panel"])
    reference = lambda t: (np.array([.02]), np.zeros(1), np.zeros(1))
    log = sim.run(PD(), reference, .8, case_id="release", plan_id="move",
                  attachments=[(0., "hold", True), (.2, "hold", False)])
    assert log.object_pose[90, 0, 2] == pytest.approx(.3, abs=.001)
    assert log.object_pose[-1, 0, 2] < .06
    assert log.q[-1, 0] == pytest.approx(.02, abs=.0002)
    assert log.metadata["attachments"][-1] == [.2, "hold", False]


def test_reject_dynamics_only_scene_and_bad_control(scene):
    sim = MujocoContactScene(scene, ["slide"], ["motor"], ["panel"])
    with pytest.raises(ValueError, match="integer multiple"):
        sim.run(PD(), stationary, .003, case_id="x", plan_id="y")
    with pytest.raises(ValueError, match="known equality"):
        sim.run(PD(), stationary, .2, case_id="x", plan_id="y", attachments=[(.1, "missing", True)])
    bad = SimpleNamespace(command=lambda *args: np.array([11.]))
    with pytest.raises(RuntimeError, match="control range"):
        sim.run(bad, stationary, .2, case_id="x", plan_id="y")
    scene.write_text(scene.read_text().replace('<geom name="floor"', '<geom contype="0" conaffinity="0" name="floor"')
                     .replace('<geom name="arm_geom"', '<geom contype="0" conaffinity="0" name="arm_geom"')
                     .replace('<geom name="panel_geom"', '<geom contype="0" conaffinity="0" name="panel_geom"'))
    with pytest.raises(ValueError, match="collision-enabled"):
        MujocoContactScene(scene, ["slide"], ["motor"], ["panel"])


def test_cito_joint_reference():
    result = SimpleNamespace(dt=np.array([.2, .3]), q=np.array([[0., 100], [.1, 200], [.2, 300]]),
                             v=np.array([[0., 0], [0., 0], [0., 0]]))
    ref = trajectory_reference(result, [0], [0])
    np.testing.assert_allclose(ref(.2)[0], [.1])
    np.testing.assert_allclose(ref(.2)[1], [0.])
    with pytest.raises(ValueError, match="outside"):
        ref(.6)


def test_trace_rejects_nan_contact_data(scene):
    log = MujocoContactScene(scene, ["slide"], ["motor"], ["panel"]).run(
        PD(), stationary, .4, case_id="x", plan_id="y")
    log.contact_gap[0] = np.nan
    with pytest.raises(ValueError, match="contact_gap"):
        log.validate()


def test_position_actuator_logs_actual_torque_not_radian_command(scene):
    scene.write_text(scene.read_text().replace('<motor name="motor" joint="slide" ctrllimited="true" ctrlrange="-10 10"/>','<position name="motor" joint="slide" kp="100" kv="20" ctrllimited="true" ctrlrange="-.1 .1" forcelimited="true" forcerange="-2 2"/>'))
    with pytest.raises(ValueError,match='torque mode'):MujocoContactScene(scene,['slide'],['motor'],['panel'])
    sim=MujocoContactScene(scene,['slide'],['motor'],['panel'],actuator_modes=['position'])
    reference=lambda t:(np.array([.04]),np.zeros(1),np.zeros(1))
    controller=SimpleNamespace(command=lambda q,v,qr,vr,ar:qr)
    log=sim.run(controller,reference,1.,case_id='position',plan_id='step')
    assert log.torque[0,0]==pytest.approx(2.)  # 4 Nm demanded, capped at 2
    assert abs(log.torque[-1,0])<.01 and log.q[-1,0]==pytest.approx(.04,abs=.0001)
    assert log.metadata['actuator_modes']==['position']


def test_passive_robot_coordinate_is_observed_without_an_actuator(scene):
    scene.write_text(scene.read_text().replace('<motor name="motor" joint="slide" ctrllimited="true" ctrlrange="-10 10"/>',''))
    sim=MujocoContactScene(scene,['slide'],[None],['panel'],actuator_modes=['passive'])
    controller=SimpleNamespace(command=lambda *args:np.zeros(1))
    log=sim.run(controller,stationary,.02,case_id='passive',plan_id='observe')
    assert sim.model.nu==0 and log.q.shape==(11,1)
    np.testing.assert_array_equal(log.torque,0.)
    bad=SimpleNamespace(command=lambda *args:np.ones(1))
    with pytest.raises(RuntimeError,match='passive observed joint'):sim.run(bad,stationary,.02,case_id='passive',plan_id='bad')
