"""Real Panda mechanics and local job boundary regressions."""
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
import numpy as np
import pytest
from contact_transfer.contracted import ContractedCase
from contact_transfer.robot import Robot
from contact_transfer.three_arm import build_three_arm_scene,run_three_arm
from contact_transfer.three_arm_view import export_three_arm_view

ROOT=Path(__file__).resolve().parents[1]
CASE=ROOT/'src/contact_transfer/scenarios/three_franka/case.json'
sys.path.insert(0,str(ROOT/'web'))
from experiments import Jobs,prepare_request


def test_real_panda_mechanics_preserved_and_independent_mujoco(tmp_path):
    mj=pytest.importorskip('mujoco')
    case=ContractedCase.load(CASE)
    source=ET.parse(ROOT/'web/models/franka_panda/robot.urdf')
    for robot in case.robots:
        assert robot.n==7
        tree=ET.parse(robot.path)
        for name in [f'panda_link{i}' for i in range(9)]:
            assert ET.tostring(tree.find(f"link[@name='{name}']/inertial"))==ET.tostring(source.find(f"link[@name='{name}']/inertial"))
    if not (ROOT/'.cache/panda_meshes/link7.stl').exists():pytest.skip('Run tools/fetch_panda_meshes.py for native mesh checks')
    from types import SimpleNamespace
    full=build_three_arm_scene(SimpleNamespace(case=case),tmp_path/'scene.xml')
    model=mj.MjModel.from_xml_path(str(tmp_path/'scene.xml'));data=mj.MjData(model)
    for robot,spec in zip(full,case.data['arms']):
        q=np.r_[spec['q_initial'],[.035,.035]] if robot.n==9 else np.array(spec['q_initial'])
        indices=[model.jnt_qposadr[mj.mj_name2id(model,mj.mjtObj.mjOBJ_JOINT,spec['id']+'_'+name)] for name in robot.joint_names]
        dofs=[model.jnt_dofadr[mj.mj_name2id(model,mj.mjtObj.mjOBJ_JOINT,spec['id']+'_'+name)] for name in robot.joint_names]
        data.qpos[indices]=q;mj.mj_forward(model,data)
        np.testing.assert_allclose(data.qfrc_bias[dofs],robot.gravity(q),atol=1e-8)
        symbolic=robot.symbolic()
        np.testing.assert_allclose(np.asarray(symbolic.mass(q)),robot.mass(q),atol=1e-9)
        np.testing.assert_allclose(np.asarray(symbolic.jacobian(q,'tip')),robot.jacobian(q,'tip'),atol=1e-9)


@pytest.mark.parametrize('payload',[{'preset':'../../tmp'},{'mass':float('nan')},{'mass':5},{'unknown':'field'},{'contacts':[]}])
def test_job_rejects_untrusted_parameters(tmp_path,payload):
    with pytest.raises((ValueError,KeyError)):prepare_request(payload,tmp_path/'job')


def test_job_cancel_and_single_resource_gate(tmp_path):
    jobs=Jobs(tmp_path)
    job=jobs.submit({})
    with pytest.raises(BlockingIOError):jobs.submit({})
    assert jobs.cancel(job['id'])['state']=='cancelled'
    with pytest.raises(ValueError):jobs.path('../escape')


def test_real_franka_free_panel_execution_and_export(tmp_path):
    pytest.importorskip('mujoco');pytest.importorskip('clarabel')
    if not (ROOT/'.cache/panda_meshes/link7.stl').exists():pytest.skip('Run tools/fetch_panda_meshes.py for physical checks')
    plan,trace,seal,report=run_three_arm(tmp_path/'run',case_path=CASE)
    assert plan.system.nu==21 and trace.q.shape[1]==23
    assert report['planner']=='fixed_contact_path' and not report['complementarity_applicable']
    assert report['tracking_passed'] and report['force_envelopes_passed']
    assert report['seal_retained_before_vent'] and seal.events==[{'time':0.,'event':'acquired'}]
    assert report['sampled_collision_check_passed'] and report['equality_constraints_used']==0
    assert max(k['translation_m'] for k in report['keyframe_errors'])<.002
    assert not report['full_geometry_certified']
    # Independent object force-balance check against prescribed acceleration.
    from contact_transfer.contacts import contact_basis,map_reaction,gravity_wrench
    for k,h in enumerate(plan.trajectory.dt):
        total=np.zeros(6)
        for c,cap in zip(plan.case.data['contracted_contacts'],plan.case.capabilities):
            force=plan.trajectory.impulses[c['group_id']][k]/h
            lam=np.r_[np.zeros(3),force] if cap.kind=='P' else force
            total+=np.asarray(map_reaction(c['point_object'],contact_basis(c['normal_object']),lam)).reshape(6)
        np.testing.assert_allclose(total[:3],0,atol=1e-8)
        assert abs(total[5]+9.81)<.2
    data=export_three_arm_view(plan,trace,report,tmp_path/'viewer')
    assert data['id']=='three_franka' and data['events']==[]
    assert len(data['instances'])==3 and data['tool_states']['gripper_contact'][-1]
    assert json.loads((tmp_path/'viewer/models/three_franka_point/model.json').read_text())['packages']
