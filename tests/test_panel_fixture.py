import json,sys,time
from pathlib import Path
import numpy as np
import pytest
from contact_transfer.control import rb_position_command
from contact_transfer.panel_fixture import PanelCase,build_panel_scene,object_reference
from contact_transfer.execution import ExecutionTrace
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'web'))
from experiments import Jobs,prepare_request


def test_rb_command_units_are_explicit_and_no_hardware_is_contacted():
    assert rb_position_command(['joint'],[np.pi],units='deg')['positions']==pytest.approx([180])
    payload=rb_position_command(['joint'],[1.]);assert payload['positions']==[1.] and payload['units']=='rad'
    assert not payload['hardware_transmission_executed']
    with pytest.raises(ValueError):rb_position_command(['joint'],[float('nan')])


def test_rb5_native_position_actuator_tracks_under_urdf_limits(tmp_path):
    pytest.importorskip('mujoco')
    if not (ROOT/'.cache/rb5_meshes/link6.stl').exists():pytest.skip('Fetch RB5 collision meshes first')
    from contact_transfer.rb_position import run_rb5_position_servo
    r,t=run_rb5_position_servo(tmp_path)
    assert r['execution_passed'],r
    assert r['urdf_effort_limits_Nm']==[10.]*6
    assert t.metadata['actuator_modes']==['position']*6
    assert not r['panel_task_executed'] and not r['hardware_transmission_executed']
    # Native full-URDF mechanics independently agree with Pinocchio.
    import mujoco as mj
    from contact_transfer.robot import Robot
    case=PanelCase.load();spec=case.data['arms'][1];T=np.eye(4);T[:3,3]=[1.5,0,0]
    robot=Robot(case.path.parent/'rb5_point.urdf',base_pose=T)
    m=mj.MjModel.from_xml_path(str(tmp_path/'scene.xml'));d=mj.MjData(m)
    joints=[mj.mj_name2id(m,mj.mjtObj.mjOBJ_JOINT,'rb5_'+n) for n in robot.joint_names];vadr=m.jnt_dofadr[joints]
    q=np.array([.03,.015,-.015,.01,0,.02]);d.qpos[m.jnt_qposadr[joints]]=q;mj.mj_forward(m,d)
    dense=np.zeros((m.nv,m.nv));mj.mj_fullM(m,d,dense) if mj.__version__.startswith('4.') else mj.mj_fullM(m,dense,d.qM)
    np.testing.assert_allclose(dense[np.ix_(vadr,vadr)],robot.mass(q),atol=1e-8)
    np.testing.assert_allclose(d.qfrc_bias[vadr],robot.gravity(q),atol=1e-8)
    body=mj.mj_name2id(m,mj.mjtObj.mjOBJ_BODY,'rb5_tip');jp=np.zeros((3,m.nv));jr=jp.copy();mj.mj_jacBody(m,d,jp,jr,body)
    native=np.vstack((d.xmat[body].reshape(3,3).T@jr[:,vadr],jp[:,vadr]));np.testing.assert_allclose(native,robot.jacobian(q,'tip'),atol=1e-8)



def test_panel_mass_and_gravity_are_independently_imported_from_urdf(tmp_path):
    mj=pytest.importorskip('mujoco')
    if not (ROOT/'.cache/panda_meshes/link7.stl').exists():pytest.skip('Fetch Panda meshes')
    case=PanelCase.load();build_panel_scene(case,tmp_path/'scene.xml');m=mj.MjModel.from_xml_path(str(tmp_path/'scene.xml'));d=mj.MjData(m);mj.mj_forward(m,d)
    dense=np.zeros((m.nv,m.nv));mj.mj_fullM(m,d,dense) if mj.__version__.startswith('4.') else mj.mj_fullM(m,dense,d.qM)
    from scipy.spatial.transform import Rotation
    pose=object_reference(case,0)[0];j=mj.mj_name2id(m,mj.mjtObj.mjOBJ_JOINT,'panel_free');adr=m.jnt_dofadr[j];B=np.zeros((m.nv,6));B[adr:adr+3,3:]=Rotation.from_quat(pose[3:]).as_matrix();B[adr+3:adr+6,:3]=np.eye(3)
    np.testing.assert_allclose(B.T@dense@B,case.panel.mass(pose),atol=1e-9);np.testing.assert_allclose(B.T@d.qfrc_bias,case.panel.gravity(pose),atol=1e-8)
    assert m.neq==0


def test_panel_retimes_load_transfer_and_retreat_with_refinement_ablation(tmp_path):
    d=prepare_request(dict(preset='panel_fixture',duration=36,refine_source=False),tmp_path/'job')
    assert not d['source_refinement'] and d['keyframes'][-1]['time']==36
    assert d['simulation']['release_time']==pytest.approx(31.2)
    assert d['simulation']['load_transfer_end']==30
    assert d['simulation']['retract_end']==pytest.approx(34.8)
    with pytest.raises(ValueError):prepare_request(dict(preset='panel_fixture',refine_source='false'),tmp_path/'bad')


@pytest.fixture(scope='module')
def panel_execution(tmp_path_factory):
    pytest.importorskip('mujoco')
    if not (ROOT/'.cache/panda_meshes/link7.stl').exists():pytest.skip('Fetch Panda meshes')
    jobs=Jobs(tmp_path_factory.mktemp('panel_jobs'));job=jobs.submit(dict(preset='panel_fixture'));deadline=time.monotonic()+300
    while time.monotonic()<deadline:
        status=jobs.get(job['id'])
        if status['state']=='completed':break
        if status['state']=='failed':pytest.fail(str(status)+'\n'+(jobs.path(job['id'])/'worker.log').read_text()[-2500:])
        time.sleep(.2)
    else:jobs.cancel(job['id']);pytest.fail('Panel worker exceeded 300 seconds')
    root=jobs.path(job['id']);return status['report'],ExecutionTrace.load(root/'result/execution.npz'),json.loads((root/'viewer/result.json').read_text())


def test_panel_main_task_refines_carries_seats_and_releases(panel_execution):
    r,t,data=panel_execution
    assert r['execution_passed'],r
    assert r['objects']==1 and r['keyframes']==8 and r['duration_seconds']==30
    assert r['source']['source_contact_positions_optimized'] and r['source']['synthetic_input']
    assert r['source']['source_object']['mass']==.03125 and r['target_mass_kg']==2
    assert r['all_tools_released'] and r['fixture_supported'] and not r['full_geometry_certified']
    assert r['jig_support_final_N']==pytest.approx(19.62,abs=.15)
    assert r['minimum_execution_torque_margin_Nm']>=-1e-6
    assert all(v['rho']>0 and v['primal_residual']<1e-6 for v in r['target_validation'])
    assert t.q.shape[1]==23 and len(data['instances'])==5 and len(data['objects'])==1
    assert sum(i['floating_base'] for i in data['instances'])==1
    assert data['instances'][-1]['role']=='static_jig'


def test_panel_job_accepts_supplied_contracted_geometry_and_rejects_bad_groups(tmp_path):
    from contact_transfer.source_transfer import source_fixture
    source=source_fixture();source['groups'][1]['position'][0]=.024
    prepare_request(dict(preset='panel_fixture',contracted_source=source),tmp_path/'good')
    assert json.loads((tmp_path/'good/source_contacts.json').read_text())['groups'][1]['position'][0]==.024
    source['groups'][1]['group_id']='unknown'
    with pytest.raises(ValueError,match='groups must match'):prepare_request(dict(preset='panel_fixture',contracted_source=source),tmp_path/'bad')
