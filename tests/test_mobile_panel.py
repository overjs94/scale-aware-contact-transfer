"""Mobile mechanics, nonholonomic reference, handover and public playback assets."""
import json,sys,time
from pathlib import Path
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from contact_transfer.mobile_panel import MobileCase,build_scene,base_reference,base_acceleration
from contact_transfer.robot import Robot
from contact_transfer.urdf import URDFTree
from contact_transfer.execution import ExecutionTrace
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'web'))
from experiments import prepare_request,Jobs


def assets():
    if not (ROOT/'.cache/rby1/LINK_19_0.obj').exists() or not (ROOT/'.cache/panda_meshes/link7.stl').exists():pytest.skip('Fetch pinned Panda and RBY1 meshes first')


def test_rby_adaptation_preserves_vendor_mechanics_and_labels_unknown_wheel_effort():
    c=MobileCase.load();source=URDFTree(c.path.parent/'rby1.urdf');adapted=c.rby.tree
    for name,link in source.links.items():
        other=adapted.links[name];assert other.mass==link.mass
        np.testing.assert_allclose(other.com,link.com);np.testing.assert_allclose(other.inertia,link.inertia)
    for j in source.joints:
        k=adapted.by_name[j.name];assert k.kind==j.kind and k.parent==j.parent and k.child==j.child
        np.testing.assert_allclose(k.p,j.p);np.testing.assert_allclose(k.R,j.R);np.testing.assert_allclose(k.axis,j.axis)
    meta=json.loads((c.path.parent/'provenance.json').read_text());assert not meta['wheel_effort_upstream_specified']
    assert meta['wheel_effort_simulation_Nm']==40 and c.rby.n==30
    source=json.loads((c.path.parent/c.data['source_file']).read_text());assert source['origin']=='synthetic_mobile_handover_fixture'
    assert next(g for g in source['groups'] if g['group_id']=='under_right')['position'][1]==.01


def test_mobile_URDF_mass_gravity_and_mixed_Jacobian_agree_with_native_simulator(tmp_path):
    mj=pytest.importorskip('mujoco');assets();c=MobileCase.load();build_scene(c,tmp_path/'scene.xml');m=mj.MjModel.from_xml_path(str(tmp_path/'scene.xml'));d=mj.MjData(m)
    joints=[mj.mj_name2id(m,mj.mjtObj.mjOBJ_JOINT,'rby_'+n) for n in c.rby.joint_names];jid=mj.mj_name2id(m,mj.mjtObj.mjOBJ_JOINT,'rby_floating_base');qa,va=m.jnt_qposadr[jid],m.jnt_dofadr[jid]
    q=c.ready.copy();q[c.arm_indices]=c.data['rby_seed'];quat=Rotation.from_euler('xyz',[.02,-.03,.1]).as_quat();pose=np.r_[.2,.1,.01,quat];d.qpos[qa:qa+7]=np.r_[pose[:3],pose[6],pose[3:6]];d.qpos[m.jnt_qposadr[joints]]=q;mj.mj_forward(m,d)
    dense=np.zeros((m.nv,m.nv));mj.mj_fullM(m,d,dense) if mj.__version__.startswith('4.') else mj.mj_fullM(m,dense,d.qM)
    B=np.zeros((m.nv,c.floating.nv));B[va:va+3,3:6]=Rotation.from_quat(quat).as_matrix();B[va+3:va+6,:3]=np.eye(3);B[m.jnt_dofadr[joints],np.arange(6,c.floating.nv)]=1
    qf=np.r_[pose,q];np.testing.assert_allclose(B.T@dense@B,c.floating.mass(qf),atol=2e-7);np.testing.assert_allclose(B.T@d.qfrc_bias,c.floating.gravity(qf),atol=2e-6)
    for name in ['tip_left','tip_right']:
        body=mj.mj_name2id(m,mj.mjtObj.mjOBJ_BODY,'rby_'+name);jp=np.zeros((3,m.nv));jr=jp.copy();mj.mj_jacBody(m,d,jp,jr,body);J=np.vstack((d.xmat[body].reshape(3,3).T@jr@B,jp@B));np.testing.assert_allclose(J,c.floating.jacobian(qf,name),atol=2e-8)
    assert m.neq==0 and m.nu==37
    # The base has a free joint, not commanded x/y slides or a mocap body.
    assert m.jnt_type[jid]==mj.mjtJoint.mjJNT_FREE and m.nmocap==0


def test_differential_path_has_no_sideways_command_and_respects_wheel_velocity():
    c=MobileCase.load();track,radius=map(float,__import__('xml.etree.ElementTree',fromlist=['parse']).parse(c.rby.path).find('mobile').get('params').split())
    for t in np.linspace(0,c.data['duration'],101):
        b,v=base_reference(c,t);a=base_acceleration(c,t)
        assert b[1]==b[2]==v[1]==v[2]==a[1]==a[2]==0
        assert abs(v[0]/radius)<c.rby.velocity[c.rby.joint_names.index('right_wheel')]
    assert base_reference(c,c.data['duration'])[0][0]==1.4


def test_mobile_API_retimes_departure_handover_and_release_together(tmp_path):
    d=prepare_request(dict(preset='mobile_panel',duration=45,refine_source=False),tmp_path/'job')
    assert d['duration']==45 and d['base_start']==10 and d['base_end']==30 and d['fixed_release']==7.5
    assert d['keyframes'][-1]['time']==45 and d['release_time']==40 and not d['source_refinement']
    with pytest.raises(ValueError):prepare_request(dict(preset='mobile_panel',duration=24),tmp_path/'bad')


@pytest.fixture(scope='module')
def mobile_execution(tmp_path_factory):
    pytest.importorskip('mujoco');assets();jobs=Jobs(tmp_path_factory.mktemp('mobile_jobs'));job=jobs.submit(dict(preset='mobile_panel'));deadline=time.monotonic()+480
    while time.monotonic()<deadline:
        status=jobs.get(job['id'])
        if status['state']=='completed':break
        if status['state']=='failed':pytest.fail(str(status)+'\n'+(jobs.path(job['id'])/'worker.log').read_text()[-3000:])
        time.sleep(.2)
    else:jobs.cancel(job['id']);pytest.fail('Mobile worker exceeded 480 seconds')
    root=jobs.path(job['id']);return status['report'],ExecutionTrace.load(root/'result/execution.npz'),json.loads((root/'viewer/result.json').read_text()),root


def test_wheel_driven_handover_extends_workspace_and_releases_into_jig(mobile_execution):
    r,t,data,root=mobile_execution
    assert r['execution_passed'],r
    assert 1.35<r['base_travel_m']<1.45 and r['duration_seconds']==36 and r['keyframes']==9
    assert all(v['provably_outside_fixed_base'] for v in r['workspace_comparison'].values())
    assert all(r['workspace_comparison'][name]['fixed_base_target_distance_m']>r['workspace_comparison'][name]['URDF_chain_radius_bound_m']+.8 for name in ['tip_right','tip_left'])
    assert r['workspace_comparison']['panda']['provably_outside_fixed_base']
    assert r['all_tools_released'] and r['fixture_supported'] and r['jig_support_final_N']==pytest.approx(19.62,abs=.15)
    assert r['carrying_contact_fraction']==[1.,1.] and min(r['wheel_ground_contact_fraction'])>.95
    assert r['wheel_odometry_error_m']<.02 and r['maximum_base_roll_pitch_rad']<.03
    assert not r['mobile_base_pose_prescribed'] and not r['object_pose_prescribed'] and not r['full_geometry_certified']
    assert not r['unexpected_penetrations_m'] and r['minimum_torque_margin_Nm']>=0
    assert t.q.shape[1]==37 and t.object_pose.shape[1]==2
    assert len(data['instances'])==4 and sum(i['floating_base'] for i in data['instances'])==2
    assert data['instances'][0]['role']=='mobile_dual_arm' and data['nq']==51
    assert not (root/'result/worker.log').exists()
