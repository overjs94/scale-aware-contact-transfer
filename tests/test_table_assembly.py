"""Two-free-body mechanics, assembly execution and Python worker integration."""
import json,sys,time
from pathlib import Path
import casadi as ca
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from contact_transfer.optimization import equality
from contact_transfer.table_assembly import AssemblyCase,build_assembly_scene,object_reference
from contact_transfer.execution import ExecutionTrace
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'web'))
from experiments import Jobs,prepare_request

@pytest.mark.parametrize('value',[ca.DM([1,0]),ca.DM([float('nan')])])
def test_constant_dynamics_rows_must_not_hide_inconsistency(value):
    with pytest.raises(ValueError,match='constant equality'):equality(ca.Opti(),value)
    equality(ca.Opti(),ca.DM.zeros(6))


def test_assembly_object_inertias_against_native_simulator(tmp_path):
    mj=pytest.importorskip('mujoco')
    if not (ROOT/'.cache/panda_meshes/link7.stl').exists():pytest.skip('Fetch Panda collision meshes first')
    case=AssemblyCase.load();build_assembly_scene(case,tmp_path/'scene.xml')
    model=mj.MjModel.from_xml_path(str(tmp_path/'scene.xml'));data=mj.MjData(model);mj.mj_forward(model,data)
    dense=np.zeros((model.nv,model.nv));mj.mj_fullM(model,data,dense) if mj.__version__.startswith('4.') else mj.mj_fullM(model,dense,data.qM)
    for part,spec,pose in zip(case.parts,case.data['objects'],object_reference(case,0)[0]):
        j=mj.mj_name2id(model,mj.mjtObj.mjOBJ_JOINT,spec['id']+'_free');adr=model.jnt_dofadr[j];R=Rotation.from_quat(pose[3:]).as_matrix()
        B=np.zeros((model.nv,6));B[adr:adr+3,3:]=R;B[adr+3:adr+6,:3]=np.eye(3)
        np.testing.assert_allclose(B.T@dense@B,part.mass(pose),atol=1e-9)
        np.testing.assert_allclose(B.T@data.qfrc_bias,part.gravity(pose),atol=1e-8)
    assert model.neq==0 and len(case.data['keyframes'])==8
    assert model.npair>0 and any(model.pair_dim==4)


@pytest.mark.parametrize('payload',[{'preset':'table_assembly','duration':20},{'preset':'table_assembly','duration':float('nan')},{'preset':'table_assembly','duration':30.0001},{'preset':'table_assembly','case':'/etc/passwd'}])
def test_assembly_job_input_is_bounded(tmp_path,payload):
    with pytest.raises(ValueError):prepare_request(payload,tmp_path/'job')


def test_assembly_job_retimes_objects_and_release_together(tmp_path):
    d=prepare_request({'preset':'table_assembly','duration':36},tmp_path/'job')
    assert d['keyframes'][-1]['time']==36
    assert d['simulation']['release_time']==pytest.approx(33.6)
    assert d['simulation']['seat_time']==pytest.approx(32.4)
    assert len(d['keyframes'])==8 and [a['kind'] for a in d['arms']]==['G','P','G']


@pytest.fixture(scope='module')
def execution(tmp_path_factory):
    pytest.importorskip('mujoco')
    if not (ROOT/'.cache/panda_meshes/link7.stl').exists():pytest.skip('Fetch Panda collision meshes first')
    jobs=Jobs(tmp_path_factory.mktemp('assembly_jobs'));job=jobs.submit({'preset':'table_assembly','duration':30});seen=set();deadline=time.monotonic()+240
    while time.monotonic()<deadline:
        status=jobs.get(job['id']);seen.add(status['stage'])
        if status['state']=='completed':break
        if status['state']=='failed':pytest.fail(str(status)+'\n'+(jobs.path(job['id'])/'worker.log').read_text()[-2500:])
        time.sleep(.2)
    else:
        jobs.cancel(job['id']);pytest.fail('Assembly worker exceeded 240 seconds')
    root=jobs.path(job['id']);trace=ExecutionTrace.load(root/'result/execution.npz');data=json.loads((root/'viewer/result.json').read_text())
    return status['report'],trace,data,root


def test_30_second_grip_support_reorient_insert_and_release(execution):
    r,t,data,root=execution
    assert r['execution_passed'],r
    assert r['duration_seconds']==30 and r['keyframes']==8
    assert r['tool_types']==['G','P','G'] and r['objects']==2 and t.q.shape[1]==25
    assert r['equality_constraints_used']==0 and r['leg_released'] and r['assembly_seated']
    assert .020<r['insertion_depth_m']<.030 and r['insertion_lateral_error_m']<.002
    assert r['sampled_collision_check_passed'] and not r['full_geometry_certified']
    initial=Rotation.from_quat(t.object_pose[0,0,3:]);k=int(round(12/.001));rot=Rotation.from_quat(t.object_pose[k,0,3:])
    assert (initial.inv()*rot).magnitude()==pytest.approx(np.deg2rad(12),abs=.015)
    assert len(data['instances'])==5 and sum(i['floating_base'] for i in data['instances'])==2
    assert not data['tool_states']['leg_gripper_contact'][-1] and data['tool_states']['socket_contact'][-1]
    assert (root/'viewer/models/assembly_tabletop/robot.urdf').is_file()
    assert not (root/'result/worker.log').exists() and not (root/'result/status.json').exists()
    # Both free bodies are observed throughout; leg was lifted off its cradle.
    lift=int(round(6/.001));assert t.object_pose[lift,1,2]>t.object_pose[0,1,2]+.10
    np.testing.assert_allclose(np.linalg.norm(t.object_pose[:,:,3:],axis=2),1,atol=1e-8)


def test_socket_is_empty_geometry_with_a_blind_floor():
    import xml.etree.ElementTree as ET
    case=AssemblyCase.load();root=ET.parse(case.path.parent/'tabletop.urdf');boxes=[]
    for g in root.findall('link/collision'):
        p=np.array(g.find('origin').get('xyz').split(),float);half=np.array(g.find('geometry/box').get('size').split(),float)/2;boxes.append((p,half))
    center=np.array(case.data['socket']['center_object'])
    inside=lambda point:any(np.all(abs(point-p)<=half+1e-10) for p,half in boxes)
    for depth in [.005,.015,.025]:assert not inside(center-[0,0,depth])
    assert inside(center-[0,0,.035])
    assert inside(center+[case.data['socket']['width']/2+.002,0,-.01])
