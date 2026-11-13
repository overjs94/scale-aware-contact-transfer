"""Passive joints, independent mechanics, native contact execution and web job boundary."""
import json
from pathlib import Path
import sys
import numpy as np
import pytest
from contact_transfer.articulated import ArticulatedCase,build_articulated_scene,run_articulated,point_jacobian
from contact_transfer.articulated_view import export_articulated_view
from contact_transfer.execution import ExecutionTrace
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'web'))
from experiments import prepare_request

def assets():
    if not (ROOT/'.cache/panda_meshes/link7.stl').exists():pytest.skip('Fetch pinned Panda meshes first')

@pytest.mark.parametrize('id',['hinged_door','folding_panel','drawer'])
def test_passive_object_mass_gravity_and_point_Jacobian_against_native(id,tmp_path):
    mj=pytest.importorskip('mujoco');assets();c=ArticulatedCase.load(id);build_articulated_scene(c,tmp_path/'scene.xml');m=mj.MjModel.from_xml_path(str(tmp_path/'scene.xml'));d=mj.MjData(m)
    j=mj.mj_name2id(m,mj.mjtObj.mjOBJ_JOINT,'object_object_joint');b=mj.mj_name2id(m,mj.mjtObj.mjOBJ_BODY,'object_moving');a=m.jnt_dofadr[j];q=[.1 if id=='drawer' else .4];d.qpos[m.jnt_qposadr[j]]=q[0];mj.mj_forward(m,d)
    assert m.nmocap==m.neq==0 and j not in m.actuator_trnid[:,0]
    assert m.nu==c.full.n
    M=np.zeros((m.nv,m.nv));mj.mj_fullM(m,d,M) if mj.__version__.startswith('4.') else mj.mj_fullM(m,M,d.qM)
    np.testing.assert_allclose(M[a,a],c.object.mass(q)[0,0],atol=1e-10)
    np.testing.assert_allclose(d.qfrc_bias[a],c.object.gravity(q)[0],atol=1e-10)
    assert c.object.jacobian(q,'moving').shape==(6,1)
    T=c.object.fk(q,'moving');point=T[:3,3]+T[:3,:3]@c.data['contact_object'];jp=np.zeros((3,m.nv));jr=jp.copy();mj.mj_jac(m,d,jp,jr,point,b)
    np.testing.assert_allclose(jp[:,a:a+1],point_jacobian(c.object,q,'moving',c.data['contact_object']),atol=1e-10)

@pytest.mark.parametrize('id',['hinged_door','folding_panel','drawer'])
def test_articulated_native_execution_and_measured_URDF_export(id,tmp_path):
    pytest.importorskip('mujoco');assets();p,t,r,f=run_articulated(id,tmp_path/'run')
    assert r['execution_passed'],r
    assert r['object_joint_actuators']==0 and not r['object_pose_prescribed']
    assert not r['network_inference_executed'] and not r['source_contact_refinement_executed']
    assert not r['unexpected_penetrations_m'] and r['peak_contact_force_N']<40
    assert not r['full_geometry_certified']
    saved=ExecutionTrace.load(tmp_path/'run/execution.npz');np.testing.assert_allclose(saved.metadata['articulated_state']['q'],t.metadata['articulated_state']['q'])
    data=export_articulated_view(p,t,r,f,tmp_path/'viewer')
    assert len(data['instances'])==2 and data['nq']==p.case.full.n+1
    assert data['q'][-1][-1]==pytest.approx(t.metadata['articulated_state']['q'][-1][0],abs=1e-8)
    assert data['q'][-1][-1]!=r['target_coordinate'] # measured, not prescribed
    assert len(data['keyframes'])==5

def test_articulated_API_retimes_only_bounded_preset_inputs(tmp_path):
    d=prepare_request(dict(preset='drawer',duration=28),tmp_path/'job')
    assert d['motion_start']==4 and d['motion_end']==22 and d['sample_dt']==.5
    assert (tmp_path/'job/drawer.urdf').exists()
    for k,payload in enumerate([dict(preset='drawer',duration=8),dict(preset='hinged_door',object_urdf='/tmp/evil'),dict(preset='folding_panel',duration=float('nan'))]):
        with pytest.raises(ValueError):prepare_request(payload,tmp_path/str(k))
