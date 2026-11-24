"""Functional/link-aware contraction and genuine free articulated multi-arm realization."""
from pathlib import Path
import json,sys
import numpy as np
import pytest
from contact_transfer.free_articulated import FreeCase,run_free,build_scene,object_reference,contact_jacobian
from contact_transfer.free_articulated_view import export_free
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'web'))
from experiments import prepare_request

def assets():
 if not (ROOT/'.cache/panda_meshes/link7.stl').exists():pytest.skip('Fetch pinned meshes')

@pytest.mark.parametrize('id',['free_box_lid','free_folding_object','free_portable_drawer'])
def test_free_multibody_mass_gravity_and_contact_jacobians_against_native(id,tmp_path):
 mj=pytest.importorskip('mujoco');assets();c=FreeCase.load(id);build_scene(c,tmp_path/'scene.xml');m=mj.MjModel.from_xml_path(str(tmp_path/'scene.xml'));d=mj.MjData(m);j=mj.mj_name2id(m,mj.mjtObj.mjOBJ_JOINT,'object_floating_base');h=mj.mj_name2id(m,mj.mjtObj.mjOBJ_JOINT,'object_articulation');qa,va=m.jnt_qposadr[j],m.jnt_dofadr[j];q=object_reference(c,8)[0];d.qpos[qa:qa+7]=np.r_[q[:3],q[6],q[3:6]];d.qpos[m.jnt_qposadr[h]]=q[-1];mj.mj_forward(m,d)
 assert m.nu==25 and m.neq==m.nmocap==0
 assert m.jnt_type[j]==mj.mjtJoint.mjJNT_FREE and h not in m.actuator_trnid[:,0]
 R=c.object.fk(q,'base')[:3,:3];B=np.zeros((m.nv,7));B[va:va+3,3:6]=R;B[va+3:va+6,:3]=np.eye(3);B[m.jnt_dofadr[h],-1]=1;M=np.zeros((m.nv,m.nv));mj.mj_fullM(m,d,M) if mj.__version__.startswith('4.') else mj.mj_fullM(m,M,d.qM)
 np.testing.assert_allclose(B.T@M@B,c.object.mass(q),atol=1e-9);np.testing.assert_allclose(B.T@d.qfrc_bias,c.object.gravity(q),atol=1e-8)
 for i,r in enumerate(c.full):
  joints=[mj.mj_name2id(m,mj.mjtObj.mjOBJ_JOINT,f'arm{i}_'+n) for n in r.joint_names];jd=m.jnt_dofadr[joints];qr=d.qpos[m.jnt_qposadr[joints]]
  np.testing.assert_allclose(M[np.ix_(jd,jd)],r.mass(qr),atol=1e-8);np.testing.assert_allclose(d.qfrc_bias[jd],r.gravity(qr),atol=1e-8)
 for s in c.data['arms']:
  body=mj.mj_name2id(m,mj.mjtObj.mjOBJ_BODY,'object_urdf_base' if s['link_id']=='base' else 'object_moving');T=c.object.fk(q,s['link_id']);point=T[:3,3]+T[:3,:3]@s['point_link'];jp=np.zeros((3,m.nv));jr=jp.copy();mj.mj_jac(m,d,jp,jr,point,body);np.testing.assert_allclose(np.vstack((jr@B,jp@B)),contact_jacobian(c.object,q,s['link_id'],s['point_link']),atol=1e-9)


def test_functional_link_identity_enables_internal_joint_control():
 c=FreeCase.load();q=object_reference(c,8)[0];maps=[contact_jacobian(c.object,q,s['link_id'],s['point_link']).T[:,3:] if s['kind']=='P' else contact_jacobian(c.object,q,s['link_id'],s['point_link']).T for s in c.data['arms']]
 assert np.linalg.matrix_rank(np.c_[tuple(maps)])==7
 # Dropping moving-link identity removes all authority over the internal hinge.
 lost=[contact_jacobian(c.object,q,'base',s['point_link']).T for s in c.data['arms']]
 assert np.linalg.matrix_rank(np.c_[tuple(lost)])==6 and all(np.allclose(J[-1],0) for J in lost)
 assert c.contraction['dense_sample_count']==125 and c.contraction['contracted_contact_count']==3
 assert [s['link_id'] for s in c.data['arms']]==['base','base','moving']
 assert c.contraction['contraction_executed'] and not c.contraction['network_inference_executed']

@pytest.mark.parametrize('id',['free_box_lid','free_folding_object','free_portable_drawer'])
def test_native_three_arm_free_articulated_realization(id,tmp_path):
 pytest.importorskip('mujoco');assets();p,t,r,f=run_free(id,tmp_path/'run');assert r['execution_passed'],r
 assert r['arm_count']==3 and r['contact_fraction']==[1.,1.,1.]
 assert not r['world_hinge'] and r['object_joint_actuators']==0 and not r['object_pose_prescribed']
 assert r['base_max_position_error_m']<.002 and not r['unexpected_penetrations_m']
 assert min(r['peak_contact_force_per_arm_N'])>1
 # Verify all 7 object generalized force equations using independently assembled maps.
 for k in [0,len(p.times)//2,-1]:
  qo,vo,ao=object_reference(p.case,p.times[k]);rhs=p.case.object.inverse_dynamics(qo,vo,ao);rhs[-1]+=p.case.data['damping']*vo[-1]+p.case.data['frictionloss']*np.tanh(vo[-1]/.001)
  actual=sum(contact_jacobian(p.case.object,qo,s['link_id'],s['point_link']).T@w for s,w in zip(p.case.data['arms'],p.wrenches[k]));np.testing.assert_allclose(actual,rhs,atol=1e-6)
 data=export_free(p,t,r,f,tmp_path/'viewer');assert len(data['instances'])==4 and len(data['objects'])==2 and data['nq']==33 and data['nv']==32
 assert data['instances'][-1]['floating_base'] and data['contact_contraction']['dense_sample_count']==125


def test_free_API_retimes_lift_and_internal_motion_and_consumes_dense_input(tmp_path):
 d=prepare_request(dict(preset='free_box_lid',duration=36),tmp_path/'job');assert d['lift_start']==4 and d['lift_end']==8 and d['motion_end']==26
 c=FreeCase.load(path=tmp_path/'job/case.json');assert c.contraction['contracted_contact_count']==3
 with pytest.raises(ValueError):prepare_request(dict(preset='free_box_lid',dense_source_groups=[]),tmp_path/'bad')
