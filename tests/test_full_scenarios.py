"""Independent native mechanics for complete-task adapters and stage records."""
from pathlib import Path
import numpy as np
import pytest
from contact_transfer.full_scenario import FullTask,FullProblem
from contact_transfer.full_scene import build_scene,support_contacts
from contact_transfer.full_validation import Geometry,numeric_contact,support_patch_contains
from contact_transfer.full_recording import save_interval,load_interval
from contact_transfer.trajectory import TrajectoryResult
from contact_transfer.multi_object import RigidCollection


@pytest.mark.parametrize('task_id,types',[('panel','GPP'),('box','GPG'),('assembly','GPG')])
def test_complete_scenes_preserve_unactuated_objects_and_native_mechanics(task_id,types,tmp_path):
    mj=pytest.importorskip('mujoco')
    if not Path('.cache/robotiq_2f85/2f85.xml').exists():pytest.skip('Fetch pinned native gripper assets')
    task=FullTask.load(task_id,tmp_path/'task');p=FullProblem.create(task,tmp_path/'problem',types);scene=build_scene(p,tmp_path/'scene.xml');g=Geometry(p,scene);q=np.r_[*[r.neutral() for r in p.robots],task.keyframes[-1]['object_q']];g.set_state(q);m,d=g.model,g.data;M=np.zeros((m.nv,m.nv));mj.mj_fullM(m,d,M) if mj.__version__.startswith('4.') else mj.mj_fullM(m,M,d.qM)
    assert m.neq==3*types.count('G') and m.nmocap==0
    for i,(r,spec,qs) in enumerate(zip(p.robots,p.specs,p.system.q_slices)):
        ids=[mj.mj_name2id(m,mj.mjtObj.mjOBJ_JOINT,spec['id']+'_'+n) for n in r.joint_names];v=m.jnt_dofadr[ids];np.testing.assert_allclose(M[np.ix_(v,v)],r.mass(q[qs]),atol=1e-8);np.testing.assert_allclose(d.qfrc_bias[v],r.gravity(q[qs]),atol=1e-8)
    obj=task.object;B=np.zeros((m.nv,obj.nv));qo=q[p.system.object_q]
    for name,body in scene.object_roots.items():
        bid=mj.mj_name2id(m,mj.mjtObj.mjOBJ_BODY,body);jid=m.body_jntadr[bid];adr=m.jnt_dofadr[jid];sl=obj.v_slices[name] if isinstance(obj,RigidCollection) else slice(0,obj.nv);frame=name if isinstance(obj,RigidCollection) else obj.tree.root;R=obj.fk(qo,frame)[:3,:3];B[adr:adr+3,sl.start+3:sl.start+6]=R;B[adr+3:adr+6,sl.start:sl.start+3]=np.eye(3);assert m.jnt_type[jid]==mj.mjtJoint.mjJNT_FREE
    if task_id=='box':
        jid=mj.mj_name2id(m,mj.mjtObj.mjOBJ_JOINT,scene.object_joints['articulation']);B[m.jnt_dofadr[jid],-1]=1;assert jid not in m.actuator_trnid[:,0]
    np.testing.assert_allclose(B.T@M@B,obj.mass(qo),atol=1e-9);np.testing.assert_allclose(B.T@d.qfrc_bias,obj.gravity(qo),atol=1e-8)
    assert all(cap.closing_force_per_finger==40 for cap in p.capabilities if cap.kind=='G')
    for c in support_contacts(p,task.keyframes[-1]['phase'],8):assert abs(numeric_contact(p,c,q)[0])<1e-10 and support_patch_contains(p,c,q)


def test_complete_interval_cache_preserves_formulation_and_rejects_other_model(tmp_path):
    r=TrajectoryResult(np.zeros((3,2)),np.zeros((3,2)),np.zeros((2,2)),np.ones(2),{'p':np.zeros((2,3))},1.,0.,0.,0.,0.,False,(1e-3,1e-8));r.contact_force_coordinates='scaled_tangent';r.configuration_update='independent_midpoint';path=tmp_path/'interval.npz';save_interval(path,r,'model-A');loaded=load_interval(path,'model-A');assert loaded.contact_force_coordinates=='scaled_tangent' and loaded.configuration_update=='independent_midpoint';np.testing.assert_array_equal(loaded.impulses['p'],r.impulses['p'])
    with pytest.raises(ValueError,match='mismatch'):load_interval(path,'model-B')
