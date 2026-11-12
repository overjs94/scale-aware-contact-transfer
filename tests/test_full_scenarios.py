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


def test_box_handle_has_connected_visual_and_collision_attachment():
    """A shared rigid link cannot substitute for missing physical material."""
    import xml.etree.ElementTree as ET
    tree=ET.parse('src/contact_transfer/scenarios/free_articulated/box_lid.urdf')
    for name in ('base','moving'):
        link=tree.find(f"link[@name='{name}']");geometries=[]
        for kind in ('visual','collision'):
            bounds=[]
            for geom in link.findall(kind):
                center=np.fromstring(geom.find('origin').get('xyz'),sep=' ');half=np.fromstring(geom.find('geometry/box').get('size'),sep=' ')/2
                bounds.append((center-half,center+half))
            # Every primitive must be connected, including the handle.
            visited={0}
            while True:
                connected={j for i in visited for j in range(len(bounds)) if np.all(np.maximum(bounds[i][0],bounds[j][0])<=np.minimum(bounds[i][1],bounds[j][1])+1e-12)}
                if connected<=visited:break
                visited|=connected
            assert len(visited)==len(bounds),f'{name}/{kind}: disconnected primitive'
            geometries.append(np.array(bounds))
        np.testing.assert_array_equal(*geometries)
    # Mass/COM/inertia are declared for the aggregate body including hardware.
    assert tree.find("link[@name='base']/inertial/mass").get('value')=='2.0'


def test_native_contact_export_preserves_measured_time_sign_and_patch_centers():
    from types import SimpleNamespace as NS
    from contact_transfer.full_view import native_contact_samples
    p=NS(task_definition=NS(features=[NS(id='grasp')]),specs=[{'id':'arm0'}]);e=NS(problem=p)
    trace=NS(metadata={'timestep':.001},time=np.array([0.,.001,.002]),contact_time=np.array([0.,0.,.001,.002]),contact_pair=np.array([['arm0_left_pad_collision_0','wood'],['wood','arm0_left_pad_collision_1'],['arm0_left_pad_collision_0','wood'],['arm0_left_pad_collision_0','fixture']]),contact_wrench=np.array([[0,0,0,2,0,0],[0,0,0,-1,0,0],[0,0,0,0,0,0],[0,0,0,100,0,0]],float),contact_position=np.array([[0,0,0],[3,0,0],[4,0,0],[9,0,0]],float),contact_gap=np.array([-.001,-.002,0,-.003]))
    result=native_contact_samples(e,trace,NS(allowed_by_group={'grasp':['wood']}),np.array([0,1,2]))
    assert result['interpolated'] is False and result['sample_times_s']==[0.,.001,.002]
    assert result['frames'][1:]==[[],[]]  # no invented contact after loss
    patch=result['frames'][0][0];np.testing.assert_allclose(patch['position_world_m'],[1,0,0]);np.testing.assert_allclose(patch['force_on_robot_world_N'],[-3,0,0]);assert patch['minimum_gap_m']==-.002


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
    # Released-tool reference geometry must use the native open mechanism,
    # without changing any arm or free-object coordinates.
    old=d.qpos.copy();opened=[s['group_id'] for s in p.specs if s['kind']=='G'];g.set_state(q,open_grippers=opened)
    for spec,locked in zip(p.specs,g.locked):
        if spec['kind']=='G':
            indices=list(locked);np.testing.assert_allclose(d.qpos[indices],m.qpos0[indices],atol=1e-12)
            assert np.linalg.norm(old[indices]-d.qpos[indices])>.1
    for adr in g.addresses:np.testing.assert_allclose(d.qpos[adr],old[adr],atol=1e-12)


def test_complete_interval_cache_preserves_formulation_and_rejects_other_model(tmp_path):
    r=TrajectoryResult(np.zeros((3,2)),np.zeros((3,2)),np.zeros((2,2)),np.ones(2),{'p':np.zeros((2,3))},1.,0.,0.,0.,0.,False,(1e-3,1e-8));r.contact_force_coordinates='scaled_tangent';r.configuration_update='independent_midpoint';path=tmp_path/'interval.npz';save_interval(path,r,'model-A');loaded=load_interval(path,'model-A');assert loaded.contact_force_coordinates=='scaled_tangent' and loaded.configuration_update=='independent_midpoint';np.testing.assert_array_equal(loaded.impulses['p'],r.impulses['p'])
    with pytest.raises(ValueError,match='mismatch'):load_interval(path,'model-B')


def test_contact_coordinates_choose_native_nonsingular_grasp_and_point_charts(tmp_path):
    import casadi as ca
    from contact_transfer.trajectory import Contact,contact_coordinate_charts,point_axis_velocity_jacobian
    if not Path('.cache/robotiq_2f85/2f85.xml').exists():pytest.skip('Fetch pinned native gripper assets')
    task=FullTask.load('box',tmp_path/'task');p=FullProblem.create(task,tmp_path/'problem','GPG');q=p.branch_reference(0);contacts=[]
    for i,(feature,cap,frame) in enumerate(zip(task.features,p.capabilities,p.frames)):
        contacts.append(Contact(feature.id,feature.surface,feature.normal,cap,robot_index=i,frame=frame,object_frame=feature.frame,attached_steps=(0,) if cap.kind=='G' else (),sticking_steps=(0,) if cap.kind=='P' else (),align_normal=cap.kind=='P'))
    charts=contact_coordinate_charts(p.system,contacts,[q])
    assert [charts[c.name]['rank'] for c in contacts]==[6,5,6]
    assert [len(charts[c.name]['independent']) for c in contacts]==[1,2,1]
    for c in contacts:
        _,J,_,_=numeric_contact(p,c,q);A=J[:,p.system.v_slices[c.robot_index]]
        if c.capability.kind=='P':
            symbol=ca.MX.sym('q',p.system.nq);actual=np.array(ca.Function('axis',[symbol],[point_axis_velocity_jacobian(p.system,c,symbol)])(q));R=task.object.fk(q[p.system.object_q],c.object_frame)[:3,:3]@c.basis();native=np.zeros_like(actual);native[:,p.system.v_slices[c.robot_index]]=R[:,:2].T@p.robots[c.robot_index].jacobian(q[p.system.q_slices[c.robot_index]],c.frame,'world_aligned')[:3];native[:,p.system.object_v]=-R[:,:2].T@task.object.jacobian(q[p.system.object_q],c.object_frame,'world_aligned')[:3];np.testing.assert_allclose(actual,native,atol=1e-10);A=np.vstack([native[:,p.system.v_slices[c.robot_index]],A]);scale=np.r_[np.full(2,.6),np.ones(3)]
        else:scale=np.r_[np.full(3,.6),np.ones(3)]
        sigma=np.linalg.svd((scale[:,None]*A)[:,charts[c.name]['dependent']],compute_uv=False)[-1]
        assert sigma>1e-7
        np.testing.assert_allclose(sigma,charts[c.name]['minimum_reference_singular_value'],atol=1e-10)


def test_required_support_load_uses_plan_when_force_feedforward_is_disabled():
    from types import SimpleNamespace
    from contact_transfer.full_assessment import planned_normal_reactions
    plan=SimpleNamespace(dt=np.array([.2,.4]),impulses={'support':np.array([[0.,0.,1.],[0.,0.,0.]])})
    def forbidden(*args):raise AssertionError('Execution commands or measured forces must not determine required contact')
    episode=SimpleNamespace(trajectories=[plan],phase=lambda t:(0,0 if t<.2 else 1,t),reactions=forbidden,force_feedforward=False)
    load=planned_normal_reactions(episode,'support',[.1,.2,.3])
    np.testing.assert_allclose(load,[5.,0.,0.])
    np.testing.assert_array_equal(load>.1,[True,False,False])


def test_repeated_panel_dwell_has_exactly_the_same_ik_branch_reference(tmp_path):
    if not Path('.cache/robotiq_2f85/2f85.xml').exists():pytest.skip('Fetch pinned native gripper assets')
    task=FullTask.load('panel',tmp_path/'task');p=FullProblem.create(task,tmp_path/'problem','GPP')
    np.testing.assert_array_equal(p.branch_reference(0),p.branch_reference(1))
    assert np.linalg.norm(p.branch_reference(2)-p.branch_reference(1))>.01
