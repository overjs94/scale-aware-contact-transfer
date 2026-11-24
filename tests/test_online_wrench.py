from dataclasses import replace
from types import SimpleNamespace

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from contact_transfer.configuration import load_task_configuration
from contact_transfer.contacts import Capability
from contact_transfer.finger_grasp import FingerGrasp
from contact_transfer.online_wrench import WrenchAllocator,operating_capabilities


@pytest.mark.parametrize('backend',['clarabel','scs'])
def test_allocator_respects_circular_friction_torque_and_inactive_interfaces(backend):
    pytest.importorskip(backend)
    from contact_transfer.conic_backends import ConicSettings
    caps=[Capability(mu=.5,normal_max=20)]*2
    solver=WrenchAllocator(caps,settings=ConicSettings(backend=backend))
    maps=[np.eye(6),np.eye(6)];T=[np.eye(6),np.eye(6)]
    desired=np.array([0,0,0,3,4,10.])
    result=solver.solve(maps,desired,np.zeros((2,6)),T,np.zeros(6),np.full(6,30.),[True,False])
    assert result['accepted']
    np.testing.assert_allclose(result['reactions'][0],desired,atol=1e-5)
    np.testing.assert_allclose(result['reactions'][1],0,atol=1e-5)
    # A box relaxation would incorrectly accept this diagonal tangential load.
    desired[3:5]=4.
    assert not solver.solve(maps,desired,np.zeros((2,6)),T,np.zeros(6),np.full(6,30.),[True,False])['accepted']
    desired[3:5]=0
    assert not solver.solve(maps,desired,np.zeros((2,6)),T,np.zeros(6),np.full(6,5.),[True,False])['accepted']


def test_gripper_operating_budget_is_separate_and_witness_is_physical():
    cap=FingerGrasp(.04,.01,.01,.8,40.)
    effective,=operating_capabilities([cap],.45)
    assert cap.closing_force_per_finger==40 and effective.closing_force_per_finger==18
    solver=WrenchAllocator([effective]);w=np.array([0,0,0,0,0,20.])
    result=solver.solve([np.eye(6)],w,[w],[np.eye(6)],np.zeros(6),np.full(6,100),[True])
    assert result['accepted'] and max(result['jaw_required'][0])<=18+1e-6
    assert effective.check(result['reactions'][0])['feasible']
    w[5]=30  # mu*(18+18) = 28.8 N.
    assert not solver.solve([np.eye(6)],w,[w],[np.eye(6)],np.zeros(6),np.full(6,100),[True])['accepted']


def test_partial_release_reports_unactuated_request_instead_of_full_dynamics_certificate():
    # The held body's force can be controlled; the released body's force
    # cannot be independently prescribed by a robot with no contact on it.
    cap=Capability(normal_max=20);M=np.zeros((2,6));M[0,5]=1
    args=([M],np.array([10.,3.]),np.zeros((1,6)),[np.zeros((1,6))],np.zeros(1),np.ones(1),[True])
    assert not WrenchAllocator([cap]).solve(*args)['accepted']
    result=WrenchAllocator([cap],project_unactuated=True).solve(*args)
    assert result['accepted'] and result['controlled_dynamics_rank']==1
    assert not result['full_dynamics_equality_enforced']
    np.testing.assert_allclose(result['unactuated_request'],[0,3],atol=1e-9)
    np.testing.assert_allclose(M@result['reactions'][0],[10,0],atol=1e-6)


def test_control_envelope_validation_and_roundtrip():
    cfg=load_task_configuration('panel',{'execution':{'online_allocation':True,'allocation_grip_fraction':.45}})
    from contact_transfer.configuration import configuration_from_dict
    assert configuration_from_dict(cfg.to_dict())==cfg
    with pytest.raises(ValueError,match='Fixed jaw'):
        replace(cfg,execution=replace(cfg.execution,allocation_grip_fraction=.6))
    with pytest.raises(ValueError,match='reserve'):
        replace(cfg,execution=replace(cfg.execution,adaptive_jaw_force=True,allocation_grip_fraction=.8))


def test_reference_transport_matches_derivative_in_rotating_actual_basis():
    from contact_transfer.online_wrench import reference_tangent
    obj=SimpleNamespace(nq=7,nv=6)
    R=Rotation.from_euler('xyz',[.3,-.2,.7]);Rd=Rotation.from_euler('xyz',[-.1,.4,-.2])
    q=np.r_[[.1,.2,.3],R.as_quat()];qr=np.r_[[-.2,.4,.5],Rd.as_quat()]
    v=np.array([.2,-.3,.4,1.,.2,-.1]);vr=np.array([-.1,.2,.3,-.2,.4,.1]);ar=np.array([.4,-.2,.1,.2,.1,-.3])
    _,velocity,acceleration=reference_tangent(obj,q,qr,v,vr,ar)
    h=1e-7
    Rn=R*Rotation.from_rotvec(h*v[:3]);Rdn=Rd*Rotation.from_rotvec(h*vr[:3])
    Qn=Rn.as_matrix().T@Rdn.as_matrix();next_velocity=np.r_[Qn@(vr[:3]+h*ar[:3]),Qn@(vr[3:]+h*ar[3:])]
    np.testing.assert_allclose((next_velocity-velocity)/h,acceleration,atol=1e-7)


def test_planning_and_native_hardware_have_distinct_explicit_force_limits(tmp_path):
    from contact_transfer.full_scenario import FullProblem,FullTask
    task=FullTask.load('panel',tmp_path/'task',configuration={'target':{'gripper_force_fraction':.6}})
    p=FullProblem.create(task,tmp_path/'models','GPP')
    assert p.capabilities[0].closing_force_per_finger==24
    assert p.hardware_capabilities[0].closing_force_per_finger==40
    assert p.specs[0]['gripper_profile']['closing_force_limit_N']==40


def test_released_object_does_not_request_an_unactuated_tracking_qp():
    from contact_transfer.full_execution import FullContactController
    c=object.__new__(FullContactController);c.t=3.;c.policy=SimpleNamespace(online_allocation=True,allocation_dt_s=.02)
    feature=SimpleNamespace(id='site',role='base')
    c.problem=SimpleNamespace(task_definition=SimpleNamespace(features=[feature]))
    c.episode=SimpleNamespace(preparation=2.,phase=lambda t:(0,0,1.),reactions=lambda t:np.zeros((1,6)),attached=lambda j,t:False,keyframes=[SimpleNamespace(phase='released')])
    c.last_active=None;c.next_allocation=0.;c.allocation_scope_logs=[];c.allocated=None
    np.testing.assert_array_equal(c.desired_reactions(None,None,None,None,None),0)
    assert not c.allocation_scope_logs[0]['qp_executed']
    c.episode.reactions=lambda t:np.ones((1,6))
    with pytest.raises(RuntimeError,match='No holding'):
        c.desired_reactions(None,None,None,None,None)


def test_native_observer_recovers_unknown_force_without_schedule_or_applied_force_input(tmp_path):
    mj=pytest.importorskip('mujoco')
    from contact_transfer.robot import Robot
    from contact_transfer.full_execution import FullContactController
    from contact_transfer.full_validation import numeric_object_point
    path=tmp_path/'object.urdf'
    path.write_text('''<robot name="object"><link name="base"><inertial><origin xyz=".02 .03 .01"/><mass value="2"/><inertia ixx=".08" iyy=".09" izz=".1" ixy="0" ixz="0" iyz="0"/></inertial></link></robot>''')
    obj=Robot(path,floating_base=True)
    model=mj.MjModel.from_xml_string('''<mujoco><option timestep=".0002" integrator="implicitfast"/><worldbody><body name="object"><freejoint/><inertial pos=".02 .03 .01" mass="2" diaginertia=".08 .09 .1"/></body></worldbody></mujoco>''')
    data=mj.MjData(model);data.qpos[3:7]=Rotation.from_euler('xyz',[.3,.2,-.4]).as_quat()[[3,0,1,2]];data.qvel[3:]=[.2,-.3,.1]
    c=object.__new__(FullContactController)
    c.problem=SimpleNamespace(task_definition=SimpleNamespace(object=obj,id='test'),specs=[])
    c.scene=SimpleNamespace(object_roots={'object':'object'},object_joints={})
    c.dt=.0002;c.policy=SimpleNamespace(disturbance_filter_s=.04);c.previous_object_v=None;c.disturbance=np.zeros(6);c.geom_frames={}
    force=np.array([2.,-3.,4.])
    expected_filtered=np.zeros(6)
    for _ in range(2500):
        mj.mj_forward(model,data)
        c.observe_object(mj,model,data,{},[])
        _,J=numeric_object_point(obj,c.object_state[0],'base',np.array([.02,.03,.01]))
        if c.previous_object_v is not None:
            expected_filtered+=c.dt/(c.policy.disturbance_filter_s+c.dt)*(J[3:].T@force-expected_filtered)
        data.xfrc_applied[1,3:]=0;data.xfrc_applied[1,:3]=force
        mj.mj_step(model,data)
    q,v,M,bias,env=c.object_state
    _,J=numeric_object_point(obj,q,'base',np.array([.02,.03,.01]))
    # Compare like-filtered moving-frame signals; comparing a low-pass output
    # to the instantaneous rotated wrench would confuse filter lag with error.
    np.testing.assert_allclose(c.disturbance,expected_filtered,atol=.003)
    # Independent native generalized mass/bias comparison in the same convention.
    R=Rotation.from_quat(q[3:7]).as_matrix();B=np.zeros((6,6));B[:3,3:]=R;B[3:,:3]=np.eye(3)
    mass=np.zeros((6,6));mj.mj_fullM(model,data,mass)
    np.testing.assert_allclose(M,B.T@mass@B,atol=1e-9)
