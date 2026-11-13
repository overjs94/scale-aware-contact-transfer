"""Independent equilibrium, sign and closure checks for uploaded-paper adapters."""
import casadi as ca
import numpy as np
from contact_transfer import Robot
from contact_transfer.contacts import Capability,SurfacePatch,contact_basis
from contact_transfer.generalized_load import GeneralizedTaskLoad
from contact_transfer.generalized_conic import fixed_generalized_margin
from contact_transfer.system import MechanicalSystem,RigidObject
from contact_transfer.paper_keyframe import Interface,realize_system_keyframe,relative_interface


def test_global_realization_matches_independent_point_socp_and_native_inverse_dynamics(slider_urdf):
    robot=Robot(slider_urdf);sys=MechanicalSystem([robot.symbolic()],RigidObject(1.,np.zeros(3),np.eye(3)*.02))
    q=np.r_[0.,0.,.5,0.,0.,.5,0.,0.,0.,1.];p=np.zeros(3);n=np.array([0.,0.,-1.]);R=contact_basis(n)
    task=GeneralizedTaskLoad(np.r_[0.,0.,0.,0.,0.,-9.81],np.eye(6)[:,5:],np.eye(1));cap=Capability(mu=.8,normal_max=20.)
    interface=Interface('support',0,'link2',None,SurfacePatch.fixed(p,n),cap,p,R)
    result=realize_system_keyframe(sys,q,q[sys.object_q],[interface],task,rho_min=.1)
    np.testing.assert_allclose(result.rho,9.81,atol=1e-5)
    _,_,_,J=relative_interface(sys,ca.DM(result.q),interface,ca.DM(result.positions[0]),ca.DM(result.rotations[0]));J=np.array(ca.DM(J))
    for lam,tau in zip(result.reactions[:,0],result.torques):
        physical_robot=np.r_[R@lam[:3],R@lam[3:]]
        expected=robot.inverse_dynamics(result.q[:3],np.zeros(3),np.zeros(3))-robot.jacobian(result.q[:3],'link2','world_aligned').T@physical_robot
        np.testing.assert_allclose(tau,expected,atol=1e-7)
    check=fixed_generalized_margin([-J.T[sys.object_v]],task,[cap]);assert check['certificate'];np.testing.assert_allclose(check['rho'],result.rho,atol=1e-5)


def test_failed_conic_balance_is_never_a_margin_certificate():
    task=GeneralizedTaskLoad(np.r_[0.,0.,0.,0.,0.,-9.81],np.eye(6)[:,5:],np.eye(1))
    # A top-facing unilateral contact cannot produce the required down reaction.
    check=fixed_generalized_margin([np.eye(6)],task,[Capability(mu=.8,normal_max=20.)])
    assert not check['certificate'] and check['status']=='PrimalInfeasible'


def test_fixed_margin_feasibility_certifies_same_box_without_claiming_optimality():
    task=GeneralizedTaskLoad(np.r_[0.,0.,0.,0.,0.,-9.81],np.eye(6)[:,5:],np.eye(1));mapping=np.eye(6);mapping[5,5]=-1;cap=Capability(mu=.8,normal_max=20.)
    check=fixed_generalized_margin([mapping],task,[cap],fixed_rho=5.)
    assert check['certificate'] and check['fixed_rho_feasibility'];assert abs(check['rho']-5.)<1e-7
    for vertex,reaction in zip(task.vertices,check['reactions'][:,0]):np.testing.assert_allclose(mapping@reaction,task.nominal+5.*task.E@task.D@vertex,atol=1e-7)
    impossible=fixed_generalized_margin([mapping],task,[cap],fixed_rho=12.);assert not impossible['certificate']


def test_stationary_conic_condensation_minimizes_weighted_robot_effort():
    task=GeneralizedTaskLoad(np.array([-10.]),np.ones((1,1)),np.ones((1,1)));mapping=np.zeros((1,6));mapping[0,5]=-1;torque_maps=[]
    for i in range(2):
        T=np.zeros((2,6));T[i,5]=1;torque_maps.append(T)
    check=fixed_generalized_margin([mapping,mapping],task,[Capability(mu=.8,normal_max=20.)]*2,torque_gravity=np.zeros(2),torque_maps=torque_maps,effort=np.ones(2)*100,fixed_rho=0.,torque_weight=np.diag([1.,4.]))
    assert check['certificate'];np.testing.assert_allclose(check['reactions'].mean(axis=0)[:,5],[8.,2.],atol=1e-6)


def test_unloaded_robot_has_zero_reaction_while_environment_supports_object(slider_urdf):
    from contact_transfer.trajectory import Contact
    robot=Robot(slider_urdf);sys=MechanicalSystem([robot.symbolic()],RigidObject(1.,np.zeros(3),np.eye(3)*.02));q=np.r_[0.,0.,.5,0.,0.,.5,0.,0.,0.,1.];n=np.array([0.,0.,-1.]);R=contact_basis(n);task=GeneralizedTaskLoad(np.r_[0.,0.,0.,0.,0.,-9.81],np.eye(6)[:,5:],np.eye(1));cap=Capability(mu=.8,normal_max=20.);interface=Interface('support',0,'link2',None,SurfacePatch.fixed(np.zeros(3),n),cap,np.zeros(3),R,force_enabled=False);environment=Contact('jig',np.zeros(3),[0,0,1],cap,plane_offset=.5)
    result=realize_system_keyframe(sys,q,q[sys.object_q],[interface],task,environment=[environment],rho_min=.1)
    np.testing.assert_array_equal(result.reactions,0.);assert result.rho>9.8
    np.testing.assert_allclose(result.environment_reactions['jig'][:,2],9.81-result.rho*task.vertices[:,0],atol=1e-5)


def test_panda_grasp_discretization_mismatch_is_first_order_not_a_solver_failure():
    from contact_transfer.panel_fixture import PanelCase
    from contact_transfer.paper_validation import grasp_discretization_probe
    c=PanelCase.load();r=c.robots[0];q=c.data['arms'][0]['q_initial'];v=np.array([.10,-.12,.08,.06,-.07,.09,.05])
    rows=grasp_discretization_probe(r,'tip',q,v,[.1,.05,.025]);error=np.array([a['max_residual'] for a in rows])
    assert error[0]>1e-3;np.testing.assert_allclose(error[:-1]/error[1:],2.,rtol=.01)
    v[1:]=0;assert grasp_discretization_probe(r,'tip',q,v,[.1])[0]['max_residual']<1e-12


def test_collision_sampling_matches_the_executed_holding_reference():
    from types import SimpleNamespace
    from scipy.spatial.transform import Rotation
    from contact_transfer.panel_fixture import PanelCase
    from contact_transfer.paper_geometry import PanelGeometry
    c=PanelCase.load();r=c.robots[0];q0=np.array(c.data['arms'][0]['q_initial']);q1=q0.copy();q1[0]+=.4
    rel=np.eye(4);rel[:3,3]=[.05,.03,.07];states=[]
    for q in [q0,q1]:
        T=r.fk(q,'tip')@rel;states.append(np.r_[q,T[:3,3],Rotation.from_matrix(T[:3,:3]).as_quat()])
    sys=SimpleNamespace(object_q=slice(7,14),object_v=slice(7,13),q_slices=[slice(0,7)])
    g=PanelGeometry.__new__(PanelGeometry);g.problem=SimpleNamespace(system=sys,robots=[r],capabilities=[SimpleNamespace(kind='G')],frames=['tip'])
    captured=[];g.discover=lambda samples:captured.extend(samples) or []
    trajectory=SimpleNamespace(q=np.array(states),v=np.zeros((2,13)),dt=np.array([1.]))
    assert g.validate(trajectory,object_interpolation='holding_grasp')
    for q in captured:
        T=r.fk(q[:7],'tip')@rel;np.testing.assert_allclose(T[:3,3],q[7:10],atol=1e-12)
    captured.clear();g.validate(trajectory)
    assert max(np.linalg.norm((r.fk(q[:7],'tip')@rel)[:3,3]-q[7:10]) for q in captured)>1e-3
