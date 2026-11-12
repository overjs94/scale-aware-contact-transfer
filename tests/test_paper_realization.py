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


def test_panda_grasp_discretization_mismatch_is_first_order_not_a_solver_failure():
    from contact_transfer.panel_fixture import PanelCase
    from contact_transfer.paper_validation import grasp_discretization_probe
    c=PanelCase.load();r=c.robots[0];q=c.data['arms'][0]['q_initial'];v=np.array([.10,-.12,.08,.06,-.07,.09,.05])
    rows=grasp_discretization_probe(r,'tip',q,v,[.1,.05,.025]);error=np.array([a['max_residual'] for a in rows])
    assert error[0]>1e-3;np.testing.assert_allclose(error[:-1]/error[1:],2.,rtol=.01)
    v[1:]=0;assert grasp_discretization_probe(r,'tip',q,v,[.1])[0]['max_residual']<1e-12
