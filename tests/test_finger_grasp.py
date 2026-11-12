import casadi as ca
import numpy as np
from contact_transfer.finger_grasp import FingerGrasp


def test_finger_resultants_shift_origin_and_coupled_normal_force_bounds():
 cap=FingerGrasp(.04,.008,.012,.8,20.);maps=cap.maps();B=np.hstack([np.array(x) for x in maps]);assert np.linalg.matrix_rank(B)==6
 zero=cap.check(np.zeros(6));assert zero['feasible']
 support=cap.check(np.array([0,0,0,0,0,-10.]));assert support['feasible'];f=support['finger_forces'];assert np.max(np.linalg.norm(f[:,:2],axis=1)-.8*f[:,2])<1e-7 and all(f[k:k+4,2].sum()<=20+1e-7 for k in [0,4])
 assert not cap.check(np.array([0,0,0,0,0,50.]))['feasible']
 T=np.eye(4);T[:3,3]=[.02,0,0];shifted=FingerGrasp(.04,.008,.012,.8,20.,interface_from_grasp=T);w=np.array([0,.2,0,0,0,-10.]);assert shifted.check(w)['feasible']


def test_finger_aperture_is_explicit_in_symbolic_resultant_map():
 cap=FingerGrasp(.04,.008,.012,.8,20.);a=ca.SX.sym('a');B=ca.horzcat(*cap.maps(a));fn=ca.Function('finger_map',[a],[B]);assert np.linalg.norm(np.array(fn(.04))-np.array(fn(.06)))>.01


def test_external_force_bound_is_in_grasp_frame_and_scales_with_impulse():
 from scipy.spatial.transform import Rotation
 from contact_transfer.optimization import solve
 T=np.eye(4);T[:3,:3]=Rotation.from_euler('z',.7).as_matrix();T[:3,3]=[.02,-.01,.03]
 unconstrained=FingerGrasp(.04,.008,.012,.8,60.,interface_from_grasp=T)
 limited=FingerGrasp(.04,.008,.012,.8,60.,interface_from_grasp=T,force_component_limits=[50,50,50])
 # The friction/closing-force model allows this load, the external bound does not.
 local=np.array([0,0,0,0,0,-55.]);force=T[:3,:3]@local[3:];w=np.r_[np.cross(T[:3,3],force),force]
 assert unconstrained.check(w)['feasible'] and not limited.check(w)['feasible']
 # Optimize an impulse using the NLP path, independently check P/h as force.
 opti=ca.Opti();P=limited.reaction(opti,scale=.1)
 opti.minimize(ca.dot(ca.DM(T[:3,:3]@np.array([0.,0.,1.])),P[3:]));sol,_=solve(opti);p=np.array(sol.value(P)).ravel();grasp_force=T[:3,:3].T@p[3:]
 assert abs(grasp_force[2]+5)<1e-6 and limited.check(p/.1)['feasible']


def test_scaled_tangent_coordinates_keep_exact_friction_and_finite_jaw_limits():
 from contact_transfer.optimization import solve
 cap=FingerGrasp(.04,.008,.012,.6,40.);opti=ca.Opti();w,u=cap.reaction_with_witness(opti,force_coordinates='scaled_tangent');opti.subject_to(w[:5]==0);opti.minimize(w[5]+1e-9*ca.sumsqr(u));sol,_=solve(opti);value=np.array(sol.value(w)).ravel();witness=np.array(sol.value(u)).reshape(8,3)
 np.testing.assert_allclose(value[5],-48.,atol=1e-5)
 assert np.max(np.linalg.norm(witness[:,:2],axis=1)-.6*witness[:,2])<1e-7
 assert all(witness[i:i+4,2].sum()<=40.+1e-7 for i in [0,4])


def test_common_motor_virtual_work_cut_couples_the_two_jaws():
 from contact_transfer.optimization import solve
 A=np.zeros((1,24));A[0,2::3]=.1;cap=FingerGrasp(.04,.008,.012,.6,40.,finger_linear_limits=(A,[4.]))
 assert not cap.check(np.array([0,0,0,0,0,-30.]))['feasible']
 for coordinates in ['cartesian','scaled_tangent']:
  opti=ca.Opti();w,u=cap.reaction_with_witness(opti,force_coordinates=coordinates);opti.subject_to(w[:5]==0);opti.minimize(w[5]);sol,_=solve(opti);value=np.array(sol.value(w)).ravel();np.testing.assert_allclose(value[5],-24.,atol=1e-5);assert cap.check(value,tolerance=2e-6)['feasible']


def test_cito_uses_finger_resultants_and_explicit_holding_aperture(slider_urdf):
 from contact_transfer import Robot
 from contact_transfer.system import MechanicalSystem,RigidObject
 from contact_transfer.trajectory import Contact,solve_cito
 r=Robot(slider_urdf).symbolic();sys=MechanicalSystem([r],RigidObject(1.,np.zeros(3),np.eye(3)*.02));q=np.r_[0,0,.5,0,0,.5,0,0,0,1];R=np.diag([1.,-1.,-1.]);T=np.eye(4);T[:3,:3]=R;cap=FingerGrasp(.04,.008,.012,.8,20.,interface_from_grasp=T);contact=Contact('holding',np.zeros(3),[0,0,-1],cap,robot_index=0,frame='link2',attached_steps=tuple(range(4)),rotation_object=R)
 result=solve_cito(sys,q,q,[contact],steps=4,duration=.4);np.testing.assert_allclose(result.apertures['holding'],.04,atol=1e-8)
 for P,h in zip(result.impulses['holding'],result.dt):assert cap.check(P/h)['feasible']
 assert result.gripper_velocity_residual<1e-6 and result.primal_residual<1e-6
