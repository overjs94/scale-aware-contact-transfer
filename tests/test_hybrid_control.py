import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from contact_transfer.hybrid_control import HybridSelection,HybridTorqueImpedance,HybridPositionAdmittance,torque_nullspace,contact_coordinates

class CartesianRobot:
 n=nq=nv=6;floating_base=False;effort=np.full(6,100.);lower=np.full(6,-1.);upper=np.full(6,1.);velocity=np.ones(6)
 def fk(self,q,frame):
  T=np.eye(4);T[:3,:3]=Rotation.from_rotvec(q[:3]).as_matrix();T[:3,3]=q[3:];return T
 def jacobian(self,q,frame):return np.eye(6)
 def mass(self,q):return np.diag([2.,3.,4.,5.,6.,7.])
 def inverse_dynamics(self,q,v,a):return self.mass(q)@a
 def integrate(self,q,v):return q+v


def test_force_direction_has_no_competing_position_stiffness():
 r=CartesianRobot();c=HybridTorqueImpedance(r,'tip',force_i=0);q=np.zeros(6);qd=q.copy();qd[5]=.2;lam=np.array([0,0,0,0,0,-10.]);tau=c.command(q,q,qd,q,q,reaction_desired=lam,reaction_measured=lam,contact_rotation_world=np.eye(3),dt=.001)
 np.testing.assert_allclose(tau,[0,0,0,0,0,10],atol=1e-6)
 assert c.last['motion_feedback_contact'][5]==0 and np.linalg.norm(c.last['joint_impedance_torque'])<1e-5
 qd[3]=.01;tau=c.command(q,q,qd,q,q,reaction_desired=lam,reaction_measured=lam,contact_rotation_world=np.eye(3),dt=.001);assert tau[3]==pytest.approx(30,abs=1e-6)


def test_force_feedback_pushes_in_when_measured_load_is_too_small_and_antiwindup():
 r=CartesianRobot();c=HybridTorqueImpedance(r,'tip',force_k=.5,force_i=0);q=np.zeros(6);desired=np.r_[np.zeros(5),-10];measured=np.r_[np.zeros(5),-6]
 tau=c.command(q,q,q,q,q,reaction_desired=desired,reaction_measured=measured,contact_rotation_world=np.eye(3),dt=.001);assert tau[5]==pytest.approx(12)
 r.effort=np.ones(6);c=HybridTorqueImpedance(r,'tip');c.command(q,q,q,q,q,reaction_desired=desired,reaction_measured=measured,contact_rotation_world=np.eye(3),dt=.001);assert c.last['saturated'];np.testing.assert_allclose(c.integral,0)


def test_rotated_contact_frame_preserves_power_and_normal_selection():
 Rc=Rotation.from_euler('y',np.pi/2).as_matrix();Rt=Rotation.from_euler('xyz',[.3,.2,-.4]).as_matrix();H=contact_coordinates(Rt,Rc);v=np.arange(6)*.1;w=np.arange(6)[::-1]*.3;assert (H@v)@(H@w)==pytest.approx(v@w)
 r=CartesianRobot();c=HybridTorqueImpedance(r,'tip',force_k=.5,force_i=0);q=np.zeros(6);d=np.r_[np.zeros(3),[-10,0,0]];m=np.r_[np.zeros(3),[-6,0,0]];tau=c.command(q,q,q,q,q,reaction_desired=d,reaction_measured=m,contact_rotation_world=Rc,dt=.001);assert tau[3]==pytest.approx(12);assert abs(tau[5])<1e-8


def test_joint_impedance_is_dynamically_orthogonal_to_contact_task():
 rng=np.random.default_rng(17);A=rng.normal(size=(7,7));M=A.T@A+np.eye(7);J=rng.normal(size=(6,7));N=torque_nullspace(M,J);posture=N.T@rng.normal(size=7);assert np.linalg.norm(J@np.linalg.solve(M,posture))<1e-6
 with pytest.raises(ValueError):HybridSelection(np.full((6,6),.2))


def test_position_admittance_uses_force_feedback_and_respects_velocity_bounds():
 r=CartesianRobot();c=HybridPositionAdmittance(r,'tip');q=np.zeros(6);qd=q.copy();qd[5]=-.5;desired=np.r_[np.zeros(5),-10];measured=np.r_[np.zeros(5),-6]
 command=c.command(q,qd,q,reaction_desired=desired,reaction_measured=measured,contact_rotation_world=np.eye(3),dt=.01);assert command[5]>0 and np.max(abs(command-q))<=.01+1e-9
 c.reset();command=c.command(q,qd,q,reaction_desired=desired,reaction_measured=desired,contact_rotation_world=np.eye(3),dt=.01);assert command[5]==pytest.approx(0,abs=1e-8)
 with pytest.raises(ValueError):c.command(q,qd,q,reaction_desired=desired,reaction_measured=desired,contact_rotation_world=np.eye(3),dt=0)


def test_feedforward_comparison_disables_force_feedback_and_keeps_motion_selection():
 r=CartesianRobot();q=np.zeros(6);qd=q.copy();qd[5]=.01;desired=np.r_[np.zeros(5),-10.]
 # Hybrid FF has no normal pose stiffness and ignores the measured load.
 c=HybridTorqueImpedance(r,'tip',force_k=0,force_i=0)
 for measured in [desired,np.zeros(6),-5*desired]:
  command=c.command(q,q,qd,q,q,reaction_desired=desired,reaction_measured=measured,contact_rotation_world=np.eye(3),dt=.001)
  assert command[5]==pytest.approx(10.,abs=1e-6)
 # Full pose impedance + the same FF explicitly restores normal tracking.
 c=HybridTorqueImpedance(r,'tip',HybridSelection(np.zeros(6)),force_k=0,force_i=0)
 command=c.command(q,q,qd,q,q,reaction_desired=desired,reaction_measured=np.zeros(6),contact_rotation_world=np.eye(3),dt=.001)
 assert command[5]==pytest.approx(40.,abs=1e-6)
