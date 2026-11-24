import casadi as ca
import numpy as np
from contact_transfer.free_articulated import FreeCase,object_reference
from contact_transfer.system import MechanicalSystem
from contact_transfer.trajectory import Contact
from contact_transfer.contacts import Capability
from contact_transfer.generalized_load import link_reaction_map


def test_articulated_system_mass_bias_integration_and_point_power_duality():
 c=FreeCase.load();o=c.object.symbolic();sys=MechanicalSystem([],o);q,v,a=object_reference(c,8);v=np.random.default_rng(11).normal(0,.1,7)
 np.testing.assert_allclose(np.array(sys.mass(ca.DM(q))),c.object.mass(q),atol=1e-10);np.testing.assert_allclose(np.array(sys.bias(ca.DM(q),ca.DM(v))).ravel(),c.object.bias(q,v),atol=1e-10)
 np.testing.assert_allclose(np.array(sys.integrate(ca.DM(q),ca.DM(.01*v))).ravel(),c.object.integrate(q,.01*v),atol=1e-10)
 point=np.array([.3,.02,.04]);R=np.eye(3);lam=np.array([.1,.2,.3,1.,2.,3.]);mapped=np.array(link_reaction_map(o,q,'moving')(ca.DM(point),ca.DM(R),ca.DM(lam))).ravel();J=c.object.jacobian(q,'moving','world_aligned');T=c.object.fk(q,'moving');from contact_transfer.free_articulated import skew
 Jp=np.vstack((J[:3],J[3:]-skew(T[:3,:3]@point)@J[:3]));w=np.r_[T[:3,:3]@lam[:3],T[:3,:3]@lam[3:]];assert mapped@v==__import__('pytest').approx(w@(Jp@v))


def test_projected_point_gap_derivative_uses_actual_contact_point(slider_urdf):
 from contact_transfer import Robot
 r=Robot(slider_urdf);c=FreeCase.load();sys=MechanicalSystem([r.symbolic()],c.object.symbolic());qo=object_reference(c,8)[0];T=c.object.fk(qo,'moving');p=np.array([.2,0,.01]);surface=T[:3,3]+T[:3,:3]@p;normal=T[:3,:3]@np.array([0,0,1]);tool=surface+T[:3,:3]@np.array([.08,.04,.02]);qr=tool # slider FK maps xyz directly
 q=np.r_[qr,qo];x=ca.SX.sym('q',sys.nq);contact=Contact('surface',p,[0,0,1],Capability(),robot_index=0,frame='link2',object_frame='moving',project_to_surface=True);gap,J,_=contact.geometry(sys,x);fn=ca.Function('projected',[x],[gap,J]);v=np.random.default_rng(3).normal(0,.1,sys.nv);h=1e-6;plus=sys.integrate(ca.DM(q),ca.DM(h*v));minus=sys.integrate(ca.DM(q),ca.DM(-h*v));fd=(float(fn(plus)[0])-float(fn(minus)[0]))/(2*h);np.testing.assert_allclose(np.array(fn(q)[1])[2]@v,fd,atol=1e-7)
 # The internal hinge column must participate in both angular and point motion.
 assert np.linalg.norm(np.array(fn(q)[1])[:,-1])>.1
