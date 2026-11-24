import casadi as ca
import numpy as np
from pathlib import Path
from contact_transfer.robot import Robot
from contact_transfer.system import MechanicalSystem
from contact_transfer.multi_object import RigidCollection,BodyContact
from contact_transfer.contacts import Capability
from contact_transfer.trajectory import Contact,solve_cito


def test_independent_objects_keep_both_se3_integrations_and_native_mechanics():
    root=Path(__file__).parents[1]/'src/contact_transfer/scenarios/table_assembly'
    bodies={n:Robot(root/f'{n}.urdf',floating_base=True) for n in ['tabletop','leg']};obj=RigidCollection(bodies);symbolic=obj.symbolic();sys=MechanicalSystem([],symbolic)
    q=np.r_[0,0,.5,0,0,0,1,.1,.2,.8,0,0,0,1];v=np.arange(12)*.01
    np.testing.assert_allclose(np.array(sys.mass(ca.DM(q))),obj.mass(q),atol=1e-12)
    np.testing.assert_allclose(np.array(sys.bias(ca.DM(q),ca.DM(v))).ravel(),obj.bias(q,v),atol=1e-12)
    nextq=np.array(sys.integrate(ca.DM(q),ca.DM(.01*v))).ravel();np.testing.assert_allclose(nextq,obj.integrate(q,.01*v),atol=1e-12)
    for s in obj.quaternion_slices:assert abs(np.linalg.norm(nextq[s])-1)<1e-12


def test_internal_contact_impulse_conserves_total_linear_momentum_and_power():
    root=Path(__file__).parents[1]/'src/contact_transfer/scenarios/table_assembly';obj=RigidCollection({n:Robot(root/f'{n}.urdf',floating_base=True) for n in ['tabletop','leg']});sys=MechanicalSystem([],obj.symbolic())
    q=np.r_[0,0,.5,0,0,0,1,.1,.2,.66,0,0,0,1];c=BodyContact('seat',[.1,.2,.02],[0,0,1],object_frame='tabletop',other_frame='leg',other_point=[0,0,-.14]);gap,J,_=c.geometry(sys,ca.DM(q));J=np.array(ca.DM(J));P=np.array([.1,-.05,.3]);impulse=J.T@P
    np.testing.assert_allclose(impulse[3:6]+impulse[9:12],0.,atol=1e-12)
    v=np.arange(12)*.02;eps=1e-7;nextq=obj.integrate(q,eps*v);a=obj.fk(q,'tabletop');b=obj.fk(q,'leg');pn=a[:3,3]+a[:3,:3]@c.point_object;po=b[:3,3]+b[:3,:3]@c.other_point
    an=obj.fk(nextq,'tabletop');bn=obj.fk(nextq,'leg');relative=((bn[:3,3]+bn[:3,:3]@c.other_point)-(an[:3,3]+an[:3,:3]@c.point_object)-(po-pn))/eps
    np.testing.assert_allclose(J@v,c.basis().T@relative,atol=1e-7);np.testing.assert_allclose(v@impulse,(J@v)@P,atol=1e-12)
    q[7:10]+=[.03,.04,.2];gap,J,_=c.geometry(sys,ca.DM(q));gnew=c.geometry(sys,ca.DM(obj.integrate(q,eps*v)))[0]
    np.testing.assert_allclose((float(gnew)-float(gap))/eps,np.array(ca.DM(J))[2]@v,atol=1e-7)


def test_constrained_midpoint_preserves_two_free_body_momenta_and_both_root_bounds(slider_urdf):
    from scipy.linalg import block_diag
    from contact_transfer.optimization import SolverSettings
    root=Path(__file__).parents[1]/'src/contact_transfer/scenarios/table_assembly';obj=RigidCollection({n:Robot(root/f'{n}.urdf',floating_base=True) for n in ['tabletop','leg']});robots=[Robot(slider_urdf),Robot(slider_urdf)];sys=MechanicalSystem([r.symbolic() for r in robots],obj.symbolic());q=np.r_[0,0,.5,.1,0,.8,0,0,.5,0,0,0,1,.1,0,.8,0,0,0,1];goal=q.copy();goal[[2,8]]+=.02;goal[[4,14]]+=.01;cap=Capability('G',H=np.eye(6),bounds=np.r_[np.ones(3)*5,np.ones(3)*50]);contacts=[Contact(name,np.zeros(3),[0,0,-1],cap,robot_index=i,frame='link2',object_frame=name,rotation_object=np.diag([1.,-1.,-1.]),attached_steps=tuple(range(4))) for i,name in enumerate(obj.names)]
    result=solve_cito(sys,q,goal,contacts,steps=4,duration=.8,dt_bounds=(.1,.3),configuration_update='grasp_consistent_midpoint',configuration_defect_bounds=(2e-5,5e-5),settings=SolverSettings(max_iterations=500));assert result.configuration_defect_bounds==(2e-5,5e-5)
    for k,h in enumerate(result.dt):
        x,y=result.q[k:k+2];v,w=result.v[k:k+2];M=block_diag(*[r.mass(x[s]) for r,s in zip(robots,sys.q_slices)],obj.mass(x[sys.object_q]));b=np.r_[*[r.bias(x[qs],v[vs]) for r,qs,vs in zip(robots,sys.q_slices,sys.v_slices)],obj.bias(x[sys.object_q],v[sys.object_v])];impulse=np.zeros(sys.nv)
        for c in contacts:
            T=obj.fk(x[sys.object_q],c.object_frame);R=T[:3,:3]@c.basis();Jr=robots[c.robot_index].jacobian(x[sys.q_slices[c.robot_index]],'link2','world_aligned');Jo=obj.jacobian(x[sys.object_q],c.object_frame,'world_aligned');J=np.zeros((6,sys.nv));J[:,sys.v_slices[c.robot_index]]=Jr;J[:,sys.object_v]=-Jo;impulse+=J.T@block_diag(R,R)@result.impulses[c.name][k]
        np.testing.assert_allclose(M@(w-v),h*(np.array(sys.B)@result.tau[k]-b)+impulse,atol=2e-6)
        predicted=obj.integrate(x[sys.object_q],.5*h*(v[sys.object_v]+w[sys.object_v]))
        for sl in obj.q_slices.values():assert np.max(abs(y[sys.object_q][sl][:3]-predicted[sl][:3]))<=2.2e-5
    np.testing.assert_allclose(result.q[-1],goal,atol=2e-6)
