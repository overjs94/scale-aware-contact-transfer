"""Native full-system checks of phase condensation, including zero-load release."""
from types import SimpleNamespace
import numpy as np
from contact_transfer.robot import Robot
from contact_transfer.system import MechanicalSystem
from contact_transfer.contacts import Capability
from contact_transfer.trajectory import Contact
from contact_transfer.generalized_load import GeneralizedTaskLoad
from contact_transfer.full_static import stationary_interval, released_arm_interval
from contact_transfer.optimization import SolverSettings


def problem(slider_urdf,tmp_path):
    path=tmp_path/'object.urdf'
    path.write_text('<robot name="object"><link name="body"><inertial><mass value="1"/><inertia ixx=".02" iyy=".02" izz=".02" ixy="0" ixz="0" iyz="0"/></inertial></link></robot>')
    arm=Robot(slider_urdf);obj=Robot(path,floating_base=True);system=MechanicalSystem([arm.symbolic()],obj.symbolic())
    p=SimpleNamespace(system=system,robots=[arm],task_definition=SimpleNamespace(id='test',object=obj),task_load=lambda q:GeneralizedTaskLoad(-obj.gravity(q),np.eye(6),np.eye(6)*.01))
    q=np.r_[0.,0.,.5,0.,0.,.5,0.,0.,0.,1.]
    supports=tuple(Contact('ground'+str(i),[x,y,0.],[0,0,1],Capability(mu=.8,normal_max=50.),plane_offset=.5,object_frame='body') for i,(x,y) in enumerate([[-.2,-.2],[.2,-.2],[-.2,.2],[.2,.2]]))
    return p,q,supports


def test_load_transfer_condensation_preserves_native_momentum_and_last_zero_tool_force(slider_urdf,tmp_path):
    p,q,supports=problem(slider_urdf,tmp_path);N=8
    tool=Contact('tool',[0,0,0],[0,0,1],Capability(mu=.8,normal_max=30.),robot_index=0,frame='link2',object_frame='body',nodes=tuple(range(N-1)),maintained_steps=tuple(range(N)),sticking_steps=tuple(range(N)))
    left=SimpleNamespace(q=q,phase='hold');right=SimpleNamespace(q=q.copy(),phase='unload')
    r,_,check=stationary_interval(p,left,right,(tool,*supports),.8,N)
    assert check['passed'] and check['residuals']['momentum']<1e-7
    assert np.max(abs(r.impulses['tool'][-1]))==0.
    np.testing.assert_allclose(sum(r.impulses[c.name][-1,2] for c in supports),9.81*r.dt[-1],atol=1e-7)
    assert r.objective_mode=='stationary_contact_conic_QP'


def test_released_arm_nlp_keeps_free_object_balanced_without_object_actuation(slider_urdf,tmp_path):
    p,q,supports=problem(slider_urdf,tmp_path);N=8
    tool=Contact('tool',[0,0,0],[0,0,1],Capability(),robot_index=0,frame='link2',object_frame='body',nodes=(),released_steps=tuple(range(N)))
    goal=q.copy();goal[2]+=.04
    r,_,check=released_arm_interval(p,SimpleNamespace(q=q),SimpleNamespace(q=goal),(tool,*supports),1.,N,SolverSettings(max_iterations=300))
    assert check['passed'] and check['residuals']['momentum']<1e-7
    np.testing.assert_allclose(r.q[-1],goal,atol=1e-8)
    np.testing.assert_allclose(r.q[:,p.system.object_q],np.tile(q[p.system.object_q],(N+1,1)),atol=1e-12)
    assert not np.any(r.impulses['tool'])
    assert r.objective_mode=='stationary_objects_release_NLP'
