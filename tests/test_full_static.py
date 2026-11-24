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
    # Inactive environment modes cannot reuse otherwise balanced reactions.
    from dataclasses import replace
    from contact_transfer.full_validation import validate_interval
    disabled=tuple(replace(c,nodes=()) for c in supports)
    rejected=validate_interval(p,r,(tool,*disabled))
    assert not rejected['passed'] and rejected['residuals']['capability']>1.


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


def test_fixed_material_target_attains_posture_and_torque_free_margin_bounds(slider_urdf,tmp_path):
    from contact_transfer.full_static import fixed_geometry_keyframe
    from contact_transfer.paper_keyframe import Interface
    from contact_transfer.contacts import SurfacePatch,contact_basis
    from contact_transfer.full_validation import validate_keyframe
    p,_,_=problem(slider_urdf,tmp_path);arm=p.robots[0];obj=p.task_definition.object;points=np.array([[-.2,-.15,0.],[.2,-.15,0.],[0.,.2,0.]]);normal=np.array([0.,0.,-1.]);qo=np.r_[0.,0.,.5,0.,0.,0.,1.];seeds=[a+qo[:3] for a in points];caps=[Capability(mu=.8,normal_max=30.) for _ in points]
    p.system=MechanicalSystem([arm.symbolic() for _ in points],obj.symbolic());p.robots=[arm]*3;p.frames=['link2']*3;p.specs=[dict(q_initial=q) for q in seeds];p.capabilities=caps;p.task_definition.features=[SimpleNamespace(id='tool'+str(i),frame='body') for i in range(3)]
    interfaces=[Interface(f.id,i,'link2','body',SurfacePatch.fixed(a,normal),c,a,contact_basis(normal)) for i,(f,a,c) in enumerate(zip(p.task_definition.features,points,caps))]
    key=fixed_geometry_keyframe(p,np.r_[*seeds,qo],interfaces,dict(positions=points,normals=np.tile(normal,(3,1))),(),SolverSettings(max_iterations=300));assert key is not None;key.interfaces=interfaces
    assert key.separable_target_certificate['same_rho_torque_feasibility']
    assert key.rho==key.separable_target_certificate['torque_free_rho_upper_bound']
    assert validate_keyframe(p,key)['passed']
    # The torque-free bound must never be used as a certificate when robot
    # actuation cannot realize its force distribution.
    p.system.effort=np.full(9,.1)
    assert fixed_geometry_keyframe(p,np.r_[*seeds,qo],interfaces,dict(positions=points,normals=np.tile(normal,(3,1))),(),SolverSettings(max_iterations=300)) is None


def test_full_execution_identity_tracks_physics_and_material_points_not_observer_state(slider_urdf,tmp_path):
    from contact_transfer.full_execution import FullEpisode
    from contact_transfer.trajectory import TrajectoryResult
    p,q,_=problem(slider_urdf,tmp_path);p.full_robots=p.robots;p.specs=[dict(id='arm',q_initial=q[:3].tolist())];p.types=('P',);p.assignment=(0,);p.refine_enabled=True;p.task_definition.keyframes=[];p.task_definition.fixtures=()
    r=TrajectoryResult(np.tile(q,(2,1)),np.zeros((2,p.system.nv)),np.zeros((1,3)),np.array([.1]),{'tool':np.zeros((1,3))},0.,0.,0.,0.,0.,False,());episode=SimpleNamespace(problem=p,trajectories=[r],keyframes=[SimpleNamespace(positions=np.zeros((1,3)),rotations=np.eye(3)[None])]);before=FullEpisode.identity(episode)
    p.specs[0]['allowed_object_geoms']=['body_collision_0'];assert FullEpisode.identity(episode)==before
    episode.keyframes[0].positions[0,0]=.001;changed=FullEpisode.identity(episode);assert changed[0]==before[0] and changed[1]!=before[1]
    p.robots[0].base_pose[0,3]+=.01;assert FullEpisode.identity(episode)[0]!=changed[0]
