from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace
import json

import numpy as np
import pytest

from contact_transfer.certified_disturbance import normalized_membership,schedule_membership,physical_hold
from contact_transfer.configuration import load_task_configuration
from contact_transfer.equation_contract import require_literal_task
from contact_transfer.full_scenario import FullTask,FullProblem
from contact_transfer.full_scene import build_scene
from contact_transfer.generalized_load import GeneralizedTaskLoad
from contact_transfer.multi_object import RigidCollection
from contact_transfer.perturbations import WrenchHold
from contact_transfer.robot import Robot


@pytest.mark.parametrize('name',['box','assembly'])
def test_functional_source_is_supported_by_real_surface_points_and_native_witnesses(name,tmp_path):
    override={'source':{'representation':'functional_group_sites','site_file':f'package:scenarios/certified_sources/{name}.json','failure_policy':'error'}}
    task=FullTask.load(name,tmp_path/'task',configuration=override)
    p=FullProblem.create(task,tmp_path/'problem','GPG')
    result=p.source(task.keyframes[0]);r=result['record']
    assert r['source_certificate'] and r['source_rho']>1.
    assert r['source_site_count']==19 and r['source_group_count']==3
    assert r['source_model_extension'] and not r['source_forces_transferred']
    assert r['source_independent_validation']['residual']<2e-6
    before=np.array(r['source_positions_before_m']);after=np.array(r['source_positions_after_m'])
    for group in range(3):
        indices=np.flatnonzero(np.array(r['source_coordinate_groups'])==group)
        np.testing.assert_allclose((after-before)[indices],np.tile((after-before)[indices[0]],(len(indices),1)),atol=1e-12)
    np.testing.assert_allclose(result['positions'],after[r['source_anchor_indices']]*task.scale)


def test_source_fixture_cannot_invent_a_contact_inside_the_solid(tmp_path):
    path=Path('src/contact_transfer/scenarios/certified_sources/box.json')
    data=json.loads(path.read_text());data['groups'][0]['sites'][0]['position_m'][1]=0.
    fixture=tmp_path/'invalid.json';fixture.write_text(json.dumps(data))
    task=FullTask.load('box',tmp_path/'task',configuration={'source':{'representation':'functional_group_sites','site_file':str(fixture),'failure_policy':'error'}})
    p=FullProblem.create(task,tmp_path/'problem','GPG')
    with pytest.raises(ValueError,match='leaves surface'):p.source(task.keyframes[0])


def free_body(tmp_path,name):
    path=tmp_path/(name+'.urdf');path.write_text('<robot name="test"><link name="body"><inertial><mass value="1"/><inertia ixx=".02" iyy=".02" izz=".02" ixy="0" ixz="0" iyz="0"/></inertial></link></robot>')
    return Robot(path,floating_base=True)


def test_correlated_multibody_forces_must_be_checked_together(tmp_path):
    obj=RigidCollection({'a':free_body(tmp_path,'a'),'b':free_body(tmp_path,'b')})
    q=np.r_[obj.bodies['a'].neutral(),obj.bodies['b'].neutral()]
    load=GeneralizedTaskLoad(-obj.gravity(q),np.vstack([.8*np.eye(6),.2*np.eye(6)]),np.eye(6))
    p=SimpleNamespace(task_definition=SimpleNamespace(object=obj),task_load=lambda q:load)
    scene=SimpleNamespace(object_frames={'a':'a_native','b':'b_native'})
    pulses=[WrenchHold('a_native',1,3,(0,0,.8)),WrenchHold('b_native',1,3,(0,0,.2))]
    assert schedule_membership(p,q,pulses,1.,scene)['inside']
    assert not schedule_membership(p,q,pulses[:1],1.,scene)['inside']
    shifted=replace(pulses[1],start=2.)
    assert not schedule_membership(p,q,[pulses[0],shifted],1.,scene)['inside']


def test_overlapping_loads_and_moment_arms_cannot_escape_the_margin_gate(tmp_path):
    obj=free_body(tmp_path,'rigid');q=obj.neutral();load=GeneralizedTaskLoad(-obj.gravity(q),np.eye(6),np.diag([.1]*3+[1.]*3))
    p=SimpleNamespace(task_definition=SimpleNamespace(object=obj),task_load=lambda q:load)
    scene=SimpleNamespace(object_frames={'body':'native'})
    a=WrenchHold('native',1,3,(0,0,.8))
    assert schedule_membership(p,q,[a],1.,scene)['inside']
    assert not schedule_membership(p,q,[a,replace(a,start=2.)],1.,scene)['inside']
    assert schedule_membership(p,q,[a,replace(a,start=5.)],1.,scene)['inside']
    assert not schedule_membership(p,q,[replace(a,point_body_m=(.2,0,0))],1.,scene)['inside']


def test_physical_articulated_load_matches_force_and_joint_torque_basis(tmp_path):
    task=FullTask.load('box',tmp_path/'task');p=FullProblem.create(task,tmp_path/'problem','GPG')
    q=np.r_[np.concatenate([s['q_initial'] for s in p.specs]),task.keyframes[2]['object_q']]
    key=SimpleNamespace(q=q);scene=build_scene(p,tmp_path/'native.xml')
    pulses=physical_hold(p,key,scene,[.4,-.3,.2,.7])
    result=schedule_membership(p,q[p.system.object_q],pulses,1.,scene)
    assert result['inside'] and result['load_to_margin_ratio']==pytest.approx(.7)
    assert len(pulses)==3  # Root wrench and an equal/opposite hinge couple.


def test_literal_mode_rejects_domain_and_integration_extensions(tmp_path):
    box=FullTask.load('box',tmp_path/'box')
    box.configuration=replace(box.configuration,formulation_mode='manuscript_literal')
    with pytest.raises(ValueError,match='one free rigid'):require_literal_task(box)
    panel=FullTask.load('panel',tmp_path/'panel')
    panel.configuration=replace(panel.configuration,formulation_mode='manuscript_literal')
    with pytest.raises(ValueError,match='independent_se3'):require_literal_task(panel)
    panel.configuration=replace(panel.configuration,transition=replace(panel.configuration.transition,configuration_update='independent_se3'))
    require_literal_task(panel)


def test_framework_badge_excludes_fixed_path_and_disabled_refinement(tmp_path,monkeypatch):
    from contact_transfer.equation_contract import audit_episode
    task=FullTask.load('panel',tmp_path/'task')
    p=SimpleNamespace(task_definition=task,refine_enabled=True)
    r=SimpleNamespace(objective_mode='motion_cost',full_geometry_checked=True)
    key=SimpleNamespace(source=dict(source_certificate=True,source_refinement_enabled=True))
    episode=SimpleNamespace(problem=p,keyframes=[key],trajectories=[r],contacts_by_interval=[()])
    monkeypatch.setattr('contact_transfer.equation_contract.literal_integration',lambda p,r:dict(passed=True))
    assert audit_episode(episode)['full_framework_stage_execution']
    r.objective_mode='fixed_path_allocation'
    assert not audit_episode(episode)['full_framework_stage_execution']
    assert not audit_episode(episode)['full_paper_pipeline_executed']
    r.objective_mode='motion_cost';p.refine_enabled=False
    assert not audit_episode(episode)['full_framework_stage_execution']


def test_margin_membership_rejects_out_of_image_and_zero_margin_nonzero_load():
    load=GeneralizedTaskLoad(np.zeros(6),np.eye(6)[:,5:],np.ones((1,1)))
    assert normalized_membership(load,np.zeros(6),0.)['inside']
    assert not normalized_membership(load,np.array([0,0,0,0,.1,0]),1.)['inside']
    assert not normalized_membership(load,np.array([0,0,0,0,0,.1]),0.)['inside']


@pytest.mark.parametrize('coordinates',['cartesian','scaled_tangent'])
def test_source_force_coordinates_preserve_the_exact_circular_cone(coordinates):
    from contact_transfer.contacts import Capability,SurfacePatch,TaskWrench
    from contact_transfer.refinement import refine_contacts
    task=TaskWrench(np.array([0.,0,0,0,0,10.]),np.eye(6)[:,3:5],np.eye(2))
    result=refine_contacts([SurfacePatch.fixed([0,0,0],[0,0,1])],task,
        [Capability(mu=.5,normal_max=20.)],point_force_coordinates=coordinates)
    assert result.rho==pytest.approx(5/np.sqrt(2),abs=2e-6)
    np.testing.assert_allclose(np.linalg.norm(result.reactions[:,0,3:5],axis=1),5.,atol=2e-6)


def test_source_witness_verification_rejects_missing_vertices_and_cone_apex_errors():
    from contact_transfer.grouped_source import verify_vertex_witnesses
    from contact_transfer.contacts import Capability
    task=GeneralizedTaskLoad(np.zeros(6),np.eye(6)[:,5:],np.eye(1))
    forces=np.zeros((2,1,6));forces[:,:,3]=1e-4
    check=verify_vertex_witnesses([np.eye(6)],task,[Capability()],0.,forces)
    assert not check['passed'] and check['residuals']['circular_friction']==pytest.approx(1e-4)
    assert not verify_vertex_witnesses([np.eye(6)],task,[Capability()],0.,forces[:1])['passed']
    assert not verify_vertex_witnesses([np.eye(6)],task,[Capability()],float('nan'),np.zeros_like(forces))['passed']


def test_zero_position_and_normal_radii_preserve_admissible_gripper_roll():
    import casadi as ca
    from contact_transfer.contacts import Capability,SurfacePatch,TaskWrench
    from contact_transfer.refinement import refine_contacts
    u=ca.SX.sym('roll',1);c,s=ca.cos(u),ca.sin(u)
    R=ca.vertcat(ca.horzcat(c,-s,0),ca.horzcat(s,c,0),ca.horzcat(0,0,1))
    patch=SurfacePatch(ca.Function('p',[u],[ca.DM.zeros(3)]),ca.Function('R',[u],[R]),np.array([0.]),np.array([np.pi/2]),np.array([.2]))
    task=TaskWrench(np.array([0.,0,0,0,0,-1]),np.eye(6)[:,:1],np.eye(1))
    result=refine_contacts([patch],task,[Capability('G',H=np.eye(6),bounds=np.array([.1,1,1,10,10,10]))],position_radius=0.,normal_radius=0.)
    assert result.rho==pytest.approx(np.sqrt(1.01),abs=2e-6)


def test_nominal_preflight_uses_nominal_load_span_instead_of_robust_image(tmp_path):
    from contact_transfer.full_pipeline import FullPlanner
    a,b=free_body(tmp_path,'a'),free_body(tmp_path,'b');obj=RigidCollection({'a':a,'b':b})
    q=np.r_[a.neutral(),b.neutral()]
    load=GeneralizedTaskLoad(-obj.gravity(q),np.vstack([.8*np.eye(6),.2*np.eye(6)]),np.eye(6))
    features=[SimpleNamespace(frame='a'),SimpleNamespace(frame='b')]
    cfg=load_task_configuration('assembly',{'target':{'robustness_enabled':False,'rho_min':0.}})
    task=SimpleNamespace(object=obj,features=features,keyframes=[dict(object_q=q)],configuration=cfg)
    p=SimpleNamespace(types=('P','P'),task_load=lambda q:load,robots=[],frames=[],specs=[])
    planner=FullPlanner.__new__(FullPlanner);planner.task=task
    planner.preflight(p)  # Necessary rank-one nominal demand fits each point.
    task.configuration=load_task_configuration('assembly')
    with pytest.raises(RuntimeError,match='wrench span'):planner.preflight(p)
    task.configuration=load_task_configuration('assembly',{'target':{'robustness_enabled':True,'rho_min':0.}})
    planner.preflight(p)  # Rank insufficiency excludes positive rho, not rho=0.


def test_default_fixture_policy_does_not_include_inactive_keyframe_names_as_phases():
    assert load_task_configuration('panel').transition.terminal_support_transitions==()


def test_source_outer_dual_distinguishes_impossible_side_force_from_bottom_support(tmp_path):
    import importlib.util
    module=importlib.util.spec_from_file_location('source_diagnosis',Path('tools/diagnose_source_feasibility.py'))
    diagnostic=importlib.util.module_from_spec(module);module.loader.exec_module(diagnostic)
    from contact_transfer.contacts import Capability
    obj=free_body(tmp_path,'dual');q=obj.neutral()
    load=GeneralizedTaskLoad(-obj.gravity(q),np.eye(6)[:,5:],np.eye(1))
    cap=Capability(mu=.8,normal_max=20.)
    side,certificate=diagnostic.outer_test(obj,q,[[0,0,0]],[[0,1,0]],['body'],load,[cap],.001)
    assert side['nominal_globally_infeasible_within_trust_region']
    z=certificate['z']/(-certificate['b']@certificate['z'])
    np.testing.assert_allclose(certificate['A'].T@z,0.,atol=1e-7)
    below,_=diagnostic.outer_test(obj,q,[[0,0,-.1]],[[0,0,-1]],['body'],load,[cap],.001)
    assert below['outer_feasible'] and not below['nominal_globally_infeasible_within_trust_region']
