"""Independent gates for controlled robustness and contact-type experiments."""
import numpy as np
import pytest
from contact_transfer.robustness_study import source_refinement,PROTOCOL
from contact_transfer.contacts import map_reaction


def test_nominal_ablation_removes_vertices_but_keeps_feasible_nominal_equilibrium():
    result,record=source_refinement(False)
    assert len(result.reactions)==1 and record['optimization_vertex_count']==1
    assert record['independent_common_polytope_vertex_count']==8
    np.testing.assert_allclose(result.positions,record['source_positions_before_m'],atol=1e-8)
    total=sum(np.array(map_reaction(p,R,w)).ravel() for p,R,w in zip(result.positions,result.rotations,result.reactions[0]))
    np.testing.assert_allclose(total,[0,0,0,0,0,-PROTOCOL['source_mass_kg']*9.81],atol=2e-6)


def test_robust_refinement_improves_same_independent_box_with_unchanged_scales():
    nominal,n=source_refinement(False);robust,r=source_refinement(True)
    assert len(robust.reactions)==8 and r['optimization_vertex_count']==8
    assert r['source_E']==n['source_E'] and r['source_D']==n['source_D']
    assert r['source_rho_after']>n['source_rho_after']+.1
    assert r['source_independent_residual']<2e-6
    displacement=np.linalg.norm(robust.positions-nominal.positions,axis=1)
    assert np.max(displacement)<=PROTOCOL['source_position_radius_m']+2e-6
    assert PROTOCOL['force_levels_N']==[0.,-4.,4.,-8.,8.,-12.,12.]


def test_nominal_ablation_cannot_claim_a_positive_margin_constraint():
    from contact_transfer.refinement import refine_contacts
    from contact_transfer.contacts import SurfacePatch,TaskWrench
    patch=SurfacePatch.fixed([0,0,0],[0,0,1]);task=TaskWrench(np.zeros(6),np.eye(6),np.eye(6))
    with pytest.raises(ValueError,match='Nominal-only'):
        refine_contacts([patch],task,rho_min=.1,geometry_weight=1.,robustness_enabled=False)


@pytest.mark.parametrize('enabled',[True,False])
def test_hold_plan_uses_real_unactuated_body_and_the_declared_wrench_point(enabled,tmp_path):
    from pathlib import Path
    mj=pytest.importorskip('mujoco')
    if not Path('.cache/panda_meshes/link7.stl').exists():pytest.skip('Fetch pinned native Panda meshes')
    from contact_transfer.robustness_study import make_plan
    from contact_transfer.full_scene import build_scene
    from contact_transfer.full_validation import cross,validate_interval
    episode,source=make_plan(tmp_path,enabled);scene=build_scene(episode.problem,tmp_path/'native-check.xml');m=mj.MjModel.from_xml_path(str(scene.path))
    assert m.nu==21 and m.neq==m.nmocap==0
    assert not any((mj.mj_id2name(m,mj.mjtObj.mjOBJ_GEOM,i) or '').startswith('jig_') for i in range(m.ngeom))
    root=mj.mj_name2id(m,mj.mjtObj.mjOBJ_BODY,next(iter(scene.object_roots.values())));joint=m.body_jntadr[root]
    assert m.jnt_type[joint]==mj.mjtJoint.mjJNT_FREE and joint not in m.actuator_trnid[:,0]
    qo=episode.keyframes[0].q[episode.problem.system.object_q];load=episode.problem.task_load(qo)
    np.testing.assert_allclose(load.E,np.vstack([cross(PROTOCOL['force_application_point_body_m']),np.eye(3)]),atol=1e-12)
    np.testing.assert_allclose(np.diag(load.D),PROTOCOL['target_force_box_scales_N'])
    assert abs(source['target_rho']-source['source_rho_after'])<2e-6
    assert all(validate_interval(episode.problem,r,c)['passed'] for r,c in zip(episode.trajectories,episode.contacts_by_interval))
