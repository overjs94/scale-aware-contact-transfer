"""Load profiles and independent certificate membership, without a physics mock."""
import numpy as np
import pytest
from contact_transfer.perturbations import WrenchHold
from contact_transfer.margin_hold_study import CASES,disturbances,initial_certificate
from contact_transfer.full_validation import cross


def test_sustained_load_has_smooth_edges_exact_plateau_and_correct_impulse():
    pulse=WrenchHold('panel',4.,5.,(0.,0.,4.),ramp_duration_s=.5)
    assert pulse.scale(3.9)==pulse.scale(9.1)==0
    assert all(pulse.scale(t)==1 for t in [4.5,5.,6.,7.,8.5])
    t=np.linspace(4,9,10001)
    np.testing.assert_allclose(np.trapezoid([pulse.scale(v)*4 for v in t],t),18.,atol=1e-7)
    assert pulse.to_dict()['linear_impulse_Ns']==[0.,0.,18.]
    for edge in [4.,4.5,8.5,9.]:
        assert abs(pulse.scale(edge+1e-6)-pulse.scale(edge-1e-6))<1e-8
    with pytest.raises(ValueError,match='ramps'):
        WrenchHold('panel',0.,1.,(0,0,1),ramp_duration_s=.6)


def certificate(rho):
    r=[0.,.18,.02]
    return dict(target_E=np.vstack([cross(r),np.eye(3)]).tolist(),
                target_D=np.diag([2.56,2.56,8.]).tolist(),target_rho=rho,
                independent_target=dict(frozen_conic=dict(rho=rho)))


@pytest.mark.parametrize('case',list(CASES))
def test_prespecified_forces_fit_on_certificate_without_ignoring_the_lever_arm(case):
    pulses=disturbances('panel',case)
    result=initial_certificate(certificate(.5978475202),pulses)
    assert result['inside_initial_static_box']
    assert result['subspace_residual']<1e-12
    if pulses:
        assert max(p.start+p.duration for p in pulses)<=12
        assert all(a.start+a.duration<=b.start for a,b in zip(pulses,pulses[1:]))
    if case=='sustained_up4':
        assert not initial_certificate(certificate(.3768107963),pulses)['inside_initial_static_box']
        wrong_point=(WrenchHold('panel',4.,5.,(0.,0.,4.),point_body_m=(0.,.3,0.)),)
        assert not initial_certificate(certificate(.5978475202),wrong_point)['inside_initial_static_box']


def test_common_nonzero_control_is_inside_both_boxes():
    pulses=disturbances('panel','common_up2')
    assert all(initial_certificate(certificate(rho),pulses)['inside_initial_static_box']
               for rho in [.5978475202,.3768107963])


@pytest.mark.parametrize('name',['margin_platform','margin_platform_compact'])
def test_rigid_payload_mass_com_and_inertia_match_the_authored_geometry(name):
    from pathlib import Path
    import json
    from contact_transfer.robot import Robot
    folder=Path('src/contact_transfer/scenarios')/name
    design=json.loads((folder/'design.json').read_text());robot=Robot(folder/'panel.urdf',floating_base=True)
    boxes=design['components'];mass=sum(c['mass_kg'] for c in boxes)
    center=sum(c['mass_kg']*np.asarray(c['center_m']) for c in boxes)/mass;I=np.zeros((3,3))
    for c in boxes:
        x,y,z=c['size_m'];delta=np.asarray(c['center_m'])-center
        I+=c['mass_kg']*(np.diag([y*y+z*z,x*x+z*z,x*x+y*y])/12+(delta@delta)*np.eye(3)-np.outer(delta,delta))
    link=robot.tree.links['panel'];assert link.mass==mass
    np.testing.assert_allclose(link.com,center,atol=1e-12)
    np.testing.assert_allclose(link.inertia,I,atol=1e-12)
    q=robot.neutral();M=robot.mass(q);np.testing.assert_allclose(M[3:,3:],mass*np.eye(3),atol=1e-12)


def test_loaded_plan_transfers_the_urdf_com_and_matches_native_inertial_origin(tmp_path):
    from pathlib import Path
    mj=pytest.importorskip('mujoco')
    if not Path('.cache/panda_meshes/link7.stl').is_file():pytest.skip('Pinned Panda meshes required')
    from contact_transfer.robustness_study import make_plan
    from contact_transfer.full_scene import build_scene
    episode,source=make_plan(tmp_path,True,hold_duration_s=13.,object_urdf='src/contact_transfer/scenarios/margin_platform/panel.urdf')
    np.testing.assert_allclose(np.asarray(source['source_com_object_m'])*4,source['target_com_object_m'])
    scene=build_scene(episode.problem,tmp_path/'native.xml');m=mj.MjModel.from_xml_path(str(scene.path))
    body=mj.mj_name2id(m,mj.mjtObj.mjOBJ_BODY,'object_urdf_panel')
    np.testing.assert_allclose(m.body_ipos[body],source['target_com_object_m'],atol=1e-12)
    assert m.body_mass[body]==2. and m.neq==0 and m.nmocap==0 and m.nu==21
