import numpy as np
import pytest

from contact_transfer.contacts import Capability, TaskWrench, contact_basis, map_reaction
from contact_transfer.conic_backends import ConicSettings, conic_available
from contact_transfer.generalized_conic import fixed_generalized_margin
from contact_transfer.generalized_load import GeneralizedTaskLoad
from contact_transfer.optimization import SolverSettings, SolverUnavailableError, solve
from contact_transfer.robot import Robot


@pytest.mark.parametrize('backend',['clarabel','scs'])
def test_conic_libraries_match_circular_friction_and_torque_limits(backend):
    if not conic_available(backend):pytest.skip('Optional solver is not installed')
    E=np.zeros((6,2));E[3:5]=np.eye(2)
    task=GeneralizedTaskLoad(np.array([0.,0,0,0,0,10]),E,np.eye(2))
    result=fixed_generalized_margin([np.eye(6)],task,[Capability(mu=.5,normal_max=20)],solver_settings=ConicSettings(backend=backend))
    assert result['certificate']
    assert result['rho']==pytest.approx(5/np.sqrt(2),abs=2e-6)
    assert result['primal_residual']<2e-6


@pytest.mark.parametrize('backend',['clarabel','scs'])
def test_fixed_rho_conic_qp_and_infeasibility_are_not_confused(backend):
    if not conic_available(backend):pytest.skip('Optional solver is not installed')
    task=GeneralizedTaskLoad(np.array([0.,0,0,0,0,10]),np.eye(6)[:,5:],np.eye(1))
    kwargs=dict(torque_gravity=np.array([1.]),torque_maps=[np.array([[0,0,0,0,0,1.]])],effort=np.array([30.]),solver_settings=ConicSettings(backend=backend))
    ok=fixed_generalized_margin([np.eye(6)],task,[Capability(normal_max=20)],fixed_rho=1.,torque_weight=np.eye(1),**kwargs)
    assert ok['certificate'] and ok['rho']==pytest.approx(1.,abs=2e-6)
    bad=fixed_generalized_margin([np.eye(6)],task,[Capability(normal_max=2)],fixed_rho=1.,**kwargs)
    assert not bad['certificate']


def test_experimental_native_backend_requires_explicit_opt_in():
    with pytest.raises(ValueError,match='experimental'):
        SolverSettings(backend='fatrop')


def test_unavailable_solver_cannot_turn_into_a_source_fallback(monkeypatch):
    import casadi as ca
    import contact_transfer.optimization as optimization
    from contact_transfer.pipeline import source_prior
    from contact_transfer.contacts import SurfacePatch
    monkeypatch.setattr(optimization,'backend_available',lambda backend:False)
    with pytest.raises(SolverUnavailableError):
        source_prior([SurfacePatch.fixed([0,0,0],[0,0,-1])],TaskWrench(np.zeros(6),np.eye(6),np.eye(6)))


def test_symbolic_graph_reuse_preserves_native_mechanics_and_isolates_limits(slider_urdf):
    r=Robot(slider_urdf);a=r.symbolic();b=r.symbolic()
    a.effort[0]=.01
    assert b.effort[0]!=.01
    q=np.array([.1,.2,.3]);np.testing.assert_allclose(b.mass(q),r.mass(q),atol=1e-10)
    r.base_pose[0,3]+=.1;c=r.symbolic()
    np.testing.assert_allclose(np.array(c.fk(q,'link2'))[:3,3],r.fk(q,'link2')[:3,3],atol=1e-10)
    slider_urdf.write_text(slider_urdf.read_text().replace('name="slider"','name="changed"'))
    # Ensure a real byte change even if the fixture has a different robot name.
    slider_urdf.write_text(slider_urdf.read_text()+'\n')
    with pytest.raises(ValueError,match='URDF changed'):
        r.symbolic()


def test_rotated_base_and_updated_gravity_match_reused_symbolic_graph(slider_urdf):
    from scipy.spatial.transform import Rotation
    r=Robot(slider_urdf);r.symbolic()
    r.base_pose[:3,:3]=Rotation.from_euler('y',.7).as_matrix();r.gravity_world[:]=[.1,.2,-9.7]
    model=r.symbolic();q=np.array([.1,.2,.3]);v=np.array([.2,-.1,.1]);a=np.array([.1,.1,-.1])
    np.testing.assert_allclose(model.gravity(q),r.gravity(q)[:,None],atol=1e-10)
    np.testing.assert_allclose(model.inverse_dynamics(q,v,a),r.inverse_dynamics(q,v,a)[:,None],atol=1e-10)
