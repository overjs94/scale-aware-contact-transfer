import numpy as np
import pytest

from contact_transfer.reference import subdivide_reference_times,conditioned_joint_spline


def test_optimizer_clock_roundoff_cannot_create_subnanosecond_reference_cells():
    nodes=2.+np.arange(17)*(.1125+9.3e-11)
    old=np.unique(np.r_[nodes,np.arange(nodes[0],nodes[-1],.02)])
    assert np.min(np.diff(old))<1e-8  # The actual dual-clock failure mechanism.
    times,indices=subdivide_reference_times(nodes,.02)
    np.testing.assert_array_equal(times[indices],nodes)
    assert np.min(np.diff(times))>=.01-1e-12
    assert np.max(np.diff(times))<=.02+1e-12
    spline,_,diagnostics=conditioned_joint_spline(times,np.sin(times)[:,None],indices,np.cos(nodes)[:,None])
    np.testing.assert_allclose(spline(nodes,1)[:,0],np.cos(nodes),atol=1e-12)
    assert diagnostics['maximum_joint_acceleration_rad_s2']<2.


def test_internal_quasistatic_endpoint_does_not_move_during_the_stationary_phase():
    nodes=np.array([0.,1.,2.]);times,indices=subdivide_reference_times(nodes,.03)
    s=np.clip(times-1,0,1);q=(3*s*s-2*s*s*s)[:,None]
    spline,velocity,diagnostics=conditioned_joint_spline(times,q,indices,np.zeros((3,1)))
    np.testing.assert_array_equal(velocity[indices],0.)
    np.testing.assert_allclose(spline(np.linspace(0,1,101)),0.,atol=1e-14)
    assert diagnostics['maximum_joint_acceleration_rad_s2']<7.


@pytest.mark.parametrize('nodes,dt',[([0,0,1],.02),([0,1,2],0),([0,np.nan,2],.02)])
def test_invalid_reference_mesh_fails_before_control(nodes,dt):
    with pytest.raises(ValueError):subdivide_reference_times(nodes,dt)
