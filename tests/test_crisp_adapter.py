import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from contact_transfer.crisp_simulation import native_contact_to_trace


def test_native_impulse_sign_units_and_rotated_frame():
    # Ground is geom1, sphere is geom2. Weight balance independently fixes the
    # expected sign and N conversion, including a nonidentity contact frame.
    R=Rotation.from_euler('xyz',[.2,-.3,.4]).as_matrix();normal=np.array([0.,0.,-1.])
    for h in [.001,.0005]:
        world_impulse=np.array([0.,0.,-.981*h])
        n,w=native_contact_to_trace(R,R@world_impulse,normal,h)
        np.testing.assert_allclose(n,[0,0,1]);np.testing.assert_allclose(w,[0,0,0,0,0,.981],atol=1e-14)
    with pytest.raises(ValueError):native_contact_to_trace(R,world_impulse,normal,0)
    with pytest.raises(ValueError):native_contact_to_trace(np.zeros((3,3)),world_impulse,normal,.001)
