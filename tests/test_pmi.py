import numpy as np
from contact_transfer.pmi import passive_midpoint_step


def test_virtual_pmi_exact_work_dissipation_balance_under_stiff_large_steps():
    rng=np.random.default_rng(3);x=rng.normal(size=6);v=rng.normal(size=6)
    for h in [.001,.1,1.]:
        for _ in range(20):
            x,v,b=passive_midpoint_step(x,v,rng.normal(size=6),h,mass=np.arange(1,7),damping=.3,stiffness=1e5)
            assert abs(b['balance_residual'])<1e-8


def test_virtual_pmi_lossless_oscillation_without_damping_or_input():
    x=np.array([1.]);v=np.array([0.]);initial=.5*1e5
    for _ in range(200):
        x,v,b=passive_midpoint_step(x,v,np.zeros(1),.1,mass=1.,damping=0.,stiffness=1e5)
    np.testing.assert_allclose(.5*v@v+.5*1e5*(x@x),initial,rtol=1e-12)
