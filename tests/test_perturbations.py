import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from contact_transfer.perturbations import WrenchPulse,WrenchSchedule

def test_pulse_profile_impulse_and_invalid_inputs():
    p=WrenchPulse('panel',2,.5,(0,8,0));t=np.linspace(2,2.5,10001)
    assert p.scale(1)==p.scale(3)==0 and p.scale(2.25)==pytest.approx(1)
    assert np.trapezoid([8*p.scale(s) for s in t],t)==pytest.approx(2)
    assert p.to_dict()['linear_impulse_Ns']==[0,2,0]
    for kwargs in [dict(duration=0),dict(start=-1),dict(force_world_N=(1,2)),dict(force_world_N=(0,np.nan,0))]:
        args=dict(body='panel',start=2,duration=.5,force_world_N=(0,8,0));args.update(kwargs)
        with pytest.raises(ValueError):WrenchPulse(**args)

def test_force_projection_uses_body_point_and_COM_moment():
    mj=pytest.importorskip('mujoco');m=mj.MjModel.from_xml_string('''<mujoco><option gravity="0 0 0"/><worldbody><body name="panel" pos=".3 .1 .7" quat=".9238795 0 0 .3826834"><freejoint/><inertial pos=".1 0 0" mass="2" diaginertia=".1 .2 .3"/><geom type="box" size=".2 .1 .05"/></body></worldbody></mujoco>''');d=mj.MjData(m);mj.mj_forward(m,d)
    p=WrenchPulse('panel',0,1,(0,8,0),(.0,.2,0),(1,0,0));WrenchSchedule([p])(mj,m,d,.5)
    R=d.xmat[1].reshape(3,3);F=np.array(p.force_world_N)
    # Generalized angular force is about the body/joint origin, not the COM.
    moment=np.array(p.moment_world_Nm)+np.cross(R@p.point_body_m,F)
    np.testing.assert_allclose(d.qfrc_applied,np.r_[F,R.T@moment],atol=1e-8)
    # Free-body angular acceleration instead depends on moment about the COM.
    moment_com=np.array(p.moment_world_Nm)+np.cross(R@(np.array(p.point_body_m)-m.body_ipos[1]),F)
    mj.mj_forward(m,d)
    np.testing.assert_allclose(d.qacc[3:],(R.T@moment_com)/m.body_inertia[1],atol=1e-8)
    with pytest.raises(ValueError):WrenchSchedule([WrenchPulse('missing',0,1,(1,0,0))])(mj,m,d,.5)
