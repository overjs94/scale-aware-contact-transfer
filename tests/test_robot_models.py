import casadi as ca
import numpy as np
import pytest

from contact_transfer import Robot
from contact_transfer.embodiments import catalog, load


@pytest.mark.parametrize("name", ["kuka_iiwa7", "franka_panda", "ur5e", "kinova_gen3",
                                  "xarm7", "unitree_g1", "rb3", "rb5"])
def test_urdf_symbolic_pinocchio_and_frame_derivatives(name):
    info = catalog()[name]
    robot = load(name)
    symbolic = robot.symbolic()
    rng = np.random.default_rng(93)
    q = robot.neutral()
    joff = 7 if robot.floating_base else 0
    q[joff:] = np.clip(q[joff:]+rng.normal(0, .08, robot.n), robot.lower+.001, robot.upper-.001)
    if robot.floating_base:
        from scipy.spatial.transform import Rotation
        q[:3], q[3:7] = [.2, -.3, .8], Rotation.from_rotvec([.2, -.1, .3]).as_quat()
    v, a = rng.normal(0, .2, robot.nv), rng.normal(0, .1, robot.nv)
    for method, args in [("mass", (q,)), ("gravity", (q,)), ("bias", (q, v)),
                         ("inverse_dynamics", (q, v, a))]:
        expected = np.asarray(getattr(robot, method)(*args))
        actual = np.asarray(getattr(symbolic, method)(*args)).reshape(expected.shape)
        np.testing.assert_allclose(actual, expected, atol=2e-8, rtol=2e-8)
    M, C, G = robot.mass(q), robot.coriolis(q, v), robot.gravity(q)
    np.testing.assert_allclose(M, M.T, atol=1e-10)
    assert np.linalg.eigvalsh(M)[0] > 0
    np.testing.assert_allclose(M@a+C@v+G, robot.inverse_dynamics(q, v, a), atol=2e-8)
    Cs = np.asarray(symbolic.coriolis(q, v))
    np.testing.assert_allclose(Cs@v+np.asarray(symbolic.gravity(q)).reshape(-1),
                               robot.bias(q, v), atol=2e-8)
    frame = info["frame"]
    np.testing.assert_allclose(symbolic.fk(q, frame), robot.fk(q, frame), atol=1e-9)
    J = robot.jacobian(q, frame)
    np.testing.assert_allclose(symbolic.jacobian(q, frame), J, atol=1e-9)
    h = 2e-6
    qp, qm = robot.integrate(q, h*v), robot.integrate(q, -h*v)
    Tp, Tm, T = robot.fk(qp, frame), robot.fk(qm, frame), robot.fk(q, frame)
    dR = T[:3, :3].T@(Tp[:3, :3]-Tm[:3, :3])/(2*h)
    physical_velocity = np.r_[dR[2, 1], dR[0, 2], dR[1, 0],
                              (Tp[:3, 3]-Tm[:3, 3])/(2*h)]
    np.testing.assert_allclose(J@v, physical_velocity, atol=2e-7)
    dJ = (robot.jacobian(qp, frame)-robot.jacobian(qm, frame))/(2*h)
    np.testing.assert_allclose(symbolic.jacobian_dot(q, v, frame), dJ, atol=3e-7)
    if not robot.floating_base:
        dM = (robot.mass(qp)-robot.mass(qm))/(2*h)
        np.testing.assert_allclose(dM-Cs-Cs.T, 0, atol=2e-7)


def test_mimic_reduction_and_fixed_inertias(tmp_path):
    path = tmp_path/"mimic.urdf"
    inertia = '<inertial><mass value="1"/><inertia ixx=".1" iyy=".1" izz=".1" ixy="0" ixz="0" iyz="0"/></inertial>'
    path.write_text(f'''<robot name="mimic"><link name="base"/>
    <link name="one">{inertia}</link><link name="two">{inertia}</link>
    <joint name="j1" type="prismatic"><parent link="base"/><child link="one"/>
    <axis xyz="0 0 1"/><limit lower="-1" upper="1" effort="30" velocity="1"/></joint>
    <joint name="j2" type="prismatic"><parent link="one"/><child link="two"/>
    <axis xyz="0 0 1"/><limit lower="-2" upper="2" effort="30" velocity="1"/>
    <mimic joint="j1" multiplier="-0.5" offset="0.1"/></joint></robot>''')
    robot = Robot(path)
    symbolic = robot.symbolic()
    assert robot.n == 1
    np.testing.assert_allclose(robot.mass([.1]), [[1.25]], atol=1e-10)
    np.testing.assert_allclose(robot.gravity([.1]), [14.715], atol=1e-10)
    np.testing.assert_allclose(symbolic.mass([.1]), robot.mass([.1]), atol=1e-10)
    np.testing.assert_allclose(symbolic.jacobian([.1], "two"), robot.jacobian([.1], "two"))


def test_export_reload_and_lie_zero_derivatives(tmp_path, slider_urdf):
    from contact_transfer.lie import se3_exp
    r = Robot(slider_urdf).symbolic()
    metadata = r.export(tmp_path/"generated", frames=["link2"])
    assert metadata["jacobian_order"] == "omega_frame_body,pdot_world"
    fn = ca.Function.load(str(tmp_path/"generated"/"mass.casadi"))
    np.testing.assert_allclose(fn([.1, .2, .3]), r.mass([.1, .2, .3]))
    v = ca.SX.sym("v", 6)
    derivative = ca.Function("dexp", [v], [ca.jacobian(ca.vec(se3_exp(v)), v)])
    assert np.isfinite(derivative(np.zeros(6))).all()


def test_missing_inertia_entries_are_not_silently_discarded(tmp_path):
    path = tmp_path/"invalid.urdf"
    path.write_text('<robot name="invalid"><link name="root"><inertial><mass value="1"/>'
                    '<inertia ixx=".1" iyy=".1" izz=".1"/></inertial></link></robot>')
    with pytest.raises(ValueError, match="all six tensor entries"):
        Robot(path)
