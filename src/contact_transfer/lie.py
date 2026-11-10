"""Lie operations supporting NumPy values and CasADi symbolic expressions."""

import casadi as ca
import numpy as np


def skew(x):
    x = ca.reshape(x, 3, 1)
    return ca.vertcat(
        ca.horzcat(0, -x[2], x[1]),
        ca.horzcat(x[2], 0, -x[0]),
        ca.horzcat(-x[1], x[0], 0),
    )


def ad(v):
    return ca.blockcat([[skew(v[:3]), ca.DM.zeros(3, 3)],
                        [skew(v[3:]), skew(v[:3])]])


def transform(R, p):
    return ca.vertcat(ca.horzcat(R, p), ca.DM([[0, 0, 0, 1]]))


def inverse(T):
    return transform(T[:3, :3].T, -T[:3, :3].T @ T[:3, 3])


def adjoint(T):
    R, p = T[:3, :3], T[:3, 3]
    return ca.blockcat([[R, ca.DM.zeros(3, 3)], [skew(p) @ R, R]])


def wrench_map(T):
    """[moment;force] at child origin -> parent origin, both rotated."""
    R, p = T[:3, :3], T[:3, 3]
    return ca.blockcat([[R, skew(p) @ R], [ca.DM.zeros(3, 3), R]])


def axis_rotation(axis, angle):
    K = skew(axis)
    return ca.DM.eye(3) + ca.sin(angle) * K + (1 - ca.cos(angle)) * K @ K


def rpy_rotation(rpy):
    r, p, y = rpy
    return np.asarray(axis_rotation([0, 0, 1], y) @
                      axis_rotation([0, 1, 0], p) @
                      axis_rotation([1, 0, 0], r), dtype=float)


def _coefficients(t2):
    # The unused branch must also have finite derivatives at zero.
    safe = ca.sqrt(ca.fmax(t2, 1e-16))
    A = ca.if_else(t2 < 1e-8, 1-t2/6+t2*t2/120, ca.sin(safe)/safe)
    B = ca.if_else(t2 < 1e-8, .5-t2/24+t2*t2/720,
                   (1-ca.cos(safe))/(safe*safe))
    C = ca.if_else(t2 < 1e-8, 1/6-t2/120+t2*t2/5040,
                   (safe-ca.sin(safe))/(safe**3))
    return A, B, C


def so3_exp(w):
    K = skew(w)
    A, B, _ = _coefficients(ca.sumsqr(w))
    return ca.DM.eye(3) + A*K + B*K@K


def se3_exp(v):
    w, u = v[:3], v[3:]
    K = skew(w)
    A, B, C = _coefficients(ca.sumsqr(w))
    return transform(ca.DM.eye(3)+A*K+B*K@K,
                     (ca.DM.eye(3)+B*K+C*K@K)@u)


def rotation_error(R, Rd):
    A = Rd.T @ R - R.T @ Rd
    return .5 * ca.vertcat(A[2, 1], A[0, 2], A[1, 0])


def rotation_cost(R, Rd):
    return .5 * ca.trace(ca.DM.eye(3) - Rd.T @ R)


def quat_rotation(q):
    """Unit quaternion in xyzw order. Caller maintains normalization."""
    x, y, z, w = q[0], q[1], q[2], q[3]
    return ca.vertcat(
        ca.horzcat(1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)),
        ca.horzcat(2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)),
        ca.horzcat(2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)),
    )


def spatial_inertia(mass, com, inertia_com):
    K = skew(com)
    return ca.blockcat([[ca.DM(inertia_com)-mass*K@K, mass*K],
                        [-mass*K, mass*ca.DM.eye(3)]])


def integrate_floating(q, delta):
    """Right-trivialized SE(3) step; q=[p_world, quaternion_xyzw, joints]."""
    w, u = delta[:3], delta[3:6]
    step = se3_exp(ca.vertcat(w, u))
    pnew = q[:3] + quat_rotation(q[3:7])@step[:3, 3]
    t2 = ca.sumsqr(w)
    t = ca.sqrt(ca.fmax(t2, 1e-16))
    scale = ca.if_else(t2 < 1e-8, .5-t2/48+t2*t2/3840, ca.sin(t/2)/t)
    scalar = ca.if_else(t2 < 1e-8, 1-t2/8+t2*t2/384, ca.cos(t/2))
    xyz = scale*w
    oldxyz, oldscalar = q[3:6], q[6]
    qnew = ca.vertcat(oldscalar*xyz+scalar*oldxyz+ca.cross(oldxyz, xyz),
                     oldscalar*scalar-ca.dot(oldxyz, xyz))
    return ca.vertcat(pnew, qnew, q[7:]+delta[6:])
