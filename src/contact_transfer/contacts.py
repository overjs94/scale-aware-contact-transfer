"""Object-on-agent reactions, angular-first wrenches, local surface patches."""

from dataclasses import dataclass
from itertools import product

import casadi as ca
import numpy as np

from .lie import transform, wrench_map


def contact_basis(normal):
    n = np.asarray(normal, float).reshape(3)
    if not np.isfinite(n).all() or np.linalg.norm(n) < 1e-12:
        raise ValueError("normal must be nonzero and finite")
    n /= np.linalg.norm(n)
    seed = np.eye(3)[np.argmin(abs(n))]
    t1 = np.cross(seed, n)
    t1 /= np.linalg.norm(t1)
    return np.column_stack((t1, np.cross(n, t1), n))


@dataclass
class SurfacePatch:
    """Differentiable p(u), R(u), R[:,2]=outward normal. dim=0 fixes geometry."""
    position: ca.Function
    rotation: ca.Function
    lower: np.ndarray
    upper: np.ndarray
    initial: np.ndarray

    @classmethod
    def plane(cls, center, normal, half_size=(.02, .02)):
        u = ca.SX.sym("u", 2)
        R = contact_basis(normal)
        half_size = np.broadcast_to(np.asarray(half_size, float), (2,)).copy()
        if np.any(half_size <= 0):
            raise ValueError("plane half-size must be positive; use fixed() for a fixed point")
        return cls(ca.Function("surface_p", [u], [ca.DM(center)+R[:, :2]@u]),
                   ca.Function("surface_R", [u], [R]), -half_size, half_size, np.zeros(2))

    @classmethod
    def fixed(cls, position, normal):
        u = ca.SX.sym("u", 0)
        return cls(ca.Function("fixed_p", [u], [ca.DM(position)]),
                   ca.Function("fixed_R", [u], [contact_basis(normal)]),
                   np.zeros(0), np.zeros(0), np.zeros(0))

    def instantiate(self, opti):
        if len(self.initial):
            u = opti.variable(len(self.initial))
            opti.subject_to(opti.bounded(self.lower, u, self.upper))
            opti.set_initial(u, self.initial)
        else:
            u = ca.DM.zeros(0)
        return u, self.position(u), self.rotation(u)


@dataclass(frozen=True)
class Capability:
    """P: Coulomb point contact; G: calibrated bounded resultant H*zeta."""
    kind: str = "P"
    mu: float = .5
    normal_max: float = 100.
    H: object = None
    bounds: object = None

    def __post_init__(self):
        if self.kind not in {"P", "G"} or self.mu < 0 or self.normal_max <= 0:
            raise ValueError("invalid contact capability")
        if self.kind == "G":
            H, b = np.asarray(self.H, float), np.asarray(self.bounds, float)
            if H.ndim != 2 or H.shape[0] != 6 or b.shape != (H.shape[1],):
                raise ValueError("gripper H must be (6,d), bounds must be (d,)")
            if not np.isfinite(H).all() or not np.isfinite(b).all() or np.any(b <= 0):
                raise ValueError("gripper capability requires finite calibrated bounds")
            if np.linalg.norm(H[:3]) > 1e-12 and np.linalg.matrix_rank(H) < 6:
                raise ValueError("Partial-moment interfaces need a matching custom closure model; use force-only or full-rank rigid holding.")

    @classmethod
    def force_gripper(cls, force_bounds):
        return cls("G", H=np.vstack((np.zeros((3, 3)), np.eye(3))), bounds=np.array(force_bounds))

    def reaction(self, opti):
        if self.kind == "P":
            f = opti.variable(3)
            opti.subject_to(opti.bounded(0, f[2], self.normal_max))
            opti.subject_to(ca.sumsqr(f[:2]) <= (self.mu*f[2])**2)
            opti.set_initial(f, [0, 0, min(10, self.normal_max/2)])
            return ca.vertcat(ca.DM.zeros(3), f)
        z = opti.variable(len(self.bounds))
        opti.subject_to(opti.bounded(-np.asarray(self.bounds), z, self.bounds))
        return ca.DM(self.H) @ z


@dataclass(frozen=True)
class TaskWrench:
    """All wrenches [moment;force] about the object origin in object axes."""
    nominal: np.ndarray
    E: np.ndarray
    D: np.ndarray

    def __post_init__(self):
        nominal, E, D = np.asarray(self.nominal), np.asarray(self.E), np.asarray(self.D)
        if nominal.shape != (6,) or E.ndim != 2 or E.shape[0] != 6:
            raise ValueError("nominal must be (6,), E must be (6,d)")
        d = E.shape[1]
        if not 1 <= d <= 6 or np.linalg.matrix_rank(E) != d:
            raise ValueError("task basis must have 1..6 independent columns")
        if D.shape != (d, d) or not np.allclose(D, np.diag(np.diag(D))) or np.any(np.diag(D) <= 0):
            raise ValueError("D must be positive diagonal")
        if not all(np.isfinite(x).all() for x in (nominal, E, D)):
            raise ValueError("task wrench data must be finite")

    @property
    def vertices(self):
        return np.array(list(product((-1., 1.), repeat=np.shape(self.E)[1])))

    def required(self, rho, vertex):
        return ca.DM(self.nominal) + rho*ca.DM(self.E)@ca.DM(self.D)@ca.DM(vertex)


def gravity_wrench(mass, com_object, R_world_object=np.eye(3), gravity=(0, 0, -9.81)):
    force = mass*np.asarray(R_world_object).T@np.asarray(gravity)
    return np.r_[np.cross(com_object, force), force]


def map_reaction(p_object, R_object_contact, reaction_contact):
    return wrench_map(transform(R_object_contact, p_object)) @ reaction_contact
