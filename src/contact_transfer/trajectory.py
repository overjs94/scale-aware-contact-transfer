"""Sec. IV restricted contact-implicit direct transcription and contact search.

The solver handles configured candidates and grasp phases. A caller-supplied
geometry oracle can add (candidate,node) pairs between restricted solves.
Full mesh validation is an explicit callback, never an implicit success claim.
"""

from dataclasses import dataclass, field

import casadi as ca
import numpy as np

from .contacts import Capability, contact_basis
from .lie import rotation_cost
from .optimization import SolverSettings, equality, solve


@dataclass
class Contact:
    name: str
    point_object: np.ndarray
    normal: np.ndarray
    capability: Capability = field(default_factory=Capability)
    robot_index: int | None = None
    frame: str | None = None
    plane_offset: float = 0.
    nodes: tuple | None = None
    attached_steps: tuple = ()
    environment_robot_index: int | None = None
    maintained_steps: tuple = ()
    sticking_steps: tuple = ()
    released_steps: tuple = ()
    group_id: str | None = None
    release_at_end: bool = False

    def __post_init__(self):
        self.point_object = np.asarray(self.point_object, float).reshape(3)
        self.normal = np.asarray(self.normal, float).reshape(3)
        if not np.isclose(np.linalg.norm(self.normal), 1):
            raise ValueError("contact normal must be unit length")
        if self.robot_index is None and self.capability.kind != "P":
            raise ValueError("environment plane contacts are unilateral")
        if set(self.released_steps)&(set(self.maintained_steps)|set(self.sticking_steps)):
            raise ValueError("released and maintained/sticking phases cannot overlap")

    def geometry(self, system, q):
        """Return gap, local relative velocity Jacobian, positional closure."""
        if self.robot_index is None and self.environment_robot_index is not None:
            j = self.environment_robot_index
            robot = system.robots[j]
            T = robot.fk(q[system.q_slices[j]], self.frame)
            Jr = robot.jacobian(q[system.q_slices[j]], self.frame, "world_aligned")
            from .lie import skew
            offset = T[:3, :3]@ca.DM(self.point_object)
            point = T[:3, 3]+offset
            J = ca.MX.zeros(3, system.nv) if isinstance(q, ca.MX) else ca.SX.zeros(3, system.nv)
            J[:, system.v_slices[j]] = Jr[3:, :]-skew(offset)@Jr[:3, :]
            R = ca.DM(contact_basis(self.normal))
            return ca.dot(ca.DM(self.normal), point)-self.plane_offset, R.T@J, None
        point = system.point_world(q, self.point_object)
        Jo = system.point_jacobian_object(q, self.point_object)
        if self.robot_index is None:
            R = ca.DM(contact_basis(self.normal))
            return ca.dot(ca.DM(self.normal), point)-self.plane_offset, R.T@Jo, None
        j = self.robot_index
        robot = system.robots[j]
        Tee = robot.fk(q[system.q_slices[j]], self.frame)
        R = system.object_pose(q)[:3, :3]@ca.DM(contact_basis(self.normal))
        Jr = robot.jacobian(q[system.q_slices[j]], self.frame, "world_aligned")
        Jp = -Jo
        Jp[:, system.v_slices[j]] += Jr[3:, :]
        gap = ca.dot(R[:, 2], Tee[:3, 3]-point)
        closure = Tee[:3, 3]-point
        if self.capability.kind == "P":
            return gap, R.T@Jp, closure
        Jw = ca.MX.zeros(3, system.nv) if isinstance(q, ca.MX) else ca.SX.zeros(3, system.nv)
        Jw[:, system.v_slices[j]] = Jr[:3, :]
        Jw[:, system.object_v.start:system.object_v.start+3] -= system.object_pose(q)[:3, :3]
        return gap, ca.vertcat(R.T@Jw, R.T@Jp), closure


@dataclass
class TrajectoryResult:
    q: np.ndarray
    v: np.ndarray
    tau: np.ndarray
    dt: np.ndarray
    impulses: dict
    objective: float
    primal_residual: float
    complementarity_residual: float
    friction_residual: float
    minimum_gap: float
    full_geometry_checked: bool
    relaxations: tuple
    gripper_velocity_residual: float = 0.


def solve_cito(system, q_start, q_goal, contacts, *, steps=12, duration=1.,
               dt_bounds=(.01, .2), desired_object_poses=None, fixed_dt=True,
               relaxations=(1e-3, 1e-5, 1e-8), acceptance=2e-5,
               kinematic_tolerance=1e-7,
               characteristic_length=1., Kp=None, kR=1., Q_tau=None, Q_v=None,
               duration_weight=1.,
               initial=None, collision_margins=None, full_geometry_validator=None,
               settings=None):
    if steps < 2 or duration <= 0 or not relaxations or min(relaxations) < 0 or kinematic_tolerance <= 0:
        raise ValueError("invalid transcription or continuation")
    if fixed_dt and not dt_bounds[0] <= duration/steps <= dt_bounds[1]:
        raise ValueError("fixed step falls outside dt_bounds")
    if characteristic_length <= 0 or kR <= 0 or duration_weight <= 0:
        raise ValueError("object length and orientation/duration weights must be positive")
    def weight(value, size, default):
        value = np.eye(size)*default if value is None else np.asarray(value, float)
        if np.ndim(value) == 0:
            value = np.eye(size)*value
        if value.shape != (size, size) or not np.allclose(value, value.T):
            raise ValueError("cost weights must be symmetric with compatible dimensions")
        if size and np.linalg.eigvalsh(value)[0] < -1e-12:
            raise ValueError("cost weights must be positive semidefinite")
        return ca.DM(value)
    Kp = weight(Kp, 3, 20.)
    Qt, Qv = weight(Q_tau, system.nu, 1e-4), weight(Q_v, system.nv, 1e-3)
    q_start, q_goal = np.asarray(q_start, float).reshape(system.nq), np.asarray(q_goal, float).reshape(system.nq)
    opti = ca.Opti()
    Q, V = opti.variable(system.nq, steps+1), opti.variable(system.nv, steps+1)
    U = opti.variable(system.nu, steps) if system.nu else ca.MX.zeros(0, steps)
    H = ca.DM.ones(steps)*duration/steps if fixed_dt else opti.variable(steps)
    if not fixed_dt:
        opti.subject_to(opti.bounded(dt_bounds[0], H, dt_bounds[1]))
        opti.set_initial(H, duration/steps)
    eps = opti.parameter()
    equality(opti, Q[:, 0]-q_start)
    # SE(3) integration preserves quaternion signs; align goal signs beforehand.
    equality(opti, Q[:, -1]-q_goal)
    equality(opti, V[:, 0])
    equality(opti, V[:, -1])
    qguess = np.column_stack([q_start+(q_goal-q_start)*i/steps for i in range(steps+1)])
    # Normalize interpolated quaternions for a finite warm start.
    base_slices = [slice(s.start+3, s.start+7) for r, s in zip(system.robots, system.q_slices) if r.floating_base]
    base_slices.append(slice(system.object_q.start+3, system.object_q.start+7))
    for s in base_slices:
        for i in range(steps+1):
            norm = np.linalg.norm(qguess[s, i])
            if norm < 1e-10:
                raise ValueError("align endpoint quaternion signs before interpolation")
            qguess[s, i] /= norm
    opti.set_initial(Q, qguess if initial is None else initial.q.T)
    opti.set_initial(V, 0 if initial is None else initial.v.T)
    if system.nu and initial is not None:
        opti.set_initial(U, initial.tau.T)
    impulses = {c.name: [] for c in contacts}
    if len(impulses) != len(contacts):
        raise ValueError("contact names must be unique")
    comp, friction, gap_records, grasp_velocities = [], [], [], []
    objective = 0
    for l in range(steps+1):
        for robot, qs, vs in zip(system.robots, system.q_slices, system.v_slices):
            qj = Q[qs.start+(7 if robot.floating_base else 0):qs.stop, l]
            vj = V[vs.start+(6 if robot.floating_base else 0):vs.stop, l]
            opti.subject_to(opti.bounded(robot.lower, qj, robot.upper))
            opti.subject_to(opti.bounded(-robot.velocity, vj, robot.velocity))
        if collision_margins is not None:
            opti.subject_to(collision_margins(Q[:, l]) >= 0)
    for l in range(steps):
        h = H[l]
        if system.nu:
            opti.subject_to(opti.bounded(-system.effort, U[:, l], system.effort))
        generalized_impulse = ca.MX.zeros(system.nv)
        for contact in contacts:
            gap, Jnext, closure = contact.geometry(system, Q[:, l+1])
            _, Jnow, _ = contact.geometry(system, Q[:, l])
            vel = Jnext@V[:, l+1]
            cap = contact.capability
            active = (contact.nodes is None or l in contact.nodes) and l not in contact.released_steps
            if cap.kind == "P":
                opti.subject_to(gap >= 0)
                gap_records.append(gap)
                if l in contact.maintained_steps or l in contact.sticking_steps:
                    equality(opti, gap)
                if l in contact.sticking_steps:
                    opti.subject_to(opti.bounded(-kinematic_tolerance, vel[:2], kinematic_tolerance))
                    grasp_velocities.append(vel[:2])
                P = opti.variable(3) if active else ca.DM.zeros(3)
                if active:
                    opti.subject_to(opti.bounded(0, P[2], h*cap.normal_max))
                    opti.subject_to(ca.sumsqr(P[:2]) <= (cap.mu*P[2])**2)
                    opti.subject_to(P[2]*gap <= eps)
                    comp.append(P[2]*gap)
                    speed = opti.variable()
                    opti.subject_to(speed >= 0)
                    opti.subject_to(speed**2 >= ca.sumsqr(vel[:2]))
                    dissipation = ca.dot(P[:2], vel[:2])+cap.mu*P[2]*speed
                    equality(opti, dissipation)
                    friction.append(dissipation)
                    if initial is not None and contact.name in initial.impulses:
                        opti.set_initial(P, initial.impulses[contact.name][l])
                generalized_impulse += Jnow.T@P
            else:
                attached = l in contact.attached_steps
                if attached:
                    warm_reaction = initial.impulses[contact.name][l] if initial is not None and contact.name in initial.impulses else None
                    P = cap.reaction(opti, scale=h, initial=warm_reaction)
                    equality(opti, closure)
                    if cap.has_moment:
                        Tee = system.robots[contact.robot_index].fk(
                            Q[system.q_slices[contact.robot_index], l+1], contact.frame)
                        Rwc = system.object_pose(Q[:, l+1])[:3, :3]@contact_basis(contact.normal)
                        opti.subject_to(rotation_cost(Tee[:3, :3], Rwc@ca.diag(ca.DM([1, -1, -1]))) <= 1e-8)
                        constrained_velocity = vel
                    else:
                        constrained_velocity = vel[3:]
                    # Exact closure plus its exact time derivative can produce
                    # redundant equalities after transcription. Keep closure
                    # exact and enforce velocity within an explicit tolerance.
                    opti.subject_to(opti.bounded(-kinematic_tolerance,
                                                 constrained_velocity, kinematic_tolerance))
                    grasp_velocities.append(constrained_velocity)
                    if l == min(contact.attached_steps):
                        # No attachment impulse: acquisition starts at zero relative velocity.
                        before = Jnow@V[:, l]
                        before = before if cap.has_moment else before[3:]
                        opti.subject_to(opti.bounded(-kinematic_tolerance, before, kinematic_tolerance))
                        grasp_velocities.append(before)
                    if l+1 not in contact.attached_steps and (l < steps-1 or contact.release_at_end):
                        equality(opti, P)  # unload before release
                else:
                    P = ca.DM.zeros(6)
                generalized_impulse += Jnow.T@P
            impulses[contact.name].append(P)
        dyn = (system.mass(Q[:, l])@(V[:, l+1]-V[:, l]) -
               h*(system.B@U[:, l]-system.bias(Q[:, l], V[:, l]))-generalized_impulse)
        equality(opti, dyn)
        equality(opti, Q[:, l+1]-system.integrate(Q[:, l], h*V[:, l+1]))
        objective += h*(ca.mtimes([U[:, l].T, Qt, U[:, l]])+
                        ca.mtimes([V[:, l].T, Qv, V[:, l]])+duration_weight)
        if desired_object_poses is not None:
            T, Td = system.object_pose(Q[:, l]), ca.DM(desired_object_poses[l])
            ep = (T[:3, 3]-Td[:3, 3])/characteristic_length
            objective += h*(.5*ca.mtimes([ep.T, Kp, ep])+kR*rotation_cost(T[:3, :3], Td[:3, :3]))
    opti.minimize(objective)
    residual = np.inf
    for relaxation in relaxations:
        opti.set_value(eps, relaxation)
        sol, residual = solve(opti, settings)
        opti.set_initial(opti.x, sol.value(opti.x))
    comp_res = max([abs(float(sol.value(x))) for x in comp], default=0.)
    fric_res = max([abs(float(sol.value(x))) for x in friction], default=0.)
    min_gap = min([float(sol.value(x)) for x in gap_records], default=0.)
    gripper_res = max([float(np.max(np.abs(sol.value(x)))) for x in grasp_velocities], default=0.)
    if max(comp_res, fric_res, -min_gap, gripper_res) > acceptance:
        raise RuntimeError("unrelaxed contact residual exceeds acceptance tolerance")
    result = TrajectoryResult(
        np.array(sol.value(Q)).reshape(system.nq, steps+1).T,
        np.array(sol.value(V)).reshape(system.nv, steps+1).T,
        np.asarray(sol.value(U)).reshape(system.nu, steps).T if system.nu else np.zeros((steps, 0)),
        np.asarray(sol.value(H)).reshape(steps),
        {name: np.array([np.asarray(sol.value(p)).reshape(-1) for p in ps])
         for name, ps in impulses.items()}, float(sol.value(objective)), residual,
        comp_res, fric_res, min_gap, False, tuple(relaxations), gripper_res)
    if full_geometry_validator is not None:
        if not full_geometry_validator(result):
            raise RuntimeError("full-geometry validation rejected trajectory")
        result.full_geometry_checked = True
    return result


def solve_with_contact_search(system, q_start, q_goal, contacts, oracle, *, max_rounds=8, **kwargs):
    """STOCS-style outer orchestration; oracle returns new Contact/node candidates.

    A geometry-specific oracle must inspect full geometry, not just current
    restricted candidates. This function does not claim to reproduce STOCS's
    complete merit-function algorithm.
    """
    candidates, initial = list(contacts), kwargs.pop("initial", None)
    for _ in range(max_rounds):
        result = solve_cito(system, q_start, q_goal, candidates, initial=initial, **kwargs)
        additions = list(oracle(result, candidates))
        if not additions:
            return result
        names = {c.name for c in candidates}
        if any(c.name in names for c in additions):
            raise ValueError("oracle must return new uniquely named candidates")
        candidates.extend(additions)
        initial = result
    raise RuntimeError("contact search budget exhausted")
