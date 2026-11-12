"""Sec. IV restricted contact-implicit direct transcription and contact search.

The solver handles configured candidates and grasp phases. A caller-supplied
geometry oracle can add (candidate,node) pairs between restricted solves.
Full mesh validation is an explicit callback, never an implicit success claim.
"""

from dataclasses import dataclass, field

import casadi as ca
import numpy as np

from .contacts import Capability, contact_basis
from .lie import rotation_cost, rotation_error
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
    object_frame: str | None = None
    project_to_surface: bool = False
    tangent_bounds: object = None
    align_normal: bool = False
    aperture_rate_max: float = .12
    rotation_object: object = None
    plane_bounds: object = None

    def __post_init__(self):
        self.point_object = np.asarray(self.point_object, float).reshape(3)
        self.normal = np.asarray(self.normal, float).reshape(3)
        if not np.isclose(np.linalg.norm(self.normal), 1):
            raise ValueError("contact normal must be unit length")
        if self.robot_index is None and self.capability.kind != "P":
            raise ValueError("environment plane contacts are unilateral")
        if set(self.released_steps)&(set(self.maintained_steps)|set(self.sticking_steps)):
            raise ValueError("released and maintained/sticking phases cannot overlap")
        if self.rotation_object is not None:
            R=np.asarray(self.rotation_object,float)
            if R.shape!=(3,3) or not np.allclose(R.T@R,np.eye(3)) or not np.isclose(np.linalg.det(R),1) or not np.allclose(R[:,2],self.normal):
                raise ValueError('Interface rotation must be proper with the prescribed outward normal')
        if self.plane_bounds is not None:
            bounds=np.asarray(self.plane_bounds,float)
            if self.robot_index is not None or self.environment_robot_index is not None or bounds.shape!=(2,2) or np.any(bounds[0]>=bounds[1]) or not np.isfinite(bounds).all():raise ValueError('Finite XY bounds currently support object/environment planes only')

    def basis(self):
        return contact_basis(self.normal) if self.rotation_object is None else np.asarray(self.rotation_object)

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
            R = ca.DM(self.basis())
            return ca.dot(ca.DM(self.normal), point)-self.plane_offset, R.T@J, None
        point = system.point_world(q, self.point_object, self.object_frame)
        Jo = system.point_jacobian_object(q, self.point_object, self.object_frame)
        if self.robot_index is None:
            R = ca.DM(self.basis())
            return ca.dot(ca.DM(self.normal), point)-self.plane_offset, R.T@Jo, None
        j = self.robot_index
        robot = system.robots[j]
        Tee = robot.fk(q[system.q_slices[j]], self.frame)
        R = system.object_pose(q, self.object_frame)[:3, :3]@ca.DM(self.basis())
        Jr = robot.jacobian(q[system.q_slices[j]], self.frame, "world_aligned")
        if self.capability.kind == 'P' and self.project_to_surface:
            # Material velocity is evaluated at the instantaneous projection,
            # not at a distant frozen surface point on the same plane.
            Tlink = system.object_pose(q, self.object_frame)
            normal_gap = ca.dot(R[:, 2], Tee[:3, 3]-point)
            closest = Tee[:3, 3]-normal_gap*R[:, 2]
            local = Tlink[:3, :3].T@(closest-Tlink[:3, 3])
            Jo = system.point_jacobian_object(q, local, self.object_frame)
        Jp = -Jo
        Jp[:, system.v_slices[j]] += Jr[3:, :]
        gap = ca.dot(R[:, 2], Tee[:3, 3]-point)
        closure = Tee[:3, 3]-point
        if self.capability.kind == "P":
            return gap, R.T@Jp, closure
        Jw = ca.MX.zeros(3, system.nv) if isinstance(q, ca.MX) else ca.SX.zeros(3, system.nv)
        Jw[:, system.v_slices[j]] = Jr[:3, :]
        Jw -= system.angular_jacobian_object(q, self.object_frame)
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
    apertures: dict = field(default_factory=dict)
    continuation: dict = field(default_factory=dict)
    grasp_velocity_model: str = 'instantaneous'
    instantaneous_grasp_velocity_residual: float = 0.
    objective_mode: str = 'motion_cost'
    configuration_update: str = 'independent_se3'
    configuration_defect_bounds: tuple = ()
    exact_sticking_velocity: bool = False
    contact_force_coordinates: str = 'cartesian'
    friction_speed_model: str = 'auxiliary_cone'
    endpoint_redundancy_eliminated: bool = False
    friction_speed_smoothing: float = 0.


def constrained_object_roots(system,contacts,step):
    """Roots whose pose is determined by rigid grasp or three sticking sites."""
    obj=system.object;roots=getattr(obj,'q_slices',None)
    roots={obj.tree.root if system.articulated_object else None:slice(0,7)} if roots is None else roots
    result=[]
    for name,qs in roots.items():
        grasps=[c for c in contacts if c.robot_index is not None and c.capability.has_moment and step in c.attached_steps and (not hasattr(obj,'q_slices') or c.object_frame==name)]
        points=[c.point_object for c in contacts if c.robot_index is not None and c.capability.kind=='P' and step in c.sticking_steps and c.object_frame==name]
        rank=np.linalg.matrix_rank(np.array(points[1:])-points[0],tol=1e-9) if len(points)>=3 else 0
        if grasps or rank>=2:result.append((name,slice(system.object_q.start+qs.start,system.object_q.start+qs.start+7)))
    return result


def solve_cito(system, q_start, q_goal, contacts, *, steps=12, duration=1.,
               dt_bounds=(.01, .2), desired_object_poses=None, fixed_dt=True,
               relaxations=(1e-3, 1e-5, 1e-8), acceptance=2e-5,
               friction_relaxations=None, velocity_relaxations=None,
               feasibility_warm_start=False,
               grasp_velocity_model='instantaneous',
               objective_mode='motion_cost',
               configuration_update='independent_se3',
               configuration_defect_bounds=(.0002,.0005),
               witness_regularization=0.,
               exact_sticking_velocity=False,
               contact_force_coordinates='cartesian',
               friction_speed_model='auxiliary_cone',
               friction_speed_smoothing=1e-6,
               eliminate_endpoint_redundancy=False,
               exact_gripper_velocity=False,
               kinematic_tolerance=1e-7,
               characteristic_length=1., Kp=None, kR=1., Q_tau=None, Q_v=None,
               duration_weight=1.,
               initial=None, collision_margins=None, full_geometry_validator=None,
               scene_constraints=None,
               failure_recording_path=None,
               settings=None):
    if steps < 2 or duration <= 0 or not relaxations or min(relaxations) < 0 or kinematic_tolerance <= 0:
        raise ValueError("invalid transcription or continuation")
    if fixed_dt and not dt_bounds[0] <= duration/steps <= dt_bounds[1]:
        raise ValueError("fixed step falls outside dt_bounds")
    if characteristic_length <= 0 or kR <= 0 or duration_weight <= 0:
        raise ValueError("object length and orientation/duration weights must be positive")
    if grasp_velocity_model not in {'instantaneous','pose_difference'}:raise ValueError('Unknown grasp velocity discretization')
    if objective_mode not in {'motion_cost','feasibility'}:raise ValueError('Unknown trajectory objective mode')
    if configuration_update not in {'independent_se3','independent_midpoint','grasp_consistent','grasp_consistent_midpoint'}:raise ValueError('Unknown configuration update')
    if contact_force_coordinates not in {'cartesian','scaled_tangent'}:raise ValueError('Unknown contact-force coordinates')
    if friction_speed_model not in {'auxiliary_cone','direct_norm','huber_norm'} or friction_speed_smoothing<=0:raise ValueError('Unknown friction speed representation or nonpositive smoothing')
    if configuration_update=='grasp_consistent' and grasp_velocity_model!='instantaneous':raise ValueError('Grasp-consistent update retains instantaneous no-slip')
    if len(configuration_defect_bounds)!=2 or min(configuration_defect_bounds)<=0:raise ValueError('Positive translational/rotational integration defect bounds required')
    if witness_regularization<0:raise ValueError('Nonnegative witness tie-break regularization required')
    friction_relaxations=tuple(0. for _ in relaxations) if friction_relaxations is None else tuple(friction_relaxations)
    velocity_relaxations=tuple(0. for _ in relaxations) if velocity_relaxations is None else tuple(velocity_relaxations)
    if len(friction_relaxations)!=len(relaxations) or len(velocity_relaxations)!=len(relaxations) or min(*friction_relaxations,*velocity_relaxations)<0 or velocity_relaxations[-1]!=0:
        raise ValueError('Continuation schedules must match; final velocity relaxation is zero')
    if friction_speed_model=='huber_norm' and friction_relaxations[-1]<=0:raise ValueError('Huber speed requires a positive work tolerance and an unrelaxed physical residual check')
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
    eps_friction,eps_velocity=opti.parameter(),opti.parameter()
    equality(opti, Q[:, 0]-q_start)
    # SE(3) integration preserves quaternion signs; align goal signs beforehand.
    endpoint_quaternions=[slice(s.start+3,s.start+7) for r,s in zip(system.robots,system.q_slices) if r.floating_base]
    endpoint_quaternions.extend(slice(system.object_q.start+s.start,system.object_q.start+s.stop) for s in getattr(system.object,'quaternion_slices',(slice(3,7),)))
    if eliminate_endpoint_redundancy:
        # Integration already preserves each unit-quaternion norm. Four fixed
        # quaternion coordinates add one redundant endpoint equality per root;
        # use the equivalent three-dimensional orientation chart instead.
        from .lie import quat_rotation
        quaternion_indices={i for s in endpoint_quaternions for i in range(s.start,s.stop)};indices=[i for i in range(system.nq) if i not in quaternion_indices];equality(opti,Q[indices,-1]-q_goal[indices])
        for s in endpoint_quaternions:
            equality(opti,rotation_error(quat_rotation(Q[s,-1]),quat_rotation(ca.DM(q_goal[s]))));opti.subject_to(ca.dot(Q[s,-1],ca.DM(q_goal[s]))>=.5)
    else:equality(opti, Q[:, -1]-q_goal)
    equality(opti, V[:, 0])
    equality(opti, V[:, -1])
    qguess = np.column_stack([q_start+(q_goal-q_start)*i/steps for i in range(steps+1)])
    # Normalize interpolated quaternions for a finite warm start.
    base_slices = [slice(s.start+3, s.start+7) for r, s in zip(system.robots, system.q_slices) if r.floating_base]
    base_slices.extend(slice(system.object_q.start+s.start,system.object_q.start+s.stop) for s in getattr(system.object,'quaternion_slices',(slice(3,7),)))
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
    comp, friction, gap_records, grasp_velocities,instantaneous_grasp_velocities = [], [], [], [], []
    apertures = {}
    for contact in contacts:
        cap = contact.capability
        if not hasattr(cap, 'aperture_required'):
            continue
        a = opti.variable(steps+1)
        apertures[contact.name] = a
        opti.set_initial(a, cap.aperture_required)
        holding_nodes = {node for l in contact.attached_steps for node in [l, l+1]}
        # A holding aperture is already fixed exactly. Adding a>=a_required
        # there duplicates an active equality and destroys strict feasibility.
        for node in range(steps+1):
            if node not in holding_nodes:opti.subject_to(opti.bounded(max(cap.aperture_min,cap.aperture_required),a[node],cap.aperture_max))
        for node in holding_nodes:
            if not 0<=node<=steps:raise ValueError('Grasp phase outside interval')
            equality(opti, a[node]-cap.aperture_required)
        if contact.attached_steps and min(contact.attached_steps)>0:
            equality(opti, a[0]-cap.aperture_max)
        if contact.attached_steps and max(contact.attached_steps)<steps-1:
            equality(opti, a[-1]-cap.aperture_max)
        for l in range(steps):
            opti.subject_to(opti.bounded(-contact.aperture_rate_max*H[l],a[l+1]-a[l],contact.aperture_rate_max*H[l]))
        if initial is not None and contact.name in getattr(initial,'apertures',{}):
            opti.set_initial(a, initial.apertures[contact.name])
    if scene_constraints is not None:
        scene_constraints(opti,Q,V,H,apertures)
    objective = 0
    for l in range(steps+1):
        for robot, qs, vs in zip(system.robots, system.q_slices, system.v_slices):
            qj = Q[qs.start+(7 if robot.floating_base else 0):qs.stop, l]
            vj = V[vs.start+(6 if robot.floating_base else 0):vs.stop, l]
            if eliminate_endpoint_redundancy and l in (0,steps):
                known=(q_start if l==0 else q_goal)[qs.start+(7 if robot.floating_base else 0):qs.stop]
                if np.any(known<robot.lower) or np.any(known>robot.upper):raise ValueError('Prescribed robot endpoint violates joint limits')
            else:
                opti.subject_to(opti.bounded(robot.lower, qj, robot.upper))
                opti.subject_to(opti.bounded(-robot.velocity, vj, robot.velocity))
        if system.articulated_object and system.object.n:
            qo = Q[system.object_q.start+7:system.object_q.stop, l]
            vo = V[system.object_v.start+6:system.object_v.stop, l]
            if eliminate_endpoint_redundancy and l in (0,steps):
                known=(q_start if l==0 else q_goal)[system.object_q.start+7:system.object_q.stop]
                if np.any(known<system.object.lower) or np.any(known>system.object.upper):raise ValueError('Prescribed object endpoint violates joint limits')
            else:
                opti.subject_to(opti.bounded(system.object.lower, qo, system.object.upper))
                opti.subject_to(opti.bounded(-system.object.velocity, vo, system.object.velocity))
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
            terminal=eliminate_endpoint_redundancy and l==steps-1
            terminal_gap=None
            if terminal and cap.kind=='P':
                terminal_gap=float(ca.evalf(contact.geometry(system,ca.DM(q_goal))[0]))
                if terminal_gap < -1e-10:raise ValueError('Prescribed terminal point penetrates its contact plane; polish the keyframe')
                if contact.align_normal and contact.robot_index is not None and (l in contact.maintained_steps or l in contact.sticking_steps):
                    endT=system.robots[contact.robot_index].fk(ca.DM(q_goal[system.q_slices[contact.robot_index]]),contact.frame);endR=system.object_pose(ca.DM(q_goal),contact.object_frame)[:3,:3]@contact.basis()
                    if np.max(abs(np.array(ca.DM(endR[:,:2].T@endT[:3,2]))))>1e-10 or float(ca.evalf(ca.dot(endR[:,2],endT[:3,2])))>-.5:raise ValueError('Terminal point tool lacks its prescribed surface-normal alignment')
            active = (contact.nodes is None or l in contact.nodes) and l not in contact.released_steps
            if cap.kind == "P":
                if not terminal and l not in contact.maintained_steps and l not in contact.sticking_steps:opti.subject_to(gap >= 0)
                gap_records.append(gap)
                if l in contact.maintained_steps or l in contact.sticking_steps:
                    if terminal:
                        if abs(terminal_gap)>1e-10:raise ValueError('Terminal maintained contact lacks exact closure; polish the keyframe')
                    else:equality(opti, gap)
                    if contact.align_normal and contact.robot_index is not None and not terminal:
                        Tee = system.robots[contact.robot_index].fk(Q[system.q_slices[contact.robot_index],l+1],contact.frame)
                        normal = system.object_pose(Q[:,l+1],contact.object_frame)[:3,:3]@ca.DM(contact.normal)
                        Rc=system.object_pose(Q[:,l+1],contact.object_frame)[:3,:3]@contact.basis()
                        equality(opti,Rc[:,:2].T@Tee[:3,2]);opti.subject_to(ca.dot(Tee[:3,2],normal)<=-.5)
                if contact.tangent_bounds is not None and active and contact.robot_index is not None:
                    Tee = system.robots[contact.robot_index].fk(Q[system.q_slices[contact.robot_index],l+1],contact.frame)
                    point = system.point_world(Q[:,l+1],contact.point_object,contact.object_frame)
                    R = system.object_pose(Q[:,l+1],contact.object_frame)[:3,:3]@contact.basis()
                    tangent = (R.T@(Tee[:3,3]-point))[:2]
                    opti.subject_to(opti.bounded(contact.tangent_bounds[0],tangent,contact.tangent_bounds[1]))
                if l in contact.sticking_steps:
                    # Exact equalities are useful for the coupled arm solve,
                    # but can overcount redundant rows in unactuated support
                    # problems. Keep the established bounded form by default.
                    if exact_sticking_velocity:
                        if l<steps-1:equality(opti,vel[:2])
                    else:opti.subject_to(opti.bounded(-kinematic_tolerance,vel[:2],kinematic_tolerance))
                    grasp_velocities.append(vel[:2])
                normal=ratio=None
                if active and contact_force_coordinates=='scaled_tangent':
                    normal=opti.variable();ratio=opti.variable(2);opti.subject_to(ca.sumsqr(ratio)<=1);P=ca.vertcat(cap.mu*normal*ratio,normal)
                else:P = opti.variable(3) if active else ca.DM.zeros(3)
                if active:
                    opti.subject_to(opti.bounded(0, P[2], h*cap.normal_max))
                    if contact_force_coordinates=='cartesian':opti.subject_to(ca.sumsqr(P[:2]) <= (cap.mu*P[2])**2)
                    # An explicitly maintained zero gap already satisfies
                    # complementarity. Avoid a redundant near-active product
                    # row parallel to its exact gap equality.
                    if not (terminal and abs(terminal_gap)<=1e-10) and (not exact_sticking_velocity or (l not in contact.maintained_steps and l not in contact.sticking_steps)):
                        opti.subject_to(P[2]*gap <= eps)
                    comp.append(P[2]*gap)
                    if getattr(contact,'patch_half_size',None) is not None:
                        tangent=contact.support_coordinates(system,Q[:,l+1])
                        for axis in range(2):
                            for sign in [-1,1]:
                                outside=sign*tangent[axis]-contact.patch_half_size[axis];opti.subject_to(P[2]*outside<=eps);comp.append(ca.fmax(P[2]*outside,0))
                    if contact.plane_bounds is not None:
                        point=system.point_world(Q[:,l+1],contact.point_object,contact.object_frame)
                        for axis in range(2):
                            for outside in [contact.plane_bounds[0][axis]-point[axis],point[axis]-contact.plane_bounds[1][axis]]:
                                opti.subject_to(P[2]*outside<=eps);comp.append(ca.fmax(P[2]*outside,0))
                    if terminal or l in contact.sticking_steps and exact_sticking_velocity:
                        # vt=0 makes the entire Coulomb disk maximally
                        # dissipative. Eliminate the redundant speed cone at
                        # its singular apex rather than add a zero-gradient row.
                        friction.append(ca.dot(P[:2],vel[:2])+cap.mu*P[2]*ca.sqrt(ca.sumsqr(vel[:2])+1e-30))
                    else:
                        if friction_speed_model=='direct_norm':speed=ca.sqrt(ca.sumsqr(vel[:2])+1e-30)
                        elif friction_speed_model=='huber_norm':speed=ca.sqrt(ca.sumsqr(vel[:2])+friction_speed_smoothing**2)-friction_speed_smoothing
                        else:
                            speed = opti.variable()
                            opti.subject_to(speed >= 0)
                            opti.subject_to(speed**2+eps_velocity**2 >= ca.sumsqr(vel[:2]))
                        dissipation = ca.dot(P[:2], vel[:2])+cap.mu*P[2]*speed
                        if not any(friction_relaxations):equality(opti,dissipation)
                        else:opti.subject_to(opti.bounded(-eps_friction,dissipation,eps_friction))
                        friction.append(ca.dot(P[:2],vel[:2])+cap.mu*P[2]*ca.sqrt(ca.sumsqr(vel[:2])+1e-30) if friction_speed_model=='huber_norm' else dissipation)
                    if initial is not None and contact.name in initial.impulses:
                        seed=initial.impulses[contact.name][l]
                        if normal is not None:opti.set_initial(normal,seed[2]);opti.set_initial(ratio,seed[:2]/(cap.mu*seed[2]) if seed[2]>1e-12 else np.zeros(2))
                        else:opti.set_initial(P,seed)
                generalized_impulse += Jnow.T@P
            else:
                attached = l in contact.attached_steps
                if attached:
                    warm_reaction = initial.impulses[contact.name][l] if initial is not None and contact.name in initial.impulses else None
                    a = apertures[contact.name][l+1] if contact.name in apertures else None
                    if hasattr(cap,'reaction_with_witness'):
                        P,witness=cap.reaction_with_witness(opti,scale=h,initial=warm_reaction,aperture=a,force_coordinates=contact_force_coordinates)
                        objective+=witness_regularization*ca.sumsqr(witness)/h
                    else:P = cap.reaction(opti, scale=h, initial=warm_reaction,aperture=a)
                    if terminal:
                        end=np.array(ca.DM(contact.geometry(system,ca.DM(q_goal))[2])).ravel()
                        if np.max(abs(end))>1e-10:raise ValueError('Terminal grasp lacks exact closure; polish the keyframe')
                    else:equality(opti, closure)
                    if cap.has_moment:
                        Tee = system.robots[contact.robot_index].fk(
                            Q[system.q_slices[contact.robot_index], l+1], contact.frame)
                        Rwc = system.object_pose(Q[:, l+1], contact.object_frame)[:3, :3]@contact.basis()
                        desired_rotation=Rwc@ca.diag(ca.DM([1, -1, -1]))
                        if not terminal:
                            equality(opti, rotation_error(Tee[:3, :3], desired_rotation))
                            opti.subject_to(rotation_cost(Tee[:3, :3], desired_rotation) <= 1.)
                        else:
                            endT=system.robots[contact.robot_index].fk(ca.DM(q_goal[system.q_slices[contact.robot_index]]),contact.frame);endR=system.object_pose(ca.DM(q_goal),contact.object_frame)[:3,:3]@contact.basis()@ca.diag(ca.DM([1.,-1.,-1.]))
                            if np.max(abs(np.array(ca.DM(rotation_error(endT[:3,:3],endR)))))>1e-10 or float(ca.evalf(rotation_cost(endT[:3,:3],endR)))>1e-12:raise ValueError('Terminal grasp lacks exact orientation; polish the keyframe')
                        constrained_velocity = vel
                    else:
                        constrained_velocity = vel[3:]
                    # Exact closure plus its exact time derivative can produce
                    # redundant equalities after transcription. Keep closure
                    # exact and enforce velocity within an explicit tolerance.
                    instantaneous_grasp_velocities.append(constrained_velocity)
                    if grasp_velocity_model=='instantaneous':
                        if not terminal:
                            if exact_gripper_velocity:equality(opti,constrained_velocity)
                            else:opti.subject_to(opti.bounded(-kinematic_tolerance-eps_velocity,constrained_velocity,kinematic_tolerance+eps_velocity))
                        grasp_velocities.append(constrained_velocity)
                    else:
                        # Explicit comparison variant, NOT silently substituted
                        # for instantaneous J(q_next)*v_next=0. Relative pose
                        # closure at both ends gives zero discrete grasp motion.
                        _,_,previous_closure=contact.geometry(system,Q[:,l])
                        Ro0=system.object_pose(Q[:,l],contact.object_frame)[:3,:3];Ro1=system.object_pose(Q[:,l+1],contact.object_frame)[:3,:3]
                        discrete=(Ro1.T@closure-Ro0.T@previous_closure)/h
                        if cap.has_moment:
                            T0=system.robots[contact.robot_index].fk(Q[system.q_slices[contact.robot_index],l],contact.frame)
                            discrete=ca.vertcat(rotation_error(Ro1.T@Tee[:3,:3],Ro0.T@T0[:3,:3])/h,discrete)
                        grasp_velocities.append(discrete)
                    if l == min(contact.attached_steps):
                        gap_before, _, closure_before = contact.geometry(system,Q[:,l])
                        if l == 0:
                            qs = ca.MX.sym('q_check',system.nq)
                            check = ca.Function('initial_grasp_check',[qs],[contact.geometry(system,qs)[2]])
                            if np.max(abs(np.asarray(check(q_start))))>acceptance:
                                raise ValueError('Initial attached grasp lacks positional closure')
                        else:
                            opti.subject_to(opti.bounded(-kinematic_tolerance,closure_before,kinematic_tolerance))
                        # No attachment impulse: acquisition starts at zero relative velocity.
                        before = Jnow@V[:, l]
                        before = before if cap.has_moment else before[3:]
                        if not (eliminate_endpoint_redundancy and l==0):opti.subject_to(opti.bounded(-kinematic_tolerance, before, kinematic_tolerance))
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
        integration_velocity=.5*(V[:,l]+V[:,l+1]) if configuration_update in {'independent_midpoint','grasp_consistent_midpoint'} else V[:,l+1]
        integrated=system.integrate(Q[:, l], h*integration_velocity)
        constrained_roots=constrained_object_roots(system,contacts,l) if configuration_update=='grasp_consistent_midpoint' else []
        root_name=system.object.tree.root if system.articulated_object else None
        root_grasp=any(c.capability.has_moment and l in c.attached_steps and c.object_frame in (None,root_name,'object' if not system.articulated_object else root_name) for c in contacts)
        if constrained_roots:
            # Constrained-configuration midpoint variant: preserve the contact
            # manifold and full instantaneous momentum/no-slip equations, with
            # an explicit bounded root integration defect. Every independent
            # joint and every released root still integrates exactly.
            skipped={i for _,sl in constrained_roots for i in range(sl.start,sl.stop)};indices=[i for i in range(system.nq) if i not in skipped];equality(opti,Q[indices,l+1]-integrated[indices])
            from .lie import quat_rotation
            for name,sl in constrained_roots:
                root=sl.start;equality(opti,ca.sumsqr(Q[root+3:root+7,l+1])-1);opti.subject_to(opti.bounded(-configuration_defect_bounds[0],Q[root:root+3,l+1]-integrated[root:root+3],configuration_defect_bounds[0]));Ractual=quat_rotation(Q[root+3:root+7,l+1]);Rpred=quat_rotation(integrated[root+3:root+7]);opti.subject_to(opti.bounded(-configuration_defect_bounds[1],rotation_error(Ractual,Rpred),configuration_defect_bounds[1]));opti.subject_to(rotation_cost(Ractual,Rpred)<=1.)
        elif configuration_update=='grasp_consistent' and root_grasp:
            # Explicit constrained-configuration variant: the root pose is
            # determined by full-rank grasp closure. Integrate independent
            # robot and internal-object coordinates; retain full momentum and
            # bounded reaction/instantaneous no-slip equations. No simulator
            # weld, prescribed object motion or additional force is introduced.
            equality(opti,Q[:system.object_q.start,l+1]-integrated[:system.object_q.start])
            equality(opti,Q[system.object_q.start+7:system.object_q.stop,l+1]-integrated[system.object_q.start+7:system.object_q.stop])
            equality(opti,ca.sumsqr(Q[system.object_q.start+3:system.object_q.start+7,l+1])-1)
            from .lie import quat_rotation
            root=system.object_q.start
            opti.subject_to(opti.bounded(-configuration_defect_bounds[0],Q[root:root+3,l+1]-integrated[root:root+3],configuration_defect_bounds[0]))
            Ractual=quat_rotation(Q[root+3:root+7,l+1]);Rpred=quat_rotation(integrated[root+3:root+7])
            opti.subject_to(opti.bounded(-configuration_defect_bounds[1],rotation_error(Ractual,Rpred),configuration_defect_bounds[1]));opti.subject_to(rotation_cost(Ractual,Rpred)<=1.)
        else:equality(opti, Q[:, l+1]-integrated)
        objective += h*(ca.mtimes([U[:, l].T, Qt, U[:, l]])+
                        ca.mtimes([V[:, l].T, Qv, V[:, l]])+duration_weight)
        if desired_object_poses is not None:
            references=desired_object_poses if isinstance(desired_object_poses,dict) else {None:desired_object_poses}
            for object_frame,poses in references.items():
                T, Td = system.object_pose(Q[:, l],object_frame), ca.DM(poses[l])
                ep = (T[:3, 3]-Td[:3, 3])/characteristic_length
                objective += h*(.5*ca.mtimes([ep.T, Kp, ep])+kR*rotation_cost(T[:3, :3], Td[:3, :3]))
    opti.minimize(objective if objective_mode=='motion_cost' else 0)
    residual = np.inf
    if feasibility_warm_start:
        opti.set_value(eps,relaxations[0]);opti.set_value(eps_friction,friction_relaxations[0]);opti.set_value(eps_velocity,velocity_relaxations[0]);opti.minimize(0)
        warm_sol,_=solve(opti,settings);opti.set_initial(opti.x,warm_sol.value(opti.x));opti.minimize(objective if objective_mode=='motion_cost' else 0)
    for relaxation,friction_relaxation,velocity_relaxation in zip(relaxations,friction_relaxations,velocity_relaxations):
        opti.set_value(eps, relaxation)
        opti.set_value(eps_friction,friction_relaxation)
        opti.set_value(eps_velocity,velocity_relaxation)
        try:sol, residual = solve(opti, settings)
        except RuntimeError:
            if failure_recording_path is not None:
                from pathlib import Path
                import json
                path=Path(failure_recording_path);path.parent.mkdir(parents=True,exist_ok=True)
                try:
                    arrays=dict(q=np.array(opti.debug.value(Q)).reshape(system.nq,steps+1).T,v=np.array(opti.debug.value(V)).reshape(system.nv,steps+1).T,tau=np.array(opti.debug.value(U)).reshape(system.nu,steps).T,dt=np.array(opti.debug.value(H)).reshape(steps))
                    arrays.update({'impulse_'+n:np.array([np.asarray(opti.debug.value(p)).reshape(-1) for p in ps]) for n,ps in impulses.items()});arrays.update({'aperture_'+n:np.asarray(opti.debug.value(a)).reshape(-1) for n,a in apertures.items()});arrays['metadata_json']=json.dumps(dict(accepted=False,purpose='warm start only; requires a successful solve and every independent gate',configuration_update=configuration_update,contact_force_coordinates=contact_force_coordinates,continuation=dict(gap=relaxation,friction=friction_relaxation,velocity=velocity_relaxation)));np.savez_compressed(path,**arrays)
                except RuntimeError:pass
            raise
        opti.set_initial(opti.x, sol.value(opti.x))
    comp_res = max([abs(float(sol.value(x))) for x in comp], default=0.)
    fric_res = max([abs(float(sol.value(x))) for x in friction], default=0.)
    min_gap = min([float(sol.value(x)) for x in gap_records], default=0.)
    gripper_res = max([float(np.max(np.abs(sol.value(x)))) for x in grasp_velocities], default=0.)
    if max(comp_res, fric_res, -min_gap, gripper_res) > acceptance:
        raise RuntimeError(f"unrelaxed contact residual exceeds acceptance tolerance: comp={comp_res:.3g}, friction={fric_res:.3g}, gap={min_gap:.3g}, velocity={gripper_res:.3g}")
    result = TrajectoryResult(
        np.array(sol.value(Q)).reshape(system.nq, steps+1).T,
        np.array(sol.value(V)).reshape(system.nv, steps+1).T,
        np.asarray(sol.value(U)).reshape(system.nu, steps).T if system.nu else np.zeros((steps, 0)),
        np.asarray(sol.value(H)).reshape(steps),
        {name: np.array([np.asarray(sol.value(p)).reshape(-1) for p in ps])
         for name, ps in impulses.items()}, float(sol.value(objective)), residual,
        comp_res, fric_res, min_gap, False, tuple(relaxations), gripper_res)
    result.apertures={name:np.asarray(sol.value(a)).reshape(-1) for name,a in apertures.items()}
    result.continuation=dict(normal_gap_Ns_m=tuple(relaxations),friction_work_Nm=friction_relaxations,velocity_m_s_or_rad_s=velocity_relaxations)
    result.grasp_velocity_model=grasp_velocity_model
    result.objective_mode=objective_mode
    result.configuration_update=configuration_update
    result.friction_speed_model=friction_speed_model
    result.endpoint_redundancy_eliminated=eliminate_endpoint_redundancy
    result.friction_speed_smoothing=friction_speed_smoothing if friction_speed_model=='huber_norm' else 0.
    result.configuration_defect_bounds=tuple(configuration_defect_bounds) if configuration_update in {'grasp_consistent','grasp_consistent_midpoint'} else ()
    result.exact_sticking_velocity=bool(exact_sticking_velocity)
    result.contact_force_coordinates=contact_force_coordinates
    result.instantaneous_grasp_velocity_residual=max([float(np.max(np.abs(sol.value(x)))) for x in instantaneous_grasp_velocities],default=0.)
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
