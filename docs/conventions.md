# Mechanical and numerical conventions

## Frames, origins, ordering

- T_WB=[R_WB,p_WB] maps body coordinates to world coordinates.
- Every six-vector uses angular/moment first and linear/force second.
- Default frame Jacobian: J_mixed v = [omega_B; pdot_W] at the named frame origin.
- Power-dual wrench: [moment_B_at_that_origin; force_W].
- world_aligned gives [omega_W; pdot_W] at that origin.
- body gives [omega_B; R_WB.T pdot_W].
- A spatial velocity about the world origin is not pdot_W.
- Changing moment origins uses the full wrench map, not just block rotations.

Pinocchio's native linear/angular order is converted inside Robot.
LOCAL_WORLD_ALIGNED is used for physical point velocity, rather than WORLD.

## Generalized coordinates

Fixed-base: independent joint coordinates in robot.joint_names order.
Continuous joints use scalar angles and are converted to Pinocchio's cos/sin
representation internally.

~~~text
Floating q = [base_position_world(3), base_quaternion_xyzw(4), joints]
Floating v = [base_omega_body(3), base_linear_velocity_body(3), joint_velocities]
~~~

Thus nq differs from nv. Use Robot.integrate(q, delta) instead of q+delta.
SymbolicRobot.configuration_derivative maps tangent velocity to qdot.
The floating base is unactuated in MechanicalSystem.

## Dynamics and reactions

~~~text
M(q) a + C(q,v) v + G(q) = tau + J_mixed.T lambda_mixed
Static keyframe: tau = G - J_mixed.T lambda_mixed
~~~

lambda is object-on-robot. Object-frame wrenches use [moment;force] about
the object origin, which need not equal the COM. Gravity includes the COM moment.
The reaction map satisfies sum W_i lambda_i = w_external at equilibrium.

Source positions are geometrically scaled. Source forces are not scaled:
target mass, COM, gravity, friction and actuation bounds are supplied again
and target reactions are recomputed.

C is nonunique. Numerical Robot.coriolis uses Pinocchio's factorization.
Symbolic fixed-base C is the Christoffel factorization obtained by polarizing
the quadratic velocity bias. Its Mdot=C+C.T identity is tested independently.
For a floating base, symbolic C=.5*d(bias-gravity)/dv satisfies
C*v=bias-gravity. No coordinate Christoffel/skew-symmetry claim is made for
that quasi-velocity factorization. A controller requiring a particular C must
select and validate it. One exact Wensing factorization is not claimed.

## Contact optimization

TaskWrench.E and D are fixed task data, not design variables.
Each box vertex has a separate reaction distribution.
The box contains the corresponding 2-norm ball; its dimensionless margin is not
the exact ellipsoidal margin. Point contact has zero moment. A bilateral force
box does not imply moment capacity. Gripper H/bounds need a calibrated inner model.
Full spatial-wrench robustness needs a rank-six combined map; two point
contacts have rank at most five.

In CITO, P is impulse and P/dt is the average reaction over the step.
Robot--object relative velocity is agent minus object, with positive normal
velocity toward separation. Its transpose gives equal-and-opposite impulses.
Plane impulses can act on the object or a named robot frame, for example a
floating robot foot. This representation does not supply a standing controller.
Do not include the same reaction in both external_load and contacts.
The current normal complementarity relaxation has units N*s*m; select its
tolerances consistently with task scale.

Attached gripper positions use exact transcription equalities. Their
relative-velocity constraints use an explicit kinematic_tolerance (default
1e-7) to avoid duplicate equality rows from closure and integration; the
achieved velocity residual is reported and checked.

Solver status alone is insufficient: primal and unrelaxed contact residuals
are checked. Full-geometry validation is reported separately.
