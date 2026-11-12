# Grasp-constraint diagnosis: numerical conditioning versus model consistency

This concerns the uploaded manuscript's rigid-body formulation and its current
implementation, not a completed validation of the full manipulation task.

## What is independently checked

The rigid panel source stage now evaluates gravity at the source keyframe's
orientation, maximizes rho without the nominal demo's extra geometry penalty,
and transfers geometry only. The target stage jointly optimizes surface
coordinates, Panda postures, per-vertex reactions/torques and rho. The gripper
uses explicit opposing finger friction patches, per-finger closing-force bounds
and the aperture derived from the panel and native finger URDF geometry.
The declared closing-force upper bound is 18 N per finger. This is a simulation
interface requirement; matching measured squeeze and replay force envelopes
remain execution checks, not results established by static optimization.

Five carry/seat keyframes passed the NLP and frozen-geometry Clarabel checks.
Native Pinocchio RNEA/Jacobians and NumPy wrench shifts independently reproduce
their force equilibrium and torque bounds. The first two target margins are
about 6.50083; the later horizontal keyframes are about 6.62603, for the declared
target disturbance box. These numbers are not execution or full-task margins.
The 12-degree source pose gives rho about 0.53207; the old horizontal source
fixture's 1.11564 must not be substituted for that pose.

## Solver-free finite-step probe

Run:

```sh
OPENBLAS_NUM_THREADS=1 python examples/grasp_discretization_diagnosis.py
```

Only native Pinocchio FK, Jacobians and SE(3) logarithms are used. The source
Panda posture comes from the scenario, and all robot mechanics come from URDF.
Let a grasped object's frame follow the tool exactly on the continuous path
q(t)=q0+t*u. Both endpoint grasp poses are therefore exact. Joint Euler
integration fixes the joint step velocity to u; object SE(3) integration fixes
its step body velocity to the pose logarithm. Their instantaneous post-step
relative-velocity defect is

```math
r_v(h)=J_b(q_0+hu)u-
       \frac{1}{h}\log\!\left[T(q_0)^{-1}T(q_0+hu)\right]^\vee.
```

| h (s) | angular defect (rad/s) | linear defect (m/s) |
|---:|---:|---:|
| 0.100 | 0.002302 | 0.0008351 |
| 0.050 | 0.001152 | 0.0004175 |
| 0.025 | 0.0005763 | 0.0002087 |

The defect is first order in h. A single-joint constant-twist control gives a
maximum defect of about 1.46e-16. Thus this discrepancy is not caused by IPOPT,
incorrect reaction signs, or an inaccurate robot dimension table.

This is a counterexample for a **prescribed continuous trajectory**, not a
proof that the free-trajectory NLP has no feasible solution. A different joint
path, additional nodes or a different initialization may satisfy the discrete
constraints. Nor does it establish that the uploaded manuscript necessarily
intends this exact interpretation of the held-grasp velocity constraint.

## Current optimization evidence and limits

Original instantaneous held-grasp velocity constraints remain the default.
Carry interval trials with motion cost, zero objective, and finer time grids
are kept distinct. Some relaxed continuation stages converge, but none of the
tested complete original carry intervals has yet passed final acceptance.
A successful relaxed stage is not a successful final trajectory.

Two numerical conditioning changes preserve the intended exact constraints:
normal alignment is expressed using two tangent components plus a sign check,
and a prescribed sticking phase eliminates the redundant zero-speed cone apex.
Friction/gap/velocity continuation schedules and the actual unrelaxed residuals
are recorded separately. The panel diagnostic's held-velocity bound is 1e-5;
the final residual acceptance remains 2e-5 and final velocity relaxation is zero.

An explicitly named `pose_difference` comparison interprets held-grasp velocity
as the change in relative pose over a step. It preserves nodal grasp closure
and reports the instantaneous velocity defect separately. It is **not silently
substituted for the original instantaneous-velocity formulation**, does not
establish a full manuscript pass, and has not yet passed the complete task.
No object weld, prescribed simulated object pose, or artificial support force
is introduced to obtain a pass.

The present evidence supports investigating **discretization/constraint
consistency as well as numerical conditioning**. It does not justify declaring
either a purely numerical problem or a proven error in the underlying physical
laws. Full source-to-realization-to-selection-to-CITO-to-MuJoCo task validation
remains incomplete.
