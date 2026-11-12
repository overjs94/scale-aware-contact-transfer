# Robotiq 2F-85 as a finite-force panel gripper

The previous 18 N per Panda finger was a declared simulation capability, not a
manufacturer specification. The selectable `robotiq_2f85` profile replaces the
hand geometry, inertia, joint tree, transmission, pad friction and grasp
capability together. Increasing an old scalar bound alone does not implement a
stronger physical gripper.

## Source and limits

The [manufacturer product sheet](https://robotiq.com/hubfs/Product-sheets/Adaptive%20Grippers/Product-sheet-Adaptive-Grippers-EN.pdf)
lists 85 mm stroke and 20–235 N grip force. The
[instruction manual](https://blog.robotiq.com/hubfs/support-files/2F-85_2F-140_UR_PDF_2026-03-17.pdf)
distinguishes jaw squeeze from externally applied forces and moments. Its
payload relation uses `W = 2 F mu / S`, with `F` at each jaw. The grip-force
range is not a permissible net Cartesian force or moment in every direction.

The physical simulation uses the BSD-2-Clause
[MuJoCo Menagerie native model](https://github.com/google-deepmind/mujoco_menagerie/tree/370c43965323c0944b433b202aac5058b5f907a9/robotiq_2f85),
including eight joint coordinates, two four-bar connect constraints, coupled
drivers, preloaded spring links, original link inertias and eight STL assets.
The original finite motor bound is ±5 Nm. Six mechanism joints are passive;
they receive no independent artificial motor. The two driver halves each
receive at most 2.5 Nm in the mounted scene. This shares the native motor's
position command through the coupled-driver equality, rather than fitting an
unbounded linear jaw actuator.

The standalone test uses the original single tendon actuator. The mounted
scene represents its servo by two scalar PD halves with a common command and
the original driver coupling. Their forces agree with the tendon model when
the drivers coincide; the soft coupling can introduce small differences. An
exact single-tendon mounted actuator adapter is not claimed.

The declared planning model uses **60 N per jaw**, pad friction `mu = 0.6`,
and an additional grasp-frame **50 N per net force component** bound. The
native component test commands **40 N per jaw**, above the previous 18 N
bound. These are conservative simulation choices, not identified hardware
ratings or a calibration of the entire 60 N grasp-wrench set. The force model
retains eight patch force sites, individual Coulomb cones, coupled per-jaw
closing-force bounds and the aperture. Its resultants shift to the paper's
interface origin before force balance and torque constraints are applied.

## Geometry and mechanics

The prepared URDF is generated from the pinned compiled native model.
Joint-anchor changes shift the collision, visual and inertial frames together.
Independent Pinocchio checks compare body FK, the unconstrained mass matrix
without separately declared armature, and gravity at four configurations.
Closed-loop constraints remain explicit native scene data; a tree URDF alone
does not implement that mechanism's constrained dynamics.

On Panda, the old hand is removed and the complete Robotiq mechanism is mounted
at the flange. The nominal parallel grasp is solved at the panel's 40 mm
thickness. A collision-distance probe chooses an edge insertion depth that
clears the metal linkage, then restricts the force patches to material inside
the panel. Passive link motion and soft joint-limit deviations are recorded
from native simulation for rendering. They are not synthesized from aperture.

Between optimized knots, the controller interpolates arm joint references with
cubic Hermite splines. Its held-object reference is recovered from the holding
tool's FK and fixed grasp transform. The sampled reference collision check uses
that same `holding_grasp` interpolation and verifies closure at every knot.
An independently linear/SE(3)-interpolated object can separate from the curved
tool path and produce false linkage interference. The original independent
interpolation rejection remains in the diagnostic record. This reference
choice does not assign the simulated object's pose; native contact still
determines its motion. It is not a continuous collision certificate.

## Low-level control

The native gripper retains its finite position servo. An outer jaw-force
admittance updates its common driver-position command. Let `a` denote the
parallel-mode aperture, `theta` the tendon coordinate, `N_L,N_R` the measured
native compressive normal forces, and `g = max(0, -da/dtheta)` the local
parallel-mode work gain. The virtual force port is

~~~math
u_k = g_k (N_d - max(N_L,N_R)),
M_v (v_{k+1}-v_k)/h + D_v (v_{k+1}+v_k)/2 = u_k,
theta^d_{k+1} = theta^d_k + h (v_{k+1}+v_k)/2.
~~~

This virtual constant-mass admittance uses passive midpoint integration.
Command projection, native servo/contact dynamics, passive curling and moving
frames are outside that elementary virtual-port energy identity. It is not a
claim of full multibody PMI or hardware passivity. The native engine uses
`implicitfast`. Summing `mj_contactForce` compressive normals on each pad
accounts for adaptive pad rotation; projecting only onto a fixed opening axis
would mismeasure squeeze.

The first lift comparison keeps the robot arms in pose/joint impedance plus
planned force feedforward, while the gripper independently closes its measured
jaw-force loop. No object actuator, weld, state overwrite or externally
prescribed supporting wrench drives the free panel. The mounted gripper's
three internal equalities are not object attachments.

## Reproduction and publication

~~~bash
python tools/fetch_robotiq_assets.py
OPENBLAS_NUM_THREADS=1 python tools/run_robotiq_clamp.py --viewer web
OPENBLAS_NUM_THREADS=1 python tools/run_paper_panel.py \
  --gripper robotiq_2f85 --control-mode impedance_feedforward \
  --steps 8 --solver-tolerance 1e-6 --iterations 1000 \
  --output outputs/robotiq_panel_native --viewer web
npm --prefix web run build
~~~

The gripper component belongs to **요소·강건성 → 그리퍼 · 구동·파지력**.
The multi-arm first lift is a separately labeled diagnostic, with its actual
planning and execution checks retained. Neither is a completed reorientation,
jig-placement and release sequence or a III-D assignment-search result. The
configuration update is the explicit `independent_midpoint` variant, not the
literal manuscript's poststep-Euler update.

The [saved first-lift evidence](robotiq-panel-execution-results.json) records
9.311 s of native execution, maximum panel error 0.2161 mm / 0.001918 rad,
maximum jaw reactions 38.914 / 42.474 N, and contact retention of
100% / 100% / 99.986%. All 147 sampled raw contact resultants at each of the
three interfaces pass their declared capabilities. The arm control is pose and
joint impedance plus planned force feedforward; measured jaw-force feedback
commands the native gripper servo. The
[component evidence](robotiq-component-results.json) separately records the
approximately 40 N free-block clamp and independent native/Pinocchio mechanics.

The default 1e-8 IPOPT run hit its iteration limit and remains rejected in the
diagnostic outputs. With 1e-6 solver tolerance, the final trajectory passes the
unchanged independent mechanics/contact gate; its native momentum residual is
9.71e-10. The 54 independent-object interpolation collision samples remain
reported, while the actual holding-reference interpolation passes. These
different checks must not be collapsed into a solver-success claim.

The public viewer serves static recordings and pinned upstream STL URLs; a
Python server is unnecessary for playback. New optimization and physics runs
use Python. The current CRISP adapter rejects this native closed-loop gripper
because it does not yet preserve its internal constraints; it must not silently
run an unconstrained substitute and claim a matched comparison.
