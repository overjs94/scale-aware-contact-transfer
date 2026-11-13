# Implementation scope

Current consolidated report (2026-11-12):
[code structure, solver/control changes and verified boundaries](system-architecture-and-progress.md).
The Robotiq-equipped three-Panda first lift now passes source refinement,
robust endpoint realization, the explicitly named midpoint CITO variant,
independent plan checks and native MuJoCo execution. This does not complete
III-D candidate search or the full reorient/jig/release sequence.

The free articulated-object tasks now execute link-aware geometric contraction of
125 supplied synthetic contact samples into three anchors, geometry-only scale
transfer, and three-arm G/P/G IK plus coupled 7D object-force/robot-torque
realization. The complete object has a free root and passive internal joint.
See [the main articulated-object validation](free-articulated-realization.md).
The fixed-fixture single-arm tasks below are component tests.

Three passive one-joint articulated extensions (hinged door, folding panel,
drawer) optimize generalized contact force/arm torque along supplied joint
paths and execute using native MuJoCo contact, with no object actuator or pose
replay. Contact-disabled controls fail all three tasks. They do not implement
learned articulated contraction or general multi-joint articulated CITO.
The mobile scenario also has deterministic same-plan/gain external-force tests;
the 25 N trial recovers position but remains a planned-capability failure.
See [measurements, equations and limitations](contact-validation-and-datasets.md).

Executable v0.1 research implementation; experimental reproduction of every
paper component is not complete.

| Component | Implemented | Current boundary |
|---|---|---|
| URDF numerical mechanics | Pinocchio M/C/G, FK/J, RNEA, reduced forward solve | Tree mechanisms; closed-loop constraints external |
| URDF symbolic mechanics | Lie recursive Newton--Euler, M/C/G, FK/J/Jdot, export | Graph construction cost grows with model size |
| Floating base | Numerical/symbolic state and dynamics; unactuated system assembly | Whole-body support controller not supplied |
| III-B | Local charts, exact quadratic friction cone, box vertices, force bounds | Smooth local charts required |
| III-C | Arm posture, contact geometry, robustness and vertex torque bounds; panel adapter has explicit opposing finger patches/aperture | Simulation grasp capacity is declared, not hardware calibrated |
| III-D | Configured type/assignment enumeration and fixed-endpoint transition callbacks | Complete Panda candidate comparison remains pending; shared-endpoint joint NLP is an optional extension |
| IV restricted CITO | Dynamics, Lie integration, complementarity, maximum dissipation, gripper phases | Candidates and attachment phases supplied |
| Contact search | Candidate/node update loop with user geometry oracle | Complete STOCS merit/oracle algorithm not reproduced |
| Collision | Differentiable margins and trajectory validator callbacks | Bundled dynamics-only models do not provide full mesh clearances |
| Control | Hybrid motion/force, pose/force feedforward comparison, joint nullspace impedance, virtual PMI position admittance | Simulation level; full nonlinear multibody/contact PMI and hardware servo validation remain pending |
| MuJoCo | URDF inertia export, state mapping, tracking and native multi-arm/tool/free-object contact scenes | The basic marker-only tracking exporter remains distinct from the full collision scene builders |
| Mesh visualization | Eight full upstream URDFs, STL/DAE loading, visual/collision selector and recorded joint/base poses | Upstream mesh network access and WebGL required; display does not enable contact dynamics |
| Fixed-geometry SOCP | Clarabel box margin, exact friction cones, fixed-posture torque bounds | Geometry/closure/collision remain NLP or scene checks |
| Contact scene replay | MJCF, scalar-joint torque/position commands, passive observed joints, free objects, contact traces and internal gripper constraints | Model/scene-specific geometry; simulation force bounds are not hardware calibration |
| Engine comparison | Portable trace, optional CRISP CANAL/SubADMM SDK client, rotated-contact frame/impulse checks and fixed-Panda/free-panel runner | Complete task/precision insertion/jam comparison remains pending; SDK installed separately |
| Three-arm contact experiment | Assumed network groups, 3 keyframes, 2 CITO solves, URDF sphere/fingers/cup, actual dynamic replay and vent/release | Authored Cartesian arms, fixed contact assignments; force-only gripper and lumped suction model |
| Solver comparison | Fresh stationary CITO trials and residual-qualified timing tool | SNOPT unavailable in validated runtime; no assembly benchmark |
| Robotiq 2F-85 | Native linkage/mass/mesh, finite clamp and measured-jaw feedback; mounted G/P/P first lift | Mounted PD halves approximate a tendon under soft coupling; CRISP closed-loop import and hardware calibration pending |
| Hardware | Position/velocity/effort separation | No physical communication or validation |

Restricted CITO tests cover stationary impulse balance and a 3 cm object lift
by two Cartesian 3-prismatic arms with attached force-only grippers. They do not
establish arbitrary regrasping, impact, full-geometry articulated dual-arm motion,
or full-scale assembly success. Gripper closure is exact in the transcription;
redundant velocity conditions use an explicit tolerance and are checked again.

The III-A learned contact prior is an input; this package does not train the
networks. Use multi-start budgets for nonconvex experiments. Vendor model
inertias/effort values are not independent calibration or payload identification.

The planning/execution workflow and CRISP integration boundary are specified in
[contact_execution.md](contact_execution.md). Synthetic contact replay tests do
not provide real dual-arm furniture transport or precision insertion evidence.

The first coupled Panda lift has a separate optimize/execute entry point,
`tools/run_paper_panel.py`. It keeps literal manuscript and midpoint
configuration transcriptions distinct, revalidates recorded plans independently,
and exposes MuJoCo/CRISP and low-level control choices. See
[CRISP, controller comparisons and the exact PMI scope](crisp-and-midpoint.md).
This runner does not mark the full reorient/jig/release or all scenarios complete.
