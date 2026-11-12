# Implementation scope

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
| III-C | Arm posture, contact geometry, robustness and vertex torque bounds | Arm helper uses fixed bases; detailed finger/aperture geometry external |
| III-D | Configured type/assignment enumeration and transition callbacks | Shared-keyframe/interval joint NLP not implemented yet |
| IV restricted CITO | Dynamics, Lie integration, complementarity, maximum dissipation, gripper phases | Candidates and attachment phases supplied |
| Contact search | Candidate/node update loop with user geometry oracle | Complete STOCS merit/oracle algorithm not reproduced |
| Collision | Differentiable margins and trajectory validator callbacks | Bundled dynamics-only models do not provide full mesh clearances |
| Control | Computed torque, Cartesian admittance, position servo | Simulation level; no hard real-time guarantee |
| MuJoCo | URDF inertia export, state mapping, headless tracking | Markers are not collision geometry |
| Mesh visualization | Eight full upstream URDFs, STL/DAE loading, visual/collision selector and recorded joint/base poses | Upstream mesh network access and WebGL required; display does not enable contact dynamics |
| Fixed-geometry SOCP | Clarabel box margin, exact friction cones, fixed-posture torque bounds | Geometry/closure/collision remain NLP or scene checks |
| Contact scene replay | Supplied MJCF, scalar-joint torque control, free objects, contact traces, equality hold/release | Real furniture collision assets and gripper calibration external |
| Engine comparison | Validated portable trace and matched endpoint comparisons | CRISP C++ runner/adapter and insertion/jam classifier not implemented |
| Three-arm contact experiment | Assumed network groups, 3 keyframes, 2 CITO solves, URDF sphere/fingers/cup, actual dynamic replay and vent/release | Authored Cartesian arms, fixed contact assignments; force-only gripper and lumped suction model |
| Solver comparison | Fresh stationary CITO trials and residual-qualified timing tool | SNOPT unavailable in validated runtime; no assembly benchmark |
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
