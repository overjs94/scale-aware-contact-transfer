# Implementation scope

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
| Hardware | Position/velocity/effort separation | No physical communication or validation |

Restricted CITO tests cover stationary impulse balance and a 3 cm object lift
by two Cartesian 3-prismatic arms with attached force-only grippers. They do not
establish arbitrary regrasping, impact, full-geometry articulated dual-arm motion,
or full-scale assembly success. Gripper closure is exact in the transcription;
redundant velocity conditions use an explicit tolerance and are checked again.

The III-A learned contact prior is an input; this package does not train the
networks. Use multi-start budgets for nonconvex experiments. Vendor model
inertias/effort values are not independent calibration or payload identification.
