# Review of the supplied source

The uploaded archive remains unchanged. Source/URDF files were extracted for
review. Historical ZIP snapshots, Git history, CSVs and build outputs are not vendored.

| Reference | Observed structure | New implementation |
|---|---|---|
| tele_opt and tele_opt_exp include/System_Param.* | Global dimensions, axes, masses, COMs, inertias | Validated URDF tree plus experiment settings |
| tele_opt/include/Kinematics/Kinematics.cpp | Adjoint Jacobian; body linear velocity rotated into world | Preserve mixed convention; generate named URDF frames |
| tele_opt/include/Dynamics/Dynamics.cpp | Lie Newton--Euler block products; C-dependent Mdot | Pinocchio and independent CasADi RNEA; independent Mdot check |
| tele_opt/src/optimization.cpp | Pose/reaction/actuation variables; NLopt COBYLA constraints | Physical quantity separation; CasADi/IPOPT derivatives |
| tele_opt_exp/src/optimization.cpp | Experiment-specific loads/contacts | Configuration and capability inputs |
| tele_opt/src/Dynamics_Control.cpp | PMI integration, task/contact feedback, bounded QP | Model/controller separation; torque and position/admittance paths |
| PMI-IK/include/*_kinematics/ | Robot-specific FK/J and dual-arm pose errors | Named URDF frames and generated functions |
| PMI-IK/src/Inverse_kinematics.cpp | Bounded QP updates and collision-aware initialization | URDF constrained IK and bounded differential servo |
| third_party | Historical IPOPT/NLopt/qpOASES/qpSWIFT source/build trees | Dependency-managed installation |

Original Mdot=C+C.T is not an independent validation of C. New fixed-base
tests differentiate M along a trajectory before checking the identity.
The original and new C factorizations need not match elementwise.

Original COBYLA callbacks are derivative-free; placeholder gradient arguments
are not treated as an error for that selected solver. New graphs provide NLP derivatives.

The mathematical conventions and problem structure are retained. This version
does not directly compile or wrap the archived ROS1 C++ packages.
