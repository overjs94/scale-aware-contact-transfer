# Validation record

Date: 2026-11-10. Runtime: CPython 3.12.14, Linux x86_64.
Exact selected package versions are recorded in requirements-validated.txt.

## Automated tests

22 pytest tests passed in 8.54 seconds on the implementation workspace.
One warning originates from an inherited anyio pytest plugin's deprecated
pytest internal import; it is not a package test failure.

| Group | Evidence |
|---|---|
| Eight URDF embodiments | Numerical Pinocchio vs symbolic M, gravity, bias and inverse dynamics |
| Jacobians | Independent pose finite differences, including G1 floating-base body velocity |
| Jdot | Independent finite differences along manifold integration |
| Coriolis | C*v+bias consistency; independent fixed-base Mdot=C+C.T check |
| Mimic joints | Reduced mass and gravity match analytic values |
| Lie functions | Finite derivatives at zero; generated function save/reload |
| III-B | Analytic 1-kg vertical support margin: rho = 9.81 |
| III-C | URDF target contact and reaction/torque sign balance |
| Unsupported task wrench | Positive certificate rejected |
| Sequence selection | All specified types compared; infeasible transition excluded |
| IK | Known reachable Panda pose recovered |
| CITO support | Gravity impulse balance and unrelaxed residual checks |
| CITO motion | Two 3-prismatic arms lift an attached object 3 cm |
| Humanoid environment contact | Floating robot-frame/plane normal velocity matches finite difference |
| MuJoCo | Panda, RB3 and RB5 computed-torque tracking and torque bounds |
| Position command | Bounded mixed-twist differential IK |

Mechanics comparison tolerances are 2e-8. Physical velocity finite-difference
tolerance is 2e-7 and Jdot tolerance is 3e-7. These are test thresholds, not
claims of physical model calibration.

## Executable examples

- Source scalar margin: 9.809999997530284, primal residual 3.55e-15.
- Two URDF Panda keyframes: target mass 2 kg, scale 2,
  target rho 12.075999994988193, primal residual 1.70e-12,
  maximum absolute vertex torque 38.88595862231509 Nm.
- RB5 headless tracking: final joint-error norm 4.308288068861311e-6 rad
  after 699 simulation steps in the CLI example.
- Python sdist and wheel built successfully. All eight URDFs, metadata and
  upstream license files are included in the wheel.

The Dockerfile/dev-container configuration and GitHub Actions workflow are
provided, but were not executed on a remote Docker/Codespaces/Actions service.
Hardware communication, full-mesh collision certification, general regrasping
and the complete shared-variable III-D NLP remain outside this validation.
