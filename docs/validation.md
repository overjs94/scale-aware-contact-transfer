# Validation record

Updated: 2026-11-11. Runtime: CPython 3.12.14, Linux x86_64.
Exact selected package versions are recorded in requirements-validated.txt.

## Automated tests

58 pytest tests passed on the implementation workspace.
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
| MuJoCo | All seven fixed-base arms: computed-torque tracking and torque bounds |
| Position command | Bounded mixed-twist differential IK |
| Source prior | Feasible zero margin distinguished from failed solve; geometry fallback and explicit group IDs |
| Contact phases | Required point maintenance/sticking and release-phase validation |
| URDF validation | Incomplete inertia tensors rejected before backend parsing |
| Clarabel SOCP | Analytic support and circular friction margins, IPOPT agreement, fixed-posture torque bound, rotated gripper map, zero/infeasible margin |
| MuJoCo contact scene | Dynamic object fall, upward 9.81 N support, ideal hold/release, closed-loop reference and rejected invalid scenes/commands |
| Execution trace | NPZ roundtrip without pickle, matching-plan comparison and nonfinite contact rejection |

Mechanics comparison tolerances are 2e-8. Physical velocity finite-difference
tolerance is 2e-7 and Jdot tolerance is 3e-7. These are test thresholds, not
claims of physical model calibration.

## Executable examples

- Source scalar margin: 9.809999997530284, primal residual 3.55e-15.
- Two URDF Panda keyframes: target mass 2 kg, scale 2,
  target rho 12.075999994988193, primal residual 1.70e-12,
  maximum absolute vertex torque 38.88595862231509 Nm.
  The source P solve fails in this configured example; its status and absent
  certificate are explicit. Learned geometry is forwarded and the target is
  solved independently. No source force or robustness certificate is scaled.
- RB5 headless tracking: final joint-error norm 4.308288068861311e-6 rad
  after 699 simulation steps in the CLI example.
- Python sdist and wheel built successfully. All eight URDFs, metadata and
  upstream license files are included in the wheel.

## Web viewer and local computation

Generated 121-frame records for all seven fixed-base arms from actual
MuJoCo closed-loop runs, a clearly labeled G1 kinematic preview, and a
9-node restricted dual-Cartesian-arm CITO trajectory (1 kg, 3 cm lift).
The Python HTTP backend returned HTTP 200 for health and an actual fresh
RB3 simulation request (0.5 s, 0.04 rad, 121 samples); maximum effort-limit
ratio was 0.25923.

Browser QA confirmed model selection, G1 labeling, pause, timeline scrubbing
and the CITO endpoint (0.8 s, 0.0300 m). The initial browser exercised the CPU
coordinate fallback. Updated Chromium QA with software WebGL loaded the actual
upstream visual and collision meshes for all eight robots. Ten browser tests
with deterministic local mesh fixtures verify recorded FK in both geometry
modes, origin/scale/joint rotation composition, mode switching and asset errors.
Eight additional Python tests verify full-URDF mechanics against the packaged
subsets and recorded joint names. Static hosting replays records and does not
execute Python. This source update does not deploy a new hosted viewer version.

The Dockerfile/dev-container configuration was not executed on a remote
Docker/Codespaces service. GitHub Actions successfully ran the initial
22-test implementation; each updated commit has a separate Actions check.
Hardware communication, full-mesh collision certification, general regrasping
and the complete shared-variable III-D NLP remain outside this validation.

## Independent wrench and solver checks

`examples/fixed_wrench_margin.py` returned IPOPT rho 9.809999997530284 and
Clarabel rho 9.810000000036933, with conic primal residual 3.69e-11.
The same eight-node stationary-support CITO passed two fresh IPOPT trials
(primal residual 2.54e-9, unrelaxed complementarity 1.94e-9). These are smoke
checks, not assembly performance results. SNOPT was reported unavailable by
the installed CasADi; no SNOPT timing or success comparison is claimed.

CRISP execution, an external C++ trace adapter, real furniture collision scenes,
precision insertion/jam/retention classification and timestep-sensitivity
experiments remain pending. See [contact_execution.md](contact_execution.md).

## Three arm contracted contact execution

The authored URDF Cartesian testbed accepts assumed network contact groups and
solves three keyframes plus two eight-step CITO transitions. Spherical point
contact, actuated parallel-jaw fingers and a pressure-limited compliant suction
seal move a 1 kg free panel; no equality/weld attachment is present in the scene.
The three dynamic keyframe errors were 0.94, 1.17 and 1.17 mm, with rotations
below 0.009 rad. The seal acquired at 0.0 s and vented at 3.0 s; the free panel
then fell to the receiver. Measured force envelopes passed over 0.3–3.0 s.

Ten Python tests cover external network-group ingestion/identity, suction NLP/SOCP margins, invalid contact predictions,
pressure-area rejection, Pinocchio/symbolic/MuJoCo tool mechanics, actual
three-keyframe execution and release, force clipping, seal loss and power duality.
Eleven browser tests now include the three-instance URDF replay and keyframe
navigation. Desktop and mobile software-WebGL QA checked actual tool geometry,
contact/seal indicators and release navigation. The gripper's finite-pad moments
are recorded but not certified by the force-only planning approximation. See
[three_arm_experiment.md](three_arm_experiment.md) for scope and reproduction.

Three-Panda extension (2026-11-11): 66 Python tests passed locally, including
upstream inertia preservation, symbolic/Pinocchio mass and Jacobians, independent
MuJoCo gravity, actual free-panel execution, invalid job requests and worker
cancellation. A separately submitted web job completed optimization → simulation
→ export and its recording is the default `three_franka` web dataset. Maximum
keyframe translation error was 0.86 mm; measured force envelopes, suction retention
and sampled non-tool collision checks passed. Wheel build with `--no-isolation`
passed and includes scenario URDFs, SRDF, provenance and license. Managed browser
preview was unavailable because the environment denied socket creation; visual
browser QA and hosted publication were not completed in this session. Browser
regressions for all three articulated instances and form/poll/error/cancel flows
are included in CI with deterministic mesh fixtures.

Free public viewer export (2026-11-11): `Contact_contraction` now contains only
the built static UI, 11 recorded examples, model URDF/manifests and licenses.
The public repository's `verify_site.py` passed checksum/reference checks for 62
static files and validates the recorded Panda report. The private source history,
optimizer and Python simulation backend were not copied. Public Pages deployment
passed after initial owner activation. The post-publish check fetched the live
HTML/JS/CSS, recording, three Panda URDFs and real upstream DAE/STL assets with
permissive CORS. The live URL is https://overjs94.github.io/Contact_contraction/.
Cloud-browser visual inspection remains blocked by ERR_TUNNEL_CONNECTION_FAILED.
The private workflow now packages export artifacts without requesting Pages or
OIDC write permissions.
