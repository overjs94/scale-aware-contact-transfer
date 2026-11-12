# CRISP engine option and passive midpoint integration

CRISP is an optional execution engine behind the same optimized plan and
engine-neutral measured-contact controller. CasADi/IPOPT still solves the
planning problem. Changing the contact engine does not complete missing
source refinement, type/assignment selection or trajectory optimization.

The first adapter supports **three fixed-base torque-controlled Panda arms
and one free rigid panel**. It imports URDF inertias, limits, collision meshes
and the supplied SRDF exclusions. It applies robot joint efforts only. The
panel has no actuator, weld or prescribed pose. Mobile RBY1 and free
articulated-object CRISP execution are not implemented by this adapter.

## SDK and build

Tested public SDK: INRoL CRISP v1.1.1, examples commit
`3d47426cac6f001b16f8d613339383c144029e88`. The Linux x86-64 release archive has
SHA-256 `55ec7189d776e67cd571b93b919e6c7d6373a42083ae3e911e830a5e6a223b7d`.
Obtain the unmodified SDK from the [official project](https://github.com/INRoL/crisp),
retain its LICENSE and NOTICE, and follow its installation instructions.
Its license is **academic/non-commercial**; this repository and the public
GitHub Pages export do not redistribute the SDK. Our C++ API client lives in
`backends/crisp` and links a separately installed SDK. Eigen 3.4 and CMake are
required in addition to a C++20 compiler.

For an SDK installed at `.cache/crisp-sdk/crisp`:

```sh
python tools/run_crisp_probe.py --sdk .cache/crisp-sdk/crisp \
  --eigen-cmake /path/to/Eigen3/cmake
```

This builds `crisp_contact_probe`, `crisp_frame_probe` and `crisp_scene_bridge` in
`.cache/crisp-adapter-build`. No CRISP dependency is added to the Python
package's default installation.

The probe runs CANAL and SubADMM at 1 ms and 0.5 ms. All four runs settle a
0.1 kg, 20 mm sphere at 20 mm height, keep contact during the measurement
window, and recover 0.981 N support. These are **contact API smoke tests**,
not manipulation, frictional-jamming or passivity validation. An additional
rotated frictionless contact compares the native impulse against independent
linear momentum balance. This detects a transposed-frame error that a
horizontal plane cannot detect. A bridge smoke test also exercises reset,
one step and clean process/SDK destruction with ground geometry present.

## Contact and state conventions

For this SDK, the ground/sphere test independently establishes that native
`con.rot[k].T * con.lambda[k]` is an **impulse on geometry 1**. Native `rot`
maps world coordinates into the contact frame. The portable
trace records wrench on geometry 2:

```math
F_2=-R_c^T\lambda/h,\qquad n_{1\to2}=-n_{\rm native}.
```

The first bridge models point Coulomb contacts, so their intrinsic moment is
zero; wrench moments at a tool origin include the actual contact-position
shift. Native `lambda` is iterated by its number of contact vectors, not by
`size.nc`, which counts constraint coordinates. URDF mesh basenames may be
reused across robots: feedback associates native geometry IDs with their
owning bodies and generates unambiguous prefixed aliases. Actuators are
mapped by joint identity, not assumed import order. Ground's negative owning
body ID is handled separately. Solver contacts are snapshotted before FK
refresh, so old impulses are not paired with refreshed contact features.
The bridge checks native
body FK against independent Pinocchio before executing a plan.

The initial adapter uses a common coefficient of friction 0.8 and no intrinsic
torsional friction. This differs explicitly from the MuJoCo soft-pad contact
model. Recorded contact impulses are from the preceding step. Contact force
filtering, phase logic and force/motion/jaw control use the shared Python
controller, rather than a prescribed object animation. A task failure must
remain a failure even when the contact solver's residual is small.

## Optimize once, compare native execution

```sh
OPENBLAS_NUM_THREADS=1 python tools/run_paper_panel.py \
  --engine mujoco --output outputs/coupled_panel_lift
OPENBLAS_NUM_THREADS=1 python tools/run_paper_panel.py \
  --replay outputs/coupled_panel_lift/plan.npz \
  --engine crisp --crisp-solver canal --output outputs/crisp_panel_lift
```

`--crisp-solver sub_admm` selects the alternative native solver.
`--timestep .0005` halves the **execution** step without changing the optimized
plan. Recorded plans preserve their discretization and contact phases; replay
rechecks model identity, keyframes, native mechanics and sampled collision
geometry. `ExecutionTrace` stores robot states, free-object poses, efforts,
contact gaps/normals/wrenches and solver/contact settings. The runner exits
nonzero for rejected plans or failed tracking/contact execution. Validation
includes jaw-force upper/lower bounds, both jaws retaining positive reaction,
and raw native resultant wrenches checked against the declared P/G capability
models at 50 ms intervals. Filtered feedback signals are not used for that
independent capability check.

## Low-level controller comparisons

`--control-mode` exposes three explicit execution choices, with the same plan:

| Mode | Normal direction of a spherical support | Gripper width |
|---|---|---|
| `hybrid_feedback` | Planned force plus measured force feedback; tangential/orientation motion tracking | Maximum-jaw force feedback plus centre impedance |
| `hybrid_feedforward` | Planned force only; tangential/orientation motion tracking | Common closing-force feedforward plus centre impedance |
| `impedance_feedforward` | Full pose tracking plus planned force feedforward | Common closing-force feedforward plus centre impedance |

For mixed Jacobian `J`, contact-coordinate map `H`, `J_c=HJ`, and
`S_m=I-S_f`, the torque command before clipping is

```math
\tau=\operatorname{RNEA}(q,v,a_d)-J^T\lambda_{ff}
 +J_c^T\{S_m(K_xe+D_x\dot e)
 +S_f[K_fH(\lambda_m-\lambda_{ff})+K_Iz]\}
 +N^T[K_q(q_d-q)+D_q(v_d-v)].
```

`N` projects joint posture impedance out of the Cartesian task. Reactions are
object-on-robot; all wrenches include the correct origin shift. The hybrid FF
mode sets `K_f=K_I=0`; the full pose FF mode additionally sets `S_f=0`.
Thus full pose FF explicitly retains normal pose stiffness. These modes are
not described as equivalent force controllers. All torque commands obey the
URDF effort limits; the dynamic object receives no commanded pose or force.

The two gripper jaw forces are distinct from the net six-dimensional interface
wrench. Common closing preload contributes internal grasp force and can cancel
in that resultant. Feedforward uses a finite constant common closing preload
and a differential centre impedance. The default FF preload is 0.7 of the
declared 18 N bound, leaving room for asymmetric load sharing; force feedback
instead regulates the maximum measured jaw force to 0.95 of that bound.
These are declared simulation settings, not vendor force calibration.

```sh
OPENBLAS_NUM_THREADS=1 python tools/run_paper_panel.py \
  --replay outputs/coupled_panel_lift/plan.npz --engine crisp \
  --control-mode impedance_feedforward --output outputs/crisp_force_ff
```

Pose tracking alone does not qualify an execution: the same report must pass
contact retention, finite jaw forces and sampled wrench-capability checks.

The default planning variant is explicitly called `independent_midpoint`:

```math
q_{r,k+1}=q_{r,k}+h(v_{r,k}+v_{r,k+1})/2,\qquad
T_{o,k+1}=T_{o,k}\exp\{h(V_{o,k}+V_{o,k+1})/2\}.
```

The momentum equation still uses the pre-step mass, bias and Jacobians.
Therefore this is **midpoint configuration transcription**, not a complete
passive midpoint multibody integrator and not a literal reproduction of the
manuscript's post-step Euler configuration equation. The literal formulation
is retained as `--configuration-update independent_se3`. The alternative
`grasp_consistent` also records and bounds its independent-integration defect;
it is never silently identified with either of the other formulations.

At the tested 12-interval first lift, the midpoint variant has native
Pinocchio momentum residual below 5.6e-11 and independent position/rotation
integration residual below 1.4e-11. The held-grasp velocity bound is 1e-5.
MuJoCo at 1 ms executes this plan with maximum panel position error 4.51 mm
and angle error 0.01693 rad. This is one optimized lift, **not the complete
reorient/jig/release experiment or a completed III-D comparison**.

## PMI belongs to the integration/control layer

The user's PMI is **passive midpoint integration**. It is not projection onto
a grasp manifold. CRISP's CANAL/SubADMM contact solver choice is separate from
the integration rule; the public API and paper do not establish that selecting
CRISP provides a passive midpoint integrator.

`pmi.passive_midpoint_step` implements a constant diagonal virtual
mass/damper/spring system with midpoint-held input:

```math
\bar v=\left(2M/h+D+hK/2\right)^{-1}
       \left(2Mv_k/h+f-Kx_k\right),\quad
x_{k+1}=x_k+h\bar v,\quad v_{k+1}=2\bar v-v_k.
```

For `H=(v^TMv+x^TKx)/2` it satisfies

```math
H_{k+1}-H_k=h f^T\bar v-h\bar v^TD\bar v.
```

The position-controlled RB/UR admittance layer now uses this virtual-system
step for its selected force axes and integrates commanded displacement with
the midpoint velocity. Unit checks cover stiff large-step work balance and
undamped lossless oscillation. The reported energy identity applies to that
primitive; changing contact frames, clipping velocities, bounded IK and the
manufacturer's unknown position servo do not inherit a global passivity proof.
No full nonlinear multibody/contact PMI guarantee is claimed.

## Recorded first-lift evidence

After correcting the native contact-frame transpose and checking impulses in
their preceding configurations, the same 12-interval plan was executed with
`impedance_feedforward` and a 12.6 N common jaw preload (0.7 × 18 N):

| Native engine | Maximum position error | Maximum angle error | Largest jaw reaction | Contact retention |
|---|---:|---:|---:|---:|
| MuJoCo, 1 ms | 0.2155 mm | 0.002106 rad | 14.710 N | 100% on all three interfaces |
| CRISP CANAL, 1 ms | 0.2071 mm | 0.003032 rad | 15.064 N | 100% on all three interfaces |

Both jaws remain loaded, both stay below the declared 18 N bound, robot
efforts stay inside URDF limits, and all 147 sampled raw resultants per
interface pass the independent P/G capability checks. No force sensor signal
is used to close a force loop in this FF mode. Native impulses/features are
matched before FK refresh; the capability checker uses the preceding pose
for CRISP step impulses, rather than introducing a false moving-frame cone
violation. This timing rule has an explicit regression test.

The current `hybrid_feedback` gains still lose the held panel in CRISP even
after those adapter corrections. That failure is retained separately. Thus
the evidence establishes a nominal pose/force-FF baseline, not a validated
rigid-contact force-feedback controller, disturbance robustness or full task.
`docs/panda-lift-execution-results.json` records the qualified measurements.

The clean `--engine plan` command was also rerun from source with the current
keyframe warm start. Its independent momentum residual is below 4e-9, with
joint/object integration and grasp closure below 2e-11. This remains the
explicitly labelled midpoint configuration variant and first lift only.

Reference: [CRISP documentation](https://inrol.github.io/crisp/docs/), including
its model construction, contact solver, API and licensing pages.
