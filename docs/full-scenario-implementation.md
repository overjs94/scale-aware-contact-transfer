# Full-scenario integration contract and execution ledger

Requested scope: furniture/table assembly, free box/lid opening, and panel
reorientation/placement/release. This file tracks implementation rather than
asserting an unexecuted full-pipeline result. The consolidated historical
[architecture report](system-architecture-and-progress.md) describes baseline
`5ad6015`, whose 131 Python and 33 browser checks passed before this work.

## Required common behavior

- All P tools are compiled from `models/tools/spherical_tip.urdf`: identical
  25 mm sphere, stem, inertia and EE-to-leading-contact-frame transform
  `[0,0,0.165] m`, identity rotation. Task orientation is not a mount bias.
- Functional groups are supplied; source refinement is executed on actual
  surface candidates. No learned inference is invented.
- Type and injective robot-assignment candidates must be evaluated and their
  rejections retained. A prescribed G/P/P or G/P/G record is not an optimized
  type-selection result. Inaccessible gripper geometries are explicitly
  inapplicable, not silently treated as point tools.
- Every accepted candidate must include all task keyframes and intervals,
  object/environment or inter-object load transfer, unload/release where
  applicable, finite grip forces, independent mechanics/capability checks,
  sampled geometry and native closed-loop execution. No nominal-path fallback
  may be labeled a passed CITO result.
- The manuscript is single-rigid-body; box internal joints and two independent
  assembly bodies are documented extensions. Integration variants and
  reference interpolation are recorded, not silently called literal Eq. (17).

## Ablation contract

| ID | Change from full method | Common quantities |
|---|---|---|
| `full` | source refinement, robust target, type selection, interval optimization, native execution | task geometry/mass, keyframes, candidate budget, tolerances and controller settings recorded |
| `no_source_refinement` | supplied source surface positions fixed | target solver and selection procedure unchanged |
| `fixed_contact_types` | task's supplied type/assignment replaces search | same endpoints/transition validator and robot limits |
| `fixed_path` | prescribed interpolated robot/object path, optimize reactions/torques only | selected tools and endpoints match full method; explicitly a reduced baseline |
| `no_arm_force_feedforward` | replay full plan with arm feedforward reaction set to zero | same pose/joint impedance and gripper squeeze loop |
| `contact_disabled` | robot-tool/object collision disabled | same plan/commands; negative physical control, expected task failure retained |
| `external_push` | inject 5 N for one second using the native force API | same optimized plan, controller gains and physical limits; replay arrow retained |

Trials record success/failure, task-end pose/insertion/hinge metrics, contact
retention, effort/jaw bounds, wrench-capability violations and planning time.
No wall-time improvement or success rate is inferred from one nominal run.
External-force trials use actual native force injection and replay arrows.

## Current work

| Item | Status |
|---|---|
| Baseline regression and architecture report | saved at `cd638a7` |
| Canonical P tool compiler / cross-scenario EE-bias check | implemented; focused check passed |
| Independent two-free-body mechanics / both quaternion integrations | implemented; focused check passed |
| Internal-contact equal/opposite impulse and power/gap checks | implemented; focused check passed |
| Target keyframe unload force flag and source-refinement switch | implemented; focused validation in progress |
| All three task adapters / scenes / independent native mechanics | implemented; three-scene mass/gravity checks passed |
| Robust target keyframes | panel 8, box 7 and assembly 9 keyframes have accepted individual solutions; full-sequence acceptance remains separate |
| Type/assignment optimization connected to complete scenarios | implemented; exhaustive configured candidates, rejections and common sequence objective retained; no accepted complete search result yet |
| Full native executions / ablation matrix / web records | runner/exporter implemented; acceptance runs in progress |

The ledger is updated from executed results. Pending cells must not produce
`full_paper_pipeline_executed=true` in public recordings.

## Current implementation and reproduction

`tools/run_full_scenarios.py` is the common entry point. `full_scenario.py`
defines tasks, source priors and native tool capabilities; `full_pipeline.py`
connects actual keyframe/interval solvers to `sequence.choose_sequence`.
`full_scene.py`, `full_validation.py`, `full_execution.py` and
`full_assessment.py` keep scene construction, independent planning validation,
closed-loop execution and measured task acceptance separate. `full_view.py`
exports measured native joints/root poses and the ablation table.

```bash
OPENBLAS_NUM_THREADS=1 .venv/bin/python tools/run_full_scenarios.py \
  --tasks panel box assembly --output outputs/full_scenarios --web web
```

Default variants are `full`, `no_source_refinement`, `fixed_contact_types`,
`fixed_path`, `no_arm_force_feedforward`, `contact_disabled`, and
`external_push` (5 N, one-second sin-squared native force pulse). Failure is a
retained trial result, not a substitute for a passed full-method trial.
`fixed_path` reports its integration defect and never claims interval CITO.
An absent accepted full plan blocks its dependent controller ablations.

Numerical changes preserve the physical gates:

- P and finger Coulomb forces can use `f=[mu*n*t_x,mu*n*t_y,n]`,
  `n>=0`, `||t||<=1`: the same circular cone with a nonsingular normal bound.
- Moving intervals use independent midpoint configuration updates, pre-state
  momentum, exact unrelaxed contact checks, and the direct tangential-speed
  norm (a 1e-15 m/s numerical floor is recorded in the implementation).
- A declared stationary dwell condenses its constant states from those same
  equations and solves the torque/finger-witness objective as a conic QP.
  It is recorded as `stationary_dwell_conic_QP`, not a moving-interval NLP.
- IPOPT primal acceptance remains 2e-6; acceptable termination additionally
  requires unscaled constraint violation <=1e-6. Objective/KKT acceptance is
  configured separately (1e-4), and final independent unrelaxed gates remain
  mandatory. A maximum-iteration result is never accepted directly.
- If maximum-margin Clarabel terminates with a numerical error, the same
  requested rho can be certified by a strictly convex fixed-rho witness QP.
  Both statuses are retained; rho is not silently reduced.
- Fixed-endpoint closures are polished with native IK and every vertex's
  torque is recomputed and rechecked. Analytically redundant endpoint rows,
  the inequality `gap>=0` alongside `gap=0`, and a holding-aperture lower bound
  alongside its exact fixed value are eliminated. Their physical feasible
  set is unchanged; the barrier problem gains strict feasibility.

The common moving-interval grid is now 16 nodes. A separate kinematic seed
solve precedes dynamics: the box lift's minimized required grasp-velocity
band was 2.34e-5 at 8 nodes and below 1e-7 at 16/32 nodes. This diagnostic
does not replace CITO. The corrected 16-node box lift has subsequently
terminated successfully as a full motion-cost CITO NLP and passed native
MuJoCo tracking/contact, finite jaw/actuator and sampled wrench checks.
That prefix does not constitute the complete box-opening task.

The full-task Robotiq model uses the tested 40 N/jaw operating point and a
native virtual-work cut for its finite 5 Nm transmission and unilateral
coupler stops. This replaces the older first-lift demo's declared 60 N box
for these new experiments; it is still an authored simulation model, not
a complete hardware capability calibration.

Stage caches carry model/formulation identities. `--keyframe-archive` is an
explicit option to reuse earlier optimized keyframes only after matching
mechanical URDF bytes, task/mode and source prior, and rerunning native
equilibrium, capability, frozen-conic and geometry checks. Archive reuse and
fresh solve wall times are distinguished. Failed `*-unaccepted.npz` files
can initialize a new solve but cannot pass any stage by themselves.
