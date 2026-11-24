# Uploaded manuscript → executable equation audit

Reference: **Contact_contraction_MS_TRO (2).pdf**, 11 pages, supplied 2026-11-12.
SHA-256: `03deabc2ff306db8582cda6ab24becab4e6681397ab4a5a4367c55ca7d9fe3ba`.
The manuscript itself is not copied into the repository or public viewer.
This audit supersedes the numerical-procedure findings in the historical v12 audit.

The new rigid-panel source/target optimizations and solver-free finite-step
grasp diagnosis are documented in
[grasp-discretization-diagnosis.md](grasp-discretization-diagnosis.md).
Static keyframe verification is executed with explicit finger resultants.
The original poststep-Euler/instantaneous-grasp carry trials have not passed;
the separately named midpoint configuration variant now passes a three-Panda
first lift, including the Robotiq MuJoCo case. See the
[current consolidated structure and progress report](system-architecture-and-progress.md).

## Scope that must be preserved

- Eq. (7): source **surface** positions/normals, common unilateral P model,
  per-box-vertex reactions, fixed E/D, maximize dimensionless rho, trust region,
  surface/normal constraints; record failed-refinement fallback separately.
- Eq. (8)–(14): transfer geometry only; jointly refine target contact geometry,
  robot posture and rho; recompute per-vertex target forces and torque bounds.
  G requires Eq. (11)'s finger-resultant model or a demonstrated inner model
  such as the appropriate force-only holding interface in Eq. (12). A freely
  chosen moment box is not a calibration or proof of an inner approximation.
  Aperture, finger clearance, collision and type-specific closure are required.
- Eq. (15)–(16): the uploaded version explicitly permits its **restricted
  numerical procedure**: enumerate hardware-compatible type/assignment
  candidates, solve robust keyframes, fix their endpoints for interval CITO,
  rank accepted candidates by common keyframe + interval costs. Shared-endpoint
  joint refinement is explicitly an extension, not the stated current solver.
- Eq. (17)–(19): optimize states, torques, impulses and durations between
  quasi-static endpoints; coupled dynamics, SE(3) integration, unilateral
  complementarity, maximum dissipation, maintained/sticking/released phases,
  finite grasp resultants with acquisition/holding/unloading, aperture/clearance,
  joint/velocity/torque/geometry bounds and residual-qualified acceptance.
  The numerical solver in this uploaded version is **CasADi + IPOPT**.
- The manuscript explicitly assumes **one manipulated rigid body**. A free
  articulated container or simultaneous tabletop/leg manipulation requires
  additional inter-link/body coordinates and constraints. Such a generalization
  must be labelled and independently verified, not called a literal 6D copy.

## Current executable status

| Part | Core implementation | Scenario execution boundary |
|---|---|---|
| III-B / (7) | `refinement.py`, `pipeline.py` | Rigid panel/mobile source fixtures call this stage. New free-object nominal baselines do not. |
| III-C / (8)–(14) | `realization.py`, `paper_keyframe.py` robust contact/posture/vertex torque NLP | The paper-panel runner executes explicit finger/aperture keyframes and independent checks. Existing mobile/free-object nominal runners remain separate; they do not automatically execute the full problem. |
| III-D / (15)–(16) | `sequence.choose_sequence` enumerates candidate combinations and intervals | Must be connected to every paper-task adapter. Supplied G/P assignments in a demo are not a completed sequence search. |
| IV / (17)–(19) | `trajectory.solve_cito` restricted dynamics/complementarity NLP | Cartesian examples and the explicitly named Panda/Robotiq first-lift midpoint variant execute it. Legacy Panda/mobile/free-articulated nominal paths remain separate. Full sequence, literal formulation and extended-object/mobile transcription validation remain pending. |
| Geometric oracle / validation | Callbacks and explicit `full_geometry_checked` | A missing collision model/oracle cannot be accepted as a full-geometry certificate. |
| Learning / III-A | Supplied predictions/group IDs | No trained grouping/anchor inference is claimed. A G grasp-frame centroid inside a handle is not a source surface contact for (7). |

A successful MuJoCo nominal run establishes that its supplied path/controller
and assumed contacts executed; it does **not** establish execution of all the
optimization stages above. Each published record must carry stage provenance.
`execution_passed` must never be substituted for `full_paper_pipeline_executed`.

## Low-level execution control

The uploaded planning formulation does not prescribe a particular hybrid,
joint-impedance or admittance controller. These are the requested execution
layer, to be specified and tested separately against its planned reactions.

Legacy saved nominal baselines use computed torque + Cartesian PD + planned
wrench feedforward, with independent finger squeeze PD. The new paper-panel
runner additionally exposes selected hybrid feedback/feedforward and projected
joint impedance. Its qualified first-lift mode is pose/joint impedance plus
planned wrench feedforward; the Robotiq adds measured jaw-force admittance.
The current CRISP hybrid-feedback trial remains failed. A normal-force limit
in an NLP is not itself a hybrid force controller.

The requested torque-arm controller uses a contact-frame orthogonal force
projector Sf and Sm=I−Sf, measured wrench feedback on Sf, motion feedback on Sm,
and dynamically consistent nullspace joint impedance. For object-on-robot
reaction lambda, the robot-on-object force error is lambda_meas−lambda_d.
With H mapping mixed tool/world twists into contact coordinates, Jc=HJ:

~~~math
\tau=\operatorname{clip}\{\operatorname{RNEA}(q,v,a_d)-J^T\lambda_d
 +J_c^T[S_m(K_x e+D_x\dot e)+S_f(K_f e_f+K_I\eta)]
 +N^T[K_q(q_d-q)+D_q(v_d-v)]\},
\quad e_f=S_fH(\lambda_{meas}-\lambda_d),\quad\dot\eta=e_f.
~~~

Here N=I−M^{-1}Jcᵀ(Jc M^{-1}Jcᵀ)^{-1}Jc, regularized near singularity.
Projection prevents posture impedance from competing with the controlled
contact normal. Integral limits, clipping and conditional integration are
explicit; arbitrary-frame/saturation passivity is not asserted.

For position-command RB/UR interfaces, selected force error drives virtual
mass/damping and then a bounded differential IK position command:

~~~math
M_a\dot v_f+D_a v_f=S_fH(\lambda_{meas}-\lambda_d),
\quad v_c=S_m(v_d+K_xe)+S_fv_f,
\quad q_{cmd}=\operatorname{Integrate}(q,\Delta t\,\dot q_{IK}).
~~~

The manufacturer's internal position servo is not inferred from URDF. No
physical robot transport or claimed vendor gain calibration is provided.
The shared controller components are in `hybrid_control.py`; initial tests
check force-feedback sign, contact-frame power duality, force/motion separation,
nullspace decoupling, antiwindup and position-command limits. Integration and
measured native force-tracking validation are separate completion gates.
