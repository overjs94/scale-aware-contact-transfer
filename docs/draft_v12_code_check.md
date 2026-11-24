# Draft v12 ↔ implementation check

Historical audit. The newly uploaded 11-page manuscript changes the numerical procedure; see [the current equation audit](manuscript-equation-audit.md). In particular, it specifies IPOPT and marks shared-endpoint joint refinement as an extension.

Reviewed the latest paired TeX/PDF snapshot, both version 12 and modified on
2026-11-10 at 22:28 KST. The manuscript is not edited by this code branch.
Line references below address Contact_contraction_MS_TRO.tex v12.
The separately uploaded Contact_contraction_MS_TRO(1).pdf has 9 pages and
retains an older evaluation-protocol/numerical-solution layout; it is not the
paired current PDF (10 pages, version 12). The current TeX/PDF pair is the
implementation reference. Changes from v11 to v12 clarify force-only gripper
notation, parameterized feasible sets and wrench dimensions; they do not
remove the solver or sequence-optimization discrepancies below.

## Main finding

The draft now selects SNOPT/SQP in Sec. IV-C (lines 1055–1091). The tested
implementation uses CasADi/IPOPT, an interior-point NLP solver. These are
different numerical procedures. SolverSettings exposes backend selection,
including an optional SNOPT plugin path, but SNOPT has not been installed,
licensed or benchmarked here. Do not report the prototype timings as SNOPT results.
Either validate that backend or align the manuscript's implementation description
with the solver actually used. No manuscript solver change is made silently.

| Draft requirement | Code | Assessment |
|---|---|---|
| III-A cross-keyframe group correspondence, lines 378–386 | pipeline.SourcePrior.group_ids; trajectory.Contact.group_id | IDs are supplied explicitly; decoding index is not treated as temporal identity. Learning/annotation correspondence is external input. |
| III-B outward normals and object-on-agent force | contacts.py, refinement.py | Same sign; local reaction is rotated into object coordinates. |
| III-B object-origin wrench and COM gravity, lines 420–440 | gravity_wrench, map_reaction | Same angular-first order; COM moment is retained. |
| Ellipsoid → enclosing box and separate vertex reactions | TaskWrench.vertices, refine_contacts | Matches conservative box certificate; not an exact ellipsoidal margin. |
| Maximize rho | refine_contacts | Default geometry regularization changed to zero to match the stated objective. |
| Source physical model | source_prior | Enforces P source interfaces; generic refine_contacts can be used separately for other research variants. |
| Failed source solve vs feasible rho=0, lines 515–529 | source_prior, tests/test_prior_phases.py | Distinct statuses and certificate flags; failed solve forwards learned geometry without a source certificate. A failed local solve is not asserted to prove global infeasibility. |
| Geometry-only scale transfer | scale_prior | Scales positions, keeps normals/IDs; does not expose scaled source forces. |
| Target load, geometry, posture, torque certificates | realize_keyframe | Matches fixed-base arm model; all task vertices have separate reactions and torque bounds. |
| lambda_n in geometry cost | realize_keyframe.normal_weight | Separate configurable normal weight. |
| General finger resultant set, lines 596–635 | Capability | Partial: calibrated force-only box or full-rank bounded rigid holding. Internal finger cones, aperture decisions and finger clearance are not implemented. Partial-moment models without matching closure are rejected. |
| Tool-to-interface transform | Named URDF fixed tool frame | Tool origin/frame must be included in URDF; not a robot-specific System_Param offset. |
| III-D type/assignment enumeration | choose_sequence | All supplied candidates compared; required transition full-geometry validation is checked by default. |
| III-D shared endpoint/interval joint refinement, lines 849–861 | sequence.py | Not implemented; candidate evaluation is not claimed to solve this complete coupled NLP. |
| IV coupled momentum balance and SE(3) update | MechanicalSystem, solve_cito | Implemented. Robot and object coupling is through contact impulses. |
| Normal complementarity + max dissipation | solve_cito | Implemented with normal continuation and smooth squared cone/speed inequalities. |
| Required point maintenance/sticking/release, lines 960–966 | maintained_steps, sticking_steps, released_steps | Added explicit phase constraints. Required phases are separate from allowed candidate nodes. |
| Gripper acquisition/holding/unloading | attached_steps, release_at_end | Phase order supplied; velocity tolerance explicitly reported. Aperture/finger-closing dynamics remain external. |
| Scaled pose/effort/velocity/duration costs | solve_cito Kp,kR,Q_tau,Q_v,characteristic_length,duration_weight | Configurable in the stated form. |
| SNOPT/SQP | SolverSettings.backend | Unresolved implementation/validation difference; tested default is IPOPT. |
| Time-active maximum-violation + merit function | solve_with_contact_search | Partial orchestration only; no full STOCS oracle/merit implementation. |
| Full-geometry + finer-step clearance validation | Validator callbacks and explicit flag | Not supplied by the dynamics-only URDF assets. Restricted test success is not a full-geometry certificate. |
| Evaluation protocol | tests, examples, web data | Unit/prototype results only. No paper-scale experiment, physical dual-arm result, timing comparison or trained contraction network result is fabricated. |

## Changes made during alignment

Source fallback/status handling; stable group-ID fields; required point contact
phases; optional release-at-end unloading; configurable paper-form trajectory
costs; separate normal cost weight; solver selection; and rejection of
unvalidated sequence transitions were added. New tests cover zero-margin vs
failed source refinement and required contact maintenance/sticking.

## Remaining priority

1. Decide and validate the final solver backend and its licensing environment.
2. Supply calibrated robot-specific gripper geometry, aperture/clearance model,
   collision meshes and collision-pair exclusions.
3. Implement the complete time-active oracle/merit update and shared-variable
   sequence NLP, then compare under the same multistart and residual budgets.
4. Verify dense/finer-step replay with real collision geometry and actual
   hardware interfaces. G1 requires whole-body support constraints/controller.

The static web viewer is recorded result playback. Its G1 scene is labeled
kinematic; its CITO scene uses Cartesian arms. These views are not substituted
for the pending manuscript experiments.
