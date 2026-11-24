# Supplementary viewer: terminology and review scope

The viewer uses the title **Scale-Aware Functional Contact Transfer from Human Hands to Multi-Arm Robots** and the manuscript's terms:

| Manuscript stage | Viewer wording |
| --- | --- |
| III-B | Task-Wrench-Robust Contact Refinement |
| III-C | Scale- and Embodiment-Aware Contact Realization |
| III-D | Sequence-wide contact-type and arm-assignment selection |
| IV | Keyframe-Guided Contact-Implicit Trajectory Optimization |
| P | Unilateral point contact with the common spherical tool |
| G | Bounded gripper-mediated contact |
| Robustness certificate | Separate equilibrium reactions at every vertex of the prescribed task-wrench disturbance box |

The English presentation layer changes titles, controls, captions, and event labels. It does not modify saved dynamics, numerical results, pass/fail criteria, optimization, or controllers. Normal repository downloads retain original recordings.

The primary tasks are panel reorientation/placement, free articulated box opening, and tabletop–leg assembly. Single-embodiment tracking and historical component checks are separate categories. The anonymous review export includes all 21 primary-task conditions, 14 matched source-robustness conditions, and both executed alternative-type comparisons. The assembly alternative-type planning rejection remains in the comparison table.

## Limits that remain explicit

- Contracted contact groups are supplied synthetic inputs; learned contraction is not run.
- Source geometric fallback is reported for box/assembly; it is not a successful source robustness certificate.
- Trajectory implementations use restricted candidates and midpoint configuration-update variants; literal execution of every manuscript equation is not claimed.
- The current native arm controller uses joint impedance and planned reaction feedforward, with the force-selection matrix set to zero. A measured-normal-force hybrid-control experiment is not established by these records.
- Stationary robustness trials isolate source refinement with fixed target material contacts. Static wrench feasibility is not a closed-loop stability proof. Failures at larger disturbances are retained.
- CRISP cross-validation, real hardware, learned inference, and general collision completeness are not validated by the viewer.

## Sharing and anonymity

Keep the research repository private during anonymous review. A repository collaborator can use branches and pull requests; private forking depends on repository/account policy. Public fork/PR access is distinct from granting a reuse license. MIT is a possible permissive license for original code only after an explicit author decision; upstream robot, gripper, mesh, and library licenses continue to apply.

The root redistribution notice has not been changed. The history audit on 2026-11-13 scanned 32 local-ref commits and 1,040 unique blobs with no high-confidence credential-pattern findings. This is a pattern scan, not a guarantee that publication is safe. History contains personal author metadata and must not be included in the anonymous export.

`tools/export_review_viewer.py` builds a separate static snapshot without Git history, own-repository links, personal identifiers, or local absolute paths. It preserves third-party legal notices. Record/plan fingerprints are consistently pseudonymized and a fresh file-integrity manifest is generated. All numerical and boolean leaves are checked against the original recordings. Already-public matching experiments can still allow indirect identification; removing explicit identifiers does not guarantee anonymity.
