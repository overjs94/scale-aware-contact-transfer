# Three-Panda tabletop and leg assembly

The 30-second scenario uses two native Panda hands and one spherical point tool.
One hand grips the tabletop while the sphere supports its opposite region; the
second hand grips the leg. The tabletop's underside faces upward for inverted
assembly. It starts at a 12° pitch, lifts by 80 mm and reorients to horizontal.
The leg is inserted downward into a blind square socket and then released.

| Time (s) | Keyframe | Behavior |
| --- | --- | --- |
| 0 | grip_and_support | Open hands around the parts; sphere touches the top |
| 2 | grasp_settled | Both hands closed; initial cradles support preparation |
| 6 | lift_tabletop | Lift top and leg away from the cradles |
| 12 | reorient_tabletop | Level the tabletop through a 12° rotation |
| 18 | approach_socket | Move leg toward the socket |
| 22 | align_leg | Align above the socket |
| 27 | insert_leg | Seat the 25 mm peg |
| 30 | assembled_and_released | Hold the top while the released leg remains seated |

The leg hand starts opening at 28 s, then retracts by 80 mm. This is a separate
mode event between the last two keyframes. The observed leg is free for the final
part of the run; no weld or attachment constraint holds it in the socket.

## Geometry, mechanics and planning

`scenarios/table_assembly/` contains the authoritative URDFs: upstream Panda
mechanics with native independent fingers, a sphere tool, and authored furniture.
The tabletop is 600 × 400 × 40 mm and 2 kg; the leg is 0.4 kg. The 21 mm square
socket is a real empty region formed by collision boxes, with a blind floor.
The 18 mm square peg extends 25 mm below the leg shoulder. Mesh collisions remain
enabled for all robots, with the upstream SRDF self-collision exclusions.

Full-pose IK supplies the prescribed SE(3) path. CasADi/IPOPT allocates spatial
contact reactions under both parts' inertial balance, synthetic gripper wrench
bounds, a Coulomb point cone and URDF-derived joint torque limits. Planned load
transfer to the socket precedes leg release. The task does not search contact
modes or the geometric assembly path, and is not full CITO.

MuJoCo then integrates both parts as free bodies under robot torque control,
physical fingers, the point tool and socket contacts. Preparation cradles are
static fixtures. There are no equality constraints and no external artificial
holding forces. Soft pads use sliding friction 0.9 and a finite torsional
friction parameter of 0.008 m; elliptic-cone `impratio=100` reduces regularized
friction creep. The timestep is 1 ms. These contact parameters and synthetic
wrench limits are research assumptions, without hardware calibration.

The default measured run passed: all required grips/support contacts were
retained, no unexpected penetration beyond the sampled threshold occurred,
and the leg remained seated after hand release. Keyframe position error before
release was at most 1.19 mm. Final insertion depth was 25.00 mm, lateral error
0.134 mm and relative angle about 0.000016 rad. A passed discrete execution does
not certify continuous collision safety, fastening or structural strength.

## Run and inspect

```sh
python tools/fetch_panda_meshes.py
contact-transfer demo-table-assembly --output outputs/table_assembly --viewer web
npm --prefix web run build
```

Outputs include the resolved case, URDF/SRDF assets, collision scene, optimized
joint/reaction plan, complete execution trace, measured contact/assembly metrics
and a validation report. `assembly_view.py` exports measured joints and both
objects' poses to the viewer. Each furniture part is loaded from its own URDF,
including the empty socket, in visual and collision modes.

The [Python server](python_server.md) accepts this scenario via the async job API,
provides progress and cancellation, and returns its own isolated recording. The
GitHub Pages site replays the uploaded default recording. Its inputs do not
execute Python on GitHub Pages.
