# Three arm contracted contact experiment

Assume a network has returned three stable object-frame contact groups. This
experiment consumes those predictions, optimizes three keyframes and two CITO
transitions, then runs three torque-controlled URDF Cartesian arms against a
free panel in MuJoCo. The three tools are a spherical point tool, a physical
parallel-jaw gripper and a pressure-limited suction cup.

## Run the complete experiment

~~~bash
python -m pip install -e ".[dev,simulation,conic]"
contact-transfer demo-three-arm --output outputs/three_arm --viewer web
npm --prefix web ci
npm --prefix web run build
python web/server.py --host 127.0.0.1 --port 8000
~~~

Open `http://127.0.0.1:8000/?model=three_arm` or select
**Three-arm contracted contacts** in the viewer. Support, lift and
transfer buttons jump to the three keyframes; vent and release jumps to tool
release. The viewer loads three URDF instances, including both finger joints,
and shows actual simulated panel translation and orientation with the receiver.
The exported recording needs no remote mesh downloads: this authored testbed
uses URDF primitive shapes. The existing vendor-arm visualizations are retained.

`python examples/three_arm_contracted.py` runs the same pipeline. The CLI exits
with an error if keyframe tracking, seal retention or the measured force
envelopes fail. `--viewer` writes a recording, not a deployment. The existing
single-arm web amplitude endpoint does not rerun this multi-arm optimization.

## Input contract

The default case and three robot/tool descriptions are in
`src/contact_transfer/scenarios/three_arm/`. Copy this directory and pass
`--case /path/to/case.json` to change the prediction, scene or physics parameters.
Robot geometry, masses, inertias and joint limits are read from the URDFs.

`case.json` declares SI units, object-frame contact coordinates and world object
poses as `[x,y,z,qx,qy,qz,qw]`. Its `contracted_contacts` are the assumed network
output; no trained network is invoked. Each group contains:

~~~json
{
  "group_id": "support_sphere",
  "arm_id": "point",
  "point_object": [-0.05, -0.06, -0.02],
  "normal_object": [0, 0, -1]
}
~~~

Use a separate network-output file without changing robot/tool settings:

~~~bash
contact-transfer demo-three-arm --contacts predictions.json --viewer web
~~~

The predictions file wraps the list in `contracted_contacts` and declares
`schema_version: 1`, `units: "SI"`, and `contact_frame: "object"`. The packaged
`predicted_contacts.json` is a complete example. The network may reorder its
groups: identity and arm assignment are matched by IDs. Saved `case.json`
contains the resolved predictions, and the case fingerprint includes these
values and the URDF bytes. Inference remains external to this package.

The other groups hold the panel's left edge and top face. `group_id` preserves
identity; `arm_id` supplies a fixed injective assignment. All three groups remain
at fixed local surface locations across the three keyframes. The loader checks
IDs, assignments, surface positions, outward unit normals and pressure/area
compatibility. This first experiment uses a box panel, translation-only target
poses and one of each tool. Changed contacts, assignment search, approach
planning and regrasping are subsequent experiments.

## Optimization and execution

| Phase | Behavior |
|---|---|
| Keyframe optimization | III-C posture and separate vertex reactions; joint torque limits and a conservative six-axis box robustness margin |
| Transition planning | Two independently solved CITO intervals, eight steps each, semi-implicit coupled robot/object dynamics, point complementarity, sticking and configured attachment |
| Warm start | URDF Jacobians initialize Cartesian posture; a rest-to-rest discrete translation and Clarabel force distribution initialize momentum balance |
| Initial preparation | 0.0–0.3 s: close physical fingers and acquire suction while the spherical tool supports the panel |
| Three keyframes | 0.3 s support at z=0.60 m; 1.5 s lift to z=0.66 m; 2.7 s transfer by +0.08 m in x |
| Hold and release | Hold to 3.0 s, vent suction, open fingers and retract tools; simulate freely through 3.8 s onto the receiver |

The Cartesian arms have nine planned translation axes. Two finger joints are
locked for the force-resultant planning model and independently actuated in
MuJoCo, for eleven simulated actuators. The controller uses planned generalized
torque plus joint feedback. The object has a free joint; no object pose is
copied from the plan during execution. No weld/equality attachment is used.
Keyframes are solved separately and their endpoints are fixed in each interval;
this is not the complete shared-variable III-D sequence NLP.

## Tool models

The **point tool** has a 25 mm sphere defined in its URDF. Planning uses a
unilateral Coulomb point with no moment. MuJoCo evaluates actual sphere–panel
contacts, with explicit pair friction and no torsional/rolling friction.

The **gripper** has two prismatic fingers, collision pads and bounded motor
torques. The physical grasp comes from finger compression and friction. Its
planning interface is the existing bounded force-resultant G model, not an
ideal physical weld. The testbed's force bounds are assumed parameters, not a
hardware-calibrated inner wrench set. Measured pad resultants and moments are
logged separately. The finite fingers can transmit moments that the force-only
planner omits; force-envelope acceptance does not certify those moments.

The **suction cup** has a collision rim and a compliant adhesive-point model.
The added S capability uses object-on-tool local reaction `f`:

~~~text
-F_adh <= f_n <= F_compression
norm(f_t) <= mu * (f_n + F_adh)
F_adh <= pressure * pi * seal_radius^2 * area_efficiency
~~~

Negative normal reaction represents tension on the cup. CITO scales the same
bounds by interval duration for impulses; the fixed-geometry SOCP uses an
offset Lorentz cone. The source-prior paper path remains unilateral P; S is an
explicit experimental tool extension.

The seal acquires only within gap, lateral and angle tolerances. Spring/damper
traction is clipped to the pressure and shifted friction limits, with equal and
opposite forces applied at one interface point. Excess gap, lateral offset or
angle loses the seal; venting removes the adhesive force. The default adhesion
limit is 15 N. This is a lumped seal approximation, not cup deformation, leakage
or fluid simulation. A broken seal never becomes an unlimited holding weld.

## Outputs and acceptance

| File | Contents |
|---|---|
| `scene.xml` | Full assembled three-arm, panel and receiver MJCF |
| `case.json` and tool URDFs | Exact input configuration and mechanical/geometry sources for reproduction |
| `plan.npz` | Optimized q/v, torques, dt and group impulses |
| `execution.npz` | Actual joint/object states, commanded torque and contact traces |
| `vacuum.npz` | Enabled/sealed state, gap, lateral error, angle and applied traction |
| `tool_wrenches.npz` | Contact plus vacuum reactions in each local contact frame, moment first |
| `report.json` | Solver residuals, keyframe errors, seal events and measured force-envelope checks |

Measured reactions include all panel–tool contact points and the suction
traction; moments are shifted to the predicted interface before rotation. Force
limits are checked during the planned task window, after preparation and before
venting. Preparation and release are separate dynamics phases. Release/retraction
is a scripted diagnostic, not optimized gentle placement. The configured
acceptance is 15 mm position and 0.12 rad rotation at each keyframe, persistent
seal before commanded venting, and force violations below 0.05 N.

The initial run tracked the three keyframes to about 1.2 mm and 0.009 rad,
retained suction until the 3.0 s vent and respected all three planned force
envelopes. The free panel then fell onto the receiver. These are results for an
authored Cartesian/tool testbed. Rail visuals are schematic and their collision
geometry is omitted; tool/panel/receiver contacts are simulated. No vendor-arm
assembly result, full robot collision certificate, measured gripper wrench
calibration, hardware result or learned network performance is claimed.

## Three real Franka Panda arms and web controls

The original `three_arm` experiment remains a synthetic Cartesian testbed. The
new `three_franka` preset uses three upstream Panda arm chains, including their
seven revolute joints, joint limits, inertias and collision meshes. The point
and suction tools replace the hand at `panda_link8`; the gripper retains the
native Panda hand and two independent finger motors. Upstream provenance and
Apache license are bundled beside the modified URDFs. Standard upstream SRDF
self-collision exclusions are retained; collisions between arms, other links,
the panel and receiver remain enabled.

```sh
python tools/fetch_panda_meshes.py
contact-transfer demo-three-arm --scenario franka --output outputs/three_franka --viewer web
npm --prefix web ci
npm --prefix web run build
.venv/bin/python web/server.py
```

Open `http://localhost:8000`. The default recording shows three actual articulated
Pandas. The experiment form accepts panel mass, three object positions/times, and
contracted-contact JSON (SI, object frame). It submits an isolated Python worker,
reports optimization/simulation/export stages, supports cancellation, and loads
the resulting actual URDF joint and object trajectories. JSON can be imported
from a file or edited directly. Tool assignment is fixed to sphere / Panda hand /
suction for this first baseline. Invalid surface points, normals, time intervals
and unreachable IK targets produce explicit errors. The server accepts same-origin
browser requests; it is a local research server without robot hardware access.
Static hosting supports recorded replay and input editing only; it disables new
computation and gives the local-server command.

The Panda planner is deliberately **fixed contact path**, not full CITO: full-pose
IK prescribes a sampled rest-to-rest joint path; IPOPT optimizes reaction forces
subject to object inertial balance, tool force envelopes, and derived inverse-
dynamics joint torque limits. Clarabel computes a six-axis box wrench margin at
each frozen keyframe. Joint velocity limits are checked. This baseline fixes
object orientation and holds the last pose instead of releasing into the arms.
Panda full CITO posture/mode search and collision-aware path optimization remain
unimplemented. The Cartesian CITO implementation remains available separately.

The default 1 kg panel execution uses 1 ms MuJoCo steps with a 0.5 s preparation,
1.6 s lift, 1.6 s transfer and 0.3 s hold. Measured keyframe translation errors
were 0.71, 0.80 and 0.86 mm; contact force envelopes and suction retention passed.
The panel is a free body; there are no weld/equality constraints. Non-tool
penetrations deeper than 0.2 mm are reported separately. Discrete simulation
collision checks are not a continuous full-geometry certificate. Finite gripper
moments are measured but are not certified by the force-only capability model.
