# Bundled embodiment models

All eight mechanics presets are offline, expanded URDF dynamics/kinematics subsets of upstream descriptions.
Mass, COM, inertial rotation, joint axes, limits and origins are retained.
Meshes/collisions and vendor simulation/controller/transmission tags were removed.
Joint damping/friction tags are omitted for ideal rigid-body model comparison.
The models are not independently calibrated and do not include identified payloads.

| Preset | Upstream source | Revision | Model license |
|---|---|---|---|
| franka_panda | [robots/panda_description/urdf/panda.urdf](https://github.com/Gepetto/example-robot-data/blob/78b6101a6387de721316b0db717351d899f97b4f/robots/panda_description/urdf/panda.urdf) | 78b6101a6387 | Apache-2.0 |
| kinova_gen3 | [kortex_description/arms/gen3/7dof/urdf/GEN3-7DOF-VISION_ARM_URDF_V12.urdf](https://github.com/Kinovarobotics/ros_kortex/blob/c3280d8a6a6b5d96590d51538e3f265710a6c77d/kortex_description/arms/gen3/7dof/urdf/GEN3-7DOF-VISION_ARM_URDF_V12.urdf) | c3280d8a6a6b | BSD-3-Clause |
| kuka_iiwa7 | [iiwa_description/urdf/iiwa7.urdf.xacro](https://github.com/epfl-lasa/iiwa_ros/blob/9d2b99f1c62e1dd57c9c351958206a7ee5066785/iiwa_description/urdf/iiwa7.urdf.xacro) | 9d2b99f1c62e | GPL-3.0-or-later |
| rb3 | [rbpodo_description/robots/rb3_730es_u.urdf](https://github.com/RainbowRobotics/rbpodo_ros2/blob/5e8294a985e7ce5e20e70564c2681130afc5f502/rbpodo_description/robots/rb3_730es_u.urdf) | 5e8294a985e7 | Apache-2.0 |
| rb5 | [rbpodo_description/robots/rb5_850e.urdf](https://github.com/RainbowRobotics/rbpodo_ros2/blob/5e8294a985e7ce5e20e70564c2681130afc5f502/rbpodo_description/robots/rb5_850e.urdf) | 5e8294a985e7 | Apache-2.0 |
| unitree_g1 | [robots/g1_description/g1_29dof.urdf](https://github.com/unitreerobotics/unitree_ros/blob/725b59b6de39a26c425632120c89f23c49a36f1e/robots/g1_description/g1_29dof.urdf) | 725b59b6de39 | BSD-3-Clause |
| ur5e | [urdf/ur.urdf.xacro](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/blob/0eb1a861f7772fceb38510d90fa2f98ec668070c/urdf/ur.urdf.xacro) | 0eb1a861f777 | BSD-3-Clause |
| xarm7 | [xarm_description/urdf/xarm_device.urdf.xacro](https://github.com/xArm-Developer/xarm_ros2/blob/ac736c92b02de7ca7e0be6f737d77d46707464d8/xarm_description/urdf/xarm_device.urdf.xacro) | ac736c92b02d | BSD-3-Clause |

Full revisions, hashes, transformations, lock settings, tool frames and license files
are stored alongside each robot. Panda fingers are locked at zero by default.
G1 defaults to a floating pelvis with 29 independent joints; root 6 DoF are unactuated.
Fixing its base does not certify standing balance or foot-contact feasibility.
RB3 means RB3-730ES U and RB5 means RB5-850E, not every hardware revision in those families.

The RB driver exposes joint position/velocity/effort commands, including a
move_servo_t path. Availability, mode compensation and timing on a physical
controller require that hardware's documentation and tests. Position/admittance
is the default selected path, not a claim that effort is unavailable.

These subsets support M/C/G/FK/J computation without external mesh packages.
For collision/contact execution, obtain matching upstream geometry and define
collision-pair exclusions, package paths and tool/object contact surfaces.
The full-geometry validator in the trajectory solver is a separate callback.

The source code and different model assets retain separate licensing identities.
The KUKA model subset includes its upstream GPL-3.0-or-later license.

To regenerate, check out the source repositories at the model.json revisions and
run tools/rebuild_models.py --references /path/to/source-checkouts.

## Viewer URDF and mesh loading

`web/models/<id>/robot.urdf` retains the upstream visual and collision elements
for all eight presets. It is separate from the dynamics-only URDF used by the
tracking simulations. A manifest beside each viewer model records the matching
upstream commit, original and prepared URDF hashes, mesh paths, modifications
and model license. Upstream copyright comments and license files are retained.

The viewer uses Three.js and URDFLoader to apply joint origins/axes, visual and
collision origins, mesh scales, materials, locked/mimic joints and recorded
floating-base world poses. STL and DAE meshes are loaded from immutable
`raw.githubusercontent.com/<owner>/<repo>/<commit>/...` URLs, including DAE
relative textures. Binary meshes are not duplicated in this repository; viewing
them requires network access to upstream assets. The loader also supports OBJ
geometry with the URDF material; external OBJ MTL materials are not imported.

The geometry selector switches between visual, collision and link-coordinate
views. Collision is shown as a wireframe; it does not enable collision dynamics
in a previously recorded tracking run. Missing mesh/texture requests produce a
visible loading error rather than silently claiming a complete robot. Without
WebGL, the viewer explicitly uses the CPU link-coordinate projection. The
synthetic dual-Cartesian-arm CITO recording has no full robot mesh model.

To regenerate the prepared descriptions in a dedicated source directory:

~~~bash
python -m pip install -e ".[model-tools]"
python tools/prepare_viewer_models.py --sources /path/to/viewer-sources --fetch
npm --prefix web ci
npm --prefix web run build
~~~

The fetch step creates sparse source checkouts at the mechanics manifests'
exact commits. Subsequent generation can omit `--fetch`. Xacro is expanded in
temporary copies; the mechanics presets and source checkouts are not rewritten.
Preparation rejects different source hashes, link/joint trees, inertias, axes,
origins or bounds before preparing a display model.

Python tests compare the full URDF against the mechanics model for all eight
presets at recorded configurations. Browser regression tests use local triangle
fixtures to verify recorded FK, mesh origin/scale/rotation, geometry-mode
switching and missing-asset errors without depending on upstream availability.
Run them with `npx --prefix web playwright install chromium` and
`npm --prefix web test`. Real upstream visual and collision meshes were also
loaded for all eight models during integration QA. URDFLoader's Apache-2.0
license is in `web/URDF-LOADER-LICENSE.txt`; Three.js retains its MIT license.
