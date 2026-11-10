# Bundled embodiment models

All eight are offline, expanded URDF dynamics/kinematics subsets of upstream descriptions.
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
