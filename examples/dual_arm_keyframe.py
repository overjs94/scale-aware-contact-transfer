"""Source-to-target box-robust realization with two URDF-based Panda arms.

Fixed local contact patches illustrate the interface. This is a keyframe
example; no mesh collision certificate or gripper finger model is claimed.
"""

import json
import numpy as np

from contact_transfer.contacts import Capability, SurfacePatch, TaskWrench, gravity_wrench
from contact_transfer.embodiments import load
from contact_transfer.realization import Arm, realize_keyframe
from contact_transfer.pipeline import source_prior, scale_prior


def main():
    q0 = np.array([0., -.4, 0., -1.8, 0., 1.4, .7])
    left_base, right_base = np.eye(4), np.eye(4)
    left_base[1, 3], right_base[1, 3] = -.3, .3
    robots = [load("franka_panda", base_pose=T) for T in (left_base, right_base)]
    tips = [r.fk(q0, "panda_link8") for r in robots]
    object_pose = np.eye(4)
    object_pose[:3, 3] = np.mean([T[:3, 3] for T in tips], axis=0)
    target_points = [T[:3, 3]-object_pose[:3, 3] for T in tips]
    normals = [-T[:3, 2] for T in tips]
    scale = 2.
    source_points = [p/scale for p in target_points]
    E = np.array([[0., 0, 0, 0, 0, 1]]).T
    D = np.array([[5.]])
    capabilities = [Capability.force_gripper([40., 40., 40.]) for _ in robots]
    # The explicitly configured masses differ; no source-force scaling is used.
    source_task = TaskWrench(gravity_wrench(.25, np.zeros(3)), E, D)
    source = source_prior([SurfacePatch.fixed(p, n) for p, n in zip(source_points, normals)],
                          source_task, [Capability(normal_max=40) for _ in robots],
                          group_ids=("left_support", "right_support"))
    transferred = scale_prior(source, scale)
    target_patches = [SurfacePatch.fixed(p, n) for p, n in
                      zip(transferred["positions"], transferred["normals"])]
    target_task = TaskWrench(gravity_wrench(2., np.zeros(3)), E, D)
    arms = [Arm(r.symbolic(), "panda_link8", q0) for r in robots]
    target = realize_keyframe(arms, target_patches, capabilities, [0, 1],
                              object_pose, target_task, characteristic_length=.6,
                              posture_weight=.1, robustness_weight=1.)
    print(json.dumps({
        "scale": scale, "source_mass": .25, "target_mass": 2.,
        "source_rho": source.source_rho, "source_status": source.source_status,
        "source_certificate": source.source_certificate, "target_rho": target.rho,
        "target_primal_residual": target.primal_residual,
        "max_absolute_joint_torque": float(np.max(abs(target.torques))),
        "target_q": [q.tolist() for q in target.q],
        "full_geometry_checked": False,
    }, indent=2))


if __name__ == "__main__":
    main()
