"""Export actual multi-robot execution to the static URDF web viewer."""

import json
import hashlib
from pathlib import Path
import shutil

import numpy as np
from scipy.spatial.transform import Rotation

from .robot import Robot


def export_three_arm_view(plan, trace, report, web_directory, frames=191):
    web = Path(web_directory); data_dir = web/"data"; data_dir.mkdir(parents=True, exist_ok=True)
    franka = plan.case.data.get("model_family") == "franka_panda"
    dataset_id = "three_franka" if franka else "three_arm"
    source_manifest = json.loads((plan.case.path.parent/"provenance.json").read_text()) if franka else {}
    indices = np.unique(np.linspace(0, len(trace.time)-1, frames).round().astype(int))
    positions = [[] for _ in indices]
    names, edges, instances = [], [], []
    offset = 0
    efforts = []
    for reduced, spec in zip(plan.case.robots, plan.case.data["arms"]):
        full = Robot(reduced.path, base_pose=reduced.base_pose)
        efforts.extend(full.effort.tolist())
        link_names = list(full.tree.links)
        edges += [[len(names)+link_names.index(j.parent), len(names)+link_names.index(j.child)] for j in full.tree.joints]
        names += [spec["id"]+"_"+n for n in link_names]
        for k, sample in enumerate(indices):
            q = trace.q[sample, offset:offset+full.nq]
            positions[k] += [full.fk(q, name)[:3, 3].tolist() for name in link_names]
        model_id = dataset_id+"_"+spec["id"]
        directory = web/"models"/model_id; directory.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(full.path, directory/"robot.urdf")
        for name in ("LICENSE.upstream", "provenance.json"):
            source = plan.case.path.parent/name
            if source.exists(): shutil.copyfile(source, directory/name)
        (directory/"model.json").write_text(json.dumps({**source_manifest, "id": model_id, "urdf": "robot.urdf",
                                                        "urdf_sha256": hashlib.sha256(full.path.read_bytes()).hexdigest(),
                                                        "packages": source_manifest.get("packages", {}),
                                                        "source": "upstream Panda mechanics with scenario tools" if franka else "authored synthetic URDF Cartesian/tool testbed"}, indent=2)+"\n")
        instances.append({"id": spec["id"], "manifest": f"./models/{model_id}/model.json", "joint_names": full.joint_names,
                          "q_indices": list(range(offset, offset+full.nq)), "dof": full.n, "floating_base": False,
                          "base_pose": {"position": full.base_pose[:3, 3].tolist(), "quaternion_xyzw": Rotation.from_matrix(full.base_pose[:3, :3]).as_quat().tolist()}})
        offset += full.nq
    object_index = len(names); names.append("panel")
    for row, sample in zip(positions, indices):
        row.append(trace.object_pose[sample, 0, :3].tolist())
    q = np.c_[trace.q[indices], trace.object_pose[indices, 0]]
    flags = {name: np.zeros(len(trace.time), bool) for name in ("point", "left_finger", "right_finger")}
    point_prefix = next(a["id"]+"_" for a, cap in zip(plan.case.data["arms"], plan.case.capabilities) if cap.kind == "P")
    for t, pair in zip(trace.contact_time, trace.contact_pair):
        if "panel_collision" not in pair:
            continue
        other = pair[0] if pair[1] == "panel_collision" else pair[1]
        sample = int(round(t/trace.metadata["timestep"]))
        if other.startswith(point_prefix): flags["point"][sample] = True
        if "left_finger" in other or "leftfinger" in other: flags["left_finger"][sample] = True
        if "right_finger" in other or "rightfinger" in other: flags["right_finger"][sample] = True
    sealed = []
    for sample in indices:
        events = [e for e in trace.metadata["seal_events"] if e["time"] <= trace.time[sample]]
        sealed.append(bool(events and events[-1]["event"] == "acquired"))
    data = {"id": dataset_id, "title": "Franka Panda × 3 · contracted contacts" if franka else "Three-arm contracted contacts", "engine": "mujoco_contact_recording",
            "report": report,
            "dof": len(trace.metadata["joint_names"]), "nq": q.shape[1], "nv": q.shape[1]-1, "floating_base": False,
            "frame": "panel", "joint_names": trace.metadata["joint_names"], "instances": instances,
            "times": trace.time[indices].round(6).tolist(), "positions": positions, "q": q.round(9).tolist(),
            "link_names": names, "edges": edges, "object": {"index": object_index, "size": plan.case.data["object"]["size"],
                                                                    "poses": trace.object_pose[indices, 0].tolist()},
            "torques": np.r_[trace.torque, trace.torque[-1:]][indices].tolist(), "errors": [None]*len(indices),
            "torque_ratio": np.max(abs(np.r_[trace.torque, trace.torque[-1:]][indices])/np.array(efforts), axis=1).tolist(),
            "keyframes": [{**k, "time": report["keyframe_errors"][i]["time"]} for i, k in enumerate(plan.case.data["keyframes"])],
            "events": [{"id": "vent_and_release", "time": trace.metadata["release_time"]}] if trace.metadata["release_time"] <= trace.time[-1] else [],
            "tool_states": {"point_contact": flags["point"][indices].tolist(),
                            "gripper_contact": (flags["left_finger"] & flags["right_finger"])[indices].tolist(), "suction_sealed": sealed},
            "environment": [{"type": "box", "size": [*(2*np.array(plan.case.data["simulation"]["fixture_half_size"])).tolist(), plan.case.data["simulation"]["fixture_thickness"]],
                             "position": [0, 0, plan.case.data["simulation"]["fixture_top"]-plan.case.data["simulation"]["fixture_thickness"]/2], "color": "#425469"}],
            "summary": {"duration": float(trace.time[-1]), "physics_dt": trace.metadata["timestep"],
                        "full_geometry_checked": False, "final_error": None,
                        "keyframe_errors": report["keyframe_errors"], "primal_residual": report["primal_residual"],
                        "seal_retained_before_vent": report["seal_retained_before_vent"], "force_envelopes_passed": report["force_envelopes_passed"]},
            "description": "Assumed contracted network contacts; three authored URDF Cartesian arms. Spherical point contact and a parallel-jaw gripper use physical MuJoCo contacts; suction uses a pressure-limited compliant seal. Three optimized keyframes and two CITO transitions are replayed by torque control. The panel moves under dynamics, then all tools release at 3.0 s. No trained network inference or vendor-arm/full-geometry certificate is claimed."}
    if franka:
        data["description"] = "Three real 7-DoF Panda URDF arms and upstream visual/collision meshes. IK prescribes a fixed-orientation contact path; IPOPT optimizes inertial reaction allocation and derived joint torques. Clarabel checks frozen keyframe wrench margins. MuJoCo integrates a free panel with sphere, native Panda fingers and pressure-limited suction. No trained network inference, full CITO or continuous collision certificate."
    (data_dir/(dataset_id+".json")).write_text(json.dumps(data, separators=(",", ":"), allow_nan=False))
    index_path = data_dir/"index.json"
    index = json.loads(index_path.read_text()) if index_path.exists() else {"models": []}
    index["models"] = [entry for entry in index["models"] if entry["id"] != dataset_id]
    index["models"].append({key: data[key] for key in ("id", "title", "engine", "dof", "floating_base")})
    index_path.write_text(json.dumps(index, indent=2)+"\n")
    return data
