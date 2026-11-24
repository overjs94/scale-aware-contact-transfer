"""Portable execution traces for MuJoCo and external CRISP C++ adapters."""

from dataclasses import dataclass, field
import json

import numpy as np
from scipy.spatial.transform import Rotation


TRACE_CONVENTIONS = {"format_version": 1, "units": "SI",
                     "pose_convention": "world_xyz_xyzw",
                     "contact_wrench_convention": "world_moment_force_on_second_geom_at_contact"}


@dataclass
class ExecutionTrace:
    """SI units, world poses xyz+xyzw; contact wrench [moment;force] in world.

    Wrenches act on contact pair's second geom, about contact_position. Signed
    gap is positive at separation. No solver/contact certificate is implied.
    """
    metadata: dict
    time: np.ndarray
    q: np.ndarray
    q_reference: np.ndarray
    torque: np.ndarray
    object_pose: np.ndarray
    contact_time: np.ndarray = field(default_factory=lambda: np.zeros(0))
    contact_pair: np.ndarray = field(default_factory=lambda: np.empty((0, 2), dtype=str))
    contact_position: np.ndarray = field(default_factory=lambda: np.empty((0, 3)))
    contact_normal: np.ndarray = field(default_factory=lambda: np.empty((0, 3)))
    contact_gap: np.ndarray = field(default_factory=lambda: np.zeros(0))
    contact_wrench: np.ndarray = field(default_factory=lambda: np.empty((0, 6)))

    def validate(self):
        for key, expected in TRACE_CONVENTIONS.items():
            if self.metadata.get(key) != expected:
                raise ValueError(f"unsupported trace convention: {key}")
        for key in ("engine", "engine_version", "case_id", "plan_id", "timestep",
                    "contact_settings", "joint_names", "object_names"):
            if key not in self.metadata:
                raise ValueError(f"missing execution metadata: {key}")
        if any(not str(self.metadata[k]).strip() for k in ("engine", "engine_version", "case_id", "plan_id")):
            raise ValueError("execution identity must be nonempty")
        if not np.isfinite(self.metadata["timestep"]) or self.metadata["timestep"] <= 0:
            raise ValueError("timestep must be positive and finite")
        n, joints, objects = len(self.time), len(self.metadata["joint_names"]), len(self.metadata["object_names"])
        c = len(self.contact_time)
        shapes = {"time": (n,), "q": (n, joints), "q_reference": (n, joints),
                  "torque": (n-1, joints), "object_pose": (n, objects, 7),
                  "contact_time": (c,), "contact_pair": (c, 2),
                  "contact_position": (c, 3), "contact_normal": (c, 3),
                  "contact_gap": (c,), "contact_wrench": (c, 6)}
        if n < 2 or not objects or np.any(np.diff(self.time) <= 0):
            raise ValueError("trace requires increasing time and at least one object")
        for name, shape in shapes.items():
            value = np.asarray(getattr(self, name))
            if value.shape != shape or (name != "contact_pair" and not np.isfinite(value).all()):
                raise ValueError(f"invalid trace array: {name}")
        if not np.allclose(np.linalg.norm(self.object_pose[:, :, 3:], axis=2), 1, atol=1e-6):
            raise ValueError("object quaternions must be unit xyzw")
        if c and (not np.allclose(np.linalg.norm(self.contact_normal, axis=1), 1, atol=1e-6)
                  or np.any(np.diff(self.contact_time) < 0)
                  or self.contact_time[0] < self.time[0] or self.contact_time[-1] > self.time[-1]):
            raise ValueError("invalid contact normals/timestamps")
        json.dumps(self.metadata, allow_nan=False)
        return self

    def save(self, path):
        self.validate()
        arrays = {name: getattr(self, name) for name in self.__dataclass_fields__ if name != "metadata"}
        np.savez_compressed(path, metadata_json=json.dumps(self.metadata, allow_nan=False), **arrays)

    @classmethod
    def load(cls, path):
        with np.load(path, allow_pickle=False) as data:
            result = cls(json.loads(str(data["metadata_json"])),
                         **{name: data[name] for name in cls.__dataclass_fields__ if name != "metadata"})
        return result.validate()


def compare_executions(first, second):
    """Compare matched endpoints; this does not classify insertion success/jams."""
    first.validate(); second.validate()
    for key in ("case_id", "plan_id", "object_names", "joint_names"):
        if first.metadata[key] != second.metadata[key]:
            raise ValueError(f"cannot compare different {key}")
    if not np.allclose(first.time[[0, -1]], second.time[[0, -1]], atol=1e-9, rtol=0):
        raise ValueError("execution intervals differ")
    if not np.allclose(first.object_pose[0, :, :3], second.object_pose[0, :, :3], atol=1e-8, rtol=0):
        raise ValueError("initial object positions differ")
    if not np.allclose(first.q[0], second.q[0], atol=1e-8, rtol=0):
        raise ValueError("initial robot postures differ")
    initial_angle = (Rotation.from_quat(first.object_pose[0, :, 3:]).inv()
                     *Rotation.from_quat(second.object_pose[0, :, 3:])).magnitude()
    if np.any(initial_angle > 1e-8):
        raise ValueError("initial object orientations differ")
    p, r = first.object_pose[-1], second.object_pose[-1]
    angles = (Rotation.from_quat(p[:, 3:]).inv()*Rotation.from_quat(r[:, 3:])).magnitude()
    return {"object_names": first.metadata["object_names"],
            "final_translation_difference_m": np.linalg.norm(p[:, :3]-r[:, :3], axis=1).tolist(),
            "final_rotation_difference_rad": angles.tolist(),
            "tracking_error_max": [float(np.max(abs(t.q-t.q_reference), initial=0)) for t in (first, second)],
            "contact_samples": [len(t.contact_time) for t in (first, second)]}
