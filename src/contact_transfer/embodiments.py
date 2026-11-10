"""Offline model catalog; every preset retains its upstream revision/license."""

from importlib.resources import files
import json
from pathlib import Path

from .robot import Robot


def catalog():
    base = files("contact_transfer").joinpath("models")
    return {p.name: json.loads(p.joinpath("model.json").read_text())
            for p in base.iterdir() if p.is_dir() and p.joinpath("model.json").is_file()}


def load(name, *, symbolic=False, **overrides):
    entries = catalog()
    if name not in entries:
        raise ValueError(f"unknown embodiment {name}; available: {', '.join(sorted(entries))}")
    data = entries[name]
    kwargs = {"floating_base": data["floating_base"], "locked_joints": data["locked_joints"]}
    kwargs.update(overrides)
    path = Path(str(files("contact_transfer").joinpath("models", name, "robot.urdf")))
    robot = Robot(path, **kwargs)
    return robot.symbolic() if symbolic else robot
