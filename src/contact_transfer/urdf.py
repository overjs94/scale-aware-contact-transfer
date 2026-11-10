"""Validated URDF tree and independent-joint mapping. No System_Param table."""

from dataclasses import dataclass
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

from .lie import rpy_rotation


def numbers(s, default):
    result = np.asarray(default if s is None else [float(x) for x in s.split()])
    if result.shape != np.asarray(default).shape or not np.isfinite(result).all():
        raise ValueError(f"invalid URDF vector: {s}")
    return result.astype(float)


def origin(element):
    e = element.find("origin")
    if e is None:
        return np.eye(3), np.zeros(3)
    return rpy_rotation(numbers(e.get("rpy"), [0, 0, 0])), numbers(e.get("xyz"), [0, 0, 0])


@dataclass
class Link:
    name: str
    mass: float
    com: np.ndarray
    inertia: np.ndarray


@dataclass
class Joint:
    name: str
    kind: str
    parent: str
    child: str
    R: np.ndarray
    p: np.ndarray
    axis: np.ndarray
    lower: float
    upper: float
    effort: float
    velocity: float
    mimic: tuple | None = None


class URDFTree:
    def __init__(self, path, locked_joints=None):
        self.path = Path(path).resolve()
        self.xml = self.path.read_text()
        root = ET.fromstring(self.xml)
        if root.tag != "robot" or any("xacro" in e.tag for e in root.iter()):
            raise ValueError("Supply an expanded URDF, not a xacro source.")
        self.name = root.get("name", "robot")
        self.locked = dict(locked_joints or {})
        self.links = {}
        for e in root.findall("link"):
            name = e.attrib["name"]
            if name in self.links:
                raise ValueError(f"duplicate link: {name}")
            mass, com, I = 0., np.zeros(3), np.zeros((3, 3))
            inertial = e.find("inertial")
            if inertial is not None:
                R, com = origin(inertial)
                mass = float(inertial.find("mass").attrib["value"])
                a = inertial.find("inertia").attrib
                if not {"ixx", "ixy", "ixz", "iyy", "iyz", "izz"} <= set(a):
                    raise ValueError(f"URDF inertia requires all six tensor entries: {name}")
                I = np.array([[float(a["ixx"]), float(a.get("ixy", 0)), float(a.get("ixz", 0))],
                              [float(a.get("ixy", 0)), float(a["iyy"]), float(a.get("iyz", 0))],
                              [float(a.get("ixz", 0)), float(a.get("iyz", 0)), float(a["izz"])]])
                if mass < 0 or not np.isfinite(mass) or not np.isfinite(I).all():
                    raise ValueError(f"invalid inertia: {name}")
                eig = np.linalg.eigvalsh(I)
                if eig[0] < -1e-9 or eig[-1] > eig[:2].sum() + 1e-6:
                    raise ValueError(f"nonphysical rotational inertia: {name}")
                I = R @ I @ R.T
            self.links[name] = Link(name, mass, com, I)
        joints = {}
        child_names = set()
        for e in root.findall("joint"):
            kind, name = e.get("type"), e.attrib["name"]
            if kind not in {"fixed", "revolute", "continuous", "prismatic"}:
                raise ValueError(f"{name}: unsupported {kind}; select floating_base for root motion")
            parent, child = e.find("parent").attrib["link"], e.find("child").attrib["link"]
            if parent not in self.links or child not in self.links or child in child_names:
                raise ValueError(f"invalid tree at joint {name}")
            child_names.add(child)
            R, p = origin(e)
            axis_e = e.find("axis")
            axis = numbers(None if axis_e is None else axis_e.get("xyz"), [1, 0, 0])
            if kind != "fixed":
                if np.linalg.norm(axis) < 1e-12:
                    raise ValueError(f"zero joint axis: {name}")
                axis /= np.linalg.norm(axis)
            limit = e.find("limit")
            a = {} if limit is None else limit.attrib
            lo, hi = (-np.inf, np.inf) if kind == "continuous" else (
                float(a.get("lower", -np.inf)), float(a.get("upper", np.inf)))
            effort, velocity = float(a.get("effort", np.inf)), float(a.get("velocity", np.inf))
            if lo > hi or effort < 0 or velocity < 0:
                raise ValueError(f"invalid limits: {name}")
            mimic_e = e.find("mimic")
            mimic = None if mimic_e is None else (
                mimic_e.attrib["joint"], float(mimic_e.get("multiplier", 1)),
                float(mimic_e.get("offset", 0)))
            if name in joints:
                raise ValueError(f"duplicate joint: {name}")
            joints[name] = Joint(name, kind, parent, child, R, p, axis,
                                 lo, hi, effort, velocity, mimic)
        roots = set(self.links)-child_names
        if len(roots) != 1:
            raise ValueError("URDF must contain one connected tree")
        self.root = roots.pop()
        self.joints = []
        visited = {self.root}
        while len(visited) < len(self.links):
            ready = [j for j in joints.values() if j.parent in visited and j.child not in visited]
            if not ready:
                raise ValueError("disconnected or cyclic URDF")
            for j in ready:
                self.joints.append(j)
                visited.add(j.child)
        self.by_name = joints
        if not set(self.locked) <= set(joints):
            raise ValueError(f"unknown locked joints: {set(self.locked)-set(joints)}")
        self.joint_names = [j.name for j in self.joints
                            if j.kind != "fixed" and j.mimic is None and j.name not in self.locked]
        self.n = len(self.joint_names)
        self.mapping = {}

        def resolve(name, seen=()):
            if name in self.mapping:
                return self.mapping[name]
            if name in seen or name not in joints:
                raise ValueError(f"invalid mimic chain: {name}")
            j = joints[name]
            a = np.zeros(self.n)
            b = 0.
            if name in self.locked:
                b = float(self.locked[name])
            elif j.kind == "fixed":
                pass
            elif j.mimic is not None:
                target, multiplier, offset = j.mimic
                ar, br = resolve(target, (*seen, name))
                a, b = multiplier*ar, multiplier*br+offset
            else:
                a[self.joint_names.index(name)] = 1.
            self.mapping[name] = (a, b)
            return a, b

        for j in self.joints:
            resolve(j.name)
        self.lower = np.full(self.n, -np.inf)
        self.upper = np.full(self.n, np.inf)
        self.velocity = np.full(self.n, np.inf)
        self.effort = np.array([joints[x].effort for x in self.joint_names])
        for j in self.joints:
            a, b = self.mapping[j.name]
            ix = np.flatnonzero(a)
            if ix.size:
                i, gain = int(ix[0]), float(a[ix[0]])
                lo, hi = sorted(((j.lower-b)/gain, (j.upper-b)/gain))
                self.lower[i], self.upper[i] = max(self.lower[i], lo), min(self.upper[i], hi)
                self.velocity[i] = min(self.velocity[i], j.velocity/abs(gain))
            elif j.kind != "fixed" and not j.lower-1e-10 <= b <= j.upper+1e-10:
                raise ValueError(f"locked joint outside limits: {j.name}")
        if np.any(self.lower > self.upper):
            raise ValueError("inconsistent mimic/locked joint bounds")

    def neutral(self):
        return np.maximum(self.lower, np.minimum(self.upper, np.zeros(self.n)))
