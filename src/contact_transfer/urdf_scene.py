"""Add URDF inertias, primitives and resolved collision meshes to MJCF."""

from copy import deepcopy
import xml.etree.ElementTree as ET

import numpy as np

from .simulation import export_mjcf, _quat, _text
from .urdf import origin


def append_urdf_robot(world, actuators, robot, prefix, *, mesh_files=None, assets=None, robot_collisions=False):
    """Use the full URDF; joint dimensions/inertias never live in Python tables.

    Mesh paths must be explicitly resolved by the caller. The simulator uses
    collision STLs; the web viewer loads original visual meshes separately.
    """
    tree = ET.fromstring(export_mjcf(robot, markers=False))
    base = deepcopy(tree.find("worldbody/body"))
    for e in base.iter():
        if e.get("name"):
            e.set("name", prefix+e.get("name"))
    world.append(base)
    for actuator in tree.find("actuator"):
        e = deepcopy(actuator)
        e.set("name", prefix+e.get("name")); e.set("joint", prefix+e.get("joint"))
        actuators.append(e)
    bodies = {e.get("name"): e for e in base.iter("body")}
    source = ET.fromstring(robot.tree.xml)
    collisions = []
    for link in source.findall("link"):
        name = link.get("name")
        body = bodies[prefix+("urdf_"+name if name == robot.tree.root else name)]
        for kind in ("visual", "collision"):
            for i, element in enumerate(link.findall(kind)):
                R, p = origin(element)
                geometry = list(element.find("geometry"))
                if len(geometry) != 1:
                    raise ValueError("one primitive per URDF geometry required")
                shape = geometry[0]
                if kind == "visual" and shape.tag == "mesh" and mesh_files is not None:
                    # MuJoCo supports the upstream collision STL, while the web
                    # viewer retains the original visual DAE/materials.
                    continue
                attrs = {"name": prefix+name+"_"+kind+"_"+str(i), "pos": _text(p), "quat": _text(_quat(R)), "mass": "0"}
                if shape.tag == "box":
                    attrs.update(type="box", size=_text(np.array(shape.get("size").split(), float)/2))
                elif shape.tag == "sphere":
                    attrs.update(type="sphere", size=shape.get("radius"))
                elif shape.tag == "cylinder":
                    attrs.update(type="cylinder", size=_text([float(shape.get("radius")), float(shape.get("length"))/2]))
                elif shape.tag == "mesh" and mesh_files is not None and assets is not None:
                    filename = shape.get("filename").rsplit("/", 1)[-1]
                    if filename not in mesh_files:
                        raise ValueError(f"missing collision mesh: {filename}; run tools/fetch_panda_meshes.py")
                    mesh_name = prefix+name+"_mesh_"+str(i)
                    ET.SubElement(assets, "mesh", name=mesh_name, file=str(mesh_files[filename]), scale=shape.get("scale", "1 1 1"))
                    attrs.update(type="mesh", mesh=mesh_name)
                else:
                    raise ValueError("Cartesian testbed importer supports box/sphere/cylinder; provide MJCF for vendor meshes")
                if kind == "visual":
                    color = element.find("material/color")
                    attrs.update(contype="0", conaffinity="0", rgba=".4 .6 .8 1" if color is None else color.get("rgba"))
                else:
                    attrs.update(contype="2", conaffinity="3" if robot_collisions else "1", rgba=".75 .8 .85 1" if shape.tag == "mesh" else "0 0 0 0", condim="3",
                                 friction=".8 0 0", solref=".006 1", solimp=".95 .99 .001")
                    collisions.append(attrs["name"])
                ET.SubElement(body, "geom", **attrs)
    return collisions
