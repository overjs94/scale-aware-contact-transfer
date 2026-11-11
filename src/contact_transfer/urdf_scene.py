"""Add URDF inertias and primitive visual/collision shapes to an MJCF scene."""

from copy import deepcopy
import xml.etree.ElementTree as ET

import numpy as np

from .simulation import export_mjcf, _quat, _text
from .urdf import origin


def append_urdf_robot(world, actuators, robot, prefix):
    """Use the full URDF; joint dimensions/inertias never live in Python tables.

    This primitive importer is for the authored Cartesian/tool testbed. General
    vendor meshes still require the separate collision-enabled scene workflow.
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
                attrs = {"name": prefix+name+"_"+kind+"_"+str(i), "pos": _text(p), "quat": _text(_quat(R)), "mass": "0"}
                if shape.tag == "box":
                    attrs.update(type="box", size=_text(np.array(shape.get("size").split(), float)/2))
                elif shape.tag == "sphere":
                    attrs.update(type="sphere", size=shape.get("radius"))
                elif shape.tag == "cylinder":
                    attrs.update(type="cylinder", size=_text([float(shape.get("radius")), float(shape.get("length"))/2]))
                else:
                    raise ValueError("Cartesian testbed importer supports box/sphere/cylinder; provide MJCF for vendor meshes")
                if kind == "visual":
                    color = element.find("material/color")
                    attrs.update(contype="0", conaffinity="0", rgba=".4 .6 .8 1" if color is None else color.get("rgba"))
                else:
                    attrs.update(contype="2", conaffinity="1", rgba="0 0 0 0", condim="3",
                                 friction=".8 0 0", solref=".006 1", solimp=".95 .99 .001")
                    collisions.append(attrs["name"])
                ET.SubElement(body, "geom", **attrs)
    return collisions
