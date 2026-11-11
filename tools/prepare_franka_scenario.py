"""Compose pinned Franka arm descriptions with point/suction tools and native jaws."""

from copy import deepcopy
import hashlib
import json
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]


def main():
    source = ROOT/"web/models/franka_panda"
    old = ROOT/"src/contact_transfer/scenarios/three_arm"
    dest = ROOT/"src/contact_transfer/scenarios/three_franka"
    dest.mkdir(parents=True, exist_ok=True)
    metadata = json.loads((source/"model.json").read_text())
    for name in ("point", "gripper", "suction"):
        robot = ET.parse(source/"robot.urdf").getroot()
        if name != "gripper":
            removed = {"panda_hand", "panda_hand_tcp", "panda_leftfinger", "panda_rightfinger"}
            for element in list(robot):
                if (element.tag == "link" and element.get("name") in removed
                        or element.tag == "joint" and element.find("child").get("link") in removed):
                    robot.remove(element)
            tool = ET.parse(old/(name+"_arm.urdf")).getroot()
            for link in tool.findall("link"):
                if link.get("name") in {"tool", "tip"}:
                    robot.append(deepcopy(link))
            for joint in tool.findall("joint"):
                if joint.get("name") in {"tool_mount", "tip_frame"}:
                    joint = deepcopy(joint)
                    if joint.get("name") == "tool_mount": joint.find("parent").set("link", "panda_link8")
                    robot.append(joint)
        else:
            # Keep the upstream finger geometry, limits and inertias; apply
            # symmetric commands to two independently simulated finger motors.
            for joint in robot.findall("joint"):
                for mimic in joint.findall("mimic"): joint.remove(mimic)
            ET.SubElement(robot, "link", name="tip")
            joint = ET.SubElement(robot, "joint", name="tip_frame", type="fixed")
            ET.SubElement(joint, "parent", link="panda_hand_tcp")
            ET.SubElement(joint, "child", link="tip")
        ET.indent(robot)
        notice = f'<!-- Modified pinned Franka URDF: {name} tool; independent gripper fingers. Upstream license retained. -->\n'
        (dest/(name+"_arm.urdf")).write_text('<?xml version="1.0"?>\n'+notice+ET.tostring(robot, encoding="unicode")+"\n")
    shutil.copyfile(source/"LICENSE.upstream", dest/"LICENSE.upstream")
    provenance = {**metadata, "modifications": ["point/suction: remove hand and attach authored tool at panda_link8",
                     "gripper: retain original hand and collision pads; remove mimic for symmetric independent finger control",
                     "preserve panda_link0..8 arm inertias, axes, limits and origins"],
                  "generated_sha256": {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in dest.glob("*.urdf")}}
    (dest/"provenance.json").write_text(json.dumps(provenance, indent=2)+"\n")


if __name__ == "__main__": main()
