"""One authoritative sphere/stem/TCP assembly for every full-scenario P arm."""
from copy import deepcopy
import hashlib
from importlib.resources import files
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from .robot import Robot


def canonical_path():
    return Path(str(files('contact_transfer').joinpath('models/tools/spherical_tip.urdf')))


def specification():
    """Read geometry, inertia and EE bias from URDF, never robot tables."""
    path=canonical_path();robot=Robot(path);root=ET.parse(path).getroot()
    return dict(id='shared_spherical_tip',urdf_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                radius_m=float(root.find("link[@name='tool']/collision/geometry/sphere").get('radius')),
                ee_to_tip=robot.fk(robot.neutral(),'tip').tolist(),
                ee_to_sphere_center=robot.fk(robot.neutral(),'tool').tolist(),
                mass_kg=sum(link.mass for link in robot.tree.links.values()),
                contact_point='leading sphere surface, local +Z',license='MIT')


def mount(arm,destination,*,ee_frame='panda_link8'):
    """Replace descendants of the declared EE by the exact same tool subtree.

    `arm` is a numerical Robot. Its robot joints, base pose and inertias are
    preserved. Surface-normal alignment is a task pose, not a mount offset.
    """
    if ee_frame not in arm.tree.links:raise ValueError('Unknown EE frame '+ee_frame)
    root=ET.parse(arm.path).getroot();removed=set();frontier={ee_frame}
    while frontier:
        children={j.find('child').get('link') for j in root.findall('joint') if j.find('parent').get('link') in frontier}
        removed|=children;frontier=children
    for element in list(root):
        if element.tag=='link' and element.get('name') in removed or element.tag=='joint' and element.find('child').get('link') in removed:root.remove(element)
    tool=ET.parse(canonical_path()).getroot()
    for element in tool:
        if element.tag=='link' and element.get('name')!='tool_flange':
            if root.find(f"link[@name='{element.get('name')}']") is not None:raise ValueError('Tool link name collides with retained arm')
            root.append(deepcopy(element))
        elif element.tag=='joint':
            joint=deepcopy(element)
            if joint.find('parent').get('link')=='tool_flange':joint.find('parent').set('link',ee_frame)
            root.append(joint)
    out=Path(destination);out.parent.mkdir(parents=True,exist_ok=True);ET.indent(root);out.write_bytes(ET.tostring(root,encoding='utf-8',xml_declaration=True))
    robot=Robot(out,base_pose=arm.base_pose);q=robot.neutral();bias=np.linalg.inv(robot.fk(q,ee_frame))@robot.fk(q,'tip');spec=specification()
    if not np.allclose(bias,spec['ee_to_tip'],atol=1e-12):raise RuntimeError('Shared sphere EE bias was not preserved')
    spec.update(ee_frame=ee_frame,prepared_urdf_sha256=hashlib.sha256(out.read_bytes()).hexdigest())
    out.with_suffix('.tool.json').write_text(json.dumps(spec,indent=2)+'\n');return robot,spec
