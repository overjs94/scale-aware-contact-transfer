"""Adapt pinned official RBY1-A URDF to standard geometry without changing mechanics."""
from copy import deepcopy
import hashlib,json,shutil
from pathlib import Path
import xml.etree.ElementTree as E
import numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[1];DEST=ROOT/'src/contact_transfer/scenarios/mobile_panel'
SHA='300a585c297d1bdf8554312c3a1659610a9823f5';RAW=f'https://raw.githubusercontent.com/RainbowRobotics/rby1-sdk/{SHA}/'

def main():
    path=DEST/'rby1.urdf';root=E.parse(path).getroot();upstream=path.read_bytes()
    if root.get('name')!='RBY1_A_v1.0':raise ValueError('Expected pinned official upstream URDF')
    # The vendor custom capsules are broad proxies that overlap end tools.
    # Use convex hulls of official per-link OBJ geometry for arm/torso collision.
    collision_assets=json.loads((DEST/'collision_assets.json').read_text())
    for link in root.findall('link'):
        for c in list(link.findall('collision')):link.remove(c)
        visual=link.find('visual/geometry/mesh')
        if visual is None:continue
        stem=Path(visual.get('filename')).stem
        for asset in collision_assets:
            if '/'+stem+'/' not in asset:continue
            c=E.SubElement(link,'collision');E.SubElement(E.SubElement(c,'geometry'),'mesh',filename=RAW+asset)
    for m in root.findall('.//visual/geometry/mesh'):m.set('filename',RAW+'models/rby1a/urdf/'+m.get('filename').removeprefix('./'))
    for j in root.findall('joint'):
        if j.get('name') in {'right_wheel','left_wheel'}:j.find('limit').set('effort','40') # declared simulation limit; upstream has no wheel effort rating
    # Official convex chassis pieces include the passive floor support geometry.
    base=root.find("link[@name='base']")
    for i in range(16):
        c=E.SubElement(base,'collision');E.SubElement(E.SubElement(c,'geometry'),'mesh',filename=RAW+f'models/rby1a/mujoco/assets/base/base_collision_{i}.obj')
    mobile=root.find('mobile');track,radius=map(float,mobile.get('params').split())
    # Wheel axial extent is read from the upstream mesh by fetch_rby1_assets.py.
    bounds=json.loads((ROOT/'.cache/rby1/mesh_bounds.json').read_text());width=bounds['WHEEL.obj']['max'][1]-bounds['WHEEL.obj']['min'][1]
    for name in ['wheel_r','wheel_l']:
        l=root.find(f"link[@name='{name}']");c=E.SubElement(l,'collision');E.SubElement(c,'origin',rpy=f'{np.pi/2} 0 0');E.SubElement(E.SubElement(c,'geometry'),'cylinder',radius=str(radius),length=str(width))
    # Authored tools: one parallel-jaw pad gripper and one spherical support.
    # Native vendor EE links/inertias stay intact; pad mass is explicitly authored.
    for side in ['right','left']:
        parent='ee_'+side;tool='contact_'+side
        l=E.SubElement(root,'link',name=tool)
        inertial=E.SubElement(l,'inertial');E.SubElement(inertial,'mass',value='.08');E.SubElement(inertial,'inertia',ixx='0.00002',iyy='0.00002',izz='0.00002',ixy='0',ixz='0',iyz='0')
        # Tool z points opposite vendor hand's -z extension.
        j=E.SubElement(root,'joint',name=tool+'_mount',type='fixed');E.SubElement(j,'parent',link=parent);E.SubElement(j,'child',link=tool);E.SubElement(j,'origin',xyz='0 0 -.12',rpy=f'{np.pi} 0 0')
        if side=='left':
            for kind in ['visual','collision']:E.SubElement(E.SubElement(E.SubElement(l,kind),'geometry'),'sphere',radius='.015')
            # tip frame is the top point of the sphere along local z.
            E.SubElement(root,'link',name='tip_left');j=E.SubElement(root,'joint',name='tip_left_mount',type='fixed');E.SubElement(j,'parent',link=tool);E.SubElement(j,'child',link='tip_left');E.SubElement(j,'origin',xyz='0 0 .015')
        else:
            # Independent authored jaws mounted on the hand, squeezing 40 mm panel.
            for side_j,sign in [('negative',-1),('positive',1)]:
                name='pad_'+side_j;lpad=E.SubElement(root,'link',name=name);inertial=E.SubElement(lpad,'inertial');E.SubElement(inertial,'mass',value='.025');E.SubElement(inertial,'inertia',ixx='.000004',iyy='.000004',izz='.000004',ixy='0',ixz='0',iyz='0')
                for kind in ['visual','collision']:E.SubElement(E.SubElement(E.SubElement(lpad,kind),'geometry'),'box',size='.035 .012 .05')
                j=E.SubElement(root,'joint',name=name+'_joint',type='prismatic');E.SubElement(j,'parent',link=tool);E.SubElement(j,'child',link=name);E.SubElement(j,'origin',xyz=f'0 {sign*.026} 0');E.SubElement(j,'axis',xyz=f'0 {sign} 0');E.SubElement(j,'limit',lower='-.004',upper='.03',effort='50',velocity='.1')
            E.SubElement(root,'link',name='tip_right');j=E.SubElement(root,'joint',name='tip_right_mount',type='fixed');E.SubElement(j,'parent',link=tool);E.SubElement(j,'child',link='tip_right')
    E.indent(root);E.ElementTree(root).write(DEST/'rby1_tools.urdf',encoding='utf-8',xml_declaration=True)
    meta=dict(id='rby1a_mobile',upstream_repository='https://github.com/RainbowRobotics/rby1-sdk',upstream_commit=SHA,upstream_path='models/rby1a/urdf/model.urdf',upstream_sha256=hashlib.sha256(upstream).hexdigest(),license='Apache-2.0',license_file='LICENSE.rby1',notice_file='NOTICE.rby1',wheel_effort_upstream_specified=False,wheel_effort_simulation_Nm=40,changes=['custom capsule proxies replaced by convex hulls of official per-link OBJ geometry; inertias/joint geometry preserved','40 Nm wheel effort is an authored simulation bound, not a manufacturer rating','official chassis convex collision meshes and wheel cylinders from official mobile/mesh geometry','authored gripper pads and spherical support; native hands remain in model'])
    (DEST/'provenance.json').write_text(json.dumps(meta,indent=2)+'\n')
    panel=ROOT/'src/contact_transfer/scenarios/panel_fixture'
    for n in ['panel.urdf','point_arm.urdf','panda.srdf','LICENSE.panda','LICENSE.authored','franka_panda_provenance.json']:shutil.copyfile(panel/n,DEST/n)
    if not (DEST/'source_contacts.json').exists():
        source=json.loads((panel/'source_contacts.json').read_text());source['origin']='synthetic_mobile_handover_fixture';source['groups'][2]['position'][1]=.01
        (DEST/'source_contacts.json').write_text(json.dumps(source,indent=2)+'\n')
if __name__=='__main__':main()
