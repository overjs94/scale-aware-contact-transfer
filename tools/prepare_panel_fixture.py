"""Author the single-rigid-panel main task without changing robot mechanics."""
from copy import deepcopy
import json,shutil
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
from contact_transfer.source_transfer import source_fixture
ROOT=Path(__file__).resolve().parents[1];DEST=ROOT/'src/contact_transfer/scenarios/panel_fixture'


def box_urdf(name,size,mass,color):
    robot=ET.Element('robot',name=name);link=ET.SubElement(robot,'link',name=name)
    if mass:
        x,y,z=size;I=mass/12*np.array([y*y+z*z,x*x+z*z,x*x+y*y]);e=ET.SubElement(link,'inertial');ET.SubElement(e,'mass',value=str(mass));ET.SubElement(e,'inertia',ixx=str(I[0]),iyy=str(I[1]),izz=str(I[2]),ixy='0',ixz='0',iyz='0')
    for kind in ['visual','collision']:
        e=ET.SubElement(link,kind);ET.SubElement(ET.SubElement(e,'geometry'),'box',size=' '.join(map(str,size)))
        if kind=='visual':ET.SubElement(ET.SubElement(e,'material',name=name+'_color'),'color',rgba=color)
    return robot


def main():
    DEST.mkdir(parents=True,exist_ok=True);old=json.loads((DEST/'case.json').read_text()) if (DEST/'case.json').exists() else {}
    base=ROOT/'src/contact_transfer/scenarios/table_assembly'
    for n in ['point_arm.urdf','gripper_arm.urdf','panda.srdf']:shutil.copyfile(base/n,DEST/n)
    for src,dst in [(ROOT/'web/models/franka_panda/LICENSE.upstream','LICENSE.panda'),(ROOT/'web/models/rb5/LICENSE.upstream','LICENSE.rb5'),(ROOT/'LICENSE','LICENSE.authored')]:shutil.copyfile(src,DEST/dst)
    for name in ['franka_panda','rb5']:(DEST/(name+'_provenance.json')).write_text((ROOT/'web/models'/name/'model.json').read_text())
    rb=ET.parse(ROOT/'web/models/rb5/robot.urdf').getroot();point=ET.parse(base/'point_arm.urdf').getroot()
    for link in point.findall('link'):
        if link.get('name') in {'tool','tip'}:rb.append(deepcopy(link))
    for joint in point.findall('joint'):
        if joint.get('name') in {'tool_mount','tip_frame'}:
            joint=deepcopy(joint)
            if joint.get('name')=='tool_mount':joint.find('parent').set('link','tcp')
            rb.append(joint)
    ET.indent(rb);ET.ElementTree(rb).write(DEST/'rb5_point.urdf',encoding='utf-8',xml_declaration=True)
    panel=box_urdf('panel',[.6,.4,.04],2.,'.72 .49 .27 1');ET.indent(panel);ET.ElementTree(panel).write(DEST/'panel.urdf',encoding='utf-8',xml_declaration=True)
    # Four independent seat pads leave the spherical support tools room to exit.
    fixture=ET.Element('robot',name='jig');link=ET.SubElement(fixture,'link',name='jig');pieces=[]
    for x in [-.22,.22]:
        for y in [-.14,.14]:
            pieces.append(([.045,.045,.01],[x+.04,y+.04,.455]))
            pieces.append(([.018,.018,.45],[x+.04,y+.04,.225]))
    for i,(size,pos) in enumerate(pieces):
        for kind in ['visual','collision']:
            e=ET.SubElement(link,kind,name='jig_'+kind+'_'+str(i));ET.SubElement(e,'origin',xyz=' '.join(map(str,pos)));ET.SubElement(ET.SubElement(e,'geometry'),'box',size=' '.join(map(str,size)))
            if kind=='visual':ET.SubElement(ET.SubElement(e,'material',name='jig_color'),'color',rgba='.22 .32 .43 1')
    ET.indent(fixture);ET.ElementTree(fixture).write(DEST/'jig.urdf',encoding='utf-8',xml_declaration=True)
    I=[0,0,0,1];tilt=Rotation.from_euler('y',12,degrees=True).as_quat().tolist()
    keys=[{'id':id,'time':t,'object_pose':p+quat} for id,t,p,quat in [
      ('grip_support',0,[0,0,.58],tilt),('grasp_settled',2,[0,0,.58],tilt),('lift',7,[0,0,.66],tilt),('reorient',12,[0,0,.66],I),
      ('above_jig',18,[.04,.04,.60],I),('seat',22,[.04,.04,.48],I),('load_transfer',25,[.04,.04,.48],I),('released',30,[.04,.04,.48],I)]]
    previous=json.loads((base/'case.json').read_text())['arms'];arms=[]
    for id,kind,pos,yaw,seed,group in [('edge_gripper','G',[-.65,0,0],0,previous[0]['q_initial'],'edge_grip'),
      ('support_left','P',[.05,-.70,0],np.pi/2,previous[1]['q_initial'],'under_left'),
      ('support_right','P',[.55,.50,0],np.arctan2(-.5,-.55),previous[1]['q_initial'],'under_right')]:
        a=dict(id=id,kind=kind,model='franka_panda',urdf='gripper_arm.urdf' if kind=='G' else 'point_arm.urdf',base_position=pos,base_rpy=[0,0,float(yaw)],tool_frame='tip',group_id=group,
               q_initial=seed,controller='torque',locked_joints={'panda_finger_joint1':.02,'panda_finger_joint2':.02} if kind=='G' else {},
               tool_quaternion_xyzw=[.5,.5,.5,.5] if kind=='G' else I,retract_world=[-.08,0,0] if kind=='G' else [0,-.04 if id=='support_left' else .04,-.055])
        prior=next((x for x in old.get('arms',[]) if x['id']==id),None)
        if prior:
            a['q_initial']=prior['q_initial'];a['tool_quaternion_xyzw']=prior['tool_quaternion_xyzw']
        arms.append(a)
    d=dict(id='panel_fixture',schema_version=1,units='SI',quaternion_order='xyzw',arms=arms,keyframes=keys,panel_urdf='panel.urdf',jig_urdf='jig.urdf',
           source_file='source_contacts.json',source_refinement=True,source_to_target_scale=4.,panel_mass=2.,
           optimization=dict(sample_dt=.25,point_mu=.8,point_normal_max=40,gripper_force_bounds=[20,20,30],gripper_moment_bounds=[.5,.5,.5],disturbance_scale=[.15,.15,.15,1,1,1]),
           simulation=dict(duration=30,timestep=.001,grasp_seconds=1.2,seat_time=22,load_transfer_end=25,release_time=26,release_seconds=.6,retract_end=29,
                           arm_kp=900,arm_kd=60,cartesian_kp=3000,cartesian_kd=40,orientation_kp=40,orientation_kd=2,finger_kp=3000,finger_kd=10,finger_open=.035,finger_closed=.017,
                           pad_friction=.9,pad_torsional_friction_m=.008,friction_impratio=100,position_kp=3000,position_kd=80,tracking_tolerance_m=.008,tracking_tolerance_rad=.06))
    (DEST/'case.json').write_text(json.dumps(d,indent=2)+'\n');(DEST/'source_contacts.json').write_text(json.dumps(source_fixture(),indent=2)+'\n')
if __name__=='__main__':main()
