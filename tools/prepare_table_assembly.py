"""Author furniture assets; Panda mechanics are copied unchanged from pinned URDFs."""
import json, shutil
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[1]
DEST=ROOT/'src/contact_transfer/scenarios/table_assembly'

def rigid_urdf(name,boxes,masses,color):
    root=ET.Element('robot',name=name);link=ET.SubElement(root,'link',name=name)
    centers=np.array([b[1] for b in boxes]);masses=np.array(masses);mass=masses.sum();com=(centers*masses[:,None]).sum(0)/mass
    I=np.zeros((3,3))
    for (size,center),m in zip(boxes,masses):
        x,y,z=size;r=np.array(center)-com
        I+=np.diag(m/12*np.array([y*y+z*z,x*x+z*z,x*x+y*y]))+m*(r@r*np.eye(3)-np.outer(r,r))
    inertial=ET.SubElement(link,'inertial');ET.SubElement(inertial,'origin',xyz=' '.join(map(str,com)))
    ET.SubElement(inertial,'mass',value=str(mass));ET.SubElement(inertial,'inertia',**{k:str(v) for k,v in zip(['ixx','ixy','ixz','iyy','iyz','izz'],[I[0,0],I[0,1],I[0,2],I[1,1],I[1,2],I[2,2]])})
    for i,(size,center) in enumerate(boxes):
        for kind in ['visual','collision']:
            e=ET.SubElement(link,kind,name=f'{name}_{kind}_{i}');ET.SubElement(e,'origin',xyz=' '.join(map(str,center)))
            ET.SubElement(ET.SubElement(e,'geometry'),'box',size=' '.join(map(str,size)))
            if kind=='visual':ET.SubElement(ET.SubElement(e,'material',name=name+'_material'),'color',rgba=color)
    ET.indent(root);ET.ElementTree(root).write(DEST/(name+'.urdf'),encoding='utf-8',xml_declaration=True)

def main():
    existing=json.loads((DEST/'case.json').read_text()) if (DEST/'case.json').exists() else {}
    DEST.mkdir(parents=True,exist_ok=True);source=ROOT/'src/contact_transfer/scenarios/three_franka'
    for n in ['point_arm.urdf','gripper_arm.urdf','panda.srdf','provenance.json','LICENSE.upstream']:shutil.copyfile(source/n,DEST/n)
    # 600 x 400 x 40 mm top with a blind square socket. Rectangular pieces
    # surround an actual empty cavity, rather than overlapping a solid slab.
    hx,hy,hz=.30,.20,.02;sx,sy=.18,.108;hole=.021;bottom=-.01
    boxes=[]
    for x0,x1,y0,y1,z0,z1 in [(-hx,sx-hole/2,-hy,hy,-hz,hz),(sx+hole/2,hx,-hy,hy,-hz,hz),
                             (sx-hole/2,sx+hole/2,-hy,sy-hole/2,-hz,hz),(sx-hole/2,sx+hole/2,sy+hole/2,hy,-hz,hz),
                             (sx-hole/2,sx+hole/2,sy-hole/2,sy+hole/2,-hz,bottom)]:
        boxes.append(([x1-x0,y1-y0,z1-z0],[(x0+x1)/2,(y0+y1)/2,(z0+z1)/2]))
    volumes=np.array([np.prod(b[0]) for b in boxes]);rigid_urdf('tabletop',boxes,2.*volumes/volumes.sum(),'.72 .49 .27 1')
    rigid_urdf('leg',[([.04,.04,.28],[0,0,0]),([.018,.018,.025],[0,0,-.1525])],[.39,.01],'.42 .27 .16 1')
    old=json.loads((source/'case.json').read_text());old_by={a['id']:a for a in old['arms']}
    Rg=Rotation.from_quat([.5,.5,.5,.5]).as_quat().tolist()
    Rleg=Rotation.from_matrix([[0,-1,0],[0,0,-1],[1,0,0]]).as_quat().tolist()
    arms=[]
    for id,kind,part,base,point,normal,quat,seed in [
        ('top_gripper','G','tabletop',[-.65,-.18,0],[-.30,-.18,0],[-1,0,0],Rg,old_by['gripper']['q_initial']),
        ('point_support','P','tabletop',[.05,-.70,0],[.10,.06,-.02],[0,0,-1],[0,0,0,1],old_by['point']['q_initial']),
        ('leg_gripper','G','leg',[.55,.50,0],[0,0,-.04],[0,1,0],Rleg,old_by['gripper']['q_initial'])]:
        arms.append(dict(id=id,kind=kind,object=part,urdf='gripper_arm.urdf' if kind=='G' else 'point_arm.urdf',
                         base_position=base,base_rpy=[0,0,float(np.arctan2(-base[1],-base[0]))],tool_frame='tip',
                         point_object=point,normal_object=normal,tool_quaternion_xyzw=quat,q_initial=seed,
                         locked_joints={'panda_finger_joint1':.02,'panda_finger_joint2':.02} if kind=='G' else {},
                         tool_collision_links=['panda_hand','panda_leftfinger','panda_rightfinger'] if kind=='G' else ['tool']))
    tilted=Rotation.from_euler('y',12,degrees=True).as_quat().tolist();I=[0,0,0,1]
    keys=[]
    for id,t,tp,lr in [('grip_and_support',0,[0,0,.58]+tilted,[.18,.34,.79]+I),
                       ('grasp_settled',2,[0,0,.58]+tilted,[.18,.34,.79]+I),
                       ('lift_tabletop',6,[0,0,.66]+tilted,[.18,.34,.90]+I),
                       ('reorient_tabletop',12,[0,0,.66]+I,[.18,.34,.90]+I),
                       ('approach_socket',18,[0,0,.66]+I,[.18,.158,.90]+I),
                       ('align_leg',22,[0,0,.66]+I,[.18,.108,.89]+I),
                       ('insert_leg',27,[0,0,.66]+I,[.18,.108,.82]+I),
                       ('assembled_and_released',30,[0,0,.66]+I,[.18,.108,.82]+I)]:keys.append(dict(id=id,time=t,tabletop=tp,leg=lr))
    d=dict(schema_version=1,id='table_assembly',units='SI',model_family='franka_panda',quaternion_order='xyzw',
           prediction_source='assumed_contracted_contacts',arms=arms,objects=[dict(id=n,urdf=n+'.urdf') for n in ['tabletop','leg']],keyframes=keys,
           socket=dict(center_object=[sx,sy,.02],width=hole,depth=.03,peg_width=.018,peg_length=.025),
           optimization=dict(sample_dt=.25,gripper_moment_bounds=[.5,.5,.5],gripper_force_bounds=[20,20,30],point_mu=.8,point_normal_max=40),
           simulation=dict(timestep=.002,duration=30,grasp_seconds=1.2,release_time=28,release_seconds=.6,retract_distance=.08,
                           seat_time=27,arm_kp=900,arm_kd=60,finger_kp=3000,finger_kd=12,finger_open=.035,finger_closed=.017,
                           tracking_tolerance_m=.008,tracking_tolerance_rad=.06,insertion_tolerance_m=.002))
    d['simulation'].update(timestep=.001,finger_kd=10,cartesian_kp=3000,cartesian_kd=40,orientation_kp=40,orientation_kd=2,pad_friction=.9,pad_torsional_friction_m=.008,friction_impratio=100)
    d['socket'].update(peg_tip_object=[0,0,-.165],leg_shoulder_object=[0,0,-.14])
    for arm in d['arms']:
        old_arm=next((a for a in existing.get('arms',[]) if a['id']==arm['id']),None)
        if old_arm:arm['q_initial']=old_arm['q_initial']
    shutil.copyfile(ROOT/'LICENSE',DEST/'LICENSE.authored')
    provenance=json.loads((DEST/'provenance.json').read_text())
    provenance['modifications']=['preserve upstream Panda arm mechanics and limits','retain independent native Panda hand fingers on two arms','replace hand with authored spherical tool on the support arm','author two rigid furniture URDFs with an empty blind socket and a separate peg']
    (DEST/'provenance.json').write_text(json.dumps(provenance,indent=2)+'\n')
    (DEST/'case.json').write_text(json.dumps(d,indent=2)+'\n')
if __name__=='__main__':main()
