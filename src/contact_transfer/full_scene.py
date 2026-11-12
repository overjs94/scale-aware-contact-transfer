"""Native scenes and candidate contacts for the three complete tasks."""
from dataclasses import dataclass
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
from .robot import Robot
from .contacts import Capability
from .trajectory import Contact
from .multi_object import RigidCollection,BodyContact
from .urdf_scene import append_urdf_robot
from .simulation import _text,_quat
from .urdf import origin


def apply_gripper(root,world,actuators,contact,robot,prefix,profile):
    """Preserve native loops/passive properties and a finite common PD servo."""
    if not profile:return
    for node in world.iter('joint'):
        name=node.get('name','').removeprefix(prefix)
        if name in profile['properties']:
            for k,v in profile['properties'][name].items():node.set(k,str(v))
    for motor in list(actuators):
        name=motor.get('joint','').removeprefix(prefix)
        if not motor.get('name','').startswith(prefix) or name not in profile['drivers']:continue
        j=robot.joint_names.index(name);attrs=dict(motor.attrib);actuators.remove(motor);weight=profile['drivers'][name];attrs.update(kp=str(weight*profile['servo_kp']),kv=str(weight*profile['servo_kd']),ctrlrange=_text([robot.lower[j],robot.upper[j]]),forcelimited='true',forcerange=_text([-robot.effort[j],robot.effort[j]]));ET.SubElement(actuators,'position',**attrs)
    equality=root.find('equality')
    if equality is None:equality=ET.SubElement(root,'equality')
    for entry in profile['equalities']:
        attrs=dict(entry['attributes'])
        for key in ['body1','body2','joint1','joint2']:
            if key in attrs:attrs[key]=prefix+attrs[key]
        ET.SubElement(equality,entry['type'],**attrs)
    for entry in profile['excludes']:ET.SubElement(contact,'exclude',**{k:prefix+v for k,v in entry.items()})


@dataclass
class FullScene:
    path: Path
    object_roots: dict
    object_frames: dict
    object_joints: dict
    object_geometries: dict
    allowed_by_group: dict
    fixtures: tuple


def build_scene(problem,path,*,timestep=.001,contacts_enabled=True):
    task=problem.task_definition;root=ET.Element('mujoco',model='full_'+task.id);ET.SubElement(root,'compiler',angle='radian',inertiafromgeom='false',fusestatic='false');ET.SubElement(root,'option',timestep=str(timestep),gravity='0 0 -9.81',integrator='implicitfast',cone='elliptic',impratio='100',iterations='100',tolerance='1e-10');assets=ET.SubElement(root,'asset');world=ET.SubElement(root,'worldbody');act=ET.SubElement(root,'actuator');contact=ET.SubElement(root,'contact')
    meshes={p.name:p.resolve() for directory in ['.cache/panda_meshes','.cache/robotiq_2f85/assets'] for p in Path(directory).glob('*.stl')};robot_geoms=[]
    for robot,spec in zip(problem.full_robots,problem.specs):
        prefix=spec['id']+'_';robot_geoms.append(append_urdf_robot(world,act,robot,prefix,mesh_files=meshes,assets=assets,robot_collisions=True));apply_gripper(root,world,act,contact,robot,prefix,spec.get('gripper_profile'))
        for pair in ET.parse(task.original.path.parent/'panda.srdf').findall('disable_collisions'):
            a,b=pair.get('link1'),pair.get('link2')
            if a in robot.tree.links and b in robot.tree.links:ET.SubElement(contact,'exclude',body1=prefix+('urdf_'+a if a==robot.tree.root else a),body2=prefix+('urdf_'+b if b==robot.tree.root else b))
        if robot.base_pose[2,3]>0:
            z=robot.base_pose[2,3];ET.SubElement(world,'geom',name=prefix+'pedestal',type='box',pos=_text([*robot.base_pose[:2,3],z/2]),size=_text([.15,.15,z/2]),contype='1',conaffinity='3')
    roots={};frames={};joints={};geometries={};qo=task.keyframes[0]['object_q']
    components=task.object.bodies if isinstance(task.object,RigidCollection) else {'object':task.object}
    for name,obj in components.items():
        prefix=name+'_';geoms=append_urdf_robot(world,act,obj,prefix,robot_collisions=True);body_name=prefix+'urdf_'+obj.tree.root;body=world.find(f"body[@name='{body_name}']");pose=qo[task.object.q_slices[name]][:7] if isinstance(task.object,RigidCollection) else qo[:7];body.set('pos',_text(pose[:3]));body.set('quat',_text(pose[[6,3,4,5]]));roots[name]=body_name
        for link in obj.tree.links:
            frames[link if not isinstance(task.object,RigidCollection) else name]=prefix+('urdf_'+link if link==obj.tree.root else link);geometries[link if not isinstance(task.object,RigidCollection) else name]=[g for g in geoms if g.startswith(prefix+link+'_collision_')]
        for n in obj.joint_names:joints[n]=prefix+n
        for motor in list(act):
            if motor.get('name','').startswith(prefix):act.remove(motor)
        if task.id=='box':
            node=next(j for j in world.iter('joint') if j.get('name')==prefix+'articulation');node.set('damping',str(task.original.data['damping']));node.set('frictionloss',str(task.original.data['frictionloss']))
    allowed={}
    for f in task.features:
        obj=task.object.bodies[f.frame] if isinstance(task.object,RigidCollection) else task.object;link=obj.tree.root if isinstance(task.object,RigidCollection) else f.frame;indices=[]
        for i,g in enumerate(ET.parse(obj.path).findall(f"link[@name='{link}']/collision")):
            shape=g.find('geometry/box')
            if shape is None:continue
            R,p=origin(g);half=np.fromstring(shape.get('size'),sep=' ')/2
            if np.all(abs(R.T@(f.surface-p))<=half+1e-8):indices.append(i)
        allowed[f.id]=[g for i,g in enumerate(geometries[f.frame]) if i in indices]
        if not allowed[f.id]:raise RuntimeError('No native surface for functional feature '+f.id)
    for spec,geoms in zip(problem.specs,robot_geoms):
        for name in geoms:
            pad='_left_pad_collision_' in name or '_right_pad_collision_' in name;point='_tool_collision_' in name
            if not (pad or point):continue
            mu=spec['gripper_profile']['mu'] if pad else .8
            for other in allowed[spec['group_id']]:
                if contacts_enabled:ET.SubElement(contact,'pair',geom1=name,geom2=other,condim='3',friction=_text([mu,mu,0,0,0]),solref='.004 1' if pad else '.008 1',solimp='.95 .99 .001')
    if not contacts_enabled:
        for g in world.iter('geom'):
            name=g.get('name','')
            if name.startswith('arm') and '_collision_' in name:g.set('contype','1');g.set('conaffinity','1')
            elif any(name.startswith(n+'_') for n in components) and '_collision_' in name:g.set('contype','2');g.set('conaffinity','0')
    fixtures=[]
    if task.id=='panel':
        jig=Robot(task.original.path.parent/task.original.data['jig_urdf']);append_urdf_robot(world,act,jig,'jig_',robot_collisions=True)
        for i,g in enumerate(ET.parse(jig.path).findall('link/collision')):
            R,p=origin(g);fixtures.append(dict(name=f'jig_{i}',position=p.tolist(),size=np.fromstring(g.find('geometry/box').get('size'),sep=' ').tolist(),quaternion_xyzw=Rotation.from_matrix(R).as_quat().tolist()))
    elif task.id=='assembly':
        for f in task.fixtures:ET.SubElement(world,'geom',name=f['name'],type='box',size=_text(np.asarray(f['size'])/2),pos=_text(f['position']),quat=_text(np.array(f['quaternion_xyzw'])[[3,0,1,2]]),contype='1',conaffinity='3');fixtures.append(f)
    ET.SubElement(world,'geom',name='floor',type='plane',size='3 3 .1',contype='1',conaffinity='3');out=Path(path);out.parent.mkdir(parents=True,exist_ok=True);ET.indent(root);out.write_bytes(ET.tostring(root,encoding='utf-8',xml_declaration=True));return FullScene(out,roots,frames,joints,geometries,allowed,tuple(fixtures))


def support_contacts(problem,phase,steps):
    """Actual jig surfaces or equal/opposite shoulder seating candidates."""
    task=problem.task_definition;out=[]
    if task.id=='panel':
        qgoal=task.keyframes[-1]['object_q'];T=task.object.fk(qgoal,task.object.tree.root);bottom=float(np.fromstring(ET.parse(task.object.path).find('link/collision/geometry/box').get('size'),sep=' ')[2]/2)
        for i,g in enumerate(ET.parse(task.original.path.parent/task.original.data['jig_urdf']).findall('link/collision')):
            R,p=origin(g);size=np.fromstring(g.find('geometry/box').get('size'),sep=' ')
            if size[2]>size[0]:continue
            point=T[:3,:3].T@(p-T[:3,3]);point[2]=-bottom;bounds=np.array([p[:2]-size[:2]/2,p[:2]+size[:2]/2]);out.append(Contact('jig_'+str(i),point,[0,0,1],Capability(mu=.8,normal_max=75.),plane_offset=p[2]+size[2]/2,plane_bounds=bounds,object_frame='panel',force_coordinates='scaled_tangent',tangent_ratio_regularization=1e-6))
    elif task.id=='assembly':
        cfg=task.original.data['socket'];center=np.asarray(cfg['center_object']);shoulder=np.asarray(cfg.get('leg_shoulder_object',[0,0,-.14]));leg=task.object.bodies['leg'];box=ET.parse(leg.path).find('link/collision/geometry/box');half=np.fromstring(box.get('size'),sep=' ')/2
        for i,xy in enumerate([[-.8*half[0],-.8*half[1]],[.8*half[0],-.8*half[1]],[0,.8*half[1]]]):out.append(BodyContact('shoulder_'+str(i),center+np.r_[xy,0.],[0,0,1],Capability(mu=.6,normal_max=30.),object_frame='tabletop',other_frame='leg',other_point=shoulder+np.r_[xy,0.],patch_half_size=.002,force_coordinates='scaled_tangent',tangent_ratio_regularization=1e-6))
    return tuple(out)
