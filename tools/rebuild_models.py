from pathlib import Path
import subprocess, json, hashlib, xml.etree.ElementTree as ET
import xacro
import argparse
parser=argparse.ArgumentParser(description='Regenerate model subsets from checked-out upstream sources.')
parser.add_argument('--references', type=Path, required=True)
args=parser.parse_args()
refs=args.references.resolve()
out=Path(__file__).resolve().parents[1]/'src/contact_transfer/models'

def show(repo,path):
 return subprocess.check_output(['git','-C',str(refs/repo),'show',f'HEAD:{path}'])
def sha(repo):
 return subprocess.check_output(['git','-C',str(refs/repo),'rev-parse','HEAD'],text=True).strip()
def add(name,repo,path,data,license_path,license_id,frame,**extras):
 dest=out/name; dest.mkdir(parents=True,exist_ok=True)
 root=ET.fromstring(data)
 for parent in list(root.iter()):
  for child in list(parent):
   if child.tag in {'visual','collision','gazebo','transmission','ros2_control','material'}:
    parent.remove(child)
 for j in root.findall('joint'):
  for d in list(j):
   if d.tag in {'dynamics','safety_controller','calibration'}:j.remove(d)
 ET.indent(root)
 clean=ET.tostring(root,encoding='unicode')
 (dest/'robot.urdf').write_text('<?xml version="1.0"?>\n<!-- Dynamics/kinematics subset. Visuals/collisions and controller tags removed. -->\n'+clean+'\n')
 (dest/'LICENSE.upstream').write_bytes(show(repo,license_path))
 entry={'name':name,'source_repository':subprocess.check_output(['git','-C',str(refs/repo),'remote','get-url','origin'],text=True).strip(),
 'commit':sha(repo),'source_path':path,'source_file_sha256':hashlib.sha256(show(repo,path)).hexdigest(),'expanded_urdf_sha256':hashlib.sha256(data).hexdigest(),'license':license_id,'license_path':license_path,
 'frame':frame,'floating_base':False,'locked_joints':{},'geometry':'dynamics_only','control_default':'position','inertia_status':'upstream_not_independently_identified',**extras}
 (dest/'model.json').write_text(json.dumps(entry,indent=2)+'\n')
 print('CREATED',name,len(data),len(clean))

def expand(repo,pkg_paths,path,mappings):
 for package,rel in pkg_paths.items():
  value=str(refs/repo/rel)
  for f in (refs/repo).rglob('*.xacro'):
   if '.git' in f.parts:continue
   text=f.read_text()
   new=text.replace('$(find '+package+')',value)
   if new!=text:f.write_text(new)
 return xacro.process_file(str(refs/repo/path),mappings=mappings).toxml().encode()

add('franka_panda','example-robot-data','robots/panda_description/urdf/panda.urdf',show('example-robot-data','robots/panda_description/urdf/panda.urdf'),'robots/panda_description/LICENSE','Apache-2.0','panda_link8',locked_joints={'panda_finger_joint1':0.0,'panda_finger_joint2':0.0},control_default='torque')
add('kuka_iiwa7','iiwa_ros','iiwa_description/urdf/iiwa7.urdf.xacro',expand('iiwa_ros',{'iiwa_description':'iiwa_description'},'iiwa_description/urdf/iiwa7.urdf.xacro',{}),'LICENSE','GPL-3.0-or-later','iiwa_link_ee',control_default='position')
add('ur5e','ur_description','urdf/ur.urdf.xacro',expand('ur_description',{'ur_description':'.'},'urdf/ur.urdf.xacro',{'ur_type':'ur5e','name':'ur5e'}),'LICENSE','BSD-3-Clause','tool0')
add('xarm7','xarm_ros2','xarm_description/urdf/xarm_device.urdf.xacro',expand('xarm_ros2',{'xarm_description':'xarm_description','xarm_controller':'xarm_controller'},'xarm_description/urdf/xarm_device.urdf.xacro',{'dof':'7','robot_type':'xarm'}),'LICENSE','BSD-3-Clause','link_eef')
add('rb3','rbpodo_ros2','rbpodo_description/robots/rb3_730es_u.urdf',show('rbpodo_ros2','rbpodo_description/robots/rb3_730es_u.urdf'),'rbpodo_description/LICENSE','Apache-2.0','link6',variant='RB3-730ES U')
add('rb5','rbpodo_ros2','rbpodo_description/robots/rb5_850e.urdf',show('rbpodo_ros2','rbpodo_description/robots/rb5_850e.urdf'),'rbpodo_description/LICENSE','Apache-2.0','link6',variant='RB5-850E')
add('kinova_gen3','ros_kortex','kortex_description/arms/gen3/7dof/urdf/GEN3-7DOF-VISION_ARM_URDF_V12.urdf',show('ros_kortex','kortex_description/arms/gen3/7dof/urdf/GEN3-7DOF-VISION_ARM_URDF_V12.urdf'),'LICENSE','BSD-3-Clause','end_effector_link')
add('unitree_g1','unitree_ros','robots/g1_description/g1_29dof.urdf',show('unitree_ros','robots/g1_description/g1_29dof.urdf'),'LICENSE','BSD-3-Clause','right_wrist_yaw_link',floating_base=True,control_default='torque')
