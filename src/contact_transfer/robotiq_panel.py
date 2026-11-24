"""Mount the validated native Robotiq mechanism on the panel's Panda arm."""
from copy import deepcopy
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
from .robot import Robot
from .urdf import origin as urdf_origin
from .robotiq import settings,calibration_id,run_native_clamp,grip_geometry,matrix,origin,text


def prepare_arm(case,directory,assets='.cache/robotiq_2f85',calibration='outputs/robotiq_2f85'):
    import mujoco as mj
    assets=Path(assets).resolve();cal=Path(calibration);out=Path(directory);out.mkdir(parents=True,exist_ok=True);cfg=settings()
    if not (cal/'report.json').exists():run_native_clamp(assets,cal)
    report=json.loads((cal/'report.json').read_text())
    if report.get('input_id')!=calibration_id(assets):report=run_native_clamp(assets,cal)
    if not report['passed'] or report['settings']!=cfg:raise ValueError('Robotiq native force calibration must pass for the current settings')
    hold=report['hold_joint_positions'];pose=np.r_[report['fixture']['position'],report['fixture']['quaternion_xyzw']];Ttcp=np.eye(4);Ttcp[:3,:3]=Rotation.from_quat(pose[3:]).as_matrix();Ttcp[:3,3]=pose[:3]
    old=case.full_robots[0];root=ET.parse(old.path).getroot();remove={'panda_hand'}
    while True:
        extra={j.find('child').get('link') for j in root.findall('joint') if j.find('parent').get('link') in remove}
        if extra<=remove:break
        remove|=extra
    for link in list(root.findall('link')):
        if link.get('name') in remove:root.remove(link)
    for joint in list(root.findall('joint')):
        if joint.find('child').get('link') in remove:root.remove(joint)
    gripper=ET.parse(cal/'model/robot.urdf').getroot();meta=json.loads((cal/'model/model.json').read_text());r=Robot(cal/'model/robot.urdf');q=np.array([hold[n] for n in r.joint_names]);Tb=r.fk(q,'base_mount');Tbt=np.linalg.inv(Tb)@Ttcp
    oldq=old.neutral();Told=np.linalg.inv(old.fk(oldq,'panda_link8'))@old.fk(oldq,'tip');yaw=Rotation.from_matrix(Told[:3,:3]@Ttcp[:3,:3].T).as_euler('xyz')[2];Rmount=Rotation.from_euler('z',yaw).as_matrix()
    for link in gripper.findall('link'):
        if link.get('name')!='robotiq_world':root.append(deepcopy(link))
    for source_joint in gripper.findall('joint'):
        joint=deepcopy(source_joint)
        if joint.find('parent').get('link')=='robotiq_world':
            joint.find('parent').set('link','panda_link8');R,p=urdf_origin(joint);joint.remove(joint.find('origin'));origin(joint,Rmount@p,Rmount@R)
        root.append(joint)
    ET.SubElement(root,'link',name='tip');joint=ET.SubElement(root,'joint',name='robotiq_tcp_mount',type='fixed');ET.SubElement(joint,'parent',link='base_mount');ET.SubElement(joint,'child',link='tip');origin(joint,Tbt[:3,3],Tbt[:3,:3]);path=out/'robotiq_arm.urdf';ET.indent(root);path.write_bytes(ET.tostring(root,encoding='utf-8',xml_declaration=True))
    m=mj.MjModel.from_xml_path(str(cal/'scene.xml'));d=mj.MjData(m);mj.mj_resetData(m,d)
    for name,value in hold.items():j=mj.mj_name2id(m,mj.mjtObj.mjOBJ_JOINT,name);d.qpos[m.jnt_qposadr[j]]=value
    mj.mj_forward(m,d);g=grip_geometry(m,d);corners=[]
    for ids,n in zip(g['pad_geoms'],g['normals']):
        side=[]
        for gid in ids:
            R=d.geom_xmat[gid].reshape(3,3);s=m.geom_size[gid];sign=-np.sign(n@R[:,1])
            for x in [-s[0],s[0]]:
                for z in [-s[2],s[2]]:side.append(Ttcp[:3,:3].T@(d.geom_xpos[gid]+R@np.array([x,sign*s[1],z])-Ttcp[:3,3]))
        corners.append(np.array(side))
    # Select the deepest pad insertion that clears the actual metal linkage.
    # This is a geometry probe only; the calibration execution stays untouched.
    panel_size=np.fromstring(ET.parse(case.panel.path).find('link/collision/geometry/box').get('size'),sep=' ');R_object_tool=Rotation.from_quat(case.data['arms'][0]['tool_quaternion_xyzw']).as_matrix();tool_size=np.abs(R_object_tool.T)@panel_size
    probe_root=ET.parse(cal/'scene.xml').getroot();probe_root.find("worldbody/body[@name='calibration_fixture']/geom").set('size',text(tool_size/2));pm=mj.MjModel.from_xml_string(ET.tostring(probe_root,encoding='unicode'));pd=mj.MjData(pm);mj.mj_resetData(pm,pd)
    for name,value in hold.items():j=mj.mj_name2id(pm,mj.mjtObj.mjOBJ_JOINT,name);pd.qpos[pm.jnt_qposadr[j]]=value
    block=mj.mj_name2id(pm,mj.mjtObj.mjOBJ_GEOM,'calibration_block');free=mj.mj_name2id(pm,mj.mjtObj.mjOBJ_JOINT,'calibration_free');adr=pm.jnt_qposadr[free];pads={mj.mj_name2id(pm,mj.mjtObj.mjOBJ_GEOM,f'{side}_pad{i}') for side in ['left','right'] for i in [1,2]};checks=[]
    maximum_inset=-min(c[:,2].min() for c in corners)
    for inset in np.linspace(.001,maximum_inset,40):
        pd.qpos[adr:adr+3]=Ttcp[:3,3]+Ttcp[:3,:3]@np.array([0.,0.,tool_size[2]/2-inset]);pd.qpos[adr+3:adr+7]=Rotation.from_matrix(Ttcp[:3,:3]).as_quat()[[3,0,1,2]];mj.mj_forward(pm,pd);minimum=.01
        for gid in range(pm.ngeom):
            if gid==block or gid in pads or not (pm.geom_contype[gid] or pm.geom_conaffinity[gid]):continue
            minimum=min(minimum,float(mj.mj_geomDistance(pm,pd,gid,block,.01,None)))
        checks.append(dict(inset_m=float(inset),minimum_nonpad_clearance_m=minimum))
    feasible=[v for v in checks if v['minimum_nonpad_clearance_m']>=.001]
    if not feasible:raise RuntimeError('No Robotiq edge pinch clears the panel and metal linkage')
    inset=feasible[-1]['inset_m'];lower=max(max(c[:,2].min() for c in corners),-inset+.001);upper=min(c[:,2].max() for c in corners)-.001;center=np.array([0.,0.,(lower+upper)/2]);half_width=min(min(c[:,0].max(),-c[:,0].min()) for c in corners)*.9;half_height=(upper-lower)/2*.9
    if min(half_width,half_height)<=0:raise ValueError('No common conservative Robotiq pad rectangle')
    native=ET.parse(assets/'2f85.xml').getroot();equations=[]
    for source_e in native.find('equality'):
        attrs=dict(source_e.attrib)
        if source_e.tag=='connect':attrs['anchor']=text(-np.asarray(meta['frames'][attrs['body1']]['native_body_to_urdf_translation']))
        equations.append(dict(type=source_e.tag,attributes=attrs))
    properties={}
    for name in hold:
        j=mj.mj_name2id(m,mj.mjtObj.mjOBJ_JOINT,name);v=m.jnt_dofadr[j];properties[name]=dict(armature=float(m.dof_armature[v]),damping=float(m.dof_damping[v]),stiffness=float(m.jnt_stiffness[j]),springref=float(m.qpos_spring[m.jnt_qposadr[j]]))
    act=native.find('actuator/general');bias=np.fromstring(act.get('biasprm'),sep=' ');drivers={j.get('joint'):float(j.get('coef')) for j in native.findall('tendon/fixed/joint')}
    profile=dict(id='robotiq_2f85',source_revision=cfg['native_revision'],hold=hold,drivers=drivers,properties=properties,equalities=equations,excludes=[dict(e.attrib) for e in native.findall('contact/exclude')],servo_kp=float(-bias[1]),servo_kd=float(-bias[2]),half_width=half_width,half_height=half_height,minimum_inset_m=inset,grasp_center_tool=center.tolist(),edge_pinch_geometry_checks=checks,mu=.6,closing_force_limit_N=cfg['declared_simulation_force_limit_per_jaw_N'],force_component_limit_N=cfg['external_force_component_limit_N'],aperture_m=cfg['calibration_fixture_size_grasp_xyz_m'][1],aperture_max_m=cfg['manufacturer_stroke_m'],force_command_N=cfg['nominal_force_command_per_jaw_N'],settings=cfg)
    spec=case.data['arms'][0];spec.update(gripper_model='robotiq_2f85',gripper_profile=profile,locked_joints=hold,urdf=path.name)
    case.full_robots=(Robot(path,base_pose=old.base_pose),*case.full_robots[1:]);case.robots=(Robot(path,base_pose=old.base_pose,locked_joints=hold),*case.robots[1:]);(out/'robotiq_profile.json').write_text(json.dumps(profile,indent=2)+'\n');return profile
