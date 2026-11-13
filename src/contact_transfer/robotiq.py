"""Pinned native 2F-85 mechanism, exact tree URDF and independent mechanics.

The native model retains its four-bar connect constraints and coupled drivers.
URDF encodes the unconstrained tree; those loop constraints remain explicit
native data. No robot inertias or linkage dimensions are duplicated in code.
"""
from copy import deepcopy
from importlib.resources import files
import hashlib
import json
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation
from .robot import Robot


def settings():
    return json.loads(files('contact_transfer').joinpath('models/robotiq_2f85/settings.json').read_text())


def calibration_id(assets):
    return hashlib.sha256(Path(__file__).read_bytes()+(Path(assets)/'2f85.xml').read_bytes()+json.dumps(settings(),sort_keys=True).encode()).hexdigest()


def text(values):return ' '.join(format(float(v),'.17g') for v in np.asarray(values).ravel())


def matrix(quaternion_wxyz):
    return Rotation.from_quat(np.asarray(quaternion_wxyz)[[1,2,3,0]]).as_matrix()


def origin(parent,position,rotation):
    ET.SubElement(parent,'origin',xyz=text(position),rpy=text(Rotation.from_matrix(rotation).as_euler('xyz')))


def default_classes(root):
    result={'':{}}
    def visit(node,inherited):
        data=deepcopy(inherited)
        for child in node:
            if child.tag!='default':data.setdefault(child.tag,{}).update(child.attrib)
        result[node.get('class','')]=data
        for child in node.findall('default'):visit(child,data)
    for node in root.findall('default'):visit(node,{})
    return result


def export_native_urdf(assets,destination):
    """Translate joint-anchor frames and raw mesh poses without approximating FK."""
    import mujoco as mj
    assets=Path(assets).resolve();destination=Path(destination);destination.parent.mkdir(parents=True,exist_ok=True)
    source=ET.parse(assets/'2f85.xml').getroot();m=mj.MjModel.from_xml_path(str(assets/'2f85.xml'))
    classes=default_classes(source);bodies={e.get('name'):e for e in source.findall('.//worldbody//body')};materials={e.get('name'):e for e in source.findall('asset/material')}
    meshes={e.get('name',Path(e.get('file')).stem):e for e in source.findall('asset/mesh')}
    force_range=np.fromstring(source.find('actuator/general').get('forcerange'),sep=' ');max_force=float(max(abs(force_range)));effort={}
    for j in source.findall('tendon/fixed/joint'):effort[j.get('joint')]=max_force*abs(float(j.get('coef')))
    shifts=np.zeros((m.nbody,3));joint_ids={}
    for b in range(1,m.nbody):
        if m.body_jntnum[b]>1:raise ValueError('Converter expects one scalar joint per native link')
        if m.body_jntnum[b]:
            j=int(m.body_jntadr[b]);shifts[b]=m.jnt_pos[j];joint_ids[b]=j
            if m.jnt_type[j]!=mj.mjtJoint.mjJNT_HINGE:raise ValueError('Unsupported native gripper joint')
    root=ET.Element('robot',name='robotiq_2f85');root.append(ET.Comment('Derived from pinned BSD native model. Four-bar constraints remain external; link frames are shifted to joint anchors.'))
    ET.SubElement(root,'link',name='robotiq_world');frame_meta={}
    for b in range(1,m.nbody):
        name=mj.mj_id2name(m,mj.mjtObj.mjOBJ_BODY,b);parent=int(m.body_parentid[b]);pname='robotiq_world' if parent==0 else mj.mj_id2name(m,mj.mjtObj.mjOBJ_BODY,parent)
        link=ET.SubElement(root,'link',name=name);inertial=ET.SubElement(link,'inertial');origin(inertial,m.body_ipos[b]-shifts[b],matrix(m.body_iquat[b]));ET.SubElement(inertial,'mass',value=str(m.body_mass[b]));I=m.body_inertia[b];ET.SubElement(inertial,'inertia',ixx=str(I[0]),iyy=str(I[1]),izz=str(I[2]),ixy='0',ixz='0',iyz='0')
        source_body=bodies[name]
        for g in source_body.findall('geom'):
            attrs={**classes.get(g.get('class',source_body.get('childclass','2f85')),{}).get('geom',{}),**g.attrib};kind='visual' if int(attrs.get('contype','1'))==0 and int(attrs.get('conaffinity','1'))==0 else 'collision'
            geom=ET.SubElement(link,kind);pos=np.fromstring(attrs.get('pos','0 0 0'),sep=' ')-shifts[b];R=matrix(np.fromstring(attrs.get('quat','1 0 0 0'),sep=' '));origin(geom,pos,R);shape=ET.SubElement(geom,'geometry')
            if attrs.get('type')=='box':ET.SubElement(shape,'box',size=text(2*np.fromstring(attrs['size'],sep=' ')))
            elif attrs.get('type','mesh')=='mesh':
                mesh=meshes[attrs['mesh']];scale=classes.get(mesh.get('class',''),{}).get('mesh',{}).get('scale','1 1 1');ET.SubElement(shape,'mesh',filename='package://robotiq_2f85/assets/'+mesh.get('file'),scale=mesh.get('scale',scale))
            else:raise ValueError('Unsupported gripper geometry')
            if kind=='visual':
                material=materials.get(attrs.get('material'));rgba=attrs.get('rgba',material.get('rgba') if material is not None else '.5 .5 .5 1');color=ET.SubElement(geom,'material',name=attrs.get('material','body_gray'));ET.SubElement(color,'color',rgba=rgba)
        if b in joint_ids:
            j=joint_ids[b];jname=mj.mj_id2name(m,mj.mjtObj.mjOBJ_JOINT,j);joint=ET.SubElement(root,'joint',name=jname,type='revolute');ET.SubElement(joint,'axis',xyz=text(m.jnt_axis[j]));ET.SubElement(joint,'limit',lower=str(m.jnt_range[j,0]),upper=str(m.jnt_range[j,1]),effort=str(effort.get(jname,0.)),velocity=str(settings()['velocity_limit_rad_s']))
        else:joint=ET.SubElement(root,'joint',name=name+'_fixed',type='fixed')
        ET.SubElement(joint,'parent',link=pname);ET.SubElement(joint,'child',link=name);R=matrix(m.body_quat[b]);origin(joint,m.body_pos[b]+R@shifts[b]-shifts[parent],R);frame_meta[name]=dict(native_body_id=b,native_body_to_urdf_translation=shifts[b].tolist())
    ET.indent(root);destination.write_bytes(ET.tostring(root,encoding='utf-8',xml_declaration=True));shutil.copyfile(assets/'LICENSE',destination.parent/'LICENSE')
    metadata=dict(source=json.loads((assets/'provenance.json').read_text()),urdf_sha256=hashlib.sha256(destination.read_bytes()).hexdigest(),frames=frame_meta,loop_constraints=ET.tostring(source.find('equality'),encoding='unicode'),tendon=ET.tostring(source.find('tendon'),encoding='unicode'),settings=settings())
    (destination.parent/'model.json').write_text(json.dumps(metadata,indent=2)+'\n');return destination,metadata


def validate_native_urdf(assets,urdf,samples=4):
    """Native fullM/gravity/FK versus independent Pinocchio, unconstrained tree."""
    import mujoco as mj
    m=mj.MjModel.from_xml_path(str(Path(assets)/'2f85.xml'));d=mj.MjData(m);r=Robot(urdf);meta=json.loads((Path(urdf).parent/'model.json').read_text());jids=[mj.mj_name2id(m,mj.mjtObj.mjOBJ_JOINT,n) for n in r.joint_names];qa=m.jnt_qposadr[jids];va=m.jnt_dofadr[jids];errors=dict(mass=0.,gravity=0.,position=0.,rotation=0.)
    for seed in range(samples):
        fraction=np.random.default_rng(seed).uniform(.1,.8,r.n);q=r.lower+fraction*(r.upper-r.lower);d.qpos[qa]=q;d.qvel[:]=0;mj.mj_forward(m,d);M=np.zeros((m.nv,m.nv));mj.mj_fullM(m,d,M) if mj.__version__.startswith('4.') else mj.mj_fullM(m,M,d.qM);physical=M[np.ix_(va,va)]-np.diag(m.dof_armature[va]);errors['mass']=max(errors['mass'],float(np.max(abs(r.mass(q)-physical))));errors['gravity']=max(errors['gravity'],float(np.max(abs(r.gravity(q)-d.qfrc_bias[va]))))
        for name,frame in meta['frames'].items():
            b=frame['native_body_id'];p=d.xpos[b]+d.xmat[b].reshape(3,3)@frame['native_body_to_urdf_translation'];T=r.fk(q,name);errors['position']=max(errors['position'],float(np.linalg.norm(T[:3,3]-p)));errors['rotation']=max(errors['rotation'],float((Rotation.from_matrix(T[:3,:3]).inv()*Rotation.from_matrix(d.xmat[b].reshape(3,3))).magnitude()))
    return dict(residuals=errors,passed=max(errors.values())<1e-8,mechanics='native MuJoCo unconstrained mass (separate armature), gravity and body FK vs independent Pinocchio',closed_loop_dynamics_claimed_by_urdf=False)


def grip_geometry(m,d):
    """Inner face centres/normals from the native, split rectangular pads."""
    import mujoco as mj
    groups=[[mj.mj_name2id(m,mj.mjtObj.mjOBJ_GEOM,f'{side}_pad{i}') for i in [1,2]] for side in ['left','right']]
    centers=np.array([d.geom_xpos[g].mean(axis=0) for g in groups]);normals=[];points=[]
    for i,g in enumerate(groups):
        n=d.geom_xmat[g[0]].reshape(3,3)[:,1].copy()
        if n@(centers[i]-centers[1-i])<0:n=-n
        normals.append(n);points.append(centers[i]-m.geom_size[g[0],1]*n)
    axis=np.asarray(normals[0])-np.asarray(normals[1]);axis/=np.linalg.norm(axis);z=sum(d.geom_xmat[g[0]].reshape(3,3)[:,2] for g in groups);z-=axis*(axis@z);z/=np.linalg.norm(z);R=np.column_stack([np.cross(axis,z),axis,z]);points=np.array(points)
    return dict(pad_geoms=groups,points=points,normals=np.array(normals),position=points.mean(axis=0),rotation=R,aperture=float(axis@(points[0]-points[1])))


def equality_jacobian(m,d):
    import mujoco as mj
    if mj.mj_isSparse(m):raise ValueError('Gripper mechanism helper requires a dense native constraint Jacobian')
    return d.efc_J.reshape(d.nefc,m.nv)[d.efc_type==mj.mjtConstraint.mjCNSTR_EQUALITY]


def solve_native_aperture(m,d,width):
    """Solve seven native loop equations plus aperture by continuation."""
    import mujoco as mj
    from scipy.optimize import least_squares
    mj.mj_forward(m,d);initial_geometry=grip_geometry(m,d);initial=initial_geometry['aperture'];seed=d.qpos.copy();lower=m.jnt_range[:,0];upper=m.jnt_range[:,1]
    couplers=[j for j in range(m.njnt) if mj.mj_id2name(m,mj.mjtObj.mjOBJ_JOINT,j).endswith('coupler_joint')]
    independent=[i for i in range(m.nq) if i not in couplers];seed[couplers]=upper[couplers]
    def residual(x,target):
        q=seed.copy();q[independent]=x;q[couplers]=upper[couplers]
        d.qpos[:]=q;mj.mj_forward(m,d);eq=d.efc_pos[d.efc_type==mj.mjtConstraint.mjCNSTR_EQUALITY]
        g=grip_geometry(m,d)
        # Preloaded spring links put the unloaded parallel-grip branch at
        # the native coupler stops. Select that branch for initialization;
        # no extra lock is imposed during subsequent native dynamics.
        return np.r_[eq,g['aperture']-target]
    for target in np.linspace(initial,width,10)[1:]:
        sol=least_squares(residual,seed[independent],args=(target,),bounds=(lower[independent],upper[independent]),xtol=1e-13,ftol=1e-13,gtol=1e-13,max_nfev=200)
        if np.max(abs(residual(sol.x,target)))>1e-8:raise RuntimeError('Native four-bar/aperture solve failed: '+json.dumps(dict(target=float(target),residual=residual(sol.x,target).tolist(),q=sol.x.tolist())))
        seed[independent]=sol.x;seed[couplers]=upper[couplers]
    d.qpos[:]=seed;d.qvel[:]=0;mj.mj_forward(m,d);return seed.copy(),grip_geometry(m,d)


def transmission(m,d,tendon_direction):
    """One native loop-compatible velocity, normalized by actuator work."""
    import mujoco as mj
    from scipy.linalg import null_space
    geometry=grip_geometry(m,d);J=[];joints=np.flatnonzero(m.jnt_type==mj.mjtJoint.mjJNT_HINGE);dofs=m.jnt_dofadr[joints];parallel=[]
    for i,j in enumerate(joints):
        if mj.mj_id2name(m,mj.mjtObj.mjOBJ_JOINT,int(j)).endswith('coupler_joint'):
            row=np.zeros(len(dofs));row[i]=1.;parallel.append(row)
    for g,point in zip(geometry['pad_geoms'],geometry['points']):
        jp=np.zeros((3,m.nv));jr=np.zeros_like(jp);mj.mj_jac(m,d,jp,jr,point,int(m.geom_bodyid[g[0]]));J.append(jp)
    # The adaptive model also has two passive curling modes. Choose a
    # parallel-pad virtual displacement for the actuator/normal-force work
    # map. These extra rows are NOT constraints imposed on native dynamics.
    Z=null_space(np.vstack([equality_jacobian(m,d)[:,dofs],*parallel]),rcond=1e-9)
    if Z.shape[1]!=1:raise RuntimeError('Parallel-grasp virtual motion must have one velocity')
    z=np.zeros(m.nv);z[dofs]=Z[:,0];denominator=tendon_direction@z
    if abs(denominator)<1e-7:raise RuntimeError('Singular gripper actuator transmission')
    z/=denominator
    derivative=float(geometry['rotation'][:,1]@(J[0]-J[1])@z)
    return z,derivative


def run_native_clamp(assets,output):
    """Finite 2F-85 motor + native four-bar clamp on a free calibration block.

    This is an explicitly labelled, given-initial-grasp component test.
    The block has a free joint; it is not a full paper algorithm result.
    """
    import mujoco as mj
    cfg=settings();assets=Path(assets).resolve();out=Path(output);out.mkdir(parents=True,exist_ok=True)
    urdf,_=export_native_urdf(assets,out/'model/robot.urdf');mechanics=validate_native_urdf(assets,urdf)
    if not mechanics['passed']:raise RuntimeError('Native/URDF mechanics do not agree')
    root=ET.parse(assets/'2f85.xml').getroot();root.find('compiler').set('meshdir',str(assets/'assets'));option=root.find('option');option.set('timestep',str(cfg['timestep_s']));option.set('integrator','implicitfast');option.set('jacobian','dense')
    # Preserve the finite native tendon, position servo and effort range.
    # A selected-force virtual PMI admittance updates its position command.
    # No jaw state or object wrench is prescribed during stepping.
    actuator=root.find('actuator/general');force_range=np.fromstring(actuator.get('forcerange'),sep=' ');gain=np.fromstring(actuator.get('gainprm'),sep=' ');bias=np.fromstring(actuator.get('biasprm'),sep=' ');ctrl_range=np.fromstring(actuator.get('ctrlrange'),sep=' ')
    pre=mj.MjModel.from_xml_string(ET.tostring(root,encoding='unicode'));pd=mj.MjData(pre);mj.mj_resetData(pre,pd)
    size=np.array(cfg['calibration_fixture_size_grasp_xyz_m']);hold,geometry=solve_native_aperture(pre,pd,size[1]);initial,_=solve_native_aperture(pre,pd,size[1]-.0002)
    body=ET.SubElement(root.find('worldbody'),'body',name='calibration_fixture',pos=text(geometry['position']),quat=text(Rotation.from_matrix(geometry['rotation']).as_quat()[[3,0,1,2]]));ET.SubElement(body,'freejoint',name='calibration_free');ET.SubElement(body,'geom',name='calibration_block',type='box',size=text(size/2),mass=str(cfg['calibration_object_mass_kg']),friction='.6 0 0',rgba='.66 .43 .22 1')
    path=out/'scene.xml';ET.indent(root);path.write_bytes(ET.tostring(root,encoding='utf-8',xml_declaration=True));m=mj.MjModel.from_xml_path(str(path));d=mj.MjData(m);mj.mj_resetData(m,d);d.qpos[:len(initial)]=initial;mj.mj_forward(m,d)
    direction=np.zeros(m.nv)
    for element in root.findall('tendon/fixed/joint'):
        jid=mj.mj_name2id(m,mj.mjtObj.mjOBJ_JOINT,element.get('joint'));direction[m.jnt_dofadr[jid]]=float(element.get('coef'))
    robot=Robot(urdf);jids=[mj.mj_name2id(m,mj.mjtObj.mjOBJ_JOINT,n) for n in robot.joint_names];qa=m.jnt_qposadr[jids];block=mj.mj_name2id(m,mj.mjtObj.mjOBJ_GEOM,'calibration_block');blockbody=mj.mj_name2id(m,mj.mjtObj.mjOBJ_BODY,'calibration_fixture');records=[];Q=[];poses=[];names=[mj.mj_id2name(m,mj.mjtObj.mjOBJ_JOINT,j) for j in range(pre.njnt)];steps=round(cfg['duration_s']/cfg['timestep_s']);theta=float(d.actuator_length[0]);virtual_velocity=np.zeros(1)
    for k in range(steps+1):
        mj.mj_forward(m,d);g=grip_geometry(m,d);Fn=np.zeros(2);unexpected=[]
        for index in range(d.ncon):
            c=d.contact[index];pair=list(map(int,c.geom))
            if block not in pair:continue
            pad=pair[1-pair.index(block)];side=next((i for i,ids in enumerate(g['pad_geoms']) if pad in ids),None)
            if side is None:unexpected.append(mj.mj_id2name(m,mj.mjtObj.mjOBJ_GEOM,pad));continue
            value=np.zeros(6);mj.mj_contactForce(m,d,index,value)
            # Adaptive pads can rotate: use the solver's actual compressive
            # contact normal, not a fixed jaw-axis projection of the force.
            Fn[side]+=value[0]
        from .pmi import passive_midpoint_step
        eq=d.efc_pos[d.efc_type==mj.mjtConstraint.mjCNSTR_EQUALITY];z,derivative=transmission(m,d,direction);port=max(0.,-derivative)*(cfg['nominal_force_command_per_jaw_N']-max(Fn));_,virtual_velocity,energy=passive_midpoint_step([0.],virtual_velocity,[port],cfg['timestep_s'],mass=cfg['force_admittance_virtual_inertia'],damping=cfg['force_admittance_virtual_damping']);theta+=cfg['timestep_s']*float(energy['midpoint_velocity'][0]);command=float(np.clip((-bias[1]*theta-bias[0])/gain[0],*ctrl_range));theta=float(command*gain[0]/(-bias[1]));records.append(dict(time=k*cfg['timestep_s'],jaw_force_N=Fn.tolist(),aperture_m=g['aperture'],motor_position_command=command,motor_force_Nm=float(d.actuator_force[0]),virtual_pmi_energy_residual=energy['balance_residual'],loop_residual=float(max(abs(eq))),unexpected=unexpected));Q.append(d.qpos[qa].copy());poses.append(np.r_[d.xpos[blockbody],d.xquat[blockbody][[1,2,3,0]]])
        if k==steps:break
        d.ctrl[0]=command;mj.mj_step(m,d)
        if not np.isfinite(d.qpos).all() or any(w.number for w in d.warning):raise RuntimeError('Native gripper simulation became invalid')
    active=[r for r in records if r['time']>=cfg['duration_s']-1.];force=np.array([r['jaw_force_N'] for r in active]);passed=bool(np.min(force)>0 and np.max(force)<=cfg['declared_simulation_force_limit_per_jaw_N'] and np.max(abs(force-cfg['nominal_force_command_per_jaw_N']))<2. and not any(r['unexpected'] for r in active))
    report=dict(scope='native closed-loop gripper clamp on FREE calibration block, given initial grasp, component test only',passed=passed,input_id=calibration_id(assets),mechanics=mechanics,model='Robotiq 2F-85',settings=cfg,maximum_jaw_force_N=force.max(axis=0).tolist(),minimum_jaw_force_N=force.min(axis=0).tolist(),mean_jaw_force_N=force.mean(axis=0).tolist(),maximum_motor_force_Nm=max(abs(r['motor_force_Nm']) for r in records),maximum_loop_residual=max(r['loop_residual'] for r in active),native_internal_equalities=int(m.neq),object_pose_prescribed=False,fixed_fixture=False,full_paper_pipeline_executed=False,fixture=dict(position=geometry['position'].tolist(),quaternion_xyzw=Rotation.from_matrix(geometry['rotation']).as_quat().tolist(),size=size.tolist()),hold_joint_positions={n:float(hold[i]) for i,n in enumerate(names)},initial_joint_positions={n:float(initial[i]) for i,n in enumerate(names)})
    (out/'report.json').write_text(json.dumps(report,indent=2)+'\n');(out/'control.json').write_text(json.dumps(records,separators=(',',':')));np.savez_compressed(out/'recording.npz',time=np.array([r['time'] for r in records]),q=np.array(Q),object_pose=np.array(poses),joint_names=np.array(robot.joint_names));return report


class RobotiqForceServo:
    """Native parallel-mode gain + virtual PMI force admittance for firmware PD.

    Returns a single driver position, distributed through the native coupled
    transmission. Passive mechanism joints never receive control commands.
    """
    def __init__(self,profile,assets='.cache/robotiq_2f85'):
        import mujoco as mj
        self.mj=mj;root=ET.parse(Path(assets)/'2f85.xml').getroot();root.find('compiler').set('meshdir',str((Path(assets)/'assets').resolve()));root.find('option').set('jacobian','dense');self.model=mj.MjModel.from_xml_string(ET.tostring(root,encoding='unicode'));self.data=mj.MjData(self.model);self.profile=profile;self.names=[mj.mj_id2name(self.model,mj.mjtObj.mjOBJ_JOINT,j) for j in range(self.model.njnt)];self.direction=np.array([profile['drivers'].get(n,0.) for n in self.names]);self.theta=sum(profile['drivers'][n]*profile['hold'][n] for n in profile['drivers']);self.virtual_velocity=np.zeros(1);self.last={}
    def command(self,position,velocity,normal_forces,desired_force,dt,attached=True):
        from .pmi import passive_midpoint_step
        d,m=self.data,self.model;d.qpos[:]=[position[n] for n in self.names];d.qvel[:]=[velocity[n] for n in self.names];self.mj.mj_forward(m,d)
        if not attached:self.theta=0.;self.virtual_velocity[:]=0;return self.theta
        _,derivative=transmission(m,d,self.direction);port=max(0.,-derivative)*(desired_force-max(normal_forces));_,self.virtual_velocity,balance=passive_midpoint_step([0.],self.virtual_velocity,[port],dt,mass=self.profile['settings']['force_admittance_virtual_inertia'],damping=self.profile['settings']['force_admittance_virtual_damping']);self.theta+=dt*float(balance['midpoint_velocity'][0]);self.theta=float(np.clip(self.theta,0.,min(m.jnt_range[[self.names.index(n) for n in self.profile['drivers']],1])));self.last=dict(force_target_N=desired_force,virtual_pmi_energy_residual=balance['balance_residual']);return self.theta
