"""Optional unmodified CRISP SDK bridge; all simulation objects remain dynamic.

The C++ process owns native stepping. Python supplies only robot actuator
commands and converts recordings to ExecutionTrace. This adapter currently
supports fixed-base torque arms and one free rigid panel, not mobile robots.
"""
import hashlib
import json
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation

from .execution import ExecutionTrace, TRACE_CONVENTIONS


def native_contact_to_trace(rotation, impulse, normal, dt):
    """CRISP R.T*lambda acts on geom1; R maps world to contact coordinates.

    Native normal points geom2->geom1. Impulse is divided by its actual step.
    This adapter models point Coulomb contact; there is no intrinsic couple.
    """
    R=np.asarray(rotation,float);P=np.asarray(impulse,float);n=np.asarray(normal,float)
    if R.shape!=(3,3) or P.shape!=(3,) or n.shape!=(3,) or not np.isfinite(dt) or dt<=0 or not all(np.isfinite(v).all() for v in [R,P,n]):
        raise ValueError('Finite contact rotation, impulse, normal and positive timestep required')
    if not np.allclose(R.T@R,np.eye(3),atol=1e-7) or not np.isclose(np.linalg.det(R),1,atol=1e-7) or not np.isclose(np.linalg.norm(n),1,atol=1e-7):
        raise ValueError('Contact frame must be a proper rotation and normal a unit vector')
    return -n,np.r_[np.zeros(3),-R.T@P/dt]


def _prepare_urdf(source,destination,prefix,mesh_files):
    """Preserve inertias/limits/collision primitives; resolve collision meshes.

    Visuals are omitted for headless stepping. Joint damping/friction are zero
    as in the MuJoCo comparison scene and the Pinocchio planning model.
    """
    tree=ET.parse(source);root=tree.getroot()
    for link in root.findall('link'):
        original=link.get('name');link.set('name',prefix+original)
        for visual in list(link.findall('visual')):link.remove(visual)
        for index,c in enumerate(link.findall('collision')):
            c.set('name',prefix+original+'_collision_'+str(index))
            mesh=c.find('geometry/mesh')
            if mesh is not None:
                name=Path(mesh.get('filename')).name
                candidate=mesh_files.get(name)
                if candidate is None or not candidate.is_file():raise FileNotFoundError('Collision mesh missing: '+name)
                mesh.set('filename',str(candidate.resolve()))
    for joint in root.findall('joint'):
        joint.set('name',prefix+joint.get('name'))
        for tag in ['parent','child']:
            node=joint.find(tag);node.set('link',prefix+node.get('link'))
        dynamics=joint.find('dynamics')
        if dynamics is not None:joint.remove(dynamics)
        ET.SubElement(joint,'dynamics',damping='0',friction='0')
        for tag in ['mimic','safety_controller']:
            for node in list(joint.findall(tag)):joint.remove(node)
    root.set('name',prefix+'model');ET.indent(tree);tree.write(destination,encoding='utf-8',xml_declaration=True)


def prepare_panel_manifest(episode,controller,directory,*,solver='canal',mu=.8):
    if solver not in {'canal','sub_admm'}:raise ValueError('Unknown CRISP solver')
    if any(s['controller']!='torque' for s in episode.problem.case.data['arms']):raise ValueError('CRISP bridge currently supports direct torque arms only')
    if any(s['model']!='franka_panda' for s in episode.problem.case.data['arms']):raise ValueError('This CRISP panel adapter currently validates Panda arms only')
    if any(s.get('gripper_profile') for s in episode.problem.case.data['arms']):raise ValueError('The native closed-loop Robotiq mechanism is currently supported by the MuJoCo adapter, not this CRISP URDF bridge')
    p=episode.problem;directory=Path(directory).resolve();directory.mkdir(parents=True,exist_ok=True)
    rows=[];exclusions=[];joint_names=[];q0=controller.reference(0)[0];offset=0
    for i,(r,spec) in enumerate(zip(controller.robots,p.case.data['arms'])):
        prefix=spec['id']+'_';mesh_files={v.name:v for v in (Path.cwd()/'.cache/panda_meshes').glob('*.stl')}
        path=directory/f'arm{i}.urdf';_prepare_urdf(r.path,path,prefix,mesh_files)
        T=r.base_pose;rows.append((path,1,*T[:3,3],*Rotation.from_matrix(T[:3,:3]).as_quat()))
        names=[prefix+n for n in r.joint_names];joint_names.extend(names)
        # Keep the same provided SRDF exclusions as the MuJoCo panel scene.
        for pair in ET.parse(p.case.path.parent/'panda.srdf').findall('disable_collisions'):
            a,b=pair.get('link1'),pair.get('link2')
            if a in r.tree.links and b in r.tree.links:exclusions.append((prefix+a,prefix+b))
        offset+=r.n
    panel_path=directory/'panel.urdf';_prepare_urdf(p.case.panel.path,panel_path,'',{})
    pose=p.case.data['keyframes'][0]['object_pose'];rows.append((panel_path,0,*pose))
    jig_path=directory/'jig.urdf';_prepare_urdf(p.case.path.parent/p.case.data['jig_urdf'],jig_path,'jig_',{})
    rows.append((jig_path,1,0,0,0,0,0,0,1))
    lines=[f'{solver} {controller.dt:.17g} {mu:.17g} {len(rows)}']
    lines.extend(' '.join(map(str,row)) for row in rows);lines.append(str(len(joint_names)))
    lines.extend(f'{n} {q:.17g}' for n,q in zip(joint_names,q0));lines.append(str(len(exclusions)))
    lines.extend(' '.join(pair) for pair in exclusions);lines.append('panel')
    manifest=directory/'scene.txt';manifest.write_text('\n'.join(lines)+'\n')
    return manifest,joint_names


def validate_native_kinematics(native,controller,initial_q):
    """Check the imported native URDF frames independently with Pinocchio."""
    maximum_position=0.;maximum_rotation=0.;checked=0
    for body in native['bodies']:
        for spec,r,a,b in zip(controller.case.data['arms'],controller.robots,controller.offsets[:-1],controller.offsets[1:]):
            prefix=spec['id']+'_';name=body['name']
            if not name.startswith(prefix) or name[len(prefix):] not in r.tree.links:continue
            expected=r.fk(initial_q[a:b],name[len(prefix):]);maximum_position=max(maximum_position,float(np.linalg.norm(expected[:3,3]-body['position'])))
            maximum_rotation=max(maximum_rotation,float((Rotation.from_matrix(expected[:3,:3]).inv()*Rotation.from_quat(body['quaternion_xyzw'])).magnitude()));checked+=1
    if checked<len(controller.robots)*7 or max(maximum_position,maximum_rotation)>1e-6:raise RuntimeError('CRISP native URDF kinematics disagree with independent Pinocchio')
    return dict(checked_frames=checked,maximum_position_error_m=maximum_position,maximum_rotation_error_rad=maximum_rotation)


def run_crisp_episode(episode,output,*,binary,solver='canal',mu=.8,progress=None):
    from .paper_execution import CoupledContactController,assess_coupled_execution
    from .paper_geometry import PanelGeometry
    from .paper_validation import validate_rigid_trajectory
    p=episode.problem
    for i,r in enumerate(episode.trajectories):
        contacts=episode.contacts_by_interval[i] if episode.contacts_by_interval else p.contacts(episode.keyframes[i],len(r.dt))
        report=validate_rigid_trajectory(p,r,contacts)
        if not report['passed'] or not PanelGeometry(p).validate(r):raise RuntimeError('Independent plan validation failed before CRISP')
    out=Path(output);out.mkdir(parents=True,exist_ok=True);controller=CoupledContactController(episode)
    manifest,names=prepare_panel_manifest(episode,controller,out/'models',solver=solver,mu=mu)
    times=[];qs=[];refs=[];efforts=[];poses=[];contacts=[];residuals=[]
    dt=controller.dt;steps=round(episode.duration/dt);binary=Path(binary).resolve()
    native_model_check=None
    with (out/'bridge.log').open('w') as log:
        process=subprocess.Popen([str(binary),str(manifest)],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=log,text=True,bufsize=1)
        try:
            for k in range(steps+1):
                while True:
                    line=process.stdout.readline()
                    if not line:raise RuntimeError(f'CRISP bridge stopped at step {k}; see {out / "bridge.log"}')
                    if line.startswith('MODEL '):
                        native_model_check=validate_native_kinematics(json.loads(line[6:]),controller,controller.reference(0)[0])
                        (out/'native-model-check.json').write_text(json.dumps(native_model_check,indent=2)+'\n')
                    if line.startswith('STATE '):state=json.loads(line[6:]);break
                    log.write(line);log.flush()
                if native_model_check is None:raise RuntimeError('CRISP bridge did not provide independent model verification data')
                q,v=np.asarray(state['q']),np.asarray(state['v']);pose=np.asarray(state['object_pose']);t=k*dt
                if not np.isfinite(np.r_[q,v,pose]).all():raise RuntimeError('Nonfinite CRISP state')
                if np.linalg.norm(pose[:3]-episode.desired_object_pose(t)[:3])>.25:
                    failure=dict(time=t,object_pose=pose.tolist(),solver_residual=state['solver_residual'],reason='Object tracking error exceeded 0.25 m; CRISP integration rejected',full_paper_pipeline_executed=False)
                    failure.update(native_state=state,last_control=controller.logs[-1] if controller.logs else None)
                    (out/'failure.json').write_text(json.dumps(failure,indent=2)+'\n')
                    raise RuntimeError(failure['reason'])
                desired=controller.reference(t);times.append(t);qs.append(q);refs.append(desired[0]);poses.append([pose]);residuals.append(state['solver_residual'])
                for c in state['contacts']:contacts.append((t,c))
                if progress and (k%max(1,round(1/dt))==0 or k==steps):progress(t,episode.duration)
                if k==steps:break
                controller.observe_contacts(q,pose,state['contacts']);u=controller.command(q,v,*desired);efforts.append(u)
                process.stdin.write('step '+' '.join(format(x,'.17g') for x in u)+'\n');process.stdin.flush()
            process.stdin.write('quit\n');process.stdin.flush();process.wait(timeout=10)
            if process.returncode:raise RuntimeError(f'CRISP bridge exited {process.returncode}; see {out / "bridge.log"}')
        finally:
            if process.poll() is None:process.kill();process.wait()
    case_id,plan_id=episode.identity()
    metadata=dict(TRACE_CONVENTIONS,engine='crisp',engine_version='external-sdk',bridge_sha256=hashlib.sha256(binary.read_bytes()).hexdigest(),case_id=case_id,plan_id=plan_id,timestep=dt,joint_names=names,object_names=['panel'],contact_settings=dict(solver=solver,mu=mu,force_source='negative native R.T*lambda / dt on geom2',force_interval='preceding native step',normal='negative native normal',torsional_friction=False),controller=episode.control_mode,jaw_force_fraction=episode.jaw_force_fraction,object_pose_prescribed=False,equality_constraints_used=0,sdk_redistributed=False)
    arrays=dict(contact_time=np.array([t for t,c in contacts]),contact_pair=np.array([c['pair'] for t,c in contacts],dtype=str).reshape(-1,2),contact_position=np.array([c['position'] for t,c in contacts]).reshape(-1,3),contact_normal=np.array([c['normal'] for t,c in contacts]).reshape(-1,3),contact_gap=np.array([c['gap'] for t,c in contacts]),contact_wrench=np.array([c['wrench'] for t,c in contacts]).reshape(-1,6))
    trace=ExecutionTrace(metadata,np.array(times),np.array(qs),np.array(refs),np.array(efforts),np.array(poses),**arrays).validate();trace.save(out/'execution.npz')
    report=assess_coupled_execution(episode,trace,controller)
    report.update(solver=solver,timestep=dt,maximum_solver_residual=float(max(residuals)),native_model_check=native_model_check)
    (out/'execution-report.json').write_text(json.dumps(report,indent=2)+'\n');(out/'control.json').write_text(json.dumps(controller.logs,separators=(',',':')))
    return trace,report,controller
