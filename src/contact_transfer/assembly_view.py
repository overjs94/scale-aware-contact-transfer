"""URDF/mesh web playback of measured two-object assembly execution."""
import hashlib,json,shutil
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
from .table_assembly import AssemblyCase,AssemblyPlan,assembly_report
from .execution import ExecutionTrace
from .urdf import origin


def export_assembly_view(plan,trace,report,flags,web_directory,frames=601):
    web=Path(web_directory);(web/'data').mkdir(parents=True,exist_ok=True)
    idx=np.unique(np.linspace(0,len(trace.time)-1,frames).round().astype(int));positions=[[] for _ in idx];names=[];edges=[];instances=[];offset=0;effort=[]
    source=json.loads((plan.case.path.parent/'provenance.json').read_text())
    for robot,spec in zip(plan.case.full_robots,plan.case.data['arms']):
        links=list(robot.tree.links);start=len(names);names += [spec['id']+'_'+n for n in links]
        edges += [[start+links.index(j.parent),start+links.index(j.child)] for j in robot.tree.joints]
        for row,k in zip(positions,idx):row += [robot.fk(trace.q[k,offset:offset+robot.n],n)[:3,3].tolist() for n in links]
        model_id='assembly_'+spec['id'];directory=web/'models'/model_id;directory.mkdir(parents=True,exist_ok=True)
        shutil.copyfile(robot.path,directory/'robot.urdf')
        shutil.copyfile(plan.case.path.parent/'LICENSE.upstream',directory/'LICENSE.upstream')
        (directory/'model.json').write_text(json.dumps({**source,'id':model_id,'urdf':'robot.urdf','urdf_sha256':hashlib.sha256(robot.path.read_bytes()).hexdigest(),'source':'Panda native hand' if spec['kind']=='G' else 'Panda with spherical point tool'},indent=2)+'\n')
        instances.append({'id':spec['id'],'role':spec['object']+'_'+spec['kind'],'manifest':f'./models/{model_id}/model.json','joint_names':robot.joint_names,'dof':robot.n,'floating_base':False,
                          'q_indices':list(range(offset,offset+robot.n)),'base_pose':{'position':robot.base_pose[:3,3].tolist(),'quaternion_xyzw':Rotation.from_matrix(robot.base_pose[:3,:3]).as_quat().tolist()}})
        effort.extend(robot.effort);offset+=robot.n
    objects=[]
    for j,(part,spec) in enumerate(zip(plan.case.parts,plan.case.data['objects'])):
        model_id='assembly_'+spec['id'];directory=web/'models'/model_id;directory.mkdir(parents=True,exist_ok=True);shutil.copyfile(part.path,directory/'robot.urdf')
        (directory/'model.json').write_text(json.dumps({'id':model_id,'urdf':'robot.urdf','packages':{},'urdf_sha256':hashlib.sha256(part.path.read_bytes()).hexdigest(),'source':'authored rigid furniture URDF; box decomposition preserves empty socket','license':'MIT','license_file':'LICENSE.authored'},indent=2)+'\n')
        license_path=plan.case.path.parent/'LICENSE.authored'
        if license_path.exists():shutil.copyfile(license_path,directory/'LICENSE.authored')
        instances.append({'id':spec['id'],'role':'free_furniture','manifest':f'./models/{model_id}/model.json','joint_names':[],'dof':0,'floating_base':True,'q_indices':list(range(offset,offset+7))});offset+=7
        index=len(names);names.append(spec['id']+'_'+spec['id'])
        for row,k in zip(positions,idx):row.append(trace.object_pose[k,j,:3].tolist())
        boxes=[]
        for g in ET.parse(part.path).findall('link/visual'):
            R,p=origin(g);boxes.append({'size':np.array(g.find('geometry/box').get('size').split(),float).tolist(),'position':p.tolist(),'quaternion_xyzw':Rotation.from_matrix(R).as_quat().tolist()})
        objects.append({'id':spec['id'],'index':index,'poses':trace.object_pose[idx,j].tolist(),'boxes':boxes})
    q=np.c_[trace.q[idx],trace.object_pose[idx].reshape(len(idx),14)]
    socket=np.zeros(len(trace.time),bool)
    for t,pair in zip(trace.contact_time,trace.contact_pair):
        if any(n.startswith('leg_collision_') for n in pair) and any(n.startswith('tabletop_collision_') for n in pair):socket[int(round(t/trace.metadata['timestep']))]=True
    # Compute the displayed insertion depth from measured poses, not references.
    Rtop=Rotation.from_quat(trace.object_pose[idx,0,3:]).as_matrix();Rleg=Rotation.from_quat(trace.object_pose[idx,1,3:]).as_matrix()
    tip=trace.object_pose[idx,1,:3]+np.einsum('nij,j->ni',Rleg,plan.case.data['socket'].get('peg_tip_object',[0,0,-.165]));local=np.einsum('nji,nj->ni',Rtop,tip-trace.object_pose[idx,0,:3])
    torques=np.r_[trace.torque,trace.torque[-1:]][idx]
    data={'id':'table_assembly','title':f'책상 조립 · Franka × 3 · {trace.time[-1]:g} s','engine':'mujoco_contact_recording','scenario':'table_assembly','dof':25,'nq':39,'nv':37,'floating_base':False,'frame':'tabletop / socket',
          'joint_names':trace.metadata['joint_names'],'instances':instances,'objects':objects,'times':trace.time[idx].round(6).tolist(),'q':q.round(9).tolist(),'positions':positions,'link_names':names,'edges':edges,
          'torques':torques.tolist(),'torque_ratio':np.max(abs(torques)/np.array(effort),axis=1).tolist(),'errors':[None]*len(idx),
          'chart_values':((plan.case.data['socket']['center_object'][2]-local[:,2])*1000).tolist(),'chart_label':'MEASURED INSERTION DEPTH','chart_unit':'mm',
          'keyframes':plan.case.data['keyframes'],'events':[{'id':'release_leg_gripper','time':plan.case.data['simulation']['release_time']}],
          'tool_states':{'point_contact':flags[idx,1].all(axis=1).tolist(),'top_gripper_contact':flags[idx,0].all(axis=1).tolist(),'leg_gripper_contact':flags[idx,2].all(axis=1).tolist(),'socket_contact':socket[idx].tolist()},
          'environment':[{'type':'box',**f,'color':'#425469'} for f in trace.metadata['fixtures']],
          'summary':{'duration':float(trace.time[-1]),'physics_dt':trace.metadata['timestep'],'full_geometry_checked':False,'keyframe_errors':report['keyframe_errors'],'primal_residual':report['primal_residual']},'report':report,
          'description':'30-second two-free-body MuJoCo execution. One native Panda gripper and one spherical point tool grip/support and reorient the tabletop by 12 degrees. A second native gripper transports and inserts the leg peg into a real empty socket, opens, then retracts. SE(3) IK paths and contact modes are prescribed; IPOPT allocates inertial wrenches/torques. No pose playback, welds, suction or artificial grasp forces drive the physics. Synthetic capability bounds and soft contact do not certify hardware or structural strength.'}
    (web/'data/table_assembly.json').write_text(json.dumps(data,separators=(',',':'),allow_nan=False))
    index_path=web/'data/index.json';index=json.loads(index_path.read_text()) if index_path.exists() else {'models':[]}
    index['models']=[m for m in index['models'] if m['id']!='table_assembly'];index['models'].append({k:data[k] for k in ['id','title','engine','dof','floating_base']});index_path.write_text(json.dumps(index,indent=2)+'\n')
    return data


def export_saved_assembly(output,web_directory):
    root=Path(output);case=AssemblyCase.load(root/'case.json');a=np.load(root/'plan.npz');report=json.loads((root/'report.json').read_text())
    plan=AssemblyPlan(case,a['time'],a['q'],a['v'],a['a'],a['reactions_world'],a['torque'],report['primal_residual'],tuple(slice(7*i,7*i+7) for i in range(3)))
    trace=ExecutionTrace.load(root/'execution.npz');flags=np.load(root/'assembly_metrics.npz')['contact_flags']
    return export_assembly_view(plan,trace,report,flags,web_directory)
