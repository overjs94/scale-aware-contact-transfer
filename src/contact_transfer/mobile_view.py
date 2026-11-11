"""Measured mobile-base and panel poses, original URDF visual/collision playback."""
import hashlib,json,shutil
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
from .execution import ExecutionTrace
from .mobile_panel import MobileCase,MobilePlan
from .urdf import origin


def export_mobile_view(plan,trace,report,flags,web_directory,frames=721):
    web=Path(web_directory);(web/'data').mkdir(parents=True,exist_ok=True);idx=np.unique(np.linspace(0,len(trace.time)-1,frames).round().astype(int));case=plan.case;n=case.rby.n
    q=np.c_[trace.object_pose[idx,1],trace.q[idx,:n],trace.q[idx,n:],trace.object_pose[idx,0]];names=[];edges=[];positions=[[] for _ in idx];instances=[]
    definitions=[('rby',case.floating,'rby1_mobile',list(range(n+7)),True),('panda',case.panda,'mobile_panda',list(range(n+7,n+7+case.panda.n)),False)]
    for id,robot,model_id,indices,floating in definitions:
        directory=web/'models'/model_id;directory.mkdir(parents=True,exist_ok=True);shutil.copyfile(robot.path,directory/'robot.urdf')
        if id=='rby':
            meta=json.loads((case.path.parent/'provenance.json').read_text());mesh_root=f"https://raw.githubusercontent.com/RainbowRobotics/rby1-sdk/{meta['upstream_commit']}/"
            document=ET.parse(robot.path)
            for mesh in document.findall('.//mesh'):
                url=mesh.get('filename')
                if url.startswith(mesh_root):mesh.set('filename','package://rby1/'+url[len(mesh_root):])
            ET.indent(document);document.write(directory/'robot.urdf',encoding='utf-8',xml_declaration=True)
            meta.update(id=model_id,urdf='robot.urdf',packages={'rby1':mesh_root.rstrip('/')},source='official RBY1-A with differential mobile base and authored tools',assets=sorted(set(m.get('filename') for m in ET.parse(robot.path).findall('.//mesh'))));shutil.copyfile(case.path.parent/'LICENSE.rby1',directory/'LICENSE.rby1');shutil.copyfile(case.path.parent/'NOTICE.rby1',directory/'NOTICE.rby1')
        else:
            meta=json.loads((case.path.parent/'franka_panda_provenance.json').read_text());meta.update(id=model_id,urdf='robot.urdf',license_file='LICENSE.upstream');shutil.copyfile(case.path.parent/'LICENSE.panda',directory/'LICENSE.upstream')
        meta['urdf_sha256']=hashlib.sha256((directory/'robot.urdf').read_bytes()).hexdigest();(directory/'model.json').write_text(json.dumps(meta,indent=2)+'\n')
        spec=dict(id=id,role='mobile_dual_arm' if id=='rby' else 'fixed_support_arm',manifest=f'./models/{model_id}/model.json',joint_names=robot.joint_names,dof=robot.n,floating_base=floating,q_indices=indices)
        if not floating:spec['base_pose']=dict(position=robot.base_pose[:3,3].tolist(),quaternion_xyzw=Rotation.from_matrix(robot.base_pose[:3,:3]).as_quat().tolist())
        instances.append(spec);links=list(robot.tree.links);start=len(names);names += [id+'_'+name for name in links];edges += [[start+links.index(j.parent),start+links.index(j.child)] for j in robot.tree.joints]
        for row,k in zip(positions,idx):
            state=np.r_[trace.object_pose[k,1],trace.q[k,:n]] if floating else trace.q[k,n:]
            row += [robot.fk(state,name)[:3,3].tolist() for name in links]
    for id,file,floating,indices in [('panel','panel.urdf',True,list(range(n+7+case.panda.n,q.shape[1]))),('jig','jig.urdf',False,[])]:
        model_id='mobile_'+id;directory=web/'models'/model_id;directory.mkdir(parents=True,exist_ok=True);path=case.path.parent/file;shutil.copyfile(path,directory/'robot.urdf');shutil.copyfile(case.path.parent/'LICENSE.authored',directory/'LICENSE.authored');(directory/'model.json').write_text(json.dumps(dict(id=model_id,urdf='robot.urdf',packages={},source='authored mobile panel fixture',license='MIT',license_file='LICENSE.authored',urdf_sha256=hashlib.sha256(path.read_bytes()).hexdigest()),indent=2)+'\n')
        instances.append(dict(id=id,role='free_furniture' if floating else 'static_jig',manifest=f'./models/{model_id}/model.json',joint_names=[],dof=0,floating_base=floating,q_indices=indices));names.append(id+'_'+ET.parse(path).find('link').get('name'))
        for row,k in zip(positions,idx):row.append(trace.object_pose[k,0,:3].tolist() if floating else [0.,0.,0.])
    boxes=[]
    for g in ET.parse(case.panel.path).findall('link/visual'):
        R,p=origin(g);boxes.append(dict(size=np.array(g.find('geometry/box').get('size').split(),float).tolist(),position=p.tolist(),quaternion_xyzw=Rotation.from_matrix(R).as_quat().tolist()))
    torques=np.r_[trace.torque,trace.torque[-1:]][idx];limits=np.r_[case.rby.effort,case.panda.effort];panel_index=names.index('panel_panel')
    data=dict(id='mobile_panel',title='작업영역 확장 · RBY1 + Panda · 36 s',engine='mujoco_contact_recording',scenario='mobile_panel',dof=case.rby.n+case.panda.n,nq=q.shape[1],nv=q.shape[1]-2,floating_base=False,frame='world / differential mobile base',joint_names=trace.metadata['joint_names'],instances=instances,objects=[dict(id='panel',index=panel_index,poses=trace.object_pose[idx,0].tolist(),boxes=boxes)],times=trace.time[idx].round(6).tolist(),q=q.round(9).tolist(),positions=positions,link_names=names,edges=edges,torques=torques.tolist(),torque_ratio=np.max(abs(torques)/limits,axis=1).tolist(),errors=[None]*len(idx),chart_values=(trace.object_pose[idx,1,0]-trace.object_pose[0,1,0]).tolist(),chart_label='MEASURED MOBILE BASE TRAVEL',chart_unit='m',keyframes=case.data['keyframes'],events=[dict(id='release_fixed_arm',time=case.data['fixed_release']),dict(id='release_RBY_tools',time=case.data['release_time'])],tool_states={name:flags[idx,i].tolist() for i,name in enumerate(['rby_gripper','rby_support','fixed_arm_support'])},environment=[dict(type='box',**f,color='#425469') for f in trace.metadata['fixtures']],summary=dict(duration=float(trace.time[-1]),physics_dt=trace.metadata['timestep'],primal_residual=plan.residual,full_geometry_checked=False),report=report,
              description='Differential-drive RBY1-A and one fixed Panda cooperate on a free panel. Panda supports the initial lift and hands over; RBY1 drives beyond fixed-base reach, seats the panel and releases both tools. Mobile base and panel poses are measured, never prescribed. Synthetic source refinement and target wrench allocation run with supplied path/modes. Chassis passive supports use low-friction skid approximation; wheel torque bounds and inner servo gains are simulation assumptions. No hardware execution or continuous geometry certificate.')
    (web/'data/mobile_panel.json').write_text(json.dumps(data,separators=(',',':'),allow_nan=False));p=web/'data/index.json';catalog=json.loads(p.read_text()) if p.exists() else {'models':[]};catalog['models']=[{k:data[k] for k in ['id','title','engine','dof','floating_base']}]+[m for m in catalog['models'] if m['id']!='mobile_panel'];p.write_text(json.dumps(catalog,indent=2)+'\n');return data


def load_saved_mobile(output):
    root=Path(output);case=MobileCase.load(root/'case.json');a=np.load(root/'plan.npz');meta=json.loads((root/'planning.json').read_text());return MobilePlan(case,a['time'],a['q'],a['v'],a['a'],a['reactions'],a['torque'],meta['residual'],meta['source'],meta['workspace'])


def export_saved_mobile(output,web_directory):
    root=Path(output);plan=load_saved_mobile(root);trace=ExecutionTrace.load(root/'execution.npz');report=json.loads((root/'report.json').read_text());flags=np.load(root/'mobile_metrics.npz')['contact_flags'];return export_mobile_view(plan,trace,report,flags,web_directory)
