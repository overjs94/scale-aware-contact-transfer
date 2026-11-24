"""Publish measured single-panel execution with original URDF/mesh manifests."""
import hashlib,json,shutil
import xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from .execution import ExecutionTrace
from .urdf import origin
from .panel_fixture import PanelCase,PanelPlan


def export_panel_view(plan,trace,report,flags,web_directory,frames=601):
    web=Path(web_directory);(web/'data').mkdir(parents=True,exist_ok=True);idx=np.unique(np.linspace(0,len(trace.time)-1,frames).round().astype(int))
    positions=[[] for _ in idx];names=[];edges=[];instances=[];offset=0;effort=[]
    for robot,spec in zip(plan.case.full_robots,plan.case.data['arms']):
        links=list(robot.tree.links);start=len(names);names += [spec['id']+'_'+n for n in links];edges += [[start+links.index(j.parent),start+links.index(j.child)] for j in robot.tree.joints]
        for row,k in zip(positions,idx):row += [robot.fk(trace.q[k,offset:offset+robot.n],n)[:3,3].tolist() for n in links]
        model_id='panel_'+spec['id'];directory=web/'models'/model_id;directory.mkdir(parents=True,exist_ok=True);shutil.copyfile(robot.path,directory/'robot.urdf')
        license_name='LICENSE.rb5' if spec['model']=='rb5' else 'LICENSE.panda';shutil.copyfile(plan.case.path.parent/license_name,directory/'LICENSE.upstream')
        source=json.loads((plan.case.path.parent/(spec['model']+'_provenance.json')).read_text());source.update(id=model_id,urdf='robot.urdf',urdf_sha256=hashlib.sha256(robot.path.read_bytes()).hexdigest(),license_file='LICENSE.upstream')
        (directory/'model.json').write_text(json.dumps(source,indent=2)+'\n')
        instances.append(dict(id=spec['id'],role='panel_'+spec['kind'],manifest=f'./models/{model_id}/model.json',joint_names=robot.joint_names,dof=robot.n,floating_base=False,q_indices=list(range(offset,offset+robot.n)),base_pose=dict(position=robot.base_pose[:3,3].tolist(),quaternion_xyzw=Rotation.from_matrix(robot.base_pose[:3,:3]).as_quat().tolist())))
        effort.extend(robot.effort);offset+=robot.n
    # Both the free panel and static jig are genuine URDF instances.
    for name,file in [('panel',plan.case.data['panel_urdf']),('jig',plan.case.data['jig_urdf'])]:
        directory=web/'models'/('fixture_'+name);directory.mkdir(parents=True,exist_ok=True);path=plan.case.path.parent/file;shutil.copyfile(path,directory/'robot.urdf');shutil.copyfile(plan.case.path.parent/'LICENSE.authored',directory/'LICENSE.authored')
        (directory/'model.json').write_text(json.dumps(dict(id='fixture_'+name,urdf='robot.urdf',packages={},urdf_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),source='authored single-rigid-panel fixture task',license='MIT',license_file='LICENSE.authored'),indent=2)+'\n')
        spec=dict(id=name,role='free_furniture' if name=='panel' else 'static_jig',manifest=f'./models/fixture_{name}/model.json',joint_names=[],dof=0,floating_base=name=='panel',q_indices=list(range(offset,offset+7)) if name=='panel' else [])
        if name=='jig':spec['base_pose']=dict(position=[0,0,0],quaternion_xyzw=[0,0,0,1])
        instances.append(spec)
    index=len(names);names.append('panel_panel')
    for row,k in zip(positions,idx):row.append(trace.object_pose[k,0,:3].tolist())
    names.append('jig_jig')
    for row in positions:row.append([0.,0.,0.])
    q=np.c_[trace.q[idx],trace.object_pose[idx,0]];torques=np.r_[trace.torque,trace.torque[-1:]][idx]
    boxes=[]
    for g in ET.parse(plan.case.path.parent/plan.case.data['panel_urdf']).findall('link/visual'):
        R,p=origin(g);boxes.append(dict(size=np.array(g.find('geometry/box').get('size').split(),float).tolist(),position=p.tolist(),quaternion_xyzw=Rotation.from_matrix(R).as_quat().tolist()))
    data=dict(id='panel_fixture',title='패널 → 지그 · Franka × 3 · 30 s',engine='mujoco_contact_recording',scenario='panel_fixture',dof=sum(r.n for r in plan.case.full_robots),nq=q.shape[1],nv=trace.q.shape[1]+6,floating_base=False,frame='panel / fixed jig',joint_names=trace.metadata['joint_names'],instances=instances,
              objects=[dict(id='panel',index=index,poses=trace.object_pose[idx,0].tolist(),boxes=boxes)],times=trace.time[idx].round(6).tolist(),q=q.round(9).tolist(),positions=positions,link_names=names,edges=edges,torques=torques.tolist(),torque_ratio=np.max(abs(torques)/effort,axis=1).tolist(),errors=[None]*len(idx),
              chart_values=(trace.object_pose[idx,0,2]*1000).tolist(),chart_label='MEASURED PANEL HEIGHT',chart_unit='mm',keyframes=plan.case.data['keyframes'],events=[dict(id='release_all_tools',time=plan.case.data['simulation']['release_time'])],tool_states={s['id']+'_contact':flags[idx,i].all(axis=1).tolist() for i,s in enumerate(plan.case.data['arms'])},
              environment=[dict(type='box',**f,color='#425469') for f in trace.metadata['fixtures']],summary=dict(duration=float(trace.time[-1]),physics_dt=trace.metadata['timestep'],full_geometry_checked=False,keyframe_errors=report['keyframe_errors'],primal_residual=report['primal_residual']),report=report,
              description='Main single-panel baseline: supplied initial grasp, lift, 12-degree reorientation, seat on a fixed jig, transfer load and release all tools. Synthetic quarter-scale contracted anchors are optimized before geometry-only transfer. Target reactions and torques are recomputed with URDF mechanics. Recorded free-body MuJoCo execution; no weld or artificial object forces. Supplied SE3 path/modes; no network inference, articulated CITO search or hardware trial.')
    (web/'data/panel_fixture.json').write_text(json.dumps(data,separators=(',',':'),allow_nan=False));index_path=web/'data/index.json';catalog=json.loads(index_path.read_text()) if index_path.exists() else {'models':[]};catalog['models']=[{k:data[k] for k in ['id','title','engine','dof','floating_base']}]+[m for m in catalog['models'] if m['id']!='panel_fixture'];index_path.write_text(json.dumps(catalog,indent=2)+'\n');return data


def export_saved_panel(output,web_directory):
    root=Path(output);case=PanelCase.load(root/'case.json');a=np.load(root/'plan.npz');report=json.loads((root/'report.json').read_text());offsets=np.cumsum([0]+[r.n for r in case.robots]);plan=PanelPlan(case,a['time'],a['q'],a['v'],a['a'],a['reactions_world'],a['torque'],report['primal_residual'],tuple(slice(x,y) for x,y in zip(offsets[:-1],offsets[1:])),report['source'],report['target_validation']);trace=ExecutionTrace.load(root/'execution.npz');flags=np.load(root/'panel_metrics.npz')['contact_flags'];return export_panel_view(plan,trace,report,flags,web_directory)
