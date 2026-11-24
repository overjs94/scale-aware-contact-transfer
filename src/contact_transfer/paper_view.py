"""Publish measured first-interval multi-arm plans, with tool provenance."""
import hashlib
import json
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
from .urdf import origin
from .paper_validation import validate_rigid_trajectory
from .paper_recording import model_fingerprint


def export_coupled_lift(episode,trace,report,web_directory,frames=401):
    """Export actual native joint/free-body poses; retain all acceptance flags."""
    web=Path(web_directory);case=episode.problem.case
    robotiq=case.data['arms'][0].get('gripper_model')=='robotiq_2f85'
    dataset='panda_robotiq_lift' if robotiq else 'panda_coupled_lift'
    idx=np.unique(np.linspace(0,len(trace.time)-1,frames).round().astype(int))
    positions=[[] for _ in idx];names=[];edges=[];instances=[];effort=[];offset=0
    for robot,spec in zip(case.full_robots,case.data['arms']):
        links=list(robot.tree.links);start=len(names);names.extend(spec['id']+'_'+n for n in links)
        edges.extend([start+links.index(j.parent),start+links.index(j.child)] for j in robot.tree.joints)
        for row,k in zip(positions,idx):row.extend(robot.fk(trace.q[k,offset:offset+robot.n],n)[:3,3].tolist() for n in links)
        model_id=dataset+'_'+spec['id'];directory=web/'models'/model_id;directory.mkdir(parents=True,exist_ok=True)
        shutil.copyfile(robot.path,directory/'robot.urdf');shutil.copyfile(case.path.parent/'LICENSE.panda',directory/'LICENSE.upstream')
        source=json.loads((case.path.parent/'franka_panda_provenance.json').read_text())
        if spec.get('gripper_profile'):
            native=json.loads((web/'models/robotiq_2f85/model.json').read_text())
            source['packages'].update(native['packages']);source['assets']+=native['assets']
            shutil.copyfile(web/'models/robotiq_2f85/LICENSE.upstream',directory/'LICENSE.robotiq')
            source.update(tool_source=native,additional_licenses=[dict(license='BSD-2-Clause',license_file='LICENSE.robotiq')],modifications=source.get('modifications',[])+['Replace Panda hand by native Robotiq 2F-85 linkage; derive inertias and joint frames from pinned MJCF'])
        source.update(id=model_id,urdf='robot.urdf',urdf_sha256=hashlib.sha256(robot.path.read_bytes()).hexdigest(),license_file='LICENSE.upstream')
        (directory/'model.json').write_text(json.dumps(source,indent=2)+'\n')
        instances.append(dict(id=spec['id'],role=spec['kind']+'_contact_arm',manifest=f'./models/{model_id}/model.json',joint_names=robot.joint_names,dof=robot.n,floating_base=False,q_indices=list(range(offset,offset+robot.n)),base_pose=dict(position=robot.base_pose[:3,3].tolist(),quaternion_xyzw=Rotation.from_matrix(robot.base_pose[:3,:3]).as_quat().tolist())))
        offset+=robot.n;effort.extend(robot.effort)
    directory=web/'models'/(dataset+'_panel');directory.mkdir(parents=True,exist_ok=True);shutil.copyfile(case.panel.path,directory/'robot.urdf');shutil.copyfile(case.path.parent/'LICENSE.authored',directory/'LICENSE.authored')
    (directory/'model.json').write_text(json.dumps(dict(id=directory.name,urdf='robot.urdf',packages={},source='Authored rigid panel',license='MIT',license_file='LICENSE.authored',urdf_sha256=hashlib.sha256(case.panel.path.read_bytes()).hexdigest()),indent=2)+'\n')
    instances.append(dict(id='panel',role='free_furniture',manifest=f'./models/{directory.name}/model.json',joint_names=[],dof=0,floating_base=True,q_indices=list(range(offset,offset+7))))
    panel_index=len(names);names.append('panel_panel')
    for row,k in zip(positions,idx):row.append(trace.object_pose[k,0,:3].tolist())
    boxes=[]
    for vis in ET.parse(case.panel.path).findall('link/visual'):
        R,p=origin(vis);boxes.append(dict(size=np.fromstring(vis.find('geometry/box').get('size'),sep=' ').tolist(),position=p.tolist(),quaternion_xyzw=Rotation.from_matrix(R).as_quat().tolist()))
    torque=np.r_[trace.torque,trace.torque[-1:]][idx];effort=np.array(effort);ratio=np.max(np.divide(abs(torque),effort,where=effort>0,out=np.zeros_like(torque)),axis=1)
    plan=episode.trajectories[0];q=np.c_[trace.q[idx],trace.object_pose[idx,0]]
    enriched={**report,'independent_plan':validate_rigid_trajectory(episode.problem,plan,episode.contacts_by_interval[0]),'algorithm_validation_passed':False,'full_paper_pipeline_executed':False,'scope':'first lift, supplied G/P/P assignment, independent midpoint configuration variant'}
    data=dict(id=dataset,title='Panda × 3 · '+('Robotiq 2F-85 · ' if robotiq else '')+'최적화된 첫 lift',scenario='coupled_panel_lift',engine='mujoco_contact_recording',multi_arm=True,dof=offset,nq=q.shape[1],nv=offset+6,floating_base=False,frame='optimized multi-arm panel lift',joint_names=trace.metadata['joint_names'],instances=instances,objects=[dict(id='panel',index=panel_index,poses=trace.object_pose[idx,0].tolist(),boxes=boxes)],times=trace.time[idx].tolist(),q=q.tolist(),positions=positions,link_names=names,edges=edges,torques=torque.tolist(),torque_ratio=ratio.tolist(),errors=[None]*len(idx),chart_values=(trace.object_pose[idx,0,2]*1000).tolist(),chart_label='MEASURED PANEL HEIGHT',chart_unit='mm',keyframes=[dict(id='initial_grasp',time=episode.preparation),dict(id='lift_complete',time=episode.preparation+episode.time[-1])],events=[],tool_states={},environment=[dict(type='box',**f,color='#425469') for f in trace.metadata['fixtures']],summary=dict(duration=float(trace.time[-1]),physics_dt=trace.metadata['timestep'],full_geometry_checked=plan.full_geometry_checked),report=enriched,description='Optimized first lift interval with a supplied G/P/P assignment. Raw native articulated-joint and free-panel poses, not commanded object motion. Independent midpoint configuration variant; not the full paper sequence or III-D search.',display=dict(description='3대 Panda가 grip + 두 point support로 패널을 들어 올립니다. 실제 기구·접촉 물리에서 측정한 동작이며 첫 lift 구간만 검증합니다.',scene_label='3-ARM PANEL LIFT · '+('ROBOTIQ 2F-85 / POINT / POINT' if robotiq else 'GRIPPER / POINT / POINT'),geometry='원본 URDF·mesh · 수동 그리퍼 링크를 포함한 native 관절 및 자유 패널 기록',chart_note='측정된 패널 높이 · source refinement + target realization + 첫 구간 CITO · midpoint 구성 업데이트 변형'),provenance=dict(case_id=trace.metadata['case_id'],plan_id=trace.metadata['plan_id'],source_model_fingerprint=model_fingerprint(episode.problem)))
    (web/'data').mkdir(parents=True,exist_ok=True);(web/f'data/{dataset}.json').write_text(json.dumps(data,separators=(',',':'),allow_nan=False))
    path=web/'data/index.json';catalog=json.loads(path.read_text());catalog['models']=[{k:data[k] for k in ['id','title','engine','dof','floating_base']}]+[m for m in catalog['models'] if m['id']!=dataset];path.write_text(json.dumps(catalog,indent=2)+'\n');return data
