"""Measured full-task URDF playback and explicit selection/ablation provenance."""
from pathlib import Path
import hashlib,json,shutil
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
from .articulated_view import boxes
from .multi_object import RigidCollection
from .full_assessment import frame_index
from .full_recording import write_json


TITLES={'panel':'패널 · 운반·재배향·안착·릴리스','box':'자유 박스 · 본체 지지·뚜껑 열기','assembly':'가구 조립 · 상판 지지·다리 삽입·릴리스'}


def export_full(episode,trace,report,scene,web_directory,*,variant='full',frames=601,ablation_results=None):
    web=Path(web_directory);p=episode.problem;task=p.task_definition;dataset='full_'+task.id+('' if variant=='full' else '_'+variant);idx=np.unique(np.linspace(0,len(trace.time)-1,frames).round().astype(int));qparts=[trace.q[idx]];robots=[(s['id'],r,s) for r,s in zip(p.full_robots,p.specs)];obj=task.object
    if isinstance(obj,RigidCollection):
        for name,r in obj.bodies.items():qparts.append(trace.object_pose[idx,frame_index(episode,trace,scene,name)]);robots.append((name,r,None))
    else:
        pose=trace.object_pose[idx,trace.metadata['object_names'].index(next(iter(scene.object_roots.values())))];coordinate=np.array(trace.metadata['articulated_state']['q'])[idx] if task.id=='box' else np.zeros((len(idx),0));qparts.append(np.c_[pose,coordinate]);robots.append(('object',obj,None))
    q=np.concatenate(qparts,axis=1);positions=[[] for _ in idx];names=[];edges=[];instances=[];offset=0;objects=[]
    for name,r,spec in robots:
        model_id=dataset+'_'+name;directory=web/'models'/model_id;directory.mkdir(parents=True,exist_ok=True);shutil.copyfile(r.path,directory/'robot.urdf');authored=spec is None;lic='LICENSE.authored' if authored else 'LICENSE.panda';shutil.copyfile(task.original.path.parent/lic,directory/lic);meta=dict(packages={},source='Authored manipulation object; dynamics and collision geometry from URDF',license='MIT') if authored else json.loads((task.original.path.parent/'franka_panda_provenance.json').read_text())
        if spec is not None:
            shutil.copyfile(task.original.path.parent/'LICENSE.authored',directory/'LICENSE.authored');meta.setdefault('additional_licenses',[]).append(dict(license='MIT',license_file='LICENSE.authored'))
            if spec.get('gripper_profile'):
                source=json.loads((web/'models/robotiq_2f85/model.json').read_text());meta['packages'].update(source['packages']);meta['assets']+=source['assets'];shutil.copyfile(web/'models/robotiq_2f85/LICENSE.upstream',directory/'LICENSE.robotiq');meta['additional_licenses'].append(dict(license='BSD-2-Clause',license_file='LICENSE.robotiq'));meta['tool_source']=source
            else:meta['point_tool']=spec['point_tool_profile']
        meta.update(id=model_id,urdf='robot.urdf',license_file=lic,urdf_sha256=hashlib.sha256(r.path.read_bytes()).hexdigest());write_json(directory/'model.json',meta);indices=list(range(offset,offset+r.nq));offset+=r.nq;instance=dict(id=name,role='free_articulated_object' if authored and task.id=='box' else 'free_furniture' if authored else 'contact_arm',manifest=f'./models/{model_id}/model.json',joint_names=r.joint_names,dof=r.n,floating_base=r.floating_base,q_indices=indices)
        if not r.floating_base:instance['base_pose']=dict(position=r.base_pose[:3,3].tolist(),quaternion_xyzw=Rotation.from_matrix(r.base_pose[:3,:3]).as_quat().tolist())
        instances.append(instance);links=list(r.tree.links);start=len(names);names.extend(name+'_'+n for n in links);edges.extend([start+links.index(j.parent),start+links.index(j.child)] for j in r.tree.joints)
        for row,k in zip(positions,range(len(idx))):row.extend(r.fk(q[k,indices],n)[:3,3].tolist() for n in links)
        if authored:
            for link in links:
                key=name if isinstance(obj,RigidCollection) else link
                if key not in scene.object_frames:continue
                objects.append(dict(id=key,index=start+links.index(link),native_body=scene.object_frames[key],poses=trace.object_pose[idx,frame_index(episode,trace,scene,key)].tolist(),boxes=boxes(ET.parse(r.path).find(f"link[@name='{link}']"))))
    torque=np.r_[trace.torque,trace.torque[-1:]][idx];limits=np.concatenate([r.effort for r in p.full_robots]);ratio=np.max(np.divide(abs(torque),limits,out=np.zeros_like(torque),where=limits>0),axis=1);selection=report['selection'];contraction=dict(synthetic_input=True,network_inference_executed=False,contracted_contact_count=len(task.features),dense_sample_count=0,source_to_target_scale=task.scale,groups=[],assignment=[])
    for i,(f,spec) in enumerate(zip(task.features,p.specs)):
        # Anchors are actual optimized target material points; dense network
        # samples are absent in this supplied-contracted-contact experiment.
        contraction['groups'].append(dict(group_id=f.id,link_id=f.frame,target_points_m=[],target_anchor_m=episode.keyframes[0].positions[i].tolist()));contraction['assignment'].append(dict(group_id=f.id,arm_id='physical_arm_'+str(p.assignment[i]),kind=p.types[i]))
    chart=np.rad2deg(np.array(trace.metadata['articulated_state']['q'])[idx,0]) if task.id=='box' else trace.object_pose[idx,0,2]*1000;environment=[dict(type='box',**f,color='#596d7d') for f in scene.fixtures];environment.extend(dict(type='box',position=[*r.base_pose[:2,3],r.base_pose[2,3]/2],size=[.3,.3,r.base_pose[2,3]],quaternion_xyzw=[0,0,0,1],color='#687888') for r in p.full_robots if r.base_pose[2,3]>0);enriched={**report,'ablation_results':ablation_results};events=[dict(id='외력 적용',time=pulse['start']+pulse['duration']/2) for pulse in trace.metadata.get('external_disturbances',[])];data=dict(id=dataset,title=TITLES[task.id]+(' · '+variant if variant!='full' else ''),scenario='full_'+task.id,full_task_recording=True,multi_arm=True,free_articulated_object=task.id=='box',articulated_object=task.id=='box',engine='mujoco_contact_recording',dof=sum(r.n for r in p.full_robots),nq=q.shape[1],nv=sum(r.nv for r in p.full_robots)+obj.nv,floating_base=False,frame='supplied contracted groups -> source/target -> type/assignment search -> all intervals -> native control',joint_names=trace.metadata['joint_names'],instances=instances,objects=objects,times=trace.time[idx].tolist(),q=q.tolist(),positions=positions,link_names=names,edges=edges,torques=torque.tolist(),torque_ratio=ratio.tolist(),errors=[None]*len(idx),chart_values=chart.tolist(),chart_label='MEASURED HINGE ANGLE' if task.id=='box' else 'MEASURED OBJECT HEIGHT',chart_unit='deg' if task.id=='box' else 'mm',keyframes=[dict(id=k['id'],time=episode.preparation+episode.offsets[i]) for i,k in enumerate(task.keyframes)],events=events,tool_states={f.id:[episode.attached(i,t) for t in trace.time[idx]] for i,f in enumerate(task.features)},environment=environment,contact_contraction=contraction,external_disturbances=trace.metadata.get('external_disturbances',[]),summary=dict(duration=float(trace.time[-1]),physics_dt=trace.metadata['timestep'],full_geometry_checked=True),report=enriched,description='All task keyframes and transitions with supplied contracted groups. See selection ledger, source fallback flags, independent mechanics, midpoint variant and raw native-contact assessment. Authored simulation parameters; hardware calibration not claimed.',display=dict(description=TITLES[task.id],scene_label='THREE-ARM FULL TASK · '+' / '.join(p.types),geometry='Panda·Robotiq 원본 mesh + 동일 spherical tip URDF · 측정된 native 관절과 자유 물체',chart_note='유형·팔 배정 후보 탐색 및 전체 구간 최적화 결과 · midpoint 구성 업데이트 변형'),provenance=dict(case_id=trace.metadata['case_id'],plan_id=trace.metadata['plan_id'],scene_sha256=trace.metadata['scene_sha256'],selected_types=p.types,selected_assignment=p.assignment,canonical_point_tool=next((s['point_tool_profile'] for s in p.specs if s['kind']=='P'),None)))
    path=web/'data'/f'{dataset}.json';path.parent.mkdir(parents=True,exist_ok=True);path.write_text(json.dumps(data,separators=(',',':'),allow_nan=False));index=web/'data/index.json';catalog=json.loads(index.read_text());catalog['models']=[{k:data[k] for k in ['id','title','engine','dof','floating_base']}]+[m for m in catalog['models'] if m['id']!=dataset];write_json(index,catalog);return data
