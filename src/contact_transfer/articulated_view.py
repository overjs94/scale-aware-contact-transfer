"""URDF playback of measured passive joint motion, not the planned object path."""
import hashlib
import json
import shutil
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
from .articulated import ArticulatedCase, ArticulatedPlan
from .execution import ExecutionTrace
from .urdf import origin

TITLES = {'hinged_door':'수동 힌지 · 문 열기', 'folding_panel':'접이식 상판 펼치기', 'drawer':'서랍 당기기'}

def boxes(link):
    result=[]
    for visual in link.findall('visual'):
        box=visual.find('geometry/box')
        if box is None:continue
        R,p=origin(visual)
        result.append(dict(size=np.fromstring(box.get('size'),sep=' ').tolist(),position=p.tolist(),quaternion_xyzw=Rotation.from_matrix(R).as_quat().tolist()))
    return result

def export_articulated_view(plan,trace,report,flags,web_directory,frames=421):
    web=Path(web_directory);(web/'data').mkdir(parents=True,exist_ok=True)
    case=plan.case;d=case.data;id=d['id'];idx=np.unique(np.linspace(0,len(trace.time)-1,frames).round().astype(int))
    coordinate=np.asarray(trace.metadata['articulated_state']['q'])[:,0]
    q=np.c_[trace.q[idx],coordinate[idx]];positions=[[] for _ in idx];names=[];edges=[];instances=[]
    for name,robot,role in [('robot',case.full,'contact_arm'),('object',case.object,'articulated_object')]:
        model_id='articulated_'+(d['tool'].lower()+'_panda' if name=='robot' else id)
        directory=web/'models'/model_id;directory.mkdir(parents=True,exist_ok=True);shutil.copyfile(robot.path,directory/'robot.urdf')
        license_name='LICENSE.panda' if name=='robot' else 'LICENSE.authored';shutil.copyfile(case.path.parent/license_name,directory/license_name)
        meta=json.loads((case.path.parent/'franka_panda_provenance.json').read_text()) if name=='robot' else dict(packages={},source='Authored passive joint furniture test fixture',license='MIT')
        meta.update(id=model_id,urdf='robot.urdf',license_file=license_name,urdf_sha256=hashlib.sha256(robot.path.read_bytes()).hexdigest())
        (directory/'model.json').write_text(json.dumps(meta,indent=2)+'\n')
        indices=list(range(case.full.n)) if name=='robot' else [case.full.n]
        instances.append(dict(id=name,role=role,manifest=f'./models/{model_id}/model.json',joint_names=robot.joint_names,dof=robot.n,floating_base=False,q_indices=indices,base_pose=dict(position=robot.base_pose[:3,3].tolist(),quaternion_xyzw=Rotation.from_matrix(robot.base_pose[:3,:3]).as_quat().tolist())))
        links=list(robot.tree.links);start=len(names);names += [name+'_'+link for link in links]
        edges += [[start+links.index(j.parent),start+links.index(j.child)] for j in robot.tree.joints]
        for row,k in zip(positions,idx):row += [robot.fk(trace.q[k] if name=='robot' else [coordinate[k]],link)[:3,3].tolist() for link in links]
    tree=ET.parse(case.object.path);torque=np.r_[trace.torque,trace.torque[-1:]][idx];angular=id!='drawer'
    data=dict(id=id,title=TITLES[id]+f" · {d['duration']:g} s",scenario=id,articulated_object=True,engine='mujoco_contact_recording',dof=case.full.n,nq=q.shape[1],nv=q.shape[1],floating_base=False,frame='world / passive object joint',joint_names=case.full.joint_names+['object_joint'],instances=instances,objects=[dict(id='moving',index=names.index('object_moving'),poses=trace.object_pose[idx,0].tolist(),boxes=boxes(tree.find("link[@name='moving']")))],times=trace.time[idx].round(6).tolist(),q=q.round(9).tolist(),positions=positions,link_names=names,edges=edges,torques=torque.tolist(),torque_ratio=np.max(abs(torque)/case.full.effort,axis=1).tolist(),errors=[None]*len(idx),chart_values=(np.rad2deg(coordinate[idx]) if angular else coordinate[idx]*1000).tolist(),chart_label='MEASURED PASSIVE JOINT',chart_unit='deg' if angular else 'mm',keyframes=[dict(id=label,time=t) for label,t in [('initial_contact',0),('begin_motion',d['motion_start']),('mid_motion',(d['motion_start']+d['motion_end'])/2),('target',d['motion_end']),('hold',d['duration'])]],events=[],tool_states={'sphere contact' if d['tool']=='P' else 'gripper contact':flags[idx].tolist()},environment=[dict(type='box',color='#687888',**box) for box in boxes(tree.find("link[@name='fixture']"))],summary=dict(duration=float(trace.time[-1]),physics_dt=trace.metadata['timestep'],primal_residual=plan.residual,full_geometry_checked=False),report=report,description='Passive '+report['passive_joint_type']+' object joint driven by robot contact. Only robot joints are actuated; measured object motion is replayed from MuJoCo. Supplied link-local contact and joint reference, generalized torque/contact-force optimization. Authored joint friction and spring, no hardware calibration, learned articulated contraction or general multi-joint planning.')
    (web/f'data/{id}.json').write_text(json.dumps(data,separators=(',',':'),allow_nan=False))
    (web/f'data/{id}-preset.json').write_text(json.dumps(d,indent=2)+'\n')
    path=web/'data/index.json';catalog=json.loads(path.read_text()) if path.exists() else {'models':[]}
    catalog['models']=[{k:data[k] for k in ['id','title','engine','dof','floating_base']}]+[m for m in catalog['models'] if m['id']!=id];path.write_text(json.dumps(catalog,indent=2)+'\n');return data

def export_saved_articulated(id,web_directory,output=None):
    root=Path(output or 'outputs/'+id);case=ArticulatedCase.load(id,root/(id+'.json'));a=np.load(root/'plan.npz');plan=ArticulatedPlan(case,a['time'],a['q'],a['v'],a['a'],a['forces'],a['torque'],json.loads((root/'report.json').read_text())['primal_residual'])
    return export_articulated_view(plan,ExecutionTrace.load(root/'execution.npz'),json.loads((root/'report.json').read_text()),np.load(root/'metrics.npz')['contact_flags'],web_directory)
