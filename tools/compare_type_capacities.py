#!/usr/bin/env python3
"""Compare type choices under common frozen anchors, pose, load basis and limits.

Frozen-keyframe costs are explicitly distinct from full-sequence objectives.
Native IK/collision and independent SOCP statuses are retained per candidate.
"""
import argparse,json
from itertools import product
from pathlib import Path
import numpy as np
from export_full_recording import load_recorded_plan
from contact_transfer.full_scenario import FullProblem,point_pose_ik
from contact_transfer.contracted import numerical_pose_ik
from contact_transfer.full_scene import build_scene
from contact_transfer.full_validation import Geometry,numeric_contact
from contact_transfer.full_static import allocation
from contact_transfer.trajectory import Contact
from contact_transfer.full_recording import write_json


def compare(root,task_id,directory):
 p,keys,_,_,report=load_recorded_plan(root,task_id);task=p.task_definition;k={'panel':3,'box':4,'assembly':3}[task_id];key=keys[k];qo=key.q[p.system.object_q];prior=np.array(key.source.get('target_positions_m',[f.surface for f in task.features]));geo=float(np.sum((key.positions-prior)**2)/.6**2);rows=[]
 reference=key.posture_reference
 if reference is None:reference=p.branch_reference(k)[:p.system.object_q.start] if task.posture_reference_rule=='sequential_ik_branch' else np.concatenate([s['q_initial'] for s in p.specs])
 for kinds in product(*(f.allowed_types for f in task.features)):
  row=dict(types=list(kinds),assignment=list(p.assignment),keyframe_index=k,keyframe_id=task.keyframes[k]['id'],source_and_object_pose_held_fixed=True,frozen_material_points_m=key.positions.tolist(),sequence_objective=None,sequence_not_reoptimized=True)
  try:
   candidate=FullProblem.create(task,Path(directory)/task_id/(''.join(kinds)),kinds,p.assignment);q=key.q.copy();contacts=[]
   for i,(f,r,frame,cap,qs) in enumerate(zip(task.features,candidate.robots,candidate.frames,candidate.capabilities,candidate.system.q_slices)):
    T=task.object.fk(qo,f.frame);point=T[:3,3]+T[:3,:3]@key.positions[i];normal=T[:3,:3]@key.rotations[i,:,2];target=np.eye(4);target[:3,3]=point;target[:3,:3]=T[:3,:3]@key.rotations[i]@np.diag([1.,-1.,-1.])
    q[qs]=point_pose_ik(r,frame,point,normal,key.q[p.system.q_slices[i]]) if cap.kind=='P' else numerical_pose_ik(r,frame,target,key.q[p.system.q_slices[i]])
    contacts.append(Contact(f.id,key.positions[i],key.rotations[i,:,2],cap,robot_index=i,frame=frame,object_frame=f.frame,rotation_object=key.rotations[i]))
   scene=build_scene(candidate,candidate.directory/'capacity-scene.xml');geometry=Geometry(candidate,scene);violations=geometry.discover([q]);row['native_geometry_passed']=not violations;row['geometry_violations']=violations
   if violations:raise RuntimeError('Frozen-keyframe native geometry rejected')
   load=candidate.task_load(qo);forces,tau,check,_=allocation(candidate,q,contacts,load,rho_min=0.);rho=check['rho'];posture=0.
   for r,qs in zip(candidate.robots,candidate.system.q_slices):posture+=.01*np.sum(((q[qs]-reference[qs])/np.maximum(r.upper-r.lower,1e-3))**2)
   row.update(planning_passed=bool(rho>=.1-2e-6),stage='independent_frozen_geometry_SOCP',rho=float(rho),rho_required=.1,solver_status=check['status'],primal_residual=check['primal_residual'],frozen_keyframe_cost=float(geo+posture-rho),geometry_cost=geo,posture_cost=float(posture),maximum_margin_reaction_forces_N={name:float(np.linalg.norm(w[:,3:],axis=1).max()) for name,w in forces.items()},maximum_margin_peak_joint_torque_Nm=float(abs(tau).max()))
   if rho>=.1:
    try:
     common_forces,common_tau,common,_=allocation(candidate,q,contacts,load,rho=.1,weighted=True);row['common_rho_metrics']=dict(rho=.1,passed=common['certificate'],peak_joint_torque_Nm=float(abs(common_tau).max()),minimum_joint_torque_margin_Nm=float(np.min(candidate.system.effort-abs(common_tau))),peak_interface_force_N={name:float(np.linalg.norm(w[:,3:],axis=1).max()) for name,w in common_forces.items()})
    except RuntimeError as error:row['common_rho_metrics']=dict(rho=.1,passed=False,failure=str(error),maximum_margin_certificate_unchanged=True)
  except (RuntimeError,ValueError) as error:row.update(planning_passed=False,stage=row.get('stage','IK_geometry_or_allocation'),failure=str(error),physical_execution_performed=False,local_failure_not_global_infeasibility=True)
  rows.append(row)
 result=dict(task=task_id,scope='frozen material geometry and object keyframe; posture re-realized for actual hardware; independent cone and torque constraints',objective_definition='common geometry cost + common posture-reference cost - independently computed rho; not a sequence objective',common_evaluation_rho=.1,selected_types=list(p.types),selected_sequence_objective=report['selection']['objective'],E=p.task_load(qo).E.tolist(),D=p.task_load(qo).D.tolist(),rows=rows)
 write_json(Path(directory)/task_id/'capacity-comparison.json',result);return result


if __name__=='__main__':
 a=argparse.ArgumentParser(description=__doc__);a.add_argument('--output',type=Path,default=Path('outputs/type_capacity_v1'));p=a.parse_args();roots={'panel':'outputs/full_scenarios_v22','box':'outputs/box_connected_v1','assembly':'outputs/full_scenarios_v15'}
 for task,root in roots.items():
  result=compare(root,task,p.output);print(task,[(r['types'],r.get('rho'),r['planning_passed']) for r in result['rows']],flush=True)
