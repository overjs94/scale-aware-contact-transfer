"""Recheck accepted reference geometry under the controller's jaw budget."""
import argparse
from pathlib import Path
import json
from types import SimpleNamespace

import numpy as np

from export_full_recording import load_recorded_plan
from contact_transfer.full_recording import write_json
from contact_transfer.full_validation import numeric_contact
from contact_transfer.generalized_conic import fixed_generalized_margin
from contact_transfer.online_wrench import operating_capabilities
from contact_transfer.trajectory import Contact


def certify(root,output,task,fraction):
    p,keys,_,_,_=load_recorded_plan(root,task,model_directory=output/task/'models')
    key=keys[2];sys=p.system;maps=[];tmaps=[]
    if key.environment_reactions:raise ValueError('This check requires a pre-handoff keyframe')
    for i,(f,frame,cap,point,R) in enumerate(zip(p.task_definition.features,p.frames,p.capabilities,key.positions,key.rotations)):
        c=Contact(f.id,point,R[:,2],cap,robot_index=i,frame=frame,object_frame=f.frame,rotation_object=R)
        _,J,_,_=numeric_contact(p,c,key.q);X=np.c_[np.zeros((sys.nv,3)),J.T] if cap.kind=='P' else J.T
        maps.append(-X[sys.object_v]);tmaps.append(np.asarray(sys.B).T@X)
    caps=operating_capabilities(p.hardware_capabilities,fraction);g=np.concatenate([r.gravity(key.q[s]) for r,s in zip(p.robots,sys.q_slices)])
    load=p.task_load(key.q[sys.object_q]);kw=dict(torque_gravity=g,torque_maps=tmaps,effort=sys.effort)
    margin=fixed_generalized_margin(maps,load,caps,**kw)
    attempts=[dict(status=margin['status'],rho=margin['rho'],residual=margin['primal_residual'],certificate=margin['certificate'])]
    if not margin['certificate']:
        # A failed maximum solve is not a bound. Obtain an explicit feasible
        # lower-bound witness, retaining every unsuccessful attempt.
        for scale in [.95,.8,.5,0.]:
            candidate=max(0.,margin['rho'])*scale
            trial=fixed_generalized_margin(maps,load,caps,fixed_rho=candidate,**kw)
            attempts.append(dict(status=trial['status'],rho=trial['rho'],residual=trial['primal_residual'],certificate=trial['certificate']))
            if trial['certificate']:margin=trial;break
    protocol=json.loads((Path('outputs/research-study/probes-v1')/task/'protocol.json').read_text())
    checks={}
    for name in ['on_interior','opposite_interior']:
        coefficient=np.array(protocol['cases'][name]);rhs=load.nominal+load.E@load.D@coefficient
        frozen=SimpleNamespace(nominal=rhs,E=np.zeros((len(rhs),1)),D=np.eye(1),vertices=np.zeros((1,1)))
        result=fixed_generalized_margin(maps,frozen,caps,fixed_rho=0.,**kw)
        checks[name]=dict(coefficient=coefficient,certificate=result['certificate'],status=result['status'],residual=result['primal_residual'],reactions=result['reactions'],finger_coordinates=result['coordinates'])
    record=dict(task=task,reference_keyframe_index=2,geometry_reoptimized=False,new_static_SOCP_executed=True,original_optimized_rho=key.rho,operating_gripper_fraction=fraction,hardware_jaw_limit_N=[c.closing_force_per_finger if c.kind=='G' else None for c in p.hardware_capabilities],operating_rho=margin['rho'],operating_certificate=margin['certificate'],operating_residual=margin['primal_residual'],operating_vertex_reactions=margin['reactions'],operating_vertex_coordinates=margin['coordinates'],operating_dimensionless_vertices=load.vertices,operating_solver_attempts=attempts,reported_margin_is_lower_bound=True,physical_load_checks=checks,scope='fixed reference pose static certificate, not closed-loop or full-trajectory guarantee')
    write_json(output/task/'certificate.json',record)
    print(task,margin['certificate'],margin['rho'],{k:v['certificate'] for k,v in checks.items()},flush=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--root',type=Path,default=Path('outputs/research-study/full-common'));parser.add_argument('--output',type=Path,default=Path('outputs/controller-study/envelope'));parser.add_argument('--fraction',type=float,default=.6);args=parser.parse_args()
    for task in ['box','panel','assembly']:certify(args.root,args.output,task,args.fraction)
