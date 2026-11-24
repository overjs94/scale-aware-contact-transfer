"""Recompute and archive independent conic witnesses for qualified source sites.

This does not rerun source geometry optimization. Each artifact contains the
actual numerical maps, loads, circular-cone limits and every vertex reaction.
"""
import argparse
import json
from pathlib import Path

import numpy as np

from contact_transfer.contacts import Capability,TaskWrench,contact_basis,gravity_wrench,map_reaction
from contact_transfer.full_recording import atomic_npz,write_json
from contact_transfer.full_scenario import FullTask,FullProblem
from contact_transfer.generalized_conic import fixed_generalized_margin
from contact_transfer.grouped_source import native_map,verify_vertex_witnesses
from contact_transfer.multi_object import RigidCollection
from contact_transfer.robot import Robot
from contact_transfer.source_transfer import load_source


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root',type=Path,default=Path('outputs/research-study/sources-final'))
    args=parser.parse_args();summary=[]
    for name in ['panel','box','assembly']:
        folder=args.root/name/('representative' if name=='panel' else 'functional_sites')
        data=json.loads((folder/'result.json').read_text())
        task=FullTask.load(name,folder/'witness-models/task');p=FullProblem.create(task,folder/'witness-models/planning',task.configuration.default_types)
        if name=='panel':fixture=load_source(task.original.path.parent/task.original.data['source_file'])
        elif name=='box':src=Robot(folder/'planning/source_box.urdf',floating_base=True)
        else:src=RigidCollection({part:Robot(folder/f'planning/source_{part}.urdf',floating_base=True) for part in task.object.names})
        for i,row in enumerate(data['keyframes']):
            points=np.asarray(row['source_positions_after_m']);normals=np.asarray(row['source_normals'])
            rotations=[contact_basis(n) for n in normals]
            if name=='panel':
                mass=row['source_object']['mass'];com=row['source_object']['com'];R=np.asarray(row['source_object_rotation_world'])
                load=TaskWrench(gravity_wrench(mass,com,R),np.eye(6),np.diag(fixture['disturbance_scale']))
                maps=[np.asarray(map_reaction(point,rotation,np.eye(6))) for point,rotation in zip(points,rotations)]
                cfg=row['effective_source_refinement'];caps=[Capability(mu=cfg['mu'],normal_max=cfg['normal_max_N']) for _ in points];rho=row['source_rho_after']
            else:
                qo=task.keyframes[i]['object_q'];obj=task.object
                qs=np.concatenate([np.r_[qo[s][:3]/task.scale,qo[s][3:]] for s in obj.q_slices.values()]) if isinstance(obj,RigidCollection) else np.r_[qo[:3]/task.scale,qo[3:]]
                load=p.task_load(qs,source_object=src)
                maps=[native_map(src,qs,frame,point,rotation) for frame,point,rotation in zip(row['source_site_frames'],points,rotations)]
                caps=[Capability(mu=row['source_friction'],normal_max=limit) for limit in row['source_normal_limits_N']];rho=row['source_rho']
            result=fixed_generalized_margin(maps,load,caps,fixed_rho=rho)
            check=verify_vertex_witnesses(maps,load,caps,rho,result['reactions'])
            if not result['certificate'] or not check['passed']:raise RuntimeError(f'{name}/{i}: independent source witness rejected: {result["status"]}, {check}')
            artifact=folder/f'key{i}-source-witness.npz'
            atomic_npz(artifact,maps=np.asarray(maps),nominal=np.asarray(load.nominal),E=np.asarray(load.E),D=np.asarray(load.D),vertices=np.asarray(load.vertices),rho=np.array(rho),reactions=result['reactions'],mu=np.array([c.mu for c in caps]),normal_max=np.array([c.normal_max for c in caps]),positions=points,normals=normals)
            summary.append(dict(task=name,keyframe=row['keyframe'],artifact=str(artifact),rho=rho,independent=check,conic_status=result['status'],new_geometry_optimization_executed=False))
            print(name,row['keyframe'],result['status'],flush=True)
    write_json(args.root/'source-witness-summary.json',summary)


if __name__=='__main__':main()
