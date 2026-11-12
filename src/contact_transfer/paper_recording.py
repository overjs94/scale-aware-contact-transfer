"""Pickle-free, explicit-discretization recording for the coupled panel lift."""
import hashlib
import json
from pathlib import Path
import numpy as np
from .paper_keyframe import KeyframeCertificate
from .trajectory import TrajectoryResult


def model_fingerprint(problem):
    paths=[r.path for r in problem.robots]+[problem.case.panel.path,problem.case.path,problem.case.path.parent/problem.case.data['source_file']]
    # Generated interface paths differ by output folder; their actual content
    # and every mechanical URDF are part of the identity.
    return hashlib.sha256(b'\0'.join(p.read_bytes() for p in paths)).hexdigest()


def save_lift(path,problem,keyframes,trajectory,contacts):
    fields=['objective','primal_residual','complementarity_residual','friction_residual','minimum_gap','full_geometry_checked','relaxations','gripper_velocity_residual','continuation','grasp_velocity_model','instantaneous_grasp_velocity_residual','objective_mode','configuration_update','configuration_defect_bounds','exact_sticking_velocity']
    meta=dict(format_version=1,scope='first coupled panel lift only',model_fingerprint=model_fingerprint(problem),trajectory={f:getattr(trajectory,f) for f in fields},impulses=list(trajectory.impulses),apertures=list(trajectory.apertures),point_phase='sticking' if all(c.capability.kind!='P' or len(c.sticking_steps)==len(trajectory.dt) for c in contacts) else 'maintained',keys=[])
    arrays=dict(q=trajectory.q,v=trajectory.v,tau=trajectory.tau,dt=trajectory.dt)
    for name,value in trajectory.impulses.items():arrays['impulse_'+name]=value
    for name,value in trajectory.apertures.items():arrays['aperture_'+name]=value
    for i,key in enumerate(keyframes):
        meta['keys'].append(dict(apertures=key.apertures,rho=float(key.rho),objective=float(key.objective),primal_residual=float(key.primal_residual)))
        for field in ['q','positions','rotations','reactions','torques']:arrays[f'key{i}_'+field]=getattr(key,field)
    arrays['metadata_json']=json.dumps(meta,allow_nan=False);np.savez_compressed(path,**arrays)


def load_lift(path,problem):
    from .paper_execution import CoupledEpisode
    from .paper_geometry import PanelGeometry
    from .paper_validation import validate_rigid_trajectory
    with np.load(path,allow_pickle=False) as data:
        meta=json.loads(str(data['metadata_json']))
        if meta['format_version']!=1 or meta['model_fingerprint']!=model_fingerprint(problem):raise ValueError('Plan/model identity mismatch')
        fields=meta['trajectory'];fields['configuration_defect_bounds']=tuple(fields['configuration_defect_bounds']);fields['relaxations']=tuple(fields['relaxations'])
        result=TrajectoryResult(**{k:data[k].copy() for k in ['q','v','tau','dt']},impulses={n:data['impulse_'+n].copy() for n in meta['impulses']},apertures={n:data['aperture_'+n].copy() for n in meta['apertures']},**fields)
        keys=[]
        for i,info in enumerate(meta['keys']):keys.append(KeyframeCertificate(**{f:data[f'key{i}_'+f].copy() for f in ['q','positions','rotations','reactions','torques']},environment_reactions={},**info))
    if len(keys)!=2 or not np.allclose(result.q[[0,-1]],np.array([k.q for k in keys]),atol=2e-5):raise ValueError('Lift endpoints do not match keyframes')
    for key in keys:
        native=problem.verify_keyframe(key)
        if not native['certificate'] or max(native[f] for f in ['native_equilibrium_residual','native_torque_residual','capability_violation','closure_m'])>2e-5:raise ValueError('Recorded keyframe rejected independently')
    contacts=problem.contacts(keys[0],len(result.dt),meta['point_phase'])
    if not validate_rigid_trajectory(problem,result,contacts)['passed'] or not PanelGeometry(problem).validate(result):raise ValueError('Recorded interval rejected independently')
    return CoupledEpisode(problem,tuple(keys),(result,),contacts_by_interval=(contacts,))
