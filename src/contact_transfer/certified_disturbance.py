"""Physical wrench schedules checked against a fixed-pose generalized polytope.

The certificate concerns the optimized static pose, not closed-loop stability
or all intervening dynamic states. Identical temporal profiles are grouped so
correlated loads on independent bodies are tested jointly in their task image.
"""
from collections import defaultdict
from dataclasses import replace
import numpy as np

from .multi_object import RigidCollection
from .perturbations import WrenchHold,WrenchPulse


def normalized_membership(load,generalized,rho,tolerance=1e-8):
    if not np.isfinite(rho) or rho<0:
        raise ValueError('A finite nonnegative certified margin is required')
    generalized=np.asarray(generalized,float)
    if generalized.shape!=np.shape(load.nominal) or not np.isfinite(generalized).all():
        raise ValueError('Finite generalized disturbance of the task dimension required')
    basis=np.asarray(load.E)@np.asarray(load.D)
    u=np.linalg.lstsq(basis,generalized,rcond=None)[0]
    residual=float(np.max(abs(basis@u-generalized)))
    coefficient=float(np.max(abs(u)))
    return dict(inside=residual<=tolerance and coefficient<=rho+tolerance,
        subspace_residual=residual,normalized_coefficient=u.tolist(),
        coefficient_infinity_norm=coefficient,rho_certified=float(rho),
        load_to_margin_ratio=coefficient/rho if rho>0 else None,
        certificate_scope='fixed reference pose static wrench box')


def generalized_physical_load(obj,q,frame,force_world,moment_world,point_body):
    T=obj.fk(q,frame);J=obj.jacobian(q,frame,'world_aligned')
    F=np.asarray(force_world,float);M=np.asarray(moment_world,float)+np.cross(T[:3,:3]@np.asarray(point_body),F)
    return J[:3].T@M+J[3:].T@F


def schedule_membership(problem,qobj,pulses,rho,scene):
    obj=problem.task_definition.object;load=problem.task_load(qobj)
    frames={body:frame for frame,body in scene.object_frames.items()}
    profiles=defaultdict(lambda:np.zeros(obj.nv))
    for pulse in pulses:
        if not isinstance(pulse,WrenchPulse):raise ValueError('A validated nonnegative unit-bounded wrench profile is required')
        if pulse.body not in frames:
            raise ValueError('Disturbance must name an actual manipulated object body')
        key=(pulse.start,pulse.duration,pulse.to_dict()['profile'],getattr(pulse,'ramp_duration_s',None))
        profiles[key]+=generalized_physical_load(obj,qobj,frames[pulse.body],pulse.force_world_N,pulse.moment_world_Nm,pulse.point_body_m)
    groups=[]
    for key,w in profiles.items():
        membership=normalized_membership(load,w,rho)
        groups.append(dict(start=key[0],end=key[0]+key[1],profile=key[2],**membership))
    # Every profile scale lies in [0,1]. A componentwise triangle bound over
    # each active set certifies even overlapping pulses, without time sampling.
    events=sorted({edge for g in groups for edge in (g['start'],g['end'])})
    maximum=0.
    for a,b in zip(events[:-1],events[1:]):
        active=[g for g in groups if g['start']<(a+b)/2<g['end']]
        if active:maximum=max(maximum,float(np.max(np.sum([abs(np.asarray(g['normalized_coefficient'])) for g in active],axis=0))))
    inside=all(g['subspace_residual']<=1e-8 for g in groups) and maximum<=rho+1e-8
    return dict(inside=bool(inside),rho_certified=float(rho),maximum_coefficient_bound=maximum,
        load_to_margin_ratio=maximum/rho if rho>0 else None,temporal_groups=groups,
        overlap_check='componentwise interval triangle bound; identical profiles summed before projection',
        certificate_scope='optimized static reference pose only',
        continuous_time_load_membership_bound=True,closed_loop_stability_certificate=False)


def physical_hold(problem,key,scene,coefficient,*,start=4.,duration=3.,ramp=.5):
    """Realize task coefficients using free-root wrenches and joint couples."""
    obj=problem.task_definition.object;q=key.q[problem.system.object_q]
    load=problem.task_load(q);coefficient=np.asarray(coefficient,float)
    if coefficient.shape!=(load.E.shape[1],) or not np.isfinite(coefficient).all():
        raise ValueError('One finite coefficient per fixed task direction required')
    required=np.asarray(load.E)@np.asarray(load.D)@coefficient
    components=obj.bodies.items() if isinstance(obj,RigidCollection) else [(None,obj)]
    pulses=[]
    for name,component in components:
        rows=obj.v_slices[name] if isinstance(obj,RigidCollection) else slice(0,obj.nv)
        local=required[rows];frame=name if name is not None else component.tree.root
        T=obj.fk(q,frame);R=T[:3,:3]
        if np.linalg.norm(local[:6])>1e-12:
            pulses.append(WrenchHold(scene.object_frames[frame],start,duration,tuple(R@local[3:6]),
                moment_world_Nm=tuple(R@local[:3]),ramp_duration_s=ramp))
        if name is not None and component.n:
            raise ValueError('Articulated independent collections need an explicit joint adapter')
    if not isinstance(obj,RigidCollection):
        # Equal/opposite child-parent loads have zero free-root resultant and
        # produce only the named joint's generalized load by virtual work.
        for j,joint_name in enumerate(obj.joint_names):
            value=required[6+j]
            if abs(value)<1e-12:continue
            joint=next(joint for joint in obj.tree.joints if joint.name==joint_name)
            J=obj.jacobian(q,joint.child,'world_aligned');child=obj.fk(q,joint.child);parent=obj.fk(q,joint.parent)
            if joint.kind in {'revolute','continuous'}:
                moment=J[:3,6+j]*value
                for frame,M in [(joint.child,moment),(joint.parent,-moment)]:
                    pulses.append(WrenchHold(scene.object_frames[frame],start,duration,(0.,0.,0.),moment_world_Nm=tuple(M),ramp_duration_s=ramp))
            elif joint.kind=='prismatic':
                force=J[3:,6+j]*value
                point=parent[:3,:3].T@(child[:3,3]-parent[:3,3])
                pulses.append(WrenchHold(scene.object_frames[joint.child],start,duration,tuple(force),ramp_duration_s=ramp))
                pulses.append(WrenchHold(scene.object_frames[joint.parent],start,duration,tuple(-force),tuple(point),ramp_duration_s=ramp))
            else:raise ValueError('Unsupported articulated disturbance joint')
    certificate=schedule_membership(problem,q,pulses,float(np.max(abs(coefficient)))+1e-10,scene)
    if not certificate['inside']:
        raise RuntimeError('Physical load adapter did not reproduce the declared task coefficient')
    return tuple(pulses)


def probe_episode(original,key_index,directory,*,duration=6.,steps=8):
    """Stationary conditional probe of an accepted mid-task optimized keyframe.

    This is separate from the full-motion execution. No object is prescribed
    in native physics; holding reactions are reallocated by the same nominal
    conic QP and all fresh mechanics/geometry gates remain mandatory.
    """
    from copy import copy,deepcopy
    from pathlib import Path
    from .full_static import stationary_interval
    from .full_trajectory import interval_contacts
    from .full_execution import FullEpisode
    from .full_validation import validate_keyframe
    p=copy(original.problem);key=deepcopy(original.keyframes[key_index]);goal=p.task_definition.keyframes[key_index]
    if key.environment_reactions:
        raise ValueError('Probe must be selected before environmental support/handoff')
    keys=tuple(dict(id=name,time=t,object_q=goal['object_q'].copy(),phase=goal['phase'])
        for name,t in [('hold_start',0.),('load_observation',duration/2),('hold_end',duration)])
    p.task_definition=replace(p.task_definition,keyframes=keys)
    p.directory=Path(directory);p.directory.mkdir(parents=True,exist_ok=True)
    if hasattr(p,'_case'):del p._case
    independent=validate_keyframe(p,key)
    if not independent['passed']:raise RuntimeError('Probe keyframe failed independent native mechanics')
    keyframes=tuple(deepcopy(key) for _ in keys);trajectories=[];contacts=[]
    for i in range(2):
        cs=interval_contacts(p,i,key,steps)
        r,cs,_=stationary_interval(p,key,key,cs,keys[i+1]['time']-keys[i]['time'],steps)
        trajectories.append(r);contacts.append(cs)
    return FullEpisode(p,keyframes,trajectories,contacts),independent
