"""Pickle-free stage caches, bound to model, formulation and solver identity."""
from dataclasses import fields
from pathlib import Path
import hashlib,json
import numpy as np
from .paper_keyframe import KeyframeCertificate
from .trajectory import TrajectoryResult


def jsonable(value):
    if isinstance(value,np.ndarray):return value.tolist()
    if isinstance(value,np.generic):return value.item()
    if isinstance(value,Path):return str(value)
    raise TypeError(type(value).__name__)


def write_json(path,value,*,compact=False):
    import os,tempfile
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True);contents=json.dumps(value,default=jsonable,allow_nan=False,**({'separators':(',',':')} if compact else {'indent':2}))+'\n'
    with tempfile.NamedTemporaryFile(mode='w',encoding='utf-8',dir=path.parent,prefix='.'+path.name,suffix='.tmp',delete=False) as handle:handle.write(contents);temporary=handle.name
    os.replace(temporary,path)


def fingerprint(problem,settings,steps):
    task=problem.task_definition
    info=dict(task=task.id,posture_reference_rule=task.posture_reference_rule,types=problem.types,assignment=problem.assignment,refine_source=problem.refine_enabled,features=[vars(f) for f in task.features],keyframes=task.keyframes,settings={k:v for k,v in vars(settings).items() if k not in {'diagnostics_path','verbose'}},steps=steps,specs=problem.specs)
    h=hashlib.sha256(json.dumps(info,sort_keys=True,default=jsonable).encode())
    objects=task.object.bodies.values() if hasattr(task.object,'bodies') else [task.object]
    for r in [*problem.full_robots,*objects]:h.update(r.path.read_bytes());h.update(np.asarray(r.base_pose,dtype='<f8').tobytes())
    # Execution/controller and validators are rerun on every load; changing an
    # assessment or controller must not pretend the optimization was rerun.
    for name in ['full_scenario','full_static','full_scene','full_trajectory','paper_keyframe','trajectory','finger_grasp','generalized_conic','source_transfer','multi_object','spherical_tool','system']:
        h.update(Path(__file__).with_name(name+'.py').read_bytes())
    return h.hexdigest()


def save_key(path,key,identity):
    names=['q','positions','rotations','reactions','torques'];meta={f.name:getattr(key,f.name) for f in fields(key) if f.name not in names+['environment_reactions']};meta.update(identity=identity,posture_reference=getattr(key,'posture_reference',None),target_provenance=getattr(key,'target_provenance',dict(method='coupled_target_NLP')),phase=key.phase,retreat_m=getattr(key,'retreat_m',.06 if key.phase in {'released','release_leg'} else 0.),source=key.source,environment=list(key.environment_reactions),interfaces=[dict(group_id=s.group_id,force_enabled=s.force_enabled) for s in key.interfaces]);a={n:getattr(key,n) for n in names};a.update({'env_'+n:v for n,v in key.environment_reactions.items()});np.savez_compressed(path,metadata_json=json.dumps(meta,default=jsonable,allow_nan=False),**a)


def load_key(path,identity):
    from types import SimpleNamespace
    with np.load(path,allow_pickle=False) as a:
        meta=json.loads(str(a['metadata_json']))
        if meta.pop('identity')!=identity:raise ValueError('Keyframe cache formulation/model mismatch')
        phase,source,interfaces=[meta.pop(n) for n in ['phase','source','interfaces']];target_provenance=meta.pop('target_provenance',dict(method='legacy_target_record'));posture_reference=meta.pop('posture_reference',None);retreat=meta.pop('retreat_m',.06 if phase in {'released','release_leg'} else 0.);env={n:a['env_'+n].copy() for n in meta.pop('environment')};key=KeyframeCertificate(**{n:a[n].copy() for n in ['q','positions','rotations','reactions','torques']},environment_reactions=env,**meta);key.posture_reference=None if posture_reference is None else np.array(posture_reference);key.target_provenance=target_provenance;key.phase=phase;key.retreat_m=retreat;key.source=source;key.interfaces=[SimpleNamespace(**s) for s in interfaces];return key


def save_interval(path,result,identity):
    names=['q','v','tau','dt'];meta={f.name:getattr(result,f.name) for f in fields(result) if f.name not in names+['impulses','apertures']};meta.update(identity=identity,impulses=list(result.impulses),apertures=list(result.apertures));a={n:getattr(result,n) for n in names};a.update({'impulse_'+n:v for n,v in result.impulses.items()});a.update({'aperture_'+n:v for n,v in result.apertures.items()});np.savez_compressed(path,metadata_json=json.dumps(meta,default=jsonable,allow_nan=False),**a)


def load_interval(path,identity):
    with np.load(path,allow_pickle=False) as a:
        meta=json.loads(str(a['metadata_json']))
        if meta.pop('identity')!=identity:raise ValueError('Interval cache formulation/model mismatch')
        impulses={n:a['impulse_'+n].copy() for n in meta.pop('impulses')};apertures={n:a['aperture_'+n].copy() for n in meta.pop('apertures')};return TrajectoryResult(**{n:a[n].copy() for n in ['q','v','tau','dt']},impulses=impulses,apertures=apertures,**meta)
