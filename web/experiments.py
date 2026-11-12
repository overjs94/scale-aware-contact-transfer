"""Bounded local experiment jobs; client input never supplies filesystem paths."""
import json
import math
from pathlib import Path
import shutil
import subprocess
import sys
import threading
import uuid

ROOT = Path(__file__).resolve().parents[1]
PRESETS = ROOT/'src/contact_transfer/scenarios'
ARTICULATED = {'hinged_door','folding_panel','drawer'}


def number(value, low, high):
    if isinstance(value,bool) or not isinstance(value,(float,int)) or not math.isfinite(value) or not low <= value <= high:
        raise ValueError(f'Expected finite number in [{low}, {high}]')
    return float(value)


def vector(value, low, high):
    if not isinstance(value,list) or len(value)!=3: raise ValueError('Expected three coordinates')
    return [number(v,low,high) for v in value]


def prepare_request(request, destination):
    if isinstance(request,dict) and request.get('preset') in ARTICULATED:
        return prepare_articulated_request(request,destination)
    if isinstance(request,dict) and request.get('preset')=='mobile_panel':
        return prepare_mobile_request(request,destination)
    if isinstance(request,dict) and request.get('preset')=='panel_fixture':
        return prepare_panel_request(request,destination)
    if isinstance(request,dict) and request.get('preset')=='table_assembly':
        return prepare_assembly_request(request,destination)
    if not isinstance(request,dict) or set(request)-{'preset','mass','keyframes','contacts'}: raise ValueError('Unknown experiment fields')
    preset=request.get('preset','three_franka')
    if preset not in {'three_franka','three_arm'}:raise ValueError('Unknown preset')
    source=PRESETS/preset
    d=json.loads((source/'case.json').read_text())
    mass=number(request.get('mass',d['object']['mass']),.1,2.)
    d['object']['inertia_diagonal']=[v*mass/d['object']['mass'] for v in d['object']['inertia_diagonal']]
    d['object']['mass']=mass
    keys=request.get('keyframes',d['keyframes'])
    if not isinstance(keys,list) or len(keys)!=3:raise ValueError('Exactly three keyframes required')
    for target,item in zip(d['keyframes'],keys):
        target['time']=number(item['time'],0,12)
        pose=item['object_pose']
        if len(pose)!=7 or pose[3:]!=[0,0,0,1]:raise ValueError('This baseline requires fixed identity object orientation')
        target['object_pose']=vector(pose[:3],-.9,1.2)+[0,0,0,1]
    if d['keyframes'][0]['time']!=0 or any(b['time']-a['time']<.5 for a,b in zip(d['keyframes'],d['keyframes'][1:])):raise ValueError('Start at 0; each interval must be at least 0.5 seconds')
    contacts=request.get('contacts',d['contracted_contacts'])
    if not isinstance(contacts,list) or len(contacts)!=3:raise ValueError('Exactly three contracted contacts required')
    for target,item in zip(d['contracted_contacts'],contacts):
        if item['arm_id']!=target['arm_id']:raise ValueError('Contact order must be point, gripper, suction')
        target['point_object']=vector(item['point_object'],-.5,.5)
        target['normal_object']=vector(item['normal_object'],-1,1)
    # ContractedCase validates surface membership, normal alignment, and limits.
    destination=Path(destination);destination.mkdir(parents=True)
    for file in source.iterdir():
        if file.is_file():shutil.copyfile(file,destination/file.name)
    (destination/'case.json').write_text(json.dumps(d,indent=2))
    from contact_transfer.contracted import ContractedCase
    ContractedCase.load(destination/'case.json')
    return d


def prepare_articulated_request(request,destination):
    if set(request)-{'preset','duration'}:raise ValueError('Articulated accepts preset and duration only')
    source=PRESETS/'articulated';d=json.loads((source/(request['preset']+'.json')).read_text())
    duration=number(request.get('duration',14),12,60)
    if not math.isclose(round(duration/d['timestep'])*d['timestep'],duration,abs_tol=1e-9):raise ValueError('Duration must match timestep')
    scale=duration/d['duration']
    for name in ['duration','motion_start','motion_end','sample_dt']:d[name]*=scale
    destination=Path(destination);destination.mkdir(parents=True)
    for name in {d['robot_urdf'],d['object_urdf'],'panda.srdf','LICENSE.panda','LICENSE.authored','franka_panda_provenance.json'}:shutil.copyfile(source/name,destination/name)
    (destination/'case.json').write_text(json.dumps(d,indent=2))
    return d


def prepare_assembly_request(request,destination):
    if set(request)-{'preset','duration'}:raise ValueError('Assembly accepts preset and duration only')
    source=PRESETS/'table_assembly'
    d=json.loads((source/'case.json').read_text())
    duration=number(request.get('duration',30),24,60)
    dt=d['simulation']['timestep']
    if not math.isclose(round(duration/dt)*dt,duration,abs_tol=1e-9):raise ValueError('Duration must be a multiple of the physics timestep')
    scale=duration/d['simulation']['duration']
    for key in d['keyframes']:key['time']*=scale
    for name in ('duration','grasp_seconds','release_time','release_seconds','seat_time'):d['simulation'][name]*=scale
    d['optimization']['sample_dt']*=scale
    destination=Path(destination);destination.mkdir(parents=True)
    for file in source.iterdir():
        if file.is_file():shutil.copyfile(file,destination/file.name)
    (destination/'case.json').write_text(json.dumps(d,indent=2))
    from contact_transfer.table_assembly import AssemblyCase
    AssemblyCase.load(destination/'case.json')
    return d


def prepare_panel_request(request,destination):
    if set(request)-{'preset','duration','refine_source','contracted_source'}:raise ValueError('Panel accepts preset, duration, refine_source and contracted_source only')
    if not isinstance(request.get('refine_source',True),bool):raise ValueError('refine_source must be boolean')
    source=PRESETS/'panel_fixture';d=json.loads((source/'case.json').read_text());duration=number(request.get('duration',30),24,60)
    dt=d['simulation']['timestep']
    if not math.isclose(round(duration/dt)*dt,duration,abs_tol=1e-9):raise ValueError('Duration must be a multiple of physics timestep')
    scale=duration/d['simulation']['duration']
    for k in d['keyframes']:k['time']*=scale
    for name in ('duration','grasp_seconds','seat_time','load_transfer_end','release_time','release_seconds','retract_end'):d['simulation'][name]*=scale
    d['optimization']['sample_dt']*=scale;d['source_refinement']=request.get('refine_source',True)
    destination=Path(destination);destination.mkdir(parents=True)
    for file in source.iterdir():
        if file.is_file():shutil.copyfile(file,destination/file.name)
    (destination/'case.json').write_text(json.dumps(d,indent=2))
    from contact_transfer.source_transfer import load_source
    if 'contracted_source' in request:
        if not isinstance(request['contracted_source'],dict):raise ValueError('contracted_source must be an object')
        (destination/d['source_file']).write_text(json.dumps(request['contracted_source'],allow_nan=False))
    contacts=load_source(destination/d['source_file'])
    if set(g['group_id'] for g in contacts['groups'])!=set(a['group_id'] for a in d['arms']):raise ValueError('Source groups must match assigned arms')
    if contacts['source_to_target_scale']!=d['source_to_target_scale']:raise ValueError('Source and target scales must match')
    from contact_transfer.panel_fixture import PanelCase
    PanelCase.load(destination/'case.json')
    return d


def prepare_mobile_request(request,destination):
    if set(request)-{'preset','duration','refine_source','contracted_source'}:raise ValueError('Unknown mobile experiment fields')
    if not isinstance(request.get('refine_source',True),bool):raise ValueError('refine_source must be boolean')
    source=PRESETS/'mobile_panel';d=json.loads((source/'case.json').read_text());duration=number(request.get('duration',36),30,60)
    if not math.isclose(round(duration/d['timestep'])*d['timestep'],duration,abs_tol=1e-9):raise ValueError('Duration must match timestep')
    scale=duration/d['duration']
    for key in d['keyframes']:key['time']*=scale
    for name in ['duration','sample_dt','base_start','base_end','fixed_release','seat_time','load_transfer_end','release_time','release_seconds','retract_end']:d[name]*=scale
    d['source_refinement']=request.get('refine_source',True)
    destination=Path(destination);destination.mkdir(parents=True)
    for file in source.iterdir():
        if file.is_file():shutil.copyfile(file,destination/file.name)
    (destination/'case.json').write_text(json.dumps(d,indent=2))
    from contact_transfer.source_transfer import load_source
    if 'contracted_source' in request:
        if not isinstance(request['contracted_source'],dict):raise ValueError('contracted_source must be an object')
        (destination/d['source_file']).write_text(json.dumps(request['contracted_source'],allow_nan=False))
    contacts=load_source(destination/d['source_file'])
    if set(g['group_id'] for g in contacts['groups'])!={'edge_grip','under_right','under_left'} or contacts['source_to_target_scale']!=4:raise ValueError('Mobile source groups/scale must match scene')
    from contact_transfer.mobile_panel import MobileCase
    MobileCase.load(destination/'case.json')
    return d


class Jobs:
    def __init__(self, root=None):
        self.root=Path(root or ROOT/'outputs/web_jobs');self.root.mkdir(parents=True,exist_ok=True)
        self.lock=threading.Lock();self.processes={}

    def submit(self, request):
        with self.lock:
            if any(p.poll() is None for p in self.processes.values()):raise BlockingIOError('An experiment is already running')
            jid=uuid.uuid4().hex
            path=self.root/jid
            try:prepare_request(request,path)
            except Exception:
                shutil.rmtree(path,ignore_errors=True);raise
            (path/'status.json').write_text(json.dumps({'state':'queued','stage':'queued'}))
            with (path/'worker.log').open('w') as log:
                self.processes[jid]=subprocess.Popen([sys.executable,str(ROOT/'web/experiment_worker.py'),str(path)],cwd=ROOT,stdout=log,stderr=log)
            return {'id':jid,**self.get(jid)}

    def path(self,jid):
        if len(jid)!=32 or any(c not in '0123456789abcdef' for c in jid):raise ValueError('Invalid job id')
        path=self.root/jid
        if not path.is_dir():raise FileNotFoundError('Unknown job')
        return path

    def get(self,jid):
        path=self.path(jid);result=json.loads((path/'status.json').read_text())
        proc=self.processes.get(jid)
        if result['state'] in {'queued','running'} and (proc is None or proc.poll() is not None):
            result={'state':'failed','stage':'failed','error':'Worker exited before completion; inspect worker.log locally'}
        return result

    def cancel(self,jid):
        with self.lock:
            path=self.path(jid);proc=self.processes.get(jid)
            if proc and proc.poll() is None:
                proc.terminate()
                try:proc.wait(timeout=10)
                except subprocess.TimeoutExpired:proc.kill();proc.wait(timeout=5)
                (path/'status.json').write_text(json.dumps({'state':'cancelled','stage':'cancelled'}))
            return self.get(jid)
