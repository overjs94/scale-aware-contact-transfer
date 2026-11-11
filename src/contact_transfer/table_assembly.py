"""Three real Panda arms assembling two free URDF furniture parts.

The prescribed SE(3) path uses full-pose IK; IPOPT allocates spatial reactions
under rigid-body inertial balance and joint torque limits. No object pose is
prescribed during MuJoCo execution, and no weld/attachment constraint is used.
"""
from dataclasses import dataclass
import hashlib
from importlib.resources import files
import json
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import casadi as ca
import numpy as np
from scipy.interpolate import CubicHermiteSpline
from scipy.spatial.transform import Rotation

from .contacts import Capability,contact_basis,map_reaction
from .contracted import numerical_pose_ik,pose_matrix
from .contact_simulation import MujocoContactScene
from .optimization import SolverSettings,equality,solve
from .robot import Robot
from .simulation import _text,_quat
from .urdf import origin
from .urdf_scene import append_urdf_robot


@dataclass
class AssemblyCase:
    path: Path
    data: dict
    robots: tuple
    full_robots: tuple
    parts: tuple

    @classmethod
    def load(cls,path=None):
        path=Path(path or str(files('contact_transfer').joinpath('scenarios/table_assembly/case.json'))).resolve()
        d=json.loads(path.read_text());keys=d['keyframes'];times=np.array([k['time'] for k in keys])
        if [o['id'] for o in d['objects']]!=['tabletop','leg']:raise ValueError('Assembly needs tabletop and leg free objects')
        if d.get('id')!='table_assembly' or d.get('units')!='SI' or len(keys)<5 or len(keys)>16:raise ValueError('Unsupported assembly scenario')
        if len(d['arms'])!=3 or [a['kind'] for a in d['arms']]!=['G','P','G']:raise ValueError('Assembly requires two grippers and one point tool')
        if times[0]!=0 or not np.isfinite(times).all() or np.any(np.diff(times)<.5):raise ValueError('Assembly times must increase from zero by at least 0.5 s')
        if not 24<=times[-1]<=60 or not np.isclose(times[-1],d['simulation']['duration']):raise ValueError('Assembly duration must be 24 to 60 seconds')
        for key in keys:
            for obj in d['objects']:pose_matrix(key[obj['id']])
        def asset(name):
            p=(path.parent/name).resolve()
            if not p.is_relative_to(path.parent):raise ValueError('Assembly assets must be local to case directory')
            return p
        full=[];reduced=[]
        for a in d['arms']:
            T=np.eye(4);T[:3,:3]=Rotation.from_euler('xyz',a['base_rpy']).as_matrix();T[:3,3]=a['base_position']
            full.append(Robot(asset(a['urdf']),base_pose=T))
            reduced.append(Robot(asset(a['urdf']),base_pose=T,locked_joints=a['locked_joints']))
            if reduced[-1].n!=7:raise ValueError('Assembly uses real 7-DoF Panda chains')
        parts=tuple(Robot(asset(o['urdf']),floating_base=True) for o in d['objects'])
        return cls(path,d,tuple(reduced),tuple(full),parts)


def smooth(s):
    s=np.clip(s,0.,1.);return 10*s**3-15*s**4+6*s**5


def object_reference(case,t):
    keys=case.data['keyframes'];times=np.array([k['time'] for k in keys])
    i=int(np.clip(np.searchsorted(times,t,side='right')-1,0,len(times)-2))
    h=times[i+1]-times[i];s=np.clip((t-times[i])/h,0,1)
    f=smooth(s);fd=(30*s*s-60*s**3+30*s**4)/h;fdd=(60*s-180*s*s+120*s**3)/h**2
    poses=[];vs=[];acc=[]
    for obj in case.data['objects']:
        a,b=np.array(keys[i][obj['id']]),np.array(keys[i+1][obj['id']]);R0=Rotation.from_quat(a[3:]);dr=(R0.inv()*Rotation.from_quat(b[3:])).as_rotvec()
        rot=R0*Rotation.from_rotvec(f*dr);R=rot.as_matrix();w=fd*dr;v=R.T@(fd*(b[:3]-a[:3]))
        poses.append(np.r_[a[:3]+f*(b[:3]-a[:3]),rot.as_quat()]);vs.append(np.r_[w,v]);acc.append(np.r_[fdd*dr,R.T@(fdd*(b[:3]-a[:3]))-np.cross(w,v)])
    return np.array(poses),np.array(vs),np.array(acc)


def tool_targets(case,t):
    poses=object_reference(case,t)[0];targets=[];ids=[o['id'] for o in case.data['objects']];cfg=case.data['simulation']
    for spec in case.data['arms']:
        T=pose_matrix(poses[ids.index(spec['object'])]);tool=np.eye(4)
        tool[:3,:3]=T[:3,:3]@Rotation.from_quat(spec['tool_quaternion_xyzw']).as_matrix();tool[:3,3]=T[:3,3]+T[:3,:3]@np.array(spec['point_object'])
        if spec['id']=='leg_gripper':
            tool[:3,3]+=np.array([0,cfg['retract_distance']*smooth((t-cfg['release_time']-cfg['release_seconds'])/max(.1,cfg['duration']-cfg['release_time']-cfg['release_seconds'])),0])
        targets.append(tool)
    return targets


@dataclass
class AssemblyPlan:
    case: AssemblyCase
    times: np.ndarray
    q: np.ndarray
    v: np.ndarray
    a: np.ndarray
    reactions_world: np.ndarray
    torque: np.ndarray
    primal_residual: float
    q_slices: tuple


def plan_assembly(case,settings=None,progress=None):
    cfg=case.data['optimization'];duration=case.data['simulation']['duration'];count=round(duration/cfg['sample_dt'])
    times=np.linspace(0,duration,count+1);Q=np.zeros((len(times),21));slices=tuple(slice(7*i,7*i+7) for i in range(3))
    seeds=[np.array(a['q_initial']) for a in case.data['arms']]
    for k,t in enumerate(times):
        for i,(r,a,target,sl) in enumerate(zip(case.robots,case.data['arms'],tool_targets(case,t),slices)):
            try:seeds[i]=numerical_pose_ik(r,a['tool_frame'],target,seeds[i])
            except RuntimeError as error:raise RuntimeError(f'{a["id"]} at {t:.2f}s: {error}') from error
            Q[k,sl]=seeds[i]
    V=np.gradient(Q,times,axis=0,edge_order=2);A=np.gradient(V,times,axis=0,edge_order=2)
    for r,sl in zip(case.robots,slices):
        if np.any(abs(V[:,sl])>r.velocity):raise RuntimeError('Assembly path exceeds joint velocity limits')
    caps=[Capability('G',H=np.eye(6),bounds=cfg['gripper_moment_bounds']+cfg['gripper_force_bounds']) if a['kind']=='G' else Capability('P',mu=cfg['point_mu'],normal_max=cfg['point_normal_max']) for a in case.data['arms']]
    opti=ca.Opti();reactions=[];torques=[];objective=0.;ids=[o['id'] for o in case.data['objects']]
    for k,t in enumerate(times):
        poses,vel,acc=object_reference(case,t);R=[Rotation.from_quat(p[3:]).as_matrix() for p in poses]
        lam=[ca.DM.zeros(6,1) if i==2 and t>=case.data['simulation']['release_time'] else c.reaction(opti) for i,c in enumerate(caps)]
        # Model the planned shoulder load transfer only after reaching the seat.
        sim=case.data['simulation'];leg_mass=float(case.parts[1].model.inertias[1].mass)
        seated=leg_mass*9.81*smooth((t-sim['seat_time'])/max(.1,sim['release_time']-sim['seat_time']))
        external=[np.zeros(6),np.zeros(6)]
        for j,(p,fworld) in enumerate([(case.data['socket']['center_object'],[0,0,-seated]),(case.data['socket'].get('leg_shoulder_object',[0,0,-.14]),[0,0,seated])]):
            f=R[j].T@fworld;external[j]=np.r_[np.cross(p,f),f]
        for j,part in enumerate(case.parts):
            wrench=sum(map_reaction(a['point_object'],contact_basis(a['normal_object']),force) for a,force in zip(case.data['arms'],lam) if a['object']==ids[j])
            required=-part.inverse_dynamics(poses[j],vel[j],acc[j])+external[j]
            equality(opti,wrench-required)
        world=[];taus=[]
        for i,(robot,spec,sl,force) in enumerate(zip(case.robots,case.data['arms'],slices,lam)):
            j=ids.index(spec['object']);Rc=R[j]@contact_basis(spec['normal_object']);w=ca.vertcat(ca.DM(Rc)@force[:3],ca.DM(Rc)@force[3:]);world.append(w)
            T=robot.fk(Q[k,sl],spec['tool_frame']);dual=ca.vertcat(ca.DM(T[:3,:3].T)@w[:3],w[3:])
            tau=ca.DM(robot.inverse_dynamics(Q[k,sl],V[k,sl],A[k,sl]))-ca.DM(robot.jacobian(Q[k,sl],spec['tool_frame']).T)@dual
            opti.subject_to(opti.bounded(-robot.effort,tau,robot.effort));taus.append(tau)
            objective+=1e-3*ca.sumsqr(force[3:])+20*ca.sumsqr(force[:3])+1e-5*ca.sumsqr(tau)
        reactions.append(ca.horzcat(*world).T);torques.append(ca.vertcat(*taus))
    opti.minimize(objective);sol,residual=solve(opti,settings or SolverSettings(options={'expand':False}))
    return AssemblyPlan(case,times,Q,V,A,np.array([sol.value(x) for x in reactions]),np.array([sol.value(x) for x in torques]),residual,slices)


def build_assembly_scene(case,destination):
    root=ET.Element('mujoco',model='three_panda_table_assembly');ET.SubElement(root,'compiler',angle='radian',inertiafromgeom='false',fusestatic='false')
    ET.SubElement(root,'option',timestep=str(case.data['simulation']['timestep']),gravity='0 0 -9.81',integrator='implicitfast',cone='elliptic',impratio=str(case.data['simulation'].get('friction_impratio',100)),iterations='100',tolerance='1e-10')
    assets=ET.SubElement(root,'asset');world=ET.SubElement(root,'worldbody');actuators=ET.SubElement(root,'actuator');contact=ET.SubElement(root,'contact')
    meshes={p.name:p.resolve() for p in (Path.cwd()/'.cache/panda_meshes').glob('*.stl')}
    if len(meshes)<8:raise ValueError('Fetch Panda meshes first: python tools/fetch_panda_meshes.py')
    for robot,spec in zip(case.full_robots,case.data['arms']):
        prefix=spec['id']+'_';geoms=append_urdf_robot(world,actuators,robot,prefix,mesh_files=meshes,assets=assets,robot_collisions=True)
        # Soft finger pads carry finite torsional friction about the contact
        # normal. This prevents a two-point jaw model's free spin of a long leg.
        for geom in geoms:
            is_pad='leftfinger_collision_' in geom or 'rightfinger_collision_' in geom
            if is_pad or spec['kind']=='P' and geom.startswith(prefix+'tool_collision_'):
                obj=next(o for o in case.data['objects'] if o['id']==spec['object'])
                count=len(ET.parse(case.path.parent/obj['urdf']).findall('link/collision'))
                cfg=case.data['simulation'];mu=cfg.get('pad_friction',.9) if is_pad else case.data['optimization']['point_mu']
                for j in range(count):
                    ET.SubElement(contact,'pair',geom1=geom,geom2=obj['id']+'_collision_'+str(j),condim='4' if is_pad else '3',
                                  friction=_text([mu,mu,cfg.get('pad_torsional_friction_m',.005) if is_pad else 0,0,0]),solref='.008 1',solimp='.95 .99 .001')
        for pair in ET.parse(case.path.parent/'panda.srdf').findall('disable_collisions'):
            a,b=pair.get('link1'),pair.get('link2')
            if a in robot.tree.links and b in robot.tree.links:
                ET.SubElement(contact,'exclude',body1=prefix+('urdf_'+a if a==robot.tree.root else a),body2=prefix+('urdf_'+b if b==robot.tree.root else b))
    for obj,pose in zip(case.data['objects'],object_reference(case,0)[0]):
        link=ET.parse(case.path.parent/obj['urdf']).find('link');body=ET.SubElement(world,'body',name=obj['id'],pos=_text(pose[:3]),quat=_text([pose[6],*pose[3:6]]))
        ET.SubElement(body,'freejoint',name=obj['id']+'_free');inertial=link.find('inertial');Ri,pi=origin(inertial);I=inertial.find('inertia')
        matrix=np.array([[float(I.get('ixx')),float(I.get('ixy')),float(I.get('ixz'))],[float(I.get('ixy')),float(I.get('iyy')),float(I.get('iyz'))],[float(I.get('ixz')),float(I.get('iyz')),float(I.get('izz'))]])
        matrix=Ri@matrix@Ri.T
        ET.SubElement(body,'inertial',pos=_text(pi),mass=inertial.find('mass').get('value'),fullinertia=_text([matrix[0,0],matrix[1,1],matrix[2,2],matrix[0,1],matrix[0,2],matrix[1,2]]))
        for i,g in enumerate(link.findall('collision')):
            R,p=origin(g);size=np.array(g.find('geometry/box').get('size').split(),float)/2
            ET.SubElement(body,'geom',name=obj['id']+'_collision_'+str(i),type='box',size=_text(size),pos=_text(p),quat=_text(_quat(R)),mass='0',contype='1',conaffinity='3',condim='3',friction='.9 0 0',solref='.008 1',solimp='.95 .99 .001',rgba='.7 .48 .27 1')
    fixtures=[];T=pose_matrix(object_reference(case,0)[0][0])
    for i,(x,y) in enumerate([(-.23,-.13),(-.23,.13),(.23,-.13),(.23,.13)]):
        p=T[:3,3]+T[:3,:3]@np.array([x,y,-.025]);fixtures.append({'name':f'top_cradle_{i}','position':p.tolist(),'size':[.05,.05,.01],'quaternion_xyzw':Rotation.from_matrix(T[:3,:3]).as_quat().tolist()})
    leg=object_reference(case,0)[0][1];fixtures.append({'name':'leg_cradle','position':[leg[0],leg[1],leg[2]-.175],'size':[.06,.06,.02],'quaternion_xyzw':[0,0,0,1]})
    for f in fixtures:
        quat=f['quaternion_xyzw'];ET.SubElement(world,'geom',name=f['name'],type='box',size=_text(np.array(f['size'])/2),pos=_text(f['position']),quat=_text([quat[3],*quat[:3]]),contype='1',conaffinity='3',rgba='.25 .32 .4 1')
    ET.SubElement(world,'geom',name='floor',type='plane',size='2 2 .1',contype='1',conaffinity='3',rgba='.15 .18 .22 1')
    ET.indent(root);Path(destination).write_bytes(ET.tostring(root,encoding='utf-8',xml_declaration=True))
    return fixtures


class AssemblyController:
    def __init__(self,plan):
        self.plan=plan;self.cfg=plan.case.data['simulation'];self.spline=CubicHermiteSpline(plan.times,plan.q,plan.v);self.t=0.
        self.slices=[];off=0
        for r in plan.case.full_robots:self.slices.append(slice(off,off+r.n));off+=r.n
    def reference(self,t):
        self.t=t;qred,vred,ared=[self.spline(t,n) for n in [0,1,2]];qs=[];vs=[];acc=[]
        for i,(r,sl) in enumerate(zip(self.plan.case.full_robots,self.plan.q_slices)):
            q=np.zeros(r.n);v=q.copy();a=q.copy();q[:7]=qred[sl];v[:7]=vred[sl];a[:7]=ared[sl]
            if r.n>7:
                close=smooth(t/self.cfg['grasp_seconds']);opened=smooth((t-self.cfg['release_time'])/self.cfg['release_seconds']) if i==2 else 0
                q[7:]=self.cfg['finger_open']+(self.cfg['finger_closed']-self.cfg['finger_open'])*close*(1-opened)
            qs.append(q);vs.append(v);acc.append(a)
        return np.concatenate(qs),np.concatenate(vs),np.concatenate(acc)
    def command(self,q,v,desired,velocity,acceleration):
        k=int(np.clip(np.searchsorted(self.plan.times,self.t)-1,0,len(self.plan.times)-1));out=[]
        for i,(r,spec,sl) in enumerate(zip(self.plan.case.full_robots,self.plan.case.data['arms'],self.slices)):
            feedback=self.cfg['arm_kp']*(desired[sl]-q[sl])+self.cfg['arm_kd']*(velocity[sl]-v[sl]);tau=r.inverse_dynamics(q[sl],v[sl],acceleration[sl]+feedback)
            w=self.plan.reactions_world[k,i]*smooth(self.t/self.cfg['grasp_seconds']);T=r.fk(q[sl],spec['tool_frame']);J=r.jacobian(q[sl],spec['tool_frame'])
            tau-=J.T@np.r_[T[:3,:3].T@w[:3],w[3:]]
            Td=r.fk(desired[sl],spec['tool_frame']);Jd=r.jacobian(desired[sl],spec['tool_frame']);actual_twist=J@v[sl];desired_twist=Jd@velocity[sl]
            error_rotation=Rotation.from_matrix(T[:3,:3].T@Td[:3,:3]).as_rotvec()
            moment=self.cfg.get('orientation_kp',120)*error_rotation+self.cfg.get('orientation_kd',10)*(T[:3,:3].T@Td[:3,:3]@desired_twist[:3]-actual_twist[:3])
            force=self.cfg.get('cartesian_kp',12000)*(Td[:3,3]-T[:3,3])+self.cfg.get('cartesian_kd',200)*(desired_twist[3:]-actual_twist[3:])
            tau+=J.T@np.r_[moment,force]
            if r.n>7:tau[7:]=r.gravity(q[sl])[7:]+self.cfg['finger_kp']*(desired[sl][7:]-q[sl][7:])-self.cfg['finger_kd']*v[sl][7:]
            out.append(np.clip(tau,-r.effort,r.effort))
        return np.concatenate(out)


def check_planned_collisions(plan,scene):
    """Discrete mesh checks, distinct from continuous path certification."""
    mj=scene.mj;model=scene.model;data=scene.data;bad={};controller=AssemblyController(plan)
    for t in plan.times:
        data.qpos[scene.qadr]=controller.reference(float(t))[0]
        for body,pose in zip(scene.objects,object_reference(plan.case,float(t))[0]):
            adr=model.jnt_qposadr[model.body_jntadr[body]];data.qpos[adr:adr+7]=np.r_[pose[:3],pose[6],pose[3:6]]
        mj.mj_forward(model,data)
        for j in range(data.ncon):
            c=data.contact[j];pair=[mj.mj_id2name(model,mj.mjtObj.mjOBJ_GEOM,int(g)) for g in c.geom]
            if c.dist<-.0005 and unexpected_pair(plan.case,pair):bad[' / '.join(pair)]=min(bad.get(' / '.join(pair),0),float(c.dist))
    if bad:raise RuntimeError('Planned mesh penetration: '+json.dumps(bad))
    return bad


def unexpected_pair(case,pair):
    arms=case.data['arms']
    is_robot=any(any(n.startswith(a['id']+'_') for a in arms) for n in pair)
    if not is_robot:return False
    return not any(any(n.startswith(a['id']+'_'+link+'_collision_') for link in a['tool_collision_links'] for n in pair)
                   and any(n.startswith(a['object']+'_collision_') for n in pair) for a in arms)


def assembly_report(plan,trace):
    case=plan.case;cfg=case.data['simulation'];dt=trace.metadata['timestep'];ids=[a['id'] for a in case.data['arms']]
    flags=np.zeros((len(trace.time),3,2),bool);world=np.zeros((len(trace.time),3,6));unexpected={};part_contact=np.zeros(len(trace.time),bool)
    poses=trace.object_pose;R=[Rotation.from_quat(poses[:,i,3:]).as_matrix() for i in range(2)]
    references=[poses[:,0 if a['object']=='tabletop' else 1,:3]+np.einsum('nij,j->ni',R[0 if a['object']=='tabletop' else 1],a['point_object']) for a in case.data['arms']]
    for t,pair,pos,gap,wrench in zip(trace.contact_time,trace.contact_pair,trace.contact_position,trace.contact_gap,trace.contact_wrench):
        k=int(round(t/dt))
        if gap<-.0005 and unexpected_pair(case,pair):unexpected[' / '.join(pair)]=min(unexpected.get(' / '.join(pair),0),float(gap))
        if any(n.startswith('leg_collision_') for n in pair) and any(n.startswith('tabletop_collision_') for n in pair):part_contact[k]=True
        for i,spec in enumerate(case.data['arms']):
            robotidx=next((j for j,n in enumerate(pair) if n.startswith(spec['id']+'_')),None)
            if robotidx is None or not pair[1-robotidx].startswith(spec['object']+'_collision_'):continue
            # Logged wrench acts on geom 2; reactions act on the robot.
            w=wrench*(1 if robotidx==1 else -1)
            world[k,i]+=np.r_[w[:3]+np.cross(pos-references[i][k],w[3:]),w[3:]]
            name=pair[robotidx]
            if spec['kind']=='P':flags[k,i,:]=True
            else:
                if 'leftfinger' in name:flags[k,i,0]=True
                if 'rightfinger' in name:flags[k,i,1]=True
    errors=[]
    for key in case.data['keyframes']:
        k=int(np.argmin(abs(trace.time-key['time'])));row={'id':key['id'],'time':float(trace.time[k])}
        for j,obj in enumerate(case.data['objects']):
            desired=np.array(key[obj['id']]);actual=poses[k,j]
            row[obj['id']]={'translation_m':float(np.linalg.norm(actual[:3]-desired[:3])),
                           'rotation_rad':float((Rotation.from_quat(desired[3:]).inv()*Rotation.from_quat(actual[3:])).magnitude())}
        errors.append(row)
    # Track the leg only while grasped; after release assembly geometry is the test.
    tracking=all(e['tabletop']['translation_m']<cfg['tracking_tolerance_m'] and e['tabletop']['rotation_rad']<cfg['tracking_tolerance_rad']
                 and (e['time']>cfg['release_time'] or (e['leg']['translation_m']<cfg['tracking_tolerance_m'] and e['leg']['rotation_rad']<cfg['tracking_tolerance_rad'])) for e in errors)
    peg_tip=np.array(case.data['socket'].get('peg_tip_object',[0,0,-.165]));socket=np.array(case.data['socket']['center_object'])
    tips=poses[:,1,:3]+np.einsum('nij,j->ni',R[1],peg_tip)
    relative=np.einsum('nji,nj->ni',R[0],tips-poses[:,0,:3]);depth=socket[2]-relative[:,2];lateral=np.linalg.norm(relative[:,:2]-socket[:2],axis=1)
    angle=(Rotation.from_quat(poses[:,0,3:]).inv()*Rotation.from_quat(poses[:,1,3:])).magnitude()
    final=trace.time>=cfg['duration']-.5
    released=not flags[final,2].any()
    seated=bool(np.all(depth[final]>.020) and np.all(depth[final]<.030) and np.max(lateral[final])<cfg['insertion_tolerance_m'] and np.max(angle[final])<.06 and part_contact[final].any())
    checks=[];local=np.zeros_like(world)
    for i,spec in enumerate(case.data['arms']):
        j=0 if spec['object']=='tabletop' else 1;Rc=R[j]@contact_basis(spec['normal_object'])
        local[:,i,:3]=np.einsum('nji,nj->ni',Rc,world[:,i,:3]);local[:,i,3:]=np.einsum('nji,nj->ni',Rc,world[:,i,3:])
        active=(trace.time>=2)&(trace.time<(cfg['release_time'] if i==2 else cfg['duration']))
        f=local[active,i,3:];moment=local[active,i,:3]
        if spec['kind']=='P':violation=max(float(np.max(-f[:,2])),float(np.max(f[:,2]-case.data['optimization']['point_normal_max'])),float(np.max(np.linalg.norm(f[:,:2],axis=1)-case.data['optimization']['point_mu']*f[:,2])))
        else:violation=float(np.max(abs(f)-np.array(case.data['optimization']['gripper_force_bounds'])))
        moment_violation=max(0.,float(np.max(abs(moment)-np.array(case.data['optimization']['gripper_moment_bounds'])))) if spec['kind']=='G' else 0.
        checks.append({'arm_id':spec['id'],'kind':spec['kind'],'maximum_force_violation_N':max(0.,violation),'maximum_moment_violation_Nm':moment_violation,'peak_force_N':np.max(abs(f),axis=0).tolist(),'peak_moment_Nm':np.max(abs(moment),axis=0).tolist(),
                       'contact_fraction':float(np.mean(flags[active,i].all(axis=1)))})
    report={'scenario':'table_assembly','model_family':'franka_panda','arms':3,'arm_dof':21,'finger_dof':4,'objects':2,'tool_types':['G','P','G'],
            'duration_seconds':float(trace.time[-1]),'leg_release_time_s':cfg['release_time'],'contact_model':{'pad_torsional_friction_m':cfg.get('pad_torsional_friction_m',.005),'pad_friction':cfg.get('pad_friction',.9),'friction_impratio':cfg.get('friction_impratio',100),'cone':'elliptic','solver':'Newton'},'timestep':dt,'keyframes':len(errors),'planner':'SE3_IK_path_with_IPOPT_dynamic_wrench_allocation',
            'primal_residual':plan.primal_residual,'keyframe_errors':errors,'tracking_passed':tracking,'tool_force_checks':checks,
            'force_envelopes_passed':all(c['maximum_force_violation_N']<.1 and c['maximum_moment_violation_Nm']<.01 for c in checks),'grasp_retention_passed':all(c['contact_fraction']>.95 for c in checks),
            'unexpected_penetrations_m':unexpected,'sampled_collision_check_passed':not unexpected,'equality_constraints_used':0,
            'insertion_depth_m':float(np.mean(depth[final])),'insertion_lateral_error_m':float(np.max(lateral[final])),'insertion_angle_rad':float(np.max(angle[final])),
            'leg_released':released,'assembly_seated':seated,'full_geometry_certified':False,'network_inference_executed':False,
            'limitations':['assumed contracted contacts; no learned-network inference','prescribed SE3 paths and mode schedule; not full contact-implicit search',
                           'synthetic gripper wrench bounds; no hardware calibration','collision STL convex hulls and MuJoCo soft contacts; no continuous collision certificate',
                           'gravity-seated peg/socket joint; no fastener, adhesive, or structural strength certificate']}
    report['execution_passed']=bool(tracking and seated and released and report['grasp_retention_passed'] and report['force_envelopes_passed'] and not unexpected)
    return report,flags,local,depth,lateral


def run_table_assembly(output='outputs/table_assembly',*,case_path=None,settings=None,progress=None,plan=None):
    output=Path(output);output.mkdir(parents=True,exist_ok=True)
    if plan is None:
        case=AssemblyCase.load(case_path)
        if progress:progress('optimization')
        plan=plan_assembly(case,settings,progress)
    case=plan.case
    (output/'case.json').write_text(json.dumps(case.data,indent=2,allow_nan=False)+'\n')
    asset_names={a['urdf'] for a in case.data['arms']}|{o['urdf'] for o in case.data['objects']}|{'panda.srdf','provenance.json','LICENSE.upstream','LICENSE.authored'}
    for name in asset_names:
        p=case.path.parent/name;destination=output/name
        if p.is_file() and p.resolve()!=destination.resolve():
            destination.parent.mkdir(parents=True,exist_ok=True)
            shutil.copyfile(p,destination)
    fixtures=build_assembly_scene(case,output/'scene.xml')
    names=[a['id']+'_'+n for a,r in zip(case.data['arms'],case.full_robots) for n in r.joint_names]
    motors=[a['id']+'_motor_'+n for a,r in zip(case.data['arms'],case.full_robots) for n in r.joint_names]
    scene=MujocoContactScene(output/'scene.xml',names,motors,[o['id'] for o in case.data['objects']])
    check_planned_collisions(plan,scene)
    if progress:progress('simulation')
    controller=AssemblyController(plan)
    identity=hashlib.sha256(json.dumps(case.data,sort_keys=True).encode()+b''.join(r.path.read_bytes() for r in (*case.robots,*case.parts))).hexdigest()
    plan_id=hashlib.sha256(plan.q.tobytes()+plan.reactions_world.tobytes()).hexdigest()
    trace=scene.run(controller,controller.reference,case.data['simulation']['duration'],case_id=identity,plan_id=plan_id,initial_q=controller.reference(0)[0],
                    progress=(lambda t,d:progress('simulation',time_s=t,duration_s=d)) if progress else None)
    trace.metadata.update(scenario='table_assembly',model_scope='three_real_Panda_arms_two_free_URDF_objects',tool_types=['G','P','G'],fixtures=fixtures)
    trace.save(output/'execution.npz');np.savez_compressed(output/'plan.npz',time=plan.times,q=plan.q,v=plan.v,a=plan.a,reactions_world=plan.reactions_world,torque=plan.torque)
    if progress:progress('validation')
    report,flags,wrenches,depth,lateral=assembly_report(plan,trace)
    (output/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    np.savez_compressed(output/'assembly_metrics.npz',contact_flags=flags,tool_wrenches=wrenches,insertion_depth=depth,lateral_error=lateral)
    return plan,trace,report,flags
