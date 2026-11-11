"""Main single-panel experiment: refined source anchors, free-body execution.

Contact assignment, SE(3) path and fixture load schedule are supplied. This is
not contact-implicit trajectory search, network inference or hardware control.
"""
from dataclasses import dataclass
from importlib.resources import files
import hashlib,json,shutil
from pathlib import Path
import xml.etree.ElementTree as ET
import casadi as ca
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.interpolate import CubicHermiteSpline
from .contacts import Capability,TaskWrench,contact_basis,map_reaction,gravity_wrench
from .conic import fixed_wrench_margin
from .contracted import numerical_pose_ik,pose_matrix
from .contact_simulation import MujocoContactScene
from .optimization import SolverSettings,equality,solve
from .robot import Robot
from .realization import Arm
from .simulation import _text,_quat
from .source_transfer import refine_source_and_transfer
from .table_assembly import smooth
from .urdf import origin
from .urdf_scene import append_urdf_robot


@dataclass
class PanelCase:
    path: Path
    data: dict
    robots: tuple
    full_robots: tuple
    panel: Robot

    @classmethod
    def load(cls,path=None):
        path=Path(path or str(files('contact_transfer').joinpath('scenarios/panel_fixture/case.json'))).resolve()
        d=json.loads(path.read_text());times=np.array([k['time'] for k in d['keyframes']],float)
        if d['id']!='panel_fixture' or d['units']!='SI' or len(d['arms'])<2:raise ValueError('Invalid panel scenario')
        if times[0]!=0 or not np.isfinite(times).all() or np.any(np.diff(times)<=0) or times[-1]!=d['simulation']['duration']:raise ValueError('Invalid panel keyframe times')
        for k in d['keyframes']:pose_matrix(k['object_pose'])
        full=[];reduced=[]
        def asset(name):
            result=(path.parent/name).resolve()
            if not result.is_relative_to(path.parent):raise ValueError('Scenario assets must be local')
            return result
        for spec in d['arms']:
            if spec['kind'] not in {'G','P'} or spec['controller'] not in {'torque','position'}:raise ValueError('Unsupported tool/controller')
            T=np.eye(4);T[:3,:3]=Rotation.from_euler('xyz',spec['base_rpy']).as_matrix();T[:3,3]=spec['base_position']
            full.append(Robot(asset(spec['urdf']),base_pose=T));reduced.append(Robot(asset(spec['urdf']),base_pose=T,locked_joints=spec['locked_joints']))
            if len(spec['q_initial'])!=reduced[-1].n:raise ValueError('Initial joint count mismatch')
        panel=Robot(asset(d['panel_urdf']),floating_base=True)
        return cls(path,d,tuple(reduced),tuple(full),panel)


def object_reference(case,t):
    keys=case.data['keyframes'];times=np.array([k['time'] for k in keys]);i=int(np.clip(np.searchsorted(times,t,side='right')-1,0,len(times)-2))
    h=times[i+1]-times[i];s=np.clip((t-times[i])/h,0,1);f=smooth(s);fd=(30*s*s-60*s**3+30*s**4)/h;fdd=(60*s-180*s*s+120*s**3)/h**2
    a,b=np.array(keys[i]['object_pose']),np.array(keys[i+1]['object_pose']);R0=Rotation.from_quat(a[3:]);dr=(R0.inv()*Rotation.from_quat(b[3:])).as_rotvec()
    rot=R0*Rotation.from_rotvec(f*dr);R=rot.as_matrix();w=fd*dr;v=R.T@(fd*(b[:3]-a[:3]))
    return np.r_[a[:3]+f*(b[:3]-a[:3]),rot.as_quat()],np.r_[w,v],np.r_[fdd*dr,R.T@(fdd*(b[:3]-a[:3]))-np.cross(w,v)]


def tool_targets(case,t):
    T=pose_matrix(object_reference(case,t)[0]);cfg=case.data['simulation'];targets=[]
    retreat=smooth((t-cfg['release_time']-cfg['release_seconds'])/(cfg['retract_end']-cfg['release_time']-cfg['release_seconds']))
    for spec in case.data['arms']:
        tool=np.eye(4);tool[:3,:3]=T[:3,:3]@Rotation.from_quat(spec['tool_quaternion_xyzw']).as_matrix()
        tool[:3,3]=T[:3,3]+T[:3,:3]@spec['point_object']+retreat*np.array(spec['retract_world']);targets.append(tool)
    return targets


@dataclass
class PanelPlan:
    case: PanelCase
    times: np.ndarray
    q: np.ndarray
    v: np.ndarray
    a: np.ndarray
    reactions_world: np.ndarray
    torque: np.ndarray
    primal_residual: float
    q_slices: tuple
    source: dict
    target_validation: list


def plan_panel(case,settings=None,progress=None):
    source=refine_source_and_transfer(case.path.parent/case.data['source_file'],enabled=case.data['source_refinement'],settings=settings)
    if not np.isclose(source['source_to_target_scale'],case.data['source_to_target_scale']):raise ValueError('Scale metadata mismatch')
    panel_size=np.array(ET.parse(case.path.parent/case.data['panel_urdf']).find('link/collision/geometry/box').get('size').split(),float)
    if not np.allclose(np.array(source['source_object']['size'])*source['source_to_target_scale'],panel_size):raise ValueError('Source scaling must match target URDF geometry')
    # Transfer geometry by group ID. Target mass, friction and torque bounds are
    # independently loaded again; no source force or source feasibility scales.
    for spec in case.data['arms']:
        i=source['source_group_ids'].index(spec['group_id']);spec['point_object']=source['target_positions_m'][i];spec['normal_object']=source['target_normals'][i]
    cfg=case.data['optimization'];sim=case.data['simulation'];times=np.linspace(0,sim['duration'],round(sim['duration']/cfg['sample_dt'])+1)
    offsets=np.cumsum([0]+[r.n for r in case.robots]);slices=tuple(slice(a,b) for a,b in zip(offsets[:-1],offsets[1:]));Q=np.zeros((len(times),offsets[-1]));seeds=[np.array(a['q_initial']) for a in case.data['arms']]
    for k,t in enumerate(times):
        for i,(robot,spec,target,sl) in enumerate(zip(case.robots,case.data['arms'],tool_targets(case,t),slices)):
            try:seeds[i]=numerical_pose_ik(robot,spec['tool_frame'],target,seeds[i])
            except RuntimeError as error:raise RuntimeError(f'{spec["id"]} at {t:.2f}s: {error}') from error
            Q[k,sl]=seeds[i]
    V=np.gradient(Q,times,axis=0,edge_order=2);A=np.gradient(V,times,axis=0,edge_order=2)
    for r,sl in zip(case.robots,slices):
        if np.any(abs(V[:,sl])>r.velocity):raise RuntimeError('Panel path exceeds URDF velocity limits')
    caps=[Capability('G',H=np.eye(6),bounds=cfg['gripper_moment_bounds']+cfg['gripper_force_bounds']) if s['kind']=='G' else Capability('P',mu=cfg['point_mu'],normal_max=cfg['point_normal_max']) for s in case.data['arms']]
    opti=ca.Opti();reactions=[];torques=[];objective=0.;mass=float(case.panel.model.inertias[1].mass)
    for k,t in enumerate(times):
        pose,vel,acc=object_reference(case,t);R=Rotation.from_quat(pose[3:]).as_matrix()
        lam=[ca.DM.zeros(6,1) if t>=sim['release_time'] else cap.reaction(opti) for cap in caps]
        env_force=mass*9.81*smooth((t-sim['seat_time'])/(sim['load_transfer_end']-sim['seat_time']))
        external=np.r_[np.zeros(3),R.T@np.array([0,0,env_force])]
        required=-case.panel.inverse_dynamics(pose,vel,acc)+external
        wrench=sum(map_reaction(s['point_object'],contact_basis(s['normal_object']),l) for s,l in zip(case.data['arms'],lam))
        if t<sim['release_time']:equality(opti,wrench-required)
        elif np.max(abs(required))>1e-8:raise RuntimeError('Released panel not balanced by planned fixture load')
        world=[];taus=[]
        for robot,spec,sl,force in zip(case.robots,case.data['arms'],slices,lam):
            Rc=R@contact_basis(spec['normal_object']);w=ca.vertcat(ca.DM(Rc)@force[:3],ca.DM(Rc)@force[3:]);world.append(w)
            T=robot.fk(Q[k,sl],spec['tool_frame']);dual=ca.vertcat(ca.DM(T[:3,:3].T)@w[:3],w[3:]);tau=ca.DM(robot.inverse_dynamics(Q[k,sl],V[k,sl],A[k,sl]))-ca.DM(robot.jacobian(Q[k,sl],spec['tool_frame']).T)@dual
            if tau.is_constant():
                if np.any(abs(np.array(ca.evalf(tau)).reshape(-1))>robot.effort+1e-9):raise RuntimeError('Released arm exceeds URDF effort')
            else:opti.subject_to(opti.bounded(-robot.effort,tau,robot.effort))
            taus.append(tau);objective+=1e-3*ca.sumsqr(force[3:])+20*ca.sumsqr(force[:3])+1e-5*ca.sumsqr(tau)
        reactions.append(ca.horzcat(*world).T);torques.append(ca.vertcat(*taus))
    opti.minimize(objective);sol,residual=solve(opti,settings or SolverSettings(options={'expand':False}))
    validation=[]
    for key in case.data['keyframes']:
        if key['time']>=sim['seat_time']:continue
        k=int(np.argmin(abs(times-key['time'])));R=Rotation.from_quat(key['object_pose'][3:]).as_matrix()
        task=TaskWrench(gravity_wrench(mass,[0,0,0],R),np.eye(6),np.diag(cfg['disturbance_scale']))
        # Common frozen-geometry conic validator includes each robot's own
        # Jacobian, gravity and URDF effort; source margin is never reused.
        kwargs=dict(arms=[Arm(r,s['tool_frame'],Q[k,sl]) for r,s,sl in zip(case.robots,case.data['arms'],slices)],q=[Q[k,sl] for sl in slices],assignment=list(range(len(case.robots))),object_pose=pose_matrix(key['object_pose']))
        check=fixed_wrench_margin([s['point_object'] for s in case.data['arms']],[contact_basis(s['normal_object']) for s in case.data['arms']],task,caps,**kwargs)
        validation.append({'keyframe':key['id'],'rho':check.rho,'primal_residual':check.primal_residual,'status':check.status})
    return PanelPlan(case,times,Q,V,A,np.array([sol.value(x) for x in reactions]),np.array([sol.value(x) for x in torques]),residual,slices,source,validation)


def build_panel_scene(case,destination):
    cfg=case.data['simulation'];root=ET.Element('mujoco',model='single_panel_fixture');ET.SubElement(root,'compiler',angle='radian',inertiafromgeom='false',fusestatic='false')
    ET.SubElement(root,'option',timestep=str(cfg['timestep']),gravity='0 0 -9.81',integrator='implicitfast',cone='elliptic',impratio=str(cfg['friction_impratio']),iterations='100',tolerance='1e-10')
    assets=ET.SubElement(root,'asset');world=ET.SubElement(root,'worldbody');actuators=ET.SubElement(root,'actuator');contact=ET.SubElement(root,'contact')
    for robot,spec in zip(case.full_robots,case.data['arms']):
        prefix=spec['id']+'_';cache='rb5_meshes' if spec['model']=='rb5' else 'panda_meshes';meshes={p.name:p.resolve() for p in (Path.cwd()/'.cache'/cache).glob('*.stl')}
        geoms=append_urdf_robot(world,actuators,robot,prefix,mesh_files=meshes,assets=assets,robot_collisions=True)
        if spec['controller']=='position':
            for motor in list(actuators):
                if not motor.get('name').startswith(prefix):continue
                name=motor.get('joint')[len(prefix):];j=robot.joint_names.index(name);attrs=dict(motor.attrib);actuators.remove(motor)
                attrs.update(kp=str(cfg['position_kp']),kv=str(cfg['position_kd']),ctrlrange=_text([robot.lower[j],robot.upper[j]]),forcerange=_text([-robot.effort[j],robot.effort[j]]));ET.SubElement(actuators,'position',**attrs)
        for geom in geoms:
            pad='leftfinger_collision_' in geom or 'rightfinger_collision_' in geom
            if pad or spec['kind']=='P' and geom.startswith(prefix+'tool_collision_'):
                mu=cfg['pad_friction'] if pad else case.data['optimization']['point_mu'];ET.SubElement(contact,'pair',geom1=geom,geom2='panel_collision_0',condim='4' if pad else '3',friction=_text([mu,mu,cfg['pad_torsional_friction_m'] if pad else 0,0,0]),solref='.008 1',solimp='.95 .99 .001')
        if spec['model']=='franka_panda':
            for pair in ET.parse(case.path.parent/'panda.srdf').findall('disable_collisions'):
                a,b=pair.get('link1'),pair.get('link2')
                if a in robot.tree.links and b in robot.tree.links:ET.SubElement(contact,'exclude',body1=prefix+('urdf_'+a if a==robot.tree.root else a),body2=prefix+('urdf_'+b if b==robot.tree.root else b))
        else:
            for joint in robot.tree.joints:ET.SubElement(contact,'exclude',body1=prefix+('urdf_'+joint.parent if joint.parent==robot.tree.root else joint.parent),body2=prefix+joint.child)
    pose=object_reference(case,0)[0];body=ET.SubElement(world,'body',name='panel',pos=_text(pose[:3]),quat=_text([pose[6],*pose[3:6]]));ET.SubElement(body,'freejoint',name='panel_free')
    link=ET.parse(case.path.parent/case.data['panel_urdf']).find('link');inertial=link.find('inertial');I=inertial.find('inertia');_,pi=origin(inertial)
    ET.SubElement(body,'inertial',pos=_text(pi),mass=inertial.find('mass').get('value'),diaginertia=_text([float(I.get('ixx')),float(I.get('iyy')),float(I.get('izz'))]))
    size=np.array(link.find('collision/geometry/box').get('size').split(),float)/2
    ET.SubElement(body,'geom',name='panel_collision_0',type='box',size=_text(size),mass='0',contype='1',conaffinity='3',condim='3',friction='.9 0 0',solref='.008 1',solimp='.95 .99 .001',rgba='.72 .49 .27 1')
    fixture=[]
    for i,g in enumerate(ET.parse(case.path.parent/case.data['jig_urdf']).findall('link/collision')):
        R,p=origin(g);size=np.array(g.find('geometry/box').get('size').split(),float);ET.SubElement(world,'geom',name='jig_collision_'+str(i),type='box',size=_text(size/2),pos=_text(p),quat=_text(_quat(R)),contype='1',conaffinity='3',rgba='.22 .32 .43 1')
        fixture.append(dict(name='jig_'+str(i),position=p.tolist(),size=size.tolist(),quaternion_xyzw=Rotation.from_matrix(R).as_quat().tolist()))
    ET.SubElement(world,'geom',name='floor',type='plane',size='2 2 .1',contype='1',conaffinity='3')
    ET.indent(root);Path(destination).write_bytes(ET.tostring(root,encoding='utf-8',xml_declaration=True));return fixture


class PanelController:
    def __init__(self,plan):
        self.plan=plan;self.cfg=plan.case.data['simulation'];self.spline=CubicHermiteSpline(plan.times,plan.q,plan.v);self.t=0.;off=0;self.slices=[];self.indices=[]
        for full,reduced in zip(plan.case.full_robots,plan.case.robots):
            self.slices.append(slice(off,off+full.n));off+=full.n;self.indices.append(np.array([full.joint_names.index(n) for n in reduced.joint_names]))
    def reference(self,t):
        self.t=t;qred,vred,ared=[self.spline(t,n) for n in [0,1,2]];qs=[];vs=[];acc=[]
        for robot,spec,sl,indices in zip(self.plan.case.full_robots,self.plan.case.data['arms'],self.plan.q_slices,self.indices):
            q=np.array([spec['locked_joints'].get(n,0.) for n in robot.joint_names]);v=np.zeros(robot.n);a=v.copy();q[indices]=qred[sl];v[indices]=vred[sl];a[indices]=ared[sl]
            fingers=[i for i,n in enumerate(robot.joint_names) if n in spec['locked_joints']]
            opened=smooth((t-self.cfg['release_time'])/self.cfg['release_seconds']);close=smooth(t/self.cfg['grasp_seconds'])
            initial=np.array([spec['locked_joints'][robot.joint_names[j]] for j in fingers]);q[fingers]=initial+(self.cfg['finger_closed']-initial)*close+(self.cfg['finger_open']-self.cfg['finger_closed'])*opened
            qs.append(q);vs.append(v);acc.append(a)
        return np.concatenate(qs),np.concatenate(vs),np.concatenate(acc)
    def command(self,q,v,desired,velocity,acceleration):
        k=int(np.clip(np.searchsorted(self.plan.times,self.t)-1,0,len(self.plan.times)-1));out=[]
        for i,(r,spec,sl,indices) in enumerate(zip(self.plan.case.full_robots,self.plan.case.data['arms'],self.slices,self.indices)):
            feedback=self.cfg['arm_kp']*(desired[sl]-q[sl])+self.cfg['arm_kd']*(velocity[sl]-v[sl]);tau=r.inverse_dynamics(q[sl],v[sl],acceleration[sl]+feedback)
            w=self.plan.reactions_world[k,i];T=r.fk(q[sl],spec['tool_frame']);J=r.jacobian(q[sl],spec['tool_frame']);tau-=J.T@np.r_[T[:3,:3].T@w[:3],w[3:]]
            Td=r.fk(desired[sl],spec['tool_frame']);Jd=r.jacobian(desired[sl],spec['tool_frame']);actual_twist=J@v[sl];desired_twist=Jd@velocity[sl]
            moment=self.cfg['orientation_kp']*Rotation.from_matrix(T[:3,:3].T@Td[:3,:3]).as_rotvec()+self.cfg['orientation_kd']*(T[:3,:3].T@Td[:3,:3]@desired_twist[:3]-actual_twist[:3])
            force=self.cfg['cartesian_kp']*(Td[:3,3]-T[:3,3])+self.cfg['cartesian_kd']*(desired_twist[3:]-actual_twist[3:]);tau+=J.T@np.r_[moment,force]
            fingers=[j for j,n in enumerate(r.joint_names) if n in spec['locked_joints']]
            if fingers:tau[fingers]=r.gravity(q[sl])[fingers]+self.cfg['finger_kp']*(desired[sl][fingers]-q[sl][fingers])-self.cfg['finger_kd']*v[sl][fingers]
            if spec['controller']=='position':
                # Simulation-only inverse of an explicitly assumed inner PD.
                # No torque is sent to a position-controlled robot.
                tau_ff=r.inverse_dynamics(desired[sl],velocity[sl],acceleration[sl])-Jd.T@np.r_[Td[:3,:3].T@w[:3],w[3:]]
                command=desired[sl]+(tau_ff+self.cfg['position_kd']*velocity[sl])/self.cfg['position_kp'];out.append(np.clip(command,r.lower,r.upper))
            else:out.append(np.clip(tau,-r.effort,r.effort))
        return np.concatenate(out)


def unexpected_pair(case,pair):
    for spec in case.data['arms']:
        robot=next((n for n in pair if n.startswith(spec['id']+'_')),None)
        if robot is None:continue
        tool=any(link+'_collision_' in robot for link in (['leftfinger','rightfinger'] if spec['kind']=='G' else ['tool']))
        if tool and any(n.startswith('panel_collision_') for n in pair):return False
        return True
    return False


def check_planned_collisions(plan,scene):
    mj,model,data=scene.mj,scene.model,scene.data;controller=PanelController(plan);bad={}
    for t in plan.times:
        data.qpos[scene.qadr]=controller.reference(float(t))[0];body=scene.objects[0];adr=model.jnt_qposadr[model.body_jntadr[body]];pose=object_reference(plan.case,float(t))[0];data.qpos[adr:adr+7]=np.r_[pose[:3],pose[6],pose[3:6]];mj.mj_forward(model,data)
        for j in range(data.ncon):
            c=data.contact[j];pair=[mj.mj_id2name(model,mj.mjtObj.mjOBJ_GEOM,int(g)) for g in c.geom]
            if c.dist<-.0005 and unexpected_pair(plan.case,pair):bad[' / '.join(pair)]=min(bad.get(' / '.join(pair),0),float(c.dist))
    if bad:raise RuntimeError('Planned mesh penetration: '+json.dumps(bad))


def panel_report(plan,trace):
    case=plan.case;cfg=case.data['simulation'];dt=trace.metadata['timestep'];poses=trace.object_pose[:,0];R=Rotation.from_quat(poses[:,3:]).as_matrix();N=len(trace.time)
    flags=np.zeros((N,len(case.robots),2),bool);world=np.zeros((N,len(case.robots),6));jig_force=np.zeros((N,3));unexpected={}
    refs=[poses[:,:3]+np.einsum('nij,j->ni',R,s['point_object']) for s in case.data['arms']]
    for t,pair,pos,gap,wrench in zip(trace.contact_time,trace.contact_pair,trace.contact_position,trace.contact_gap,trace.contact_wrench):
        k=int(round(t/dt))
        if gap<-.0005 and unexpected_pair(case,pair):unexpected[' / '.join(pair)]=min(unexpected.get(' / '.join(pair),0),float(gap))
        if any(n.startswith('jig_collision_') for n in pair) and any(n.startswith('panel_collision_') for n in pair):jig_force[k]+=wrench[3:]*(1 if pair[1].startswith('panel_') else -1)
        for i,spec in enumerate(case.data['arms']):
            j=next((j for j,n in enumerate(pair) if n.startswith(spec['id']+'_')),None)
            if j is None or not pair[1-j].startswith('panel_collision_'):continue
            w=wrench*(1 if j==1 else -1);world[k,i]+=np.r_[w[:3]+np.cross(pos-refs[i][k],w[3:]),w[3:]]
            if spec['kind']=='P':flags[k,i,:]=True
            else:
                if 'leftfinger' in pair[j]:flags[k,i,0]=True
                if 'rightfinger' in pair[j]:flags[k,i,1]=True
    errors=[]
    for key in case.data['keyframes']:
        k=int(np.argmin(abs(trace.time-key['time'])));desired=np.array(key['object_pose']);actual=poses[k]
        errors.append({'id':key['id'],'time':float(trace.time[k]),'panel':{'translation_m':float(np.linalg.norm(actual[:3]-desired[:3])),'rotation_rad':float((Rotation.from_quat(desired[3:]).inv()*Rotation.from_quat(actual[3:])).magnitude())}})
    tracking=all(e['panel']['translation_m']<cfg['tracking_tolerance_m'] and e['panel']['rotation_rad']<cfg['tracking_tolerance_rad'] for e in errors)
    carrying=(trace.time>=2)&(trace.time<cfg['seat_time']);final=trace.time>=cfg['duration']-.5;checks=[]
    for i,spec in enumerate(case.data['arms']):
        Rc=R@contact_basis(spec['normal_object']);f=np.einsum('nji,nj->ni',Rc,world[:,i,3:])[carrying];m=np.einsum('nji,nj->ni',Rc,world[:,i,:3])[carrying]
        if spec['kind']=='P':violation=max(0.,float(np.max(-f[:,2])),float(np.max(f[:,2]-case.data['optimization']['point_normal_max'])),float(np.max(np.linalg.norm(f[:,:2],axis=1)-case.data['optimization']['point_mu']*f[:,2])));mv=0.
        else:violation=max(0.,float(np.max(abs(f)-case.data['optimization']['gripper_force_bounds'])));mv=max(0.,float(np.max(abs(m)-case.data['optimization']['gripper_moment_bounds'])))
        checks.append({'arm_id':spec['id'],'kind':spec['kind'],'contact_fraction':float(np.mean(flags[carrying,i].all(axis=1))),'maximum_force_violation_N':violation,'maximum_moment_violation_Nm':mv})
    mass=float(case.panel.model.inertias[1].mass);released=not flags[final].any();supported=bool(np.mean(jig_force[final,2])>.95*mass*9.81)
    effort=np.concatenate([r.effort for r in case.full_robots]);torque_margin=float(np.min(effort-abs(trace.torque)))
    report={'scenario':'panel_fixture','arms':len(case.robots),'arm_dof':sum(r.n for r in case.robots),'objects':1,'duration_seconds':float(trace.time[-1]),'keyframes':len(errors),'keyframe_errors':errors,
            'planner':'refined_source_geometry_SE3_IK_IPOPT_dynamic_wrench_allocation','source':plan.source,'target_validation':plan.target_validation,'target_mass_kg':mass,
            'controllers':[{'arm_id':s['id'],'model':s['model'],'command':s['controller'],'units':'rad' if s['controller']=='position' else 'Nm','hardware_transmission_executed':False} for s in case.data['arms']],
            'primal_residual':plan.primal_residual,'tracking_passed':tracking,'tool_force_checks':checks,'grasp_retention_passed':all(c['contact_fraction']>.95 for c in checks),'force_envelopes_passed':all(c['maximum_force_violation_N']<.1 and c['maximum_moment_violation_Nm']<.01 for c in checks),
            'jig_support_final_N':float(np.mean(jig_force[final,2])),'panel_weight_N':mass*9.81,'fixture_supported':supported,'all_tools_released':released,'minimum_execution_torque_margin_Nm':torque_margin,
            'unexpected_penetrations_m':unexpected,'sampled_collision_check_passed':not unexpected,'equality_constraints_used':0,'full_geometry_certified':False,'network_inference_executed':False,
            'timestep':dt,'initial_grasp_supplied':True,'limitations':['synthetic contracted input; no hand demonstration or network inference','supplied assignment, SE3 path and load-transfer schedule; not CITO search','synthetic grasp bounds and assumed servo gains; no hardware calibration','collision mesh convex hulls and soft contacts; no continuous certificate']}
    report['execution_passed']=bool(tracking and supported and released and report['grasp_retention_passed'] and report['force_envelopes_passed'] and not unexpected and torque_margin>=-1e-6)
    return report,flags,jig_force


def run_panel(output='outputs/panel_fixture',*,case_path=None,settings=None,progress=None,plan=None):
    output=Path(output);output.mkdir(parents=True,exist_ok=True)
    if plan is None:
        if progress:progress('source_refinement_and_optimization')
        plan=plan_panel(PanelCase.load(case_path),settings,progress)
    case=plan.case;(output/'case.json').write_text(json.dumps(case.data,indent=2)+'\n')
    names={a['urdf'] for a in case.data['arms']}|{case.data['panel_urdf'],case.data['jig_urdf'],case.data['source_file'],'panda.srdf','franka_panda_provenance.json','rb5_provenance.json','LICENSE.panda','LICENSE.rb5','LICENSE.authored'}
    for name in names:
        src=case.path.parent/name;dst=output/name
        if src.exists() and src.resolve()!=dst.resolve():shutil.copyfile(src,dst)
    fixtures=build_panel_scene(case,output/'scene.xml');joint_names=[s['id']+'_'+n for s,r in zip(case.data['arms'],case.full_robots) for n in r.joint_names];motors=[s['id']+'_motor_'+n for s,r in zip(case.data['arms'],case.full_robots) for n in r.joint_names];modes=[s['controller'] for s,r in zip(case.data['arms'],case.full_robots) for n in r.joint_names]
    scene=MujocoContactScene(output/'scene.xml',joint_names,motors,['panel'],actuator_modes=modes);check_planned_collisions(plan,scene)
    if progress:progress('simulation')
    controller=PanelController(plan);trace=scene.run(controller,controller.reference,case.data['simulation']['duration'],case_id=hashlib.sha256(json.dumps(case.data,sort_keys=True).encode()).hexdigest(),plan_id=hashlib.sha256(plan.q.tobytes()+plan.reactions_world.tobytes()).hexdigest(),initial_q=controller.reference(0)[0],progress=(lambda t,d:progress('simulation',time_s=t,duration_s=d)) if progress else None)
    trace.metadata.update(scenario='panel_fixture',fixtures=fixtures,initial_grasp_supplied=True);trace.save(output/'execution.npz');np.savez_compressed(output/'plan.npz',time=plan.times,q=plan.q,v=plan.v,a=plan.a,reactions_world=plan.reactions_world,torque=plan.torque)
    report,flags,jig=panel_report(plan,trace);(output/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n');np.savez_compressed(output/'panel_metrics.npz',contact_flags=flags,jig_force=jig)
    return plan,trace,report,flags
