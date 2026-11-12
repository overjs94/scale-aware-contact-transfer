"""Differential-drive RBY1 + fixed Panda handover beyond fixed-base reach.

Free mobile base driven by wheel torques and physical floor contacts. Supplied
straight corridor/path and mode schedule; source anchors and target reactions
are optimized. No mobile/object pose is assigned during execution.
"""
from dataclasses import dataclass
from importlib.resources import files
import hashlib,json,shutil
from pathlib import Path
import xml.etree.ElementTree as ET
import casadi as ca
import numpy as np
from scipy.interpolate import CubicHermiteSpline
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from .robot import Robot
from .contacts import Capability,contact_basis,map_reaction
from .contracted import numerical_pose_ik,pose_matrix
from .optimization import equality,solve,SolverSettings
from .source_transfer import refine_source_and_transfer
from .table_assembly import smooth
from .panel_fixture import object_reference as panel_reference
from .simulation import _text,_quat
from .urdf import origin
from .urdf_scene import append_urdf_robot
from .contact_simulation import MujocoContactScene


@dataclass
class MobileCase:
    path: Path
    data: dict
    rby: Robot
    floating: Robot
    ik: Robot
    panda: Robot
    panel: Robot
    ready: np.ndarray
    arm_indices: np.ndarray

    @classmethod
    def load(cls,path=None):
        path=Path(path or str(files('contact_transfer').joinpath('scenarios/mobile_panel/case.json'))).resolve();d=json.loads(path.read_text())
        if d['id']!='mobile_panel' or d['units']!='SI' or not 30<=d['duration']<=60:raise ValueError('Invalid mobile panel case')
        def asset(n):
            p=(path.parent/n).resolve()
            if not p.is_relative_to(path.parent):raise ValueError('Assets must be local')
            return p
        r=Robot(asset(d['rby_urdf']));ready=r.neutral()
        for i,n in enumerate(r.joint_names):
            if n.startswith('torso_'):ready[i]=d['torso'][int(n[-1])]
        locked={n:ready[i] for i,n in enumerate(r.joint_names) if not ('_arm_' in n)}
        ik=Robot(r.path,locked_joints=locked);indices=np.array([r.joint_names.index(n) for n in ik.joint_names])
        T=np.eye(4);T[:3,3]=d['panda_base'];T[:3,:3]=Rotation.from_euler('z',d['panda_yaw']).as_matrix();panda=Robot(asset(d['panda_urdf']),base_pose=T)
        mobile=ET.parse(r.path).find('mobile')
        if mobile.get('type')!='differential':raise ValueError('RBY1-A requires differential drive')
        times=np.array([k['time'] for k in d['keyframes']]);
        if times[0]!=0 or times[-1]!=d['duration'] or np.any(np.diff(times)<=0):raise ValueError('Invalid keyframe timing')
        return cls(path,d,r,Robot(r.path,floating_base=True),ik,panda,Robot(asset('panel.urdf'),floating_base=True),ready,indices)


def base_reference(case,t):
    d=case.data;h=d['base_end']-d['base_start'];s=np.clip((t-d['base_start'])/h,0,1)
    return np.array([d['base_distance']*smooth(s),0.,0.]),np.array([d['base_distance']*(30*s*s-60*s**3+30*s**4)/h,0.,0.])


def base_acceleration(case,t):
    d=case.data;h=d['base_end']-d['base_start'];s=np.clip((t-d['base_start'])/h,0,1)
    return np.array([d['base_distance']*(60*s-180*s*s+120*s**3)/h**2,0.,0.])


def object_reference(case,t):
    d=case.data;pose,v,a=panel_reference(case,t);b,bd=base_reference(case,t)
    # During transport, keep tool poses relative to the nonholonomic base.
    if case.data['base_start']<=t<=case.data['base_end']:
        h=case.data['base_end']-case.data['base_start'];s=(t-case.data['base_start'])/h;pose[0]=d['keyframes'][0]['object_pose'][0]+b[0]
        R=Rotation.from_quat(pose[3:]).as_matrix();v[3:]=R.T@bd;a[3:]=R.T@np.array([case.data['base_distance']*(60*s-180*s*s+120*s**3)/h**2,0,0])
    return pose,v,a


def targets(case,t,source):
    d=case.data;pose=object_reference(case,t)[0];T=pose_matrix(pose);result=[]
    # Order: RBY right gripper, RBY left spherical support, fixed Panda support.
    for i,gid in enumerate(['edge_grip','under_right','under_left']):
        j=source['source_group_ids'].index(gid);p=np.array(source['target_positions_m'][j]);tool=np.eye(4);tool[:3,:3]=T[:3,:3]@Rotation.from_quat(d['tool_rotations'][i]).as_matrix();tool[:3,3]=T[:3,3]+T[:3,:3]@p
        if i==2 and t>=d['fixed_release']:
            hold=pose_matrix(object_reference(case,d['fixed_release'])[0]);tool[:3,3]=hold[:3,3]+hold[:3,:3]@p+np.array([0,.35,-.10])*smooth((t-d['fixed_release'])/(d['base_start']-d['fixed_release']))
        if i<2:tool[:3,3]+=np.array([-.06 if i==1 else 0,-.08 if i==0 else .03,-.025 if i==1 else 0])*smooth((t-d['release_time']-d['release_seconds'])/(d['retract_end']-d['release_time']-d['release_seconds']))
        result.append(tool)
    return result


def dual_ik(case,Ts,seed):
    def residual(q):
        errors=[]
        for frame,T in zip(['tip_right','tip_left'],Ts):
            A=case.ik.fk(q,frame);errors.extend(A[:3,3]-T[:3,3]);errors.extend(Rotation.from_matrix(T[:3,:3].T@A[:3,:3]).as_rotvec())
        return np.r_[errors,1e-5*(q-seed)]
    sol=least_squares(residual,np.clip(seed,case.ik.lower+1e-8,case.ik.upper-1e-8),bounds=(case.ik.lower+1e-9,case.ik.upper-1e-9),max_nfev=450,ftol=1e-10,xtol=1e-10,gtol=1e-10)
    if np.max(abs(residual(sol.x)[:12]))>2e-5:raise RuntimeError('RBY1 dual-arm IK failed')
    return sol.x


def reach_bound(robot,frame,shoulder):
    # All joint-origin translations from shoulder to tool, plus fixed tool offsets.
    child={j.child:j for j in robot.tree.joints};current=frame;radius=0.
    while current!=shoulder:
        j=child[current];radius+=np.linalg.norm(j.p);current=j.parent
    return float(radius)


@dataclass
class MobilePlan:
    case: MobileCase
    times: np.ndarray
    q: np.ndarray
    v: np.ndarray
    a: np.ndarray
    reactions: np.ndarray
    torque: np.ndarray
    residual: float
    source: dict
    workspace: dict


def plan_mobile(case,settings=None):
    d=case.data;source=refine_source_and_transfer(case.path.parent/d['source_file'],enabled=d['source_refinement'],settings=settings)
    size=np.array(ET.parse(case.panel.path).find('link/collision/geometry/box').get('size').split(),float)
    if not np.allclose(np.array(source['source_object']['size'])*source['source_to_target_scale'],size):raise ValueError('Source scale must match target URDF panel')
    times=np.linspace(0,d['duration'],round(d['duration']/d['sample_dt'])+1);Q=[]
    seed=np.array(d['rby_seed']) if d['rby_seed'] is not None else case.ready[case.arm_indices].copy();pseed=np.array(d['panda_seed']);track,radius=map(float,ET.parse(case.rby.path).find('mobile').get('params').split())
    for t in times:
        Ts=targets(case,t,source);b,_=base_reference(case,t);local=[T.copy() for T in Ts[:2]]
        for T in local:T[:3,3]-=np.r_[b[:2],0.]
        seed=dual_ik(case,local,seed);pseed=numerical_pose_ik(case.panda,'tip',Ts[2],pseed)
        q=case.ready.copy();q[case.arm_indices]=seed
        for wheel in ['right_wheel','left_wheel']:q[case.rby.joint_names.index(wheel)]=-b[0]/radius
        Q.append(np.r_[q,pseed])
    Q=np.array(Q);V=np.gradient(Q,times,axis=0,edge_order=2);A=np.gradient(V,times,axis=0,edge_order=2);n=case.rby.n
    if np.any(abs(V[:,:n])>case.rby.velocity) or np.any(abs(V[:,n:])>case.panda.velocity):raise RuntimeError('Mobile reference exceeds URDF velocity')
    cfg=d['optimization'];caps=[Capability('G',H=np.eye(6),bounds=cfg['gripper_moment_bounds']+cfg['gripper_force_bounds']),Capability('P',mu=cfg['point_mu'],normal_max=40),Capability('P',mu=cfg['point_mu'],normal_max=40)]
    gids=['edge_grip','under_right','under_left'];points=[source['target_positions_m'][source['source_group_ids'].index(g)] for g in gids];normals=[source['target_normals'][source['source_group_ids'].index(g)] for g in gids]
    opt=ca.Opti();L=[];Tau=[];cost=0.;mass=float(case.panel.model.inertias[1].mass)
    for k,t in enumerate(times):
        pose,v,a=object_reference(case,t);R=Rotation.from_quat(pose[3:]).as_matrix()
        required=-case.panel.inverse_dynamics(pose,v,a)+np.r_[np.zeros(3),R.T@np.array([0,0,mass*9.81*smooth((t-d['seat_time'])/(d['load_transfer_end']-d['seat_time']))])]
        total=max(0.,-required[5])*(-points[0][0])/(points[1][0]-points[0][0]);share=1. if t>=d['fixed_release'] else .6
        initial=[np.zeros(6),np.r_[np.zeros(5),total*share],np.r_[np.zeros(5),total*(1-share)]]
        remainder=required-sum(np.asarray(map_reaction(points[i],contact_basis(normals[i]),initial[i])).reshape(6) for i in [1,2]);W=np.asarray(map_reaction(points[0],contact_basis(normals[0]),np.eye(6)));initial[0]=np.linalg.solve(W,remainder)
        lam=[]
        for i,c in enumerate(caps):
            inactive=t>=d['release_time'] if i<2 else t>=d['fixed_release'];lam.append(ca.DM.zeros(6,1) if inactive else c.reaction(opt,initial=initial[i]))
        equality(opt,sum(map_reaction(p,contact_basis(nm),l) for p,nm,l in zip(points,normals,lam))-required)
        world=[ca.vertcat(ca.DM(R@contact_basis(nm))@l[:3],ca.DM(R@contact_basis(nm))@l[3:]) for nm,l in zip(normals,lam)];b,bd=base_reference(case,t);qf=np.r_[b[:2],0.,0.,0.,0.,1.,Q[k,:n]];vf=np.r_[np.zeros(3),bd,V[k,:n]];af=np.r_[np.zeros(3),base_acceleration(case,t),A[k,:n]];tr=ca.DM(case.floating.inverse_dynamics(qf,vf,af)[6:]);tp=ca.DM(case.panda.inverse_dynamics(Q[k,n:],V[k,n:],A[k,n:]))
        for i,(robot,q,frame,w) in enumerate([(case.rby,Q[k,:n],'tip_right',world[0]),(case.rby,Q[k,:n],'tip_left',world[1]),(case.panda,Q[k,n:],'tip',world[2])]):
            T=robot.fk(q,frame);dual=ca.vertcat(ca.DM(T[:3,:3].T)@w[:3],w[3:]);term=ca.DM(robot.jacobian(q,frame).T)@dual
            if i<2:tr-=term
            else:tp-=term
        tau=ca.vertcat(tr,tp);limits=np.r_[case.rby.effort,case.panda.effort]
        if tau.is_constant():
            if np.any(abs(np.array(ca.evalf(tau)).reshape(-1))>limits+1e-8):raise RuntimeError('Released joints exceed effort')
        else:opt.subject_to(opt.bounded(-limits,tau,limits))
        for l in lam:cost+=1e-3*ca.sumsqr(l[3:])+20*ca.sumsqr(l[:3])
        cost+=1e-5*ca.sumsqr(tau);L.append(ca.horzcat(*world).T);Tau.append(tau)
    opt.minimize(cost);sol,residual=solve(opt,settings or SolverSettings(options={'expand':False}))
    workspace=workspace_comparison(case,Q,source)
    return MobilePlan(case,times,Q,V,A,np.array([sol.value(x) for x in L]),np.array([sol.value(x) for x in Tau]),residual,source,workspace)


def workspace_comparison(case,Q,source):
    n=case.rby.n;d=case.data;rows={}
    for i,(frame,shoulder) in enumerate([('tip_right','link_right_arm_0'),('tip_left','link_left_arm_0')]):
        distance=float(np.linalg.norm(targets(case,d['base_end'],source)[i][:3,3]-case.rby.fk(Q[0,:n],shoulder)[:3,3]));bound=reach_bound(case.rby,frame,shoulder)
        rows[frame]=dict(fixed_base_target_distance_m=distance,URDF_chain_radius_bound_m=bound,provably_outside_fixed_base=distance>bound,comparison_scope='same_fixed_torso_ready_posture')
    pose=object_reference(case,d['base_end'])[0];R=Rotation.from_quat(pose[3:]).as_matrix();p=source['target_positions_m'][source['source_group_ids'].index('under_left')];target=pose[:3]+R@p;shoulder=case.panda.fk(Q[0,n:],'panda_link1')[:3,3];distance=float(np.linalg.norm(target-shoulder));bound=reach_bound(case.panda,'tip','panda_link1')
    rows['panda']=dict(fixed_base_target_distance_m=distance,URDF_chain_radius_bound_m=bound,provably_outside_fixed_base=distance>bound,comparison_scope='counterfactual_Panda_keeps_same_panel_anchor')
    return rows


def build_scene(case,destination):
    d=case.data;root=ET.Element('mujoco',model='mobile_RBY1_Panda_panel');ET.SubElement(root,'compiler',angle='radian',inertiafromgeom='false',fusestatic='false');ET.SubElement(root,'option',timestep=str(d['timestep']),gravity='0 0 -9.81',integrator='implicitfast',cone='elliptic',impratio='100',iterations='80',tolerance='1e-9')
    world=ET.SubElement(root,'worldbody');assets=ET.SubElement(root,'asset');act=ET.SubElement(root,'actuator');contact=ET.SubElement(root,'contact')
    for robot,prefix,cache in [(case.floating,'rby_',Path('.cache/rby1')),(case.panda,'panda_',Path('.cache/panda_meshes'))]:
        meshes={p.name:p.resolve() for p in cache.iterdir() if p.suffix in {'.obj','.stl'}};geoms=append_urdf_robot(world,act,robot,prefix,mesh_files=meshes,assets=assets,robot_collisions=True)
        for joint in robot.tree.joints:ET.SubElement(contact,'exclude',body1=prefix+('urdf_'+joint.parent if joint.parent==robot.tree.root else joint.parent),body2=prefix+joint.child)
        if prefix=='rby_':
            parents={j.child:j.parent for j in robot.tree.joints}
            for child,parent in parents.items():
                grand=parents.get(parent)
                if grand is not None:ET.SubElement(contact,'exclude',body1=prefix+('urdf_'+grand if grand==robot.tree.root else grand),body2=prefix+child)
        for geom in geoms:
            if 'pad_negative_collision' in geom or 'pad_positive_collision' in geom or geom in ['rby_contact_left_collision_0','panda_tool_collision_0']:
                ispad='pad_' in geom;ET.SubElement(contact,'pair',geom1=geom,geom2='panel_collision_0',condim='4' if ispad else '3',friction=_text([.9,.9,.008 if ispad else 0,0,0]),solref='.008 1',solimp='.95 .99 .001')
            if prefix=='rby_' and ('base_collision_' in geom):ET.SubElement(contact,'pair',geom1=geom,geom2='floor',condim='3',friction='.015 .015 0 0 0',solref='.008 1')
        if prefix=='panda_':
            for pair in ET.parse(case.path.parent/'panda.srdf').findall('disable_collisions'):
                a,b=pair.get('link1'),pair.get('link2')
                if a in robot.tree.links and b in robot.tree.links:ET.SubElement(contact,'exclude',body1=prefix+('urdf_'+a if a==robot.tree.root else a),body2=prefix+('urdf_'+b if b==robot.tree.root else b))
    # Replace RBY actuators with explicit inner position servos, preserving wheel motors.
    cfg=d['control']
    for motor in list(act):
        if not motor.get('name').startswith('rby_'):continue
        name=motor.get('joint')[4:]
        if name in ['right_wheel','left_wheel']:continue
        attrs=dict(motor.attrib);act.remove(motor);j=case.rby.joint_names.index(name);attrs.update(kp=str(cfg['pad_kp'] if name.startswith('pad_') else cfg['position_kp']),kv=str(cfg['pad_kd'] if name.startswith('pad_') else cfg['position_kd']),ctrlrange=_text([case.rby.lower[j],case.rby.upper[j]]),forcerange=_text([-case.rby.effort[j],case.rby.effort[j]]));ET.SubElement(act,'position',**attrs)
    pose=object_reference(case,0)[0];body=ET.SubElement(world,'body',name='panel',pos=_text(pose[:3]),quat=_text([pose[6],*pose[3:6]]));ET.SubElement(body,'freejoint',name='panel_free');l=case.panel.tree.links['panel'];I=l.inertia;ET.SubElement(body,'inertial',mass=str(l.mass),pos=_text(l.com),fullinertia=_text([I[0,0],I[1,1],I[2,2],I[0,1],I[0,2],I[1,2]]));size=np.array(ET.parse(case.panel.path).find('link/collision/geometry/box').get('size').split(),float)/2;ET.SubElement(body,'geom',name='panel_collision_0',type='box',size=_text(size),mass='0',contype='1',conaffinity='3',solref='.008 1')
    fixtures=[]
    for i,g in enumerate(ET.parse(case.path.parent/'jig.urdf').findall('link/collision')):
        R,p=origin(g);size=np.array(g.find('geometry/box').get('size').split(),float);ET.SubElement(world,'geom',name='jig_'+str(i),type='box',pos=_text(p),size=_text(size/2),contype='1',conaffinity='3');fixtures.append(dict(name='jig_'+str(i),position=p.tolist(),size=size.tolist(),quaternion_xyzw=[0,0,0,1]))
    ET.SubElement(world,'geom',name='floor',type='plane',size='4 3 .1',contype='1',conaffinity='3',friction='1 0 0',solref='.008 1');ET.indent(root);Path(destination).write_bytes(ET.tostring(root,encoding='utf-8',xml_declaration=True));return fixtures


def allowed_pair(pair):
    if 'panel_collision_0' in pair:
        return any(any(n.startswith(prefix) for prefix in ['rby_pad_','rby_contact_left_collision_','panda_tool_collision_','jig_']) for n in pair)
    if 'floor' in pair:
        return any(n.startswith(('rby_base_collision_','rby_wheel_r_collision_','rby_wheel_l_collision_','panda_panda_link0_collision_')) for n in pair)
    return False


def planned_collisions(plan,scene):
    mj,m,data=scene.mj,scene.model,scene.data;controller=MobileController(plan,scene);bad={}
    for t in plan.times:
        data.qpos[scene.qadr]=controller.reference(float(t))[0]
        b,_=base_reference(plan.case,t)
        for body,pose in zip(scene.objects,[object_reference(plan.case,t)[0],np.r_[b[:2],0.,0.,0.,0.,1.]]):
            adr=m.jnt_qposadr[m.body_jntadr[body]];data.qpos[adr:adr+7]=np.r_[pose[:3],pose[6],pose[3:6]]
        mj.mj_forward(m,data)
        for c in data.contact:
            pair=[mj.mj_id2name(m,mj.mjtObj.mjOBJ_GEOM,int(g)) for g in c.geom]
            if c.dist<-.002 and not allowed_pair(pair):bad[' / '.join(pair)]=min(bad.get(' / '.join(pair),0),float(c.dist))
    return bad


class MobileController:
    def __init__(self,plan,scene):
        self.plan=plan;self.case=plan.case;self.scene=scene;self.spline=CubicHermiteSpline(plan.times,plan.q,plan.v);self.t=0.;self.n=self.case.rby.n
        self.track,self.radius=map(float,ET.parse(self.case.rby.path).find('mobile').get('params').split());self.base_body=int(scene.objects[1]);self.base_j=scene.model.body_jntadr[self.base_body];self.base_qadr=scene.model.jnt_qposadr[self.base_j];self.base_vadr=scene.model.jnt_dofadr[self.base_j]
    def reference(self,t):
        self.t=t;q,v,a=[self.spline(t,j) for j in [0,1,2]];cfg=self.case.data['control'];opened=smooth((t-self.case.data['release_time'])/self.case.data['release_seconds'])
        for name in ['pad_negative_joint','pad_positive_joint']:q[self.case.rby.joint_names.index(name)]=cfg['pad_closed']*smooth(t/.6)+(cfg['pad_open']-cfg['pad_closed'])*opened
        return q,v,a
    def command(self,q,v,desired,velocity,acceleration):
        case=self.case;d=case.data;cfg=d['control'];mjdata=self.scene.data;n=self.n;k=int(np.clip(np.searchsorted(self.plan.times,self.t)-1,0,len(self.plan.times)-1));quat=mjdata.xquat[self.base_body][[1,2,3,0]];R=Rotation.from_quat(quat).as_matrix();pos=mjdata.xpos[self.base_body];bvel=mjdata.qvel[self.base_vadr:self.base_vadr+6]
        qf=np.r_[pos,quat,q[:n]];vf=np.r_[bvel[3:],R.T@bvel[:3],v[:n]];af=np.r_[np.zeros(3),R.T@base_acceleration(case,self.t)-np.cross(bvel[3:],R.T@bvel[:3]),acceleration[:n]];tau=case.floating.inverse_dynamics(qf,vf,af)
        for i,frame in enumerate(['tip_right','tip_left']):
            T=case.floating.fk(qf,frame);w=self.plan.reactions[k,i];tau-=case.floating.jacobian(qf,frame).T@np.r_[T[:3,:3].T@w[:3],w[3:]]
        ff=tau[6:];cmd=desired[:n]+(ff+cfg['position_kd']*velocity[:n])/cfg['position_kp'];cmd=np.clip(cmd,case.rby.lower,case.rby.upper)
        for name in ['pad_negative_joint','pad_positive_joint']:cmd[case.rby.joint_names.index(name)]=desired[case.rby.joint_names.index(name)]
        b,bd=base_reference(case,self.t);yaw=Rotation.from_quat(quat).as_euler('xyz')[2];forward=np.clip(bd[0]+cfg['base_kp']*(b[0]-pos[0]),-cfg['base_speed_max'],cfg['base_speed_max']);omega=np.clip(-cfg['heading_kp']*yaw-1.5*pos[1],-cfg['yaw_speed_max'],cfg['yaw_speed_max']);track,radius=self.track,self.radius
        for name,wanted in [('right_wheel',-(forward+omega*track/2)/radius),('left_wheel',-(forward-omega*track/2)/radius)]:
            j=case.rby.joint_names.index(name);cmd[j]=np.clip(cfg['wheel_kv']*(wanted-v[j]),-cfg['wheel_effort'],cfg['wheel_effort'])
        rp=case.panda;qp,vp=q[n:],v[n:];feedback=900*(desired[n:]-qp)+60*(velocity[n:]-vp);tp=rp.inverse_dynamics(qp,vp,acceleration[n:]+feedback);T=rp.fk(qp,'tip');J=rp.jacobian(qp,'tip');w=self.plan.reactions[k,2];tp-=J.T@np.r_[T[:3,:3].T@w[:3],w[3:]]
        Td=rp.fk(desired[n:],'tip');Jd=rp.jacobian(desired[n:],'tip');tw=J@vp;twd=Jd@velocity[n:];moment=40*Rotation.from_matrix(T[:3,:3].T@Td[:3,:3]).as_rotvec()+2*(T[:3,:3].T@Td[:3,:3]@twd[:3]-tw[:3]);force=3000*(Td[:3,3]-T[:3,3])+40*(twd[3:]-tw[3:]);tp+=J.T@np.r_[moment,force]
        return np.r_[cmd,np.clip(tp,-rp.effort,rp.effort)]


def run_mobile(output='outputs/mobile_panel',*,case_path=None,plan=None,progress=None,disturbances=()):
    output=Path(output);output.mkdir(parents=True,exist_ok=True)
    if plan is None:
        if progress:progress('optimization')
        plan=plan_mobile(MobileCase.load(case_path))
    case=plan.case
    np.savez_compressed(output/'plan.npz',time=plan.times,q=plan.q,v=plan.v,a=plan.a,reactions=plan.reactions,torque=plan.torque)
    (output/'planning.json').write_text(json.dumps(dict(source=plan.source,workspace=plan.workspace,residual=plan.residual),indent=2)+'\n')
    asset_names={case.data['rby_urdf'],case.data['panda_urdf'],case.data['source_file'],'rby1.urdf','panel.urdf','jig.urdf','panda.srdf','provenance.json','collision_assets.json','franka_panda_provenance.json','LICENSE.authored','LICENSE.panda','LICENSE.rby1','NOTICE.rby1'}
    for name in asset_names:
        path=case.path.parent/name
        if path.is_file() and path.resolve()!=(output/name).resolve():shutil.copyfile(path,output/name)
    (output/'case.json').write_text(json.dumps(case.data,indent=2)+'\n')
    fixtures=build_scene(case,output/'scene.xml');names=['rby_'+n for n in case.rby.joint_names]+['panda_'+n for n in case.panda.joint_names];motors=['rby_motor_'+n for n in case.rby.joint_names]+['panda_motor_'+n for n in case.panda.joint_names];modes=['torque' if 'wheel' in n else 'position' for n in case.rby.joint_names]+['torque']*case.panda.n
    scene=MujocoContactScene(output/'scene.xml',names,motors,['panel','rby_urdf_base'],actuator_modes=modes);controller=MobileController(plan,scene)
    bad=planned_collisions(plan,scene)
    if bad:raise RuntimeError('Planned mesh penetration: '+json.dumps(bad))
    if progress:progress('simulation')
    from .perturbations import WrenchSchedule
    schedule=WrenchSchedule(disturbances)
    trace=scene.run(controller,controller.reference,case.data['duration'],external_forces=schedule,case_id=hashlib.sha256(json.dumps(case.data,sort_keys=True).encode()).hexdigest(),plan_id=hashlib.sha256(plan.q.tobytes()+plan.reactions.tobytes()).hexdigest(),initial_q=controller.reference(0)[0],progress=(lambda t,d:progress('simulation',time_s=t,duration_s=d)) if progress else None);trace.metadata.update(external_disturbances=schedule.to_list(),scenario='mobile_panel',fixtures=fixtures,mobile_base_pose_prescribed=False,object_pose_prescribed=False)
    trace.save(output/'execution.npz');np.savez_compressed(output/'plan.npz',time=plan.times,q=plan.q,v=plan.v,a=plan.a,reactions=plan.reactions,torque=plan.torque)
    report,flags=mobile_report(plan,trace);(output/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n');(output/'case.json').write_text(json.dumps(case.data,indent=2)+'\n');np.savez_compressed(output/'mobile_metrics.npz',contact_flags=flags)
    return plan,trace,report,flags


def mobile_report(plan,trace):
    d=plan.case.data;N=len(trace.time);dt=trace.metadata['timestep'];flags=np.zeros((N,3),bool);pad_flags=np.zeros((N,2),bool);wheel_flags=np.zeros((N,2),bool);jig=np.zeros(N);floor=np.zeros(N);world=np.zeros((N,3,6));bad={};poses=trace.object_pose[:,0];base=trace.object_pose[:,1]
    Robj=Rotation.from_quat(poses[:,3:]).as_matrix();gids=['edge_grip','under_right','under_left'];points=[plan.source['target_positions_m'][plan.source['source_group_ids'].index(g)] for g in gids];normals=[plan.source['target_normals'][plan.source['source_group_ids'].index(g)] for g in gids]
    refs=[poses[:,:3]+np.einsum('nij,j->ni',Robj,p) for p in points]
    classifications={}
    for t,pair,pos,gap,w in zip(trace.contact_time,trace.contact_pair,trace.contact_position,trace.contact_gap,trace.contact_wrench):
        k=int(round(t/dt));key=(pair[0],pair[1]);info=classifications.get(key)
        if info is None:
            panel='panel_collision_0' in pair;tool=-1;pad=-1;wheel=-1
            if panel:
                for i,needle in enumerate(['rby_pad_','rby_contact_left_collision_','panda_tool_collision_']):
                    if any(n.startswith(needle) for n in pair):tool=i
                if any(n.startswith('rby_pad_negative') for n in pair):pad=0
                if any(n.startswith('rby_pad_positive') for n in pair):pad=1
            ground='floor' in pair and any(n.startswith('rby_') for n in pair)
            if ground:
                for j,side in enumerate(['wheel_r','wheel_l']):
                    if any(n.startswith('rby_'+side+'_collision_') for n in pair):wheel=j
            info=(tool,pad,wheel,ground,panel and any(n.startswith('jig_') for n in pair),allowed_pair(pair));classifications[key]=info
        tool,pad,wheel,ground,is_jig,allowed=info
        if tool>=0:
            flags[k,tool]=True;wr=w*(1 if pair[0]=='panel_collision_0' else -1);world[k,tool]+=np.r_[wr[:3]+np.cross(pos-refs[tool][k],wr[3:]),wr[3:]]
            if pad>=0:pad_flags[k,pad]=True
        if is_jig:jig[k]+=w[5]*(1 if pair[1]=='panel_collision_0' else -1)
        if ground:floor[k]+=abs(w[5])
        if wheel>=0:wheel_flags[k,wheel]=True
        if gap<-.002 and not allowed:bad[' / '.join(pair)]=min(bad.get(' / '.join(pair),0),float(gap))
    flags[:,0]=pad_flags.all(axis=1)
    errors=[]
    for key in d['keyframes']:
        k=int(np.argmin(abs(trace.time-key['time'])));p=object_reference(plan.case,key['time'])[0];errors.append(dict(id=key['id'],time=key['time'],translation_m=float(np.linalg.norm(poses[k,:3]-p[:3])),rotation_rad=float((Rotation.from_quat(p[3:]).inv()*Rotation.from_quat(poses[k,3:])).magnitude())))
    final=trace.time>=d['duration']-.5;carry=(trace.time>=d['base_start'])&(trace.time<d['seat_time']);tilt=Rotation.from_quat(base[:,3:]).as_euler('xyz')[:,:2]
    tracking=all(e['translation_m']<.015 and e['rotation_rad']<.08 for e in errors);released=not flags[final].any();supported=float(np.mean(jig[final]));limits=np.r_[plan.case.rby.effort,plan.case.panda.effort]
    force_checks=[]
    for i in range(3):
        Rc=Robj@contact_basis(normals[i]);active=(trace.time>=2)&(trace.time<(d['seat_time'] if i<2 else d['fixed_release']))
        local=np.c_[np.einsum('nji,nj->ni',Rc,world[:,i,:3]),np.einsum('nji,nj->ni',Rc,world[:,i,3:])][active];f=local[:,3:]
        if i==0:
            violation=max(0.,float(np.max(abs(f)-d['optimization']['gripper_force_bounds'])));moment=max(0.,float(np.max(abs(local[:,:3])-d['optimization']['gripper_moment_bounds'])))
        else:violation=max(0.,float(np.max(-f[:,2])),float(np.max(f[:,2]-40)),float(np.max(np.linalg.norm(f[:,:2],axis=1)-d['optimization']['point_mu']*f[:,2])));moment=0.
        force_checks.append(dict(tool=['rby_gripper','rby_support','panda_support'][i],maximum_force_violation_N=violation,maximum_moment_violation_Nm=moment,contact_fraction=float(flags[active,i].mean())))
    wheel_indices=[plan.case.rby.joint_names.index(n) for n in ['right_wheel','left_wheel']];track,radius=map(float,ET.parse(plan.case.rby.path).find('mobile').get('params').split());odometry=-radius*np.mean(trace.q[:,wheel_indices]-trace.q[0,wheel_indices],axis=1)
    rotational=[i for i,n in enumerate(plan.case.rby.joint_names) if plan.case.rby.tree.by_name[n].kind!='prismatic']+list(range(plan.case.rby.n,len(limits)))
    report=dict(scenario='mobile_panel',duration_seconds=float(trace.time[-1]),keyframes=len(errors),keyframe_errors=errors,source=plan.source,workspace_comparison=workspace_comparison(plan.case,plan.q,plan.source),base_travel_m=float(base[-1,0]-base[0,0]),base_final_position=base[-1,:3].tolist(),maximum_base_roll_pitch_rad=float(np.max(abs(tilt))),wheel_ground_contact_fraction=wheel_flags[carry].mean(axis=0).tolist(),wheel_odometry_final_m=float(odometry[-1]),wheel_odometry_error_m=float(abs(odometry[-1]-(base[-1,0]-base[0,0]))),force_checks=force_checks,force_envelopes_passed=all(c['maximum_force_violation_N']<.1 and c['maximum_moment_violation_Nm']<.01 for c in force_checks),tracking_passed=tracking,fixture_supported=supported>18.5,jig_support_final_N=supported,all_tools_released=released,carrying_contact_fraction=flags[carry,:2].mean(axis=0).tolist(),unexpected_penetrations_m=bad,minimum_torque_margin_Nm=float(np.min(limits[rotational]-abs(trace.torque[:,rotational]))),actuator_limits_passed=bool(np.all(abs(trace.torque)<=limits+1e-6)),control_settings=d['control'],validation_tolerances=dict(translation_m=.015,rotation_rad=.08,force_N=.1,moment_Nm=.01,unexpected_penetration_m=.002),planner_scope='supplied straight base/SE3 path; coupled nominal arm/object inertial wrench allocation',ground_reactions_optimized=False,primal_residual=plan.residual,full_geometry_certified=False,mobile_base_pose_prescribed=False,object_pose_prescribed=False,equality_constraints_used=0,network_inference_executed=False,hardware_transmission_executed=False,limitations=['supplied initial grasp, straight corridor, SE3 path and handover schedule','synthetic source and grasp capability; assumed inner servos and 40 Nm wheel bound','official chassis support meshes skid at low friction; passive caster rolling not modeled','convex hulls of official arm OBJ and chassis pieces; native hand visual geometry has no collision model; no continuous geometry certificate','mechanical parent/grandparent self-collision exclusions'])
    report['execution_passed']=bool(tracking and released and supported>18.5 and np.all(flags[carry,:2].mean(axis=0)>.95) and not bad and report['maximum_base_roll_pitch_rad']<.08 and report['base_travel_m']>.9*d['base_distance'] and report['force_envelopes_passed'] and report['actuator_limits_passed'] and report['minimum_torque_margin_Nm']>=-1e-6 and np.min(wheel_flags[carry].mean(axis=0))>.95)
    return report,flags
