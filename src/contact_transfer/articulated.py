"""Three passive articulated-object contact prototypes; no object actuator/pose replay."""
from dataclasses import dataclass
from importlib.resources import files
import hashlib,json,shutil
from pathlib import Path
import xml.etree.ElementTree as E
import casadi as ca
import numpy as np
from scipy.interpolate import CubicHermiteSpline
from scipy.spatial.transform import Rotation
from .robot import Robot
from .contracted import numerical_pose_ik
from .optimization import equality,solve
from .contact_simulation import MujocoContactScene
from .table_assembly import smooth
from .simulation import _text,_quat
from .urdf_scene import append_urdf_robot
from .urdf import origin

@dataclass
class ArticulatedCase:
    path: Path
    data: dict
    robot: Robot
    full: Robot
    object: Robot
    @classmethod
    def load(cls,id='hinged_door',path=None):
        if id not in {'hinged_door','folding_panel','drawer'}:raise ValueError('Unknown articulated preset')
        path=Path(path or str(files('contact_transfer').joinpath('scenarios/articulated/'+id+'.json'))).resolve();d=json.loads(path.read_text())
        if d['id']!=id:raise ValueError('Articulated preset ID mismatch')
        T=np.eye(4);T[:3,3]=d['robot_base'];T[:3,:3]=Rotation.from_euler('z',d['robot_yaw']).as_matrix();locked={f'panda_finger_joint{i}':.02 for i in [1,2]} if d['tool']=='G' else {}
        def asset(n):
            p=(path.parent/n).resolve()
            if not p.is_relative_to(path.parent):raise ValueError('Assets must be local')
            return p
        return cls(path,d,Robot(asset(d['robot_urdf']),base_pose=T,locked_joints=locked),Robot(asset(d['robot_urdf']),base_pose=T),Robot(asset(d['object_urdf'])))


def coordinate_reference(case,t):
    d=case.data;h=d['motion_end']-d['motion_start'];s=np.clip((t-d['motion_start'])/h,0,1);delta=d['target_coordinate']-d['initial_coordinate'];return np.array([d['initial_coordinate']+delta*smooth(s)]),np.array([delta*(30*s*s-60*s**3+30*s**4)/h]),np.array([delta*(60*s-180*s*s+120*s**3)/h**2])


def contact_target(case,t):
    q=coordinate_reference(case,t)[0];T=case.object.fk(q,'moving');target=np.eye(4);target[:3,3]=T[:3,3]+T[:3,:3]@(np.asarray(case.data['contact_object'])+case.data.get('contact_preload_m',0.)*np.asarray(case.data['inward_object']));target[:3,:3]=T[:3,:3]@Rotation.from_quat(case.data['tool_rotation_object_xyzw']).as_matrix();return target


def point_jacobian(robot,q,frame,p):
    T=robot.fk(q,frame);J=robot.jacobian(q,frame,'world_aligned');r=T[:3,:3]@p;skew=np.array([[0,-r[2],r[1]],[r[2],0,-r[0]],[-r[1],r[0],0]]);return J[3:]-skew@J[:3]

@dataclass
class ArticulatedPlan:
    case: ArticulatedCase
    times: np.ndarray
    q: np.ndarray
    v: np.ndarray
    a: np.ndarray
    forces: np.ndarray
    torque: np.ndarray
    residual: float


def plan_articulated(case):
    d=case.data;times=np.linspace(0,d['duration'],round(d['duration']/d['sample_dt'])+1);Q=[];seed=np.array(d['q_seed'])
    for t in times:
        try:seed=numerical_pose_ik(case.robot,'tip',contact_target(case,t),seed)
        except RuntimeError as e:raise RuntimeError(f'{d["id"]} IK at {t:.2f}s: {e}') from e
        Q.append(seed.copy())
    Q=np.array(Q);V=np.gradient(Q,times,axis=0,edge_order=2);A=np.gradient(V,times,axis=0,edge_order=2)
    if np.any(abs(V)>case.robot.velocity):raise RuntimeError('Articulated path exceeds robot joint velocity')
    opt=ca.Opti();forces=[];torques=[];cost=0.
    for k,t in enumerate(times):
        qo,vo,ao=coordinate_reference(case,t);T=case.object.fk(qo,'moving');direction=T[:3,:3]@d['inward_object'];coefficient=float(point_jacobian(case.object,qo,'moving',d['contact_object'])[:,0]@direction)
        required=float(case.object.inverse_dynamics(qo,vo,ao)[0]+d['damping']*vo[0]+d['stiffness']*qo[0]+d['frictionloss']*np.tanh(vo[0]/.001))
        F=opt.variable();opt.subject_to(opt.bounded(0,F,d['normal_max_N']));equality(opt,coefficient*F-required);opt.set_initial(F,max(0,required/coefficient));w=-ca.DM(direction)*F;forces.append(w)
        tau=ca.DM(case.robot.inverse_dynamics(Q[k],V[k],A[k]))-ca.DM(case.robot.jacobian(Q[k],'tip')[3:].T)@w;opt.subject_to(opt.bounded(-case.robot.effort,tau,case.robot.effort));torques.append(tau);cost+=ca.sumsqr(F)+1e-4*ca.sumsqr(tau)
    opt.minimize(cost);sol,res=solve(opt);return ArticulatedPlan(case,times,Q,V,A,np.array([sol.value(w) for w in forces]),np.array([sol.value(x) for x in torques]),res)


def build_articulated_scene(case,path,*,contacts_enabled=True):
    d=case.data;root=E.Element('mujoco',model=d['id']);E.SubElement(root,'compiler',angle='radian',inertiafromgeom='false',fusestatic='false');E.SubElement(root,'option',timestep=str(d['timestep']),integrator='implicitfast',cone='elliptic',impratio='100',iterations='80',tolerance='1e-10');world=E.SubElement(root,'worldbody');act=E.SubElement(root,'actuator');assets=E.SubElement(root,'asset');contact=E.SubElement(root,'contact')
    meshes={p.name:p.resolve() for p in Path('.cache/panda_meshes').glob('*.stl')};geoms=append_urdf_robot(world,act,case.full,'robot_',mesh_files=meshes,assets=assets,robot_collisions=True);object_geoms=append_urdf_robot(world,act,case.object,'object_',robot_collisions=True)
    for motor in list(act):
        if motor.get('name').startswith('object_'):act.remove(motor)
    joint=next(j for j in world.findall('.//joint') if j.get('name')=='object_object_joint');joint.set('damping',str(d['damping']));joint.set('frictionloss',str(d['frictionloss']));joint.set('stiffness',str(d['stiffness']));joint.set('springref','0')
    for pair in E.parse(case.path.parent/'panda.srdf').findall('disable_collisions'):
        a,b=pair.get('link1'),pair.get('link2')
        if a in case.full.tree.links and b in case.full.tree.links:E.SubElement(contact,'exclude',body1='robot_'+('urdf_'+a if a==case.full.tree.root else a),body2='robot_'+('urdf_'+b if b==case.full.tree.root else b))
    for j in case.object.tree.joints:E.SubElement(contact,'exclude',body1='object_'+('urdf_'+j.parent if j.parent==case.object.tree.root else j.parent),body2='object_'+j.child)
    if contacts_enabled:
        for name in geoms:
            pad='leftfinger_collision_' in name or 'rightfinger_collision_' in name;point=name.startswith('robot_tool_collision_')
            if not (pad or point):continue
            for other in object_geoms:
                if not other.startswith('object_moving_collision_'):continue
                E.SubElement(contact,'pair',geom1=name,geom2=other,condim='4' if pad else '3',friction=_text([.9,.9,.008 if pad else 0,0,0]),solref='.008 1',solimp='.95 .99 .001')
    else:
        # Controlled ablation: disable only robot-object collision pairs.
        for geom in world.findall('.//geom'):
            if geom.get('name','').startswith('object_'):geom.set('contype','4');geom.set('conaffinity','0')
    E.SubElement(world,'geom',name='floor',type='plane',size='3 3 .1',contype='1',conaffinity='3');E.indent(root);Path(path).write_bytes(E.tostring(root,encoding='utf-8',xml_declaration=True))


class ArticulatedController:
    def __init__(self,plan):self.plan=plan;self.spline=CubicHermiteSpline(plan.times,plan.q,plan.v);self.t=0.
    def reference(self,t):
        self.t=t;q,v,a=[self.spline(t,j) for j in [0,1,2]]
        if self.plan.case.data['tool']=='G':q=np.r_[q,.02+(.017-.02)*smooth(t/.6),.02+(.017-.02)*smooth(t/.6)];v=np.r_[v,0,0];a=np.r_[a,0,0]
        return q,v,a
    def command(self,q,v,qr,vr,ar):
        r=self.plan.case.full;k=int(np.clip(np.searchsorted(self.plan.times,self.t)-1,0,len(self.plan.times)-1));tau=r.inverse_dynamics(q,v,ar+900*(qr-q)+60*(vr-v));T=r.fk(q,'tip');J=r.jacobian(q,'tip');w=self.plan.forces[k];tau-=J[3:].T@w
        Td=r.fk(qr,'tip');Jd=r.jacobian(qr,'tip');tw=J@v;td=Jd@vr;moment=40*Rotation.from_matrix(T[:3,:3].T@Td[:3,:3]).as_rotvec()+2*(T[:3,:3].T@Td[:3,:3]@td[:3]-tw[:3]);force=3000*(Td[:3,3]-T[:3,3])+40*(td[3:]-tw[3:]);tau+=J.T@np.r_[moment,force]
        if r.n>7:tau[7:]=r.gravity(q)[7:]+3000*(qr[7:]-q[7:])-10*v[7:]
        return np.clip(tau,-r.effort,r.effort)


def permitted_pair(pair):
    tool=any(n.startswith(('robot_tool_collision_','robot_panda_leftfinger_collision_','robot_panda_rightfinger_collision_')) for n in pair)
    if tool and any(n.startswith('object_moving_collision_') for n in pair):return True
    if any(n.startswith('robot_panda_link0_collision_') for n in pair) and ('floor' in pair or any(n.startswith('object_fixture_collision_') for n in pair)):return True
    return 'floor' in pair and all(n=='floor' or n.startswith('object_fixture_collision_') for n in pair)


def sampled_collisions(plan,model,data):
    import mujoco as mj
    case=plan.case;names=['robot_'+n for n in case.full.joint_names];ids=[mj.mj_name2id(model,mj.mjtObj.mjOBJ_JOINT,n) for n in names];j=mj.mj_name2id(model,mj.mjtObj.mjOBJ_JOINT,'object_object_joint');controller=ArticulatedController(plan);bad={}
    for t in plan.times:
        data.qpos[model.jnt_qposadr[ids]]=controller.reference(t)[0];data.qpos[model.jnt_qposadr[j]]=coordinate_reference(case,t)[0][0];mj.mj_forward(model,data)
        for c in data.contact:
            pair=[mj.mj_id2name(model,mj.mjtObj.mjOBJ_GEOM,int(g)) for g in c.geom]
            if c.dist<-.001 and not permitted_pair(pair):bad[' / '.join(pair)]=min(bad.get(' / '.join(pair),0),float(c.dist))
    return bad


def run_articulated(id='hinged_door',output=None,*,case_path=None,plan=None,contacts_enabled=True,progress=None):
    root=Path(output or 'outputs/'+id);root.mkdir(parents=True,exist_ok=True);plan=plan or plan_articulated(ArticulatedCase.load(id,case_path));case=plan.case;d=case.data;build_articulated_scene(case,root/'scene.xml',contacts_enabled=contacts_enabled)
    names=['robot_'+n for n in case.full.joint_names];motors=['robot_motor_'+n for n in case.full.joint_names];scene=MujocoContactScene(root/'scene.xml',names,motors,['object_moving'],articulated_object_joints={'object_moving':'object_object_joint'});controller=ArticulatedController(plan)
    bad=sampled_collisions(plan,scene.model,scene.data)
    if bad:raise RuntimeError('Planned geometry penetration: '+json.dumps(bad))
    trace=scene.run(controller,controller.reference,d['duration'],initial_q=controller.reference(0)[0],initial_articulated_q={'object_object_joint':d['initial_coordinate']},case_id=hashlib.sha256(json.dumps(d,sort_keys=True).encode()).hexdigest(),plan_id=hashlib.sha256(plan.q.tobytes()+plan.forces.tobytes()).hexdigest(),progress=progress)
    theta=np.array(trace.metadata['articulated_state']['q'])[:,0];vel=np.array(trace.metadata['articulated_state']['v'])[:,0];reference=np.array([coordinate_reference(case,t)[0][0] for t in trace.time]);contact_flags=np.zeros(len(trace.time),bool);wrenches=np.zeros((len(trace.time),6));bad={}
    for t,pair,pos,gap,w in zip(trace.contact_time,trace.contact_pair,trace.contact_position,trace.contact_gap,trace.contact_wrench):
        k=int(round(t/d['timestep']));tool=next((j for j,n in enumerate(pair) if n.startswith(('robot_tool_collision_','robot_panda_leftfinger_collision_','robot_panda_rightfinger_collision_'))),None);moving=any(n.startswith('object_moving_collision_') for n in pair)
        if moving and tool is not None:contact_flags[k]=True;force=w*(1 if tool==1 else -1);contact_pos=case.object.fk([theta[k]],'moving');anchor=contact_pos[:3,3]+contact_pos[:3,:3]@d['contact_object'];wrenches[k]+=np.r_[force[:3]+np.cross(pos-anchor,force[3:]),force[3:]]
        elif gap<-.001 and not permitted_pair(pair):bad[' / '.join(pair)]=min(bad.get(' / '.join(pair),0),float(gap))
    settled=trace.time>=d['duration']-1;active=(trace.time>=d['motion_start']+d['duration']/14)&(trace.time<=d['motion_end']);tolerance=.012 if id=='drawer' else np.deg2rad(2);final_error=float(np.max(abs(theta[settled]-d['target_coordinate'])));tracking=float(np.max(abs(theta-reference)));margin=float(np.min(case.full.effort-abs(trace.torque)));kind=case.object.tree.by_name['object_joint'].kind
    report=dict(scenario=id,articulated_object=True,passive_joint_type=kind,object_joint_actuators=0,object_pose_prescribed=False,initial_grasp_or_contact_supplied=True,contacts_enabled=contacts_enabled,initial_coordinate=d['initial_coordinate'],target_coordinate=d['target_coordinate'],final_coordinate=float(theta[-1]),coordinate_units='m' if id=='drawer' else 'rad',final_error=final_error,maximum_tracking_error=tracking,contact_fraction=float(contact_flags[active].mean()),peak_contact_force_N=float(np.max(np.linalg.norm(wrenches[:,3:],axis=1))),minimum_actuator_margin=margin,unexpected_penetrations_m=bad,primal_residual=plan.residual,duration_seconds=d['duration'],engine_version=trace.metadata['engine_version'],full_geometry_certified=False,network_inference_executed=False,source_contact_refinement_executed=False,planner_scope='supplied link-local contact and joint path; generalized object torque/contact-force allocation',limitations=['authored hinge/rail mass, friction and spring; no hardware calibration','provided contact and initial grasp; no learned articulated contraction','one passive joint per fixture; no general multi-joint articulated planning','soft contact and convex robot meshes; no continuous geometry certificate'])
    report['execution_passed']=bool(final_error<tolerance and report['contact_fraction']>.95 and margin>=-1e-6 and not bad)
    trace.save(root/'execution.npz');np.savez_compressed(root/'plan.npz',time=plan.times,q=plan.q,v=plan.v,a=plan.a,forces=plan.forces,torque=plan.torque);np.savez_compressed(root/'metrics.npz',coordinate=theta,velocity=vel,reference=reference,contact_flags=contact_flags,wrenches=wrenches);(root/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    for name in {d['robot_urdf'],d['object_urdf'],'panda.srdf','LICENSE.panda','LICENSE.authored','franka_panda_provenance.json'}:
        src=case.path.parent/name
        if src.resolve()!=(root/name).resolve():shutil.copyfile(src,root/name)
    (root/(id+'.json')).write_text(json.dumps(d,indent=2)+'\n');return plan,trace,report,contact_flags
