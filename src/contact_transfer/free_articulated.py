"""Three-arm realization of link-aware contracted contacts on a FREE articulated object."""
from dataclasses import dataclass
from pathlib import Path
from importlib.resources import files
import hashlib,json,shutil
import xml.etree.ElementTree as E
import numpy as np
import casadi as ca
from scipy.spatial.transform import Rotation
from scipy.interpolate import CubicHermiteSpline
from .robot import Robot
from .functional_contraction import contract_dense_contacts
from .contracted import numerical_pose_ik
from .optimization import equality,solve
from .table_assembly import smooth
from .contact_simulation import MujocoContactScene
from .urdf_scene import append_urdf_robot
from .simulation import _text,_quat
from .perturbations import WrenchSchedule
IDS={'free_box_lid','free_folding_object','free_portable_drawer'}

def blend(t,start,end,delta):
 h=end-start;s=np.clip((t-start)/h,0,1);return delta*smooth(s),delta*(30*s*s-60*s**3+30*s**4)/h,delta*(60*s-180*s*s+120*s**3)/h**2

def object_reference(case,t):
 d=case.data;z,zd,zdd=blend(t,d['lift_start'],d['lift_end'],d['base_lift_m']);pitch,pd,pdd=blend(t,d['lift_start'],d['lift_end'],d['base_pitch_rad']);theta,td,tdd=blend(t,d['motion_start'],d['motion_end'],d['target_coordinate']-d['initial_coordinate']);theta+=d['initial_coordinate'];R=Rotation.from_euler('y',pitch);w=np.array([0,pd,0]);v=R.inv().apply([0,0,zd]);return np.r_[np.array(d['base_initial'][:3])+[0,0,z],R.as_quat(),theta],np.r_[w,v,td],np.r_[[0,pdd,0],R.inv().apply([0,0,zdd])-np.cross(w,v),tdd]

def skew(r):return np.array([[0,-r[2],r[1]],[r[2],0,-r[0]],[-r[1],r[0],0]])
def contact_jacobian(obj,q,link,p):
 T=obj.fk(q,link);J=obj.jacobian(q,link,'world_aligned');return np.vstack((J[:3],J[3:]-skew(T[:3,:3]@p)@J[:3]))

@dataclass
class FreeCase:
 path: Path
 data: dict
 robots: tuple
 full: tuple
 object: Robot
 contraction: dict
 @classmethod
 def load(cls,id='free_box_lid',path=None):
  if id not in IDS:raise ValueError('Unknown free articulated task')
  path=Path(path or str(files('contact_transfer').joinpath('scenarios/free_articulated/'+id+'.json'))).resolve();d=json.loads(path.read_text())
  if d['id']!=id:raise ValueError('Case ID mismatch')
  if d.get('units')!='SI' or len(d['arms'])!=3:raise ValueError('Three-arm SI case required')
  contraction=contract_dense_contacts(d);robots=[];full=[]
  def asset(n):
   p=(path.parent/n).resolve()
   if not p.is_relative_to(path.parent):raise ValueError('Assets must be local')
   return p
  for spec in d['arms']:
   g=next(g for g in contraction['groups'] if g['group_id']==spec['group_id']);spec.update(kind=g['kind'],link_id=g['link_id'],point_link=g['target_anchor_m'],inward_link=g['inward_link']);T=np.eye(4);T[:3,3]=spec['robot_base'];T[:3,:3]=Rotation.from_euler('z',spec['robot_yaw']).as_matrix();full.append(Robot(asset(spec['robot_urdf']),base_pose=T));robots.append(Robot(asset(spec['robot_urdf']),base_pose=T,locked_joints={f'panda_finger_joint{i}':.02 for i in [1,2]} if spec['kind']=='G' else {}))
  obj=Robot(asset(d['object_urdf']),floating_base=True)
  for spec in d['arms']:
   if spec['kind'] not in {'P','G'} or spec['link_id'] not in obj.tree.links or not np.isclose(np.linalg.norm(spec['inward_link']),1):raise ValueError('Invalid link-aware group')
   permitted=[];point=np.array(spec['point_link']);normal=np.array(spec['inward_link'])
   for i,g in enumerate(E.parse(obj.path).findall(f"link[@name='{spec['link_id']}']/collision")):
    p0=np.fromstring(g.find('origin').get('xyz'),sep=' ');half=np.fromstring(g.find('geometry/box').get('size'),sep=' ')/2;local=point-p0
    match=np.all(abs(local)<=half+1e-8) if spec['kind']=='G' else (np.all(abs(local)<=half+1e-8) and any(abs(normal[j])>.99 and abs(local[j]+normal[j]*half[j])<1e-8 for j in range(3)))
    if match:permitted.append(f"object_{spec['link_id']}_collision_{i}")
   if not permitted:raise ValueError('Contracted anchor outside its intended handle/patch')
   spec['allowed_object_geoms']=permitted
  return cls(path,d,tuple(robots),tuple(full),obj,contraction)

def targets(case,t):
 qo=object_reference(case,t)[0];out=[]
 for spec in case.data['arms']:
  L=case.object.fk(qo,spec['link_id']);T=np.eye(4);T[:3,:3]=L[:3,:3]@Rotation.from_quat(spec['tool_rotation_link_xyzw']).as_matrix();T[:3,3]=L[:3,3]+L[:3,:3]@(np.asarray(spec['point_link'])+spec['contact_preload_m']*np.asarray(spec['inward_link']));out.append(T)
 return out

@dataclass
class FreePlan:
 case: FreeCase
 times: np.ndarray
 q: np.ndarray
 v: np.ndarray
 a: np.ndarray
 wrenches: np.ndarray
 torque: np.ndarray
 residual: float


def plan_free(case):
 d=case.data;times=np.linspace(0,d['duration'],round(d['duration']/d['sample_dt'])+1);Q=[];seeds=[np.array(s['q_seed']) for s in d['arms']]
 for t in times:
  row=[]
  for i,(r,T) in enumerate(zip(case.robots,targets(case,t))):
   try:seeds[i]=numerical_pose_ik(r,'tip',T,seeds[i])
   except RuntimeError as e:raise RuntimeError(f'arm{i} IK at {t:.2f}s: {e}') from e
   row.extend(seeds[i])
  Q.append(row)
 Q=np.asarray(Q);V=np.gradient(Q,times,axis=0,edge_order=2);A=np.gradient(V,times,axis=0,edge_order=2);off=np.cumsum([0]+[r.n for r in case.robots]);opt=ca.Opti();wrenches=[];torques=[];cost=0
 for k,t in enumerate(times):
  qo,vo,ao=object_reference(case,t);required=case.object.inverse_dynamics(qo,vo,ao);required[-1]+=d['damping']*vo[-1]+d['frictionloss']*np.tanh(vo[-1]/.001);total=ca.DM.zeros(case.object.nv,1);ws=[];taus=[]
  for i,(r,s) in enumerate(zip(case.robots,d['arms'])):
   sl=slice(off[i],off[i+1]);L=case.object.fk(qo,s['link_id']);force=opt.variable(3)
   if s['kind']=='P':
    n=L[:3,:3]@s['inward_link'];fn=ca.dot(n,force);opt.subject_to(opt.bounded(0,fn,40));opt.subject_to(ca.sumsqr(force-ca.DM(n)*fn)<=.8**2*fn**2);moment=ca.DM.zeros(3,1)
   else:
    opt.subject_to(opt.bounded(-40,force,40));moment=opt.variable(3);opt.subject_to(opt.bounded(-.25,moment,.25))
   opt.set_initial(force,[0,0,8]);w=ca.vertcat(moment,force);ws.append(w);total+=ca.DM(contact_jacobian(case.object,qo,s['link_id'],s['point_link']).T)@w;T=r.fk(Q[k,sl],'tip');reaction=ca.vertcat(-ca.DM(T[:3,:3].T)@moment,-force);tau=ca.DM(r.inverse_dynamics(Q[k,sl],V[k,sl],A[k,sl]))-ca.DM(r.jacobian(Q[k,sl],'tip').T)@reaction;opt.subject_to(opt.bounded(-r.effort,tau,r.effort));taus.append(tau);cost+=ca.sumsqr(force)+500*ca.sumsqr(moment)+1e-4*ca.sumsqr(tau)
   if np.any(abs(V[:,sl])>r.velocity):raise RuntimeError('Robot velocity bound')
  equality(opt,total-required);wrenches.append(ca.horzcat(*ws).T);torques.append(ca.vertcat(*taus))
 opt.minimize(cost);sol,res=solve(opt);return FreePlan(case,times,Q,V,A,np.array([sol.value(w) for w in wrenches]),np.array([sol.value(t) for t in torques]),res)

class FreeController:
 def __init__(self,plan):
  self.plan=plan;self.spline=CubicHermiteSpline(plan.times,plan.q,plan.v);self.t=0.;self.qoff=np.cumsum([0]+[r.n for r in plan.case.robots]);self.foff=np.cumsum([0]+[r.n for r in plan.case.full])
 def reference(self,t):
  self.t=t;ref=[[],[],[]]
  for r,s,a,b in zip(self.plan.case.full,self.plan.case.data['arms'],self.qoff[:-1],self.qoff[1:]):
   for k in range(3):
    value=self.spline(t,k)[a:b]
    if s['kind']=='G':value=np.r_[value,[.017]*2 if k==0 else [0,0]]
    ref[k].extend(value)
  return tuple(np.asarray(v) for v in ref)
 def command(self,q,v,qr,vr,ar):
  k=int(np.clip(np.searchsorted(self.plan.times,self.t)-1,0,len(self.plan.times)-1));out=[]
  for i,(r,a,b) in enumerate(zip(self.plan.case.full,self.foff[:-1],self.foff[1:])):
   x,y,xd,yd,ad=[z[a:b] for z in [q,v,qr,vr,ar]];T=r.fk(x,'tip');Td=r.fk(xd,'tip');J=r.jacobian(x,'tip');Jd=r.jacobian(xd,'tip');w=self.plan.wrenches[k,i];reaction=np.r_[-T[:3,:3].T@w[:3],-w[3:]];tw=J@y;td=Jd@yd;moment=40*Rotation.from_matrix(T[:3,:3].T@Td[:3,:3]).as_rotvec()+2*(T[:3,:3].T@Td[:3,:3]@td[:3]-tw[:3]);force=3000*(Td[:3,3]-T[:3,3])+40*(td[3:]-tw[3:]);tau=r.inverse_dynamics(x,y,ad+900*(xd-x)+60*(yd-y))-J.T@reaction+J.T@np.r_[moment,force]
   if r.n>7:tau[7:]=r.gravity(x)[7:]+6000*(xd[7:]-x[7:])-10*y[7:]
   out.extend(np.clip(tau,-r.effort,r.effort))
  return np.asarray(out)

def build_scene(case,path,contacts_enabled=True):
 d=case.data;root=E.Element('mujoco',model=d['id']);E.SubElement(root,'compiler',angle='radian',inertiafromgeom='false',fusestatic='false');E.SubElement(root,'option',timestep=str(d['timestep']),integrator='implicitfast',cone='elliptic',impratio='100',iterations='80',tolerance='1e-10');world=E.SubElement(root,'worldbody');act=E.SubElement(root,'actuator');assets=E.SubElement(root,'asset');contact=E.SubElement(root,'contact');meshes={p.name:p.resolve() for p in Path('.cache/panda_meshes').glob('*.stl')};armgeoms=[]
 for i,r in enumerate(case.full):
  prefix=f'arm{i}_';armgeoms.append(append_urdf_robot(world,act,r,prefix,mesh_files=meshes,assets=assets,robot_collisions=True))
  for pair in E.parse(case.path.parent/'panda.srdf').findall('disable_collisions'):
   a,b=pair.get('link1'),pair.get('link2')
   if a in r.tree.links and b in r.tree.links:E.SubElement(contact,'exclude',body1=prefix+('urdf_'+a if a==r.tree.root else a),body2=prefix+('urdf_'+b if b==r.tree.root else b))
  if r.base_pose[2,3]>0:
   g=E.SubElement(world,'geom',name=f'pedestal{i}',type='box',size=_text([.15,.15,r.base_pose[2,3]/2]),pos=_text([*r.base_pose[:2,3],r.base_pose[2,3]/2]),contype='1',conaffinity='3')
 object_geoms=append_urdf_robot(world,act,case.object,'object_',robot_collisions=True);base=world.find("body[@name='object_urdf_base']");base.set('pos',_text(d['base_initial'][:3]));base.set('quat',_text(np.array(d['base_initial'])[ [6,3,4,5] ]))
 for motor in list(act):
  if motor.get('name').startswith('object_'):act.remove(motor)
 joint=next(j for j in world.findall('.//joint') if j.get('name')=='object_articulation');joint.set('damping',str(d['damping']));joint.set('frictionloss',str(d['frictionloss']));E.SubElement(contact,'exclude',body1='object_urdf_base',body2='object_moving')
 for i,(s,geoms) in enumerate(zip(d['arms'],armgeoms)):
  for name in geoms:
   pad='leftfinger_collision_' in name or 'rightfinger_collision_' in name;point='_tool_collision_' in name
   if not (pad or point):continue
   for other in object_geoms:
    if not other.startswith('object_'+s['link_id']+'_collision_') or not contacts_enabled:continue
    E.SubElement(contact,'pair',geom1=name,geom2=other,condim='4' if pad else '3',friction=_text([.9,.9,.008 if pad else 0,0,0]),solref='.008 1',solimp='.95 .99 .001')
 if not contacts_enabled:
  for g in world.findall('.//geom'):
   if '_collision_' not in g.get('name',''):continue
   if g.get('name','').startswith('arm'):g.set('contype','1');g.set('conaffinity','1')
   if g.get('name','').startswith('object_'):g.set('contype','2');g.set('conaffinity','0')
 E.SubElement(world,'geom',name='floor',type='plane',size='3 3 .1',contype='1',conaffinity='3');E.indent(root);Path(path).write_bytes(E.tostring(root,encoding='utf-8',xml_declaration=True))

def allowed(pair,case):
 for i,s in enumerate(case.data['arms']):
  tool=any(n.startswith(f'arm{i}_') and ('_tool_collision_' in n or '_panda_leftfinger_collision_' in n or '_panda_rightfinger_collision_' in n) for n in pair)
  if tool and any(n in s['allowed_object_geoms'] for n in pair):return True
 if any('_panda_link0_collision_' in n for n in pair) and any(n=='floor' or n.startswith('pedestal') for n in pair):return True
 return False

def planned_collisions(plan,m,data):
 import mujoco as mj
 names=[f'arm{i}_'+n for i,r in enumerate(plan.case.full) for n in r.joint_names];joints=[mj.mj_name2id(m,mj.mjtObj.mjOBJ_JOINT,n) for n in names];base=mj.mj_name2id(m,mj.mjtObj.mjOBJ_JOINT,'object_floating_base');hinge=mj.mj_name2id(m,mj.mjtObj.mjOBJ_JOINT,'object_articulation');qa=m.jnt_qposadr[base];controller=FreeController(plan);bad={}
 for t in plan.times:
  qo=object_reference(plan.case,t)[0];data.qpos[qa:qa+7]=np.r_[qo[:3],qo[6],qo[3:6]];data.qpos[m.jnt_qposadr[hinge]]=qo[-1];data.qpos[m.jnt_qposadr[joints]]=controller.reference(t)[0];mj.mj_forward(m,data)
  for c in data.contact:
   pair=[mj.mj_id2name(m,mj.mjtObj.mjOBJ_GEOM,int(g)) for g in c.geom]
   if c.dist<-.001 and not allowed(pair,plan.case):bad[' / '.join(pair)]=min(bad.get(' / '.join(pair),0),float(c.dist))
 return bad

def run_free(id='free_box_lid',output=None,*,case_path=None,plan=None,contacts_enabled=True,disturbances=(),progress=None,control_mode='nominal'):
 root=Path(output or 'outputs/'+id);root.mkdir(parents=True,exist_ok=True);plan=plan or plan_free(FreeCase.load(id,case_path));case=plan.case;d=case.data;build_scene(case,root/'scene.xml',contacts_enabled);names=[f'arm{i}_'+n for i,r in enumerate(case.full) for n in r.joint_names];motors=[f'arm{i}_motor_'+n for i,r in enumerate(case.full) for n in r.joint_names];scene=MujocoContactScene(root/'scene.xml',names,motors,['object_urdf_base','object_moving'],articulated_object_joints={'object_moving':'object_articulation'});bad=planned_collisions(plan,scene.model,scene.data)
 if bad:raise RuntimeError('Planned geometry penetration '+json.dumps(bad))
 if control_mode not in {'nominal','hybrid_torque'}:raise ValueError('Unknown control mode')
 controller=HybridFreeController(plan) if control_mode=='hybrid_torque' else FreeController(plan);schedule=WrenchSchedule(disturbances);trace=scene.run(controller,controller.reference,d['duration'],initial_q=controller.reference(0)[0],initial_articulated_q={'object_articulation':d['initial_coordinate']},external_forces=schedule,case_id=hashlib.sha256(json.dumps(d,sort_keys=True).encode()).hexdigest(),plan_id=hashlib.sha256(plan.q.tobytes()+plan.wrenches.tobytes()).hexdigest(),progress=progress);trace.metadata['external_disturbances']=schedule.to_list();theta=np.asarray(trace.metadata['articulated_state']['q'])[:,0];N=len(trace.time);flags=np.zeros((N,3),bool);pads=np.zeros((N,3,2),bool);forces=np.zeros((N,3,3));bad={}
 for t,pair,gap,w in zip(trace.contact_time,trace.contact_pair,trace.contact_gap,trace.contact_wrench):
  k=round(t/d['timestep'])
  for i,s in enumerate(d['arms']):
   tool=next((j for j,n in enumerate(pair) if n.startswith(f'arm{i}_') and ('_tool_collision_' in n or '_panda_leftfinger_collision_' in n or '_panda_rightfinger_collision_' in n)),None)
   if tool is not None and any(n in s['allowed_object_geoms'] for n in pair):
    flags[k,i]=True;forces[k,i]+=w[3:]*(1 if tool==1 else -1)
    for z,tag in enumerate(['leftfinger','rightfinger']):pads[k,i,z]|=tag in pair[tool]
  if gap<-.002 and not allowed(pair,case):bad[' / '.join(pair)]=min(bad.get(' / '.join(pair),0),float(gap))
 for i,s in enumerate(d['arms']):
  if s['kind']=='G':flags[:,i]=pads[:,i].all(axis=1)
 desired=np.array([object_reference(case,t)[0] for t in trace.time]);pe=np.linalg.norm(trace.object_pose[:,0,:3]-desired[:,:3],axis=1);re=(Rotation.from_quat(desired[:,3:7]).inv()*Rotation.from_quat(trace.object_pose[:,0,3:])).magnitude();active=(trace.time>=3)&(trace.time<=d['motion_end']);settled=trace.time>=d['duration']-1;limits=np.concatenate([r.effort for r in case.full]);report=dict(scenario=id,free_articulated_object=True,multi_arm=True,arm_count=3,base_joint_type='free',internal_joint_type=case.object.tree.by_name['articulation'].kind,object_joint_actuators=0,object_pose_prescribed=False,world_hinge=False,contacts_enabled=contacts_enabled,coordinate_units='m' if id=='free_portable_drawer' else 'rad',target_coordinate=d['target_coordinate'],final_coordinate=float(theta[-1]),final_error=float(np.max(abs(theta[settled]-d['target_coordinate']))),base_max_position_error_m=float(pe[active].max()),base_final_position_error_m=float(pe[-1]),base_max_rotation_error_rad=float(re[active].max()),contact_fraction=flags[active].mean(axis=0).tolist(),peak_contact_force_per_arm_N=np.max(np.linalg.norm(forces,axis=2),axis=0).tolist(),minimum_actuator_margin=float(np.min(limits-abs(trace.torque))),unexpected_penetrations_m=bad,primal_residual=plan.residual,contraction=case.contraction,network_inference_executed=False,full_geometry_certified=False,external_disturbances=schedule.to_list(),duration_seconds=d['duration'],limitations=['synthetic dense patches; functional group IDs and arm assignment supplied','authored mechanics/contact parameters; no hardware calibration','prescribed reference path; no general articulated CITO','native hinge/rail parent-neighbor collision exclusion and joint stops'])
 report.update(planning_mode='nominal_path_baseline',full_paper_pipeline_executed=False,paper_sections_executed={'III-B':False,'III-C':False,'III-D':False,'IV':False},control_mode='computed_torque_cartesian_PD_wrench_feedforward',measured_force_feedback=False)
 if control_mode=='hybrid_torque':
  error=np.asarray(controller.force_errors);active_force=active[:-1];report.update(control_mode='hybrid_motion_force_nullspace_joint_impedance',measured_force_feedback=True,force_measurement='native contact wrench aggregation, moment shifted to current tool origin, 10 ms lowpass',force_normal_RMSE_N=np.sqrt(np.mean(error[active_force,:,5]**2,axis=0)).tolist(),controller_saturation_fraction=np.mean(controller.saturations,axis=0).tolist())
  np.savez_compressed(root/'control.npz',measured_reactions=controller.measured_history,desired_reactions=controller.commanded_history,force_error_contact=controller.force_errors,saturated=controller.saturations)
 report['execution_passed']=bool(report['final_error']<(.012 if id=='free_portable_drawer' else np.deg2rad(2)) and report['base_max_position_error_m']<.02 and report['base_max_rotation_error_rad']<.10 and np.all(np.array(report['contact_fraction'])>.95) and not bad and report['minimum_actuator_margin']>=-1e-6)
 trace.save(root/'execution.npz');np.savez_compressed(root/'plan.npz',time=plan.times,q=plan.q,v=plan.v,a=plan.a,wrenches=plan.wrenches,torque=plan.torque);np.savez_compressed(root/'metrics.npz',coordinate=theta,flags=flags,forces=forces);(root/'report.json').write_text(json.dumps(report,indent=2)+'\n');(root/'case.json').write_text(json.dumps(d,indent=2)+'\n')
 for name in {d['object_urdf'],'point_arm.urdf','gripper_arm.urdf','panda.srdf','LICENSE.panda','LICENSE.authored','franka_panda_provenance.json','provenance.json'}:
  src=case.path.parent/name
  if src.resolve()!=(root/name).resolve():shutil.copyfile(src,root/name)
 return plan,trace,report,flags

class HybridFreeController(FreeController):
 """Native measured contact-wrench feedback for all three torque-controlled arms."""
 def __init__(self,plan):
  super().__init__(plan)
  from .hybrid_control import HybridTorqueImpedance
  self.servos=[HybridTorqueImpedance(r,'tip',joint_k=np.r_[np.full(7,40.),[6000,6000]] if r.n==9 else 40.,joint_d=np.r_[np.full(7,8.),[10,10]] if r.n==9 else 8.) for r in plan.case.full];self.ids={};self.measured=np.zeros((3,6));self.filtered_world=np.zeros((3,6));self.previous_tool_origins=None;self.contact_rotations=[np.eye(3) for _ in range(3)];self.anchor_world=np.zeros((3,3));self.force_errors=[];self.measured_history=[];self.commanded_history=[];self.saturations=[];self.dt=plan.case.data['timestep']
 def observe(self,mj,model,data,t):
  if not self.ids:
   self.ids={link:mj.mj_name2id(model,mj.mjtObj.mjOBJ_BODY,'object_urdf_base' if link=='base' else 'object_'+link) for link in ['base','moving']}
  from .contacts import contact_basis
  q=data.qpos[[model.jnt_qposadr[mj.mj_name2id(model,mj.mjtObj.mjOBJ_JOINT,f'arm{i}_'+n)] for i,r in enumerate(self.plan.case.full) for n in r.joint_names]];sums=np.zeros((3,6));origins=[];tool_rotations=[]
  for i,(r,s,a,b) in enumerate(zip(self.plan.case.full,self.plan.case.data['arms'],self.foff[:-1],self.foff[1:])):
   T=r.fk(q[a:b],'tip');origins.append(T[:3,3].copy());tool_rotations.append(T[:3,:3].copy());body=self.ids[s['link_id']];R=data.xmat[body].reshape(3,3);self.anchor_world[i]=data.xpos[body]+R@s['point_link'];self.contact_rotations[i]=R@contact_basis(s['inward_link'])
   for k in range(data.ncon):
    contact=data.contact[k];names=[mj.mj_id2name(model,mj.mjtObj.mjOBJ_GEOM,int(g)) for g in contact.geom];tool=next((j for j,n in enumerate(names) if n.startswith(f'arm{i}_') and ('_tool_collision_' in n or '_panda_leftfinger_collision_' in n or '_panda_rightfinger_collision_' in n)),None)
    if tool is None or not any(n in s['allowed_object_geoms'] for n in names):continue
    local=np.zeros(6);mj.mj_contactForce(model,data,k,local);frame=contact.frame.reshape(3,3);sign=1 if tool==1 else -1;F=sign*(frame.T@local[:3]);M=sign*(frame.T@local[3:])+np.cross(contact.pos-T[:3,3],F);sums[i]+=np.r_[M,F]
  # Explicit 10 ms low-pass estimate; only native forces, never optimizer values.
  alpha=self.dt/(.01+self.dt)
  if self.previous_tool_origins is not None:
   for i in range(3):self.filtered_world[i,:3]+=np.cross(self.previous_tool_origins[i]-origins[i],self.filtered_world[i,3:])
  self.filtered_world+=alpha*(sums-self.filtered_world);self.previous_tool_origins=np.array(origins)
  for i in range(3):self.measured[i]=np.r_[tool_rotations[i].T@self.filtered_world[i,:3],self.filtered_world[i,3:]]
 def command(self,q,v,qr,vr,ar):
  k=int(np.clip(np.searchsorted(self.plan.times,self.t)-1,0,len(self.plan.times)-1));out=[];desired=[];errors=[];sat=[]
  for i,(r,servo,a,b) in enumerate(zip(self.plan.case.full,self.servos,self.foff[:-1],self.foff[1:])):
   T=r.fk(q[a:b],'tip');w=self.plan.wrenches[k,i];moment=w[:3]+np.cross(self.anchor_world[i]-T[:3,3],w[3:]);reaction=np.r_[-T[:3,:3].T@moment,-w[3:]];out.extend(servo.command(*(z[a:b] for z in [q,v,qr,vr,ar]),reaction_desired=reaction,reaction_measured=self.measured[i],contact_rotation_world=self.contact_rotations[i],dt=self.dt));desired.append(reaction);errors.append(servo.last['force_error_contact']);sat.append(servo.last['saturated'])
  self.measured_history.append(self.measured.copy());self.commanded_history.append(np.array(desired));self.force_errors.append(np.array(errors));self.saturations.append(sat);return np.asarray(out)
