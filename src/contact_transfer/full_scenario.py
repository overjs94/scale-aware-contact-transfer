"""Full-task definitions and genuine P/G candidate problem construction.

Legacy nominal runners remain distinct. No candidate is a full result until
its robust keyframes, every transition, geometry and native execution pass.
The assembly source's point-model failure may use the manuscript's explicit
geometric-prior fallback, with no transferred source certificate.
"""
from copy import deepcopy
from dataclasses import dataclass
from pathlib import Path
from types import SimpleNamespace
import json
import xml.etree.ElementTree as ET
import casadi as ca
import numpy as np
from scipy.spatial.transform import Rotation
from .robot import Robot
from .panel_fixture import PanelCase
from .free_articulated import FreeCase,object_reference as box_reference
from .table_assembly import AssemblyCase,build_assembly_scene
from .paper_panel import planar_patch
from .paper_free import source_urdf
from .paper_keyframe import Interface,realize_system_keyframe
from .contacts import Capability,SurfacePatch,contact_basis
from .finger_grasp import FingerGrasp
from .multi_object import RigidCollection,BodyContact
from .system import MechanicalSystem
from .generalized_load import GeneralizedTaskLoad,link_reaction_map
from .pipeline import source_prior,scale_prior
from .source_transfer import refine_source_and_transfer,load_source
from .contracted import numerical_pose_ik,pose_matrix
from .optimization import SolverSettings
from .spherical_tool import mount as mount_point


def release_ik(robot,frame,target,seed):
    """Once contact is released, position retreat leaves wrist orientation free."""
    from scipy.optimize import least_squares
    seed=np.asarray(seed);target=np.asarray(target)
    def error(q):
        T=robot.fk(q,frame);return np.r_[10*(T[:3,3]-target[:3,3]),.01*Rotation.from_matrix(target[:3,:3].T@T[:3,:3]).as_rotvec(),.001*(q-seed)]
    fit=least_squares(error,np.clip(seed,robot.lower+1e-5,robot.upper-1e-5),bounds=(robot.lower+1e-6,robot.upper-1e-6),max_nfev=200,ftol=1e-12,xtol=1e-12,gtol=1e-12)
    if np.linalg.norm(robot.fk(fit.x,frame)[:3,3]-target[:3,3])>2e-5:raise RuntimeError('Released retreat position is unreachable')
    return fit.x


def point_pose_ik(robot,frame,position,normal,seed):
    """A spherical P tool constrains position and axis, leaving axial roll free."""
    from scipy.optimize import least_squares
    seed=np.asarray(seed);position=np.asarray(position);normal=np.asarray(normal)
    def residual(q):
        T=robot.fk(q,frame);return np.r_[10*(T[:3,3]-position),T[:3,2]+normal,1e-7*(q-seed)]
    fit=least_squares(residual,np.clip(seed,robot.lower+1e-5,robot.upper-1e-5),bounds=(robot.lower+1e-6,robot.upper-1e-6),max_nfev=240,ftol=1e-12,xtol=1e-12,gtol=1e-12)
    if np.linalg.norm(residual(fit.x)[:6])>2e-5:raise RuntimeError(f'Point position/axis IK failed at {frame}')
    return fit.x


def polish_keyframe(problem,key):
    """Remove numerical endpoint closure error before fixed-endpoint CITO.

    Geometry and rho stay fixed. Recompute every vertex's torque with native
    inverse dynamics after the tiny IK correction, then independently verify.
    """
    active={s.group_id for s in key.interfaces};largest=0.;obj=problem.task_definition.object;sys=problem.system;u_offsets=np.r_[0,np.cumsum([r.n for r in problem.robots])]
    for i,(f,r,frame,spec,qs) in enumerate(zip(problem.task_definition.features,problem.robots,problem.frames,problem.specs,sys.q_slices)):
        if f.id not in active:continue
        old=key.q[qs].copy();T=obj.fk(key.q[sys.object_q],f.frame);Rc=T[:3,:3]@key.rotations[i];goal=np.eye(4);goal[:3,3]=T[:3,3]+T[:3,:3]@key.positions[i]
        if problem.types[i]=='G':goal[:3,:3]=Rc@np.diag([1.,-1.,-1.])
        else:
            current=r.fk(old,frame)[:3,:3];goal[:3,:3]=Rotation.align_vectors(np.array([-Rc[:,2]]),np.array([current[:,2]]))[0].as_matrix()@current
        new=point_pose_ik(r,frame,goal[:3,3],Rc[:,2],old) if problem.types[i]=='P' else numerical_pose_ik(r,frame,goal,old);largest=max(largest,float(np.max(abs(new-old))));key.q[qs]=new;span=np.maximum(r.upper-r.lower,1e-3);reference=np.array(spec['q_initial']);key.objective+=.01*(np.sum(((new-reference)/span)**2)-np.sum(((old-reference)/span)**2));J=r.jacobian(new,frame,'world_aligned');rotation=np.zeros((6,6));rotation[:3,:3]=rotation[3:,3:]=Rc;key.torques[:,u_offsets[i]:u_offsets[i+1]]=np.array([r.gravity(new)-J.T@rotation@w for w in key.reactions[:,i]])
    if largest>1e-3:raise RuntimeError(f'Endpoint polish {largest:.3g} rad exceeds 1e-3; resolve the target keyframe')
    key.endpoint_polish_max_rad=largest;return key


@dataclass
class Feature:
    id: str
    frame: str
    surface: np.ndarray
    normal: np.ndarray
    tcp: np.ndarray
    tool_rotation: np.ndarray
    box_size: np.ndarray
    allowed_types: tuple
    edge_pinch: bool=False


def nearest_box(robot,frame,point):
    from .urdf import origin
    boxes=[]
    for e in ET.parse(robot.path).findall(f"link[@name='{frame}']/collision"):
        b=e.find('geometry/box')
        if b is None:continue
        R,p=origin(e);half=np.fromstring(b.get('size'),sep=' ')/2;distance=np.linalg.norm(np.maximum(abs(R.T@(point-p))-half,0));boxes.append((distance,2*half))
    if not boxes:raise ValueError('Functional feature requires an explicit box surface')
    return min(boxes,key=lambda v:v[0])[1]


@dataclass
class FullTask:
    id: str
    original: object
    object: object
    features: tuple
    keyframes: tuple
    arms: tuple
    seeds: tuple
    gripper_template: Path
    scale: float
    fixtures: tuple=()
    @classmethod
    def load(cls,id,directory):
        out=Path(directory);out.mkdir(parents=True,exist_ok=True)
        if id=='panel':
            c=PanelCase.load();features=[];raw=load_source(c.path.parent/c.data['source_file'])
            for s,g in zip(c.data['arms'],raw['groups']):
                p=np.array(g['position'])*raw['source_to_target_scale'];n=np.array(g['normal']);Rt=Rotation.from_quat(s['tool_quaternion_xyzw']).as_matrix();features.append(Feature(g['group_id'],'panel',p,n,p,Rt,nearest_box(c.panel,'panel',p),('P','G') if s['kind']=='G' else ('P',),s['kind']=='G'))
            keys=[dict(id=k['id'],time=k['time'],object_q=np.array(k['object_pose']),phase='released' if k['id']=='released' else 'unload' if k['id']=='load_transfer' else 'hold') for k in c.data['keyframes']]
            keys[-1]['freeze_holding_posture']=True;keys[-2]['freeze_posture']=True;keys.insert(-1,dict(id='open_gripper',time=keys[-2]['time']+.8,object_q=keys[-1]['object_q'].copy(),phase='released',retreat_m=0.,freeze_posture=True))
            return cls(id,c,c.panel,tuple(features),tuple(keys),c.full_robots,tuple(np.array(s['q_initial']) for s in c.data['arms']),c.path.parent/'gripper_arm.urdf',4.)
        if id=='box':
            c=FreeCase.load('free_box_lid');features=[]
            for i,s in enumerate(c.data['arms']):
                p=np.array(s['point_link']);n=-np.array(s['inward_link']);size=nearest_box(c.object,s['link_id'],p);surface=p.copy()
                if s['kind']=='G':n=np.array([0.,-1. if i==0 else 1.,0.]);surface=p+n*(abs(n)@size/2)
                features.append(Feature(s['group_id'],s['link_id'],surface,n,p,Rotation.from_quat(s['tool_rotation_link_xyzw']).as_matrix(),size,('P','G') if s['kind']=='G' else ('P',)))
            times=sorted(set([0.,c.data['lift_start'],c.data['lift_end'],c.data['motion_start'],(c.data['motion_start']+c.data['motion_end'])/2,(c.data['motion_start']+3*c.data['motion_end'])/4,c.data['motion_end'],c.data['duration']]))
            keys=tuple(dict(id=['initial','lift_start','lift_done','open_start','opening','opening_late','open_done','hold_final'][i],time=t,object_q=box_reference(c,t)[0],phase='hold') for i,t in enumerate(times))
            return cls(id,c,c.object,tuple(features),keys,c.full,tuple(np.array(s['q_seed']) for s in c.data['arms']),c.path.parent/'gripper_arm.urdf',c.data['source_to_target_scale'])
        if id=='assembly':
            c=AssemblyCase.load();obj=RigidCollection({s['id']:r for s,r in zip(c.data['objects'],c.parts)});features=[]
            for s in c.data['arms']:
                part=obj.bodies[s['object']];p=np.array(s['point_object']);n=np.array(s['normal_object']);size=nearest_box(part,part.tree.root,p);surface=p.copy()
                if s['id']=='leg_gripper':surface+=n*(abs(n)@size/2)
                features.append(Feature(s['id'],s['object'],surface,n,p,Rotation.from_quat(s['tool_quaternion_xyzw']).as_matrix(),size,('P','G') if s['kind']=='G' else ('P',),s['id']=='top_gripper'))
            keys=[dict(id=k['id'],time=k['time'],object_q=np.concatenate([k[n] for n in obj.names]),phase='release_leg' if i==len(c.data['keyframes'])-1 else 'hold') for i,k in enumerate(c.data['keyframes'])]
            before=dict(id='unload_leg',time=c.data['simulation']['release_time'],object_q=keys[-1]['object_q'].copy(),phase='unload_leg',freeze_posture=True);keys.insert(-1,before);keys.insert(-1,dict(id='open_leg_gripper',time=before['time']+.8,object_q=keys[-1]['object_q'].copy(),phase='release_leg',retreat_m=0.,freeze_posture=True));keys[-1]['freeze_holding_posture']=True
            fixtures=tuple(build_assembly_scene(c,out/'legacy-fixture-source.xml'))
            return cls(id,c,obj,tuple(features),tuple(keys),c.full_robots,tuple(np.array(s['q_initial']) for s in c.data['arms']),c.path.parent/'gripper_arm.urdf',4.,fixtures)
        raise ValueError('Full tasks are panel, box and assembly')


def compiled_gripper(task,arm,feature,out):
    """Reuse the verified native mounting/calibration, on the actual arm base."""
    from .robotiq_panel import prepare_arm
    original=Robot(task.gripper_template,base_pose=arm.base_pose)
    root=ET.Element('robot',name='local_grasp_feature');link=ET.SubElement(root,'link',name='feature');geom=ET.SubElement(ET.SubElement(link,'collision'),'geometry');ET.SubElement(geom,'box',size=' '.join(map(str,feature.box_size)));feature_path=out/'feature.urdf';feature_path.parent.mkdir(parents=True,exist_ok=True);feature_path.write_bytes(ET.tostring(root))
    spec=dict(tool_quaternion_xyzw=Rotation.from_matrix(feature.tool_rotation).as_quat().tolist())
    case=SimpleNamespace(full_robots=(original,),robots=(original,),panel=SimpleNamespace(path=feature_path),data={'arms':[spec]})
    profile=prepare_arm(case,out)
    # The full-task planner uses the tested 40 N operating point as a limit and an
    # additional native virtual-work motor cut, rather than assuming every
    # wrench under a declared 60 N jaw box fits the finite 5 Nm transmission.
    profile['closing_force_limit_N']=min(profile['closing_force_limit_N'],profile['settings']['nominal_force_command_per_jaw_N'])
    profile['actuator_linear_cut']=native_actuator_cut(profile)
    return case.full_robots[0],profile


def native_actuator_cut(profile):
    import mujoco as mj
    from .robotiq import equality_jacobian
    from scipy.linalg import null_space
    assets=Path('.cache/robotiq_2f85').resolve();root=ET.parse(assets/'2f85.xml').getroot();root.find('compiler').set('meshdir',str(assets/'assets'));root.find('option').set('jacobian','dense');m=mj.MjModel.from_xml_string(ET.tostring(root,encoding='unicode'));d=mj.MjData(m);direction=np.zeros(m.nv)
    for name,value in profile['hold'].items():j=mj.mj_name2id(m,mj.mjtObj.mjOBJ_JOINT,name);d.qpos[m.jnt_qposadr[j]]=value
    for name,weight in profile['drivers'].items():j=mj.mj_name2id(m,mj.mjtObj.mjOBJ_JOINT,name);direction[m.jnt_dofadr[j]]=weight
    mj.mj_forward(m,d);Z=null_space(equality_jacobian(m,d),rcond=1e-9);stops=[]
    for name in profile['hold']:
        if name.endswith('coupler_joint'):
            j=mj.mj_name2id(m,mj.mjtObj.mjOBJ_JOINT,name);row=np.zeros(m.nv);row[m.jnt_dofadr[j]]=1.;stops.append(row)
    G=Z.T@np.c_[direction,np.array(stops).T]
    if G.shape!=(3,3) or np.linalg.matrix_rank(G)!=3:raise RuntimeError('Native parallel-grasp motor/upper-stop projection is singular')
    W=-np.linalg.solve(G,Z.T);report=json.loads(Path('outputs/robotiq_2f85/report.json').read_text());R=Rotation.from_quat(report['fixture']['quaternion_xyzw']).as_matrix();p=np.array(report['fixture']['position']);force_columns=[];gravity_map=np.zeros((m.nv,3))
    from .robotiq import grip_geometry
    g=grip_geometry(m,d);side_y=[(R.T@(point-p))[1] for point in g['points']]
    for sign in [-1,1]:
        side=int(np.argmax(np.array(side_y)*sign));body=int(m.geom_bodyid[g['pad_geoms'][side][0]]);basis=R@contact_basis([0,sign,0])
        for x,zsite in [(-profile['half_width'],-profile['half_height']),(-profile['half_width'],profile['half_height']),(profile['half_width'],-profile['half_height']),(profile['half_width'],profile['half_height'])]:
            point=p+R@(np.array(profile['grasp_center_tool'])+np.array([x,sign*profile['aperture_m']/2,zsite]));jp=np.zeros((3,m.nv));jr=jp.copy();mj.mj_jac(m,d,jp,jr,point,body);force_columns.append(jp.T@basis)
    for body in range(1,m.nbody):
        jp=np.zeros((3,m.nv));jr=jp.copy();mj.mj_jacBodyCom(m,d,jp,jr,body);gravity_map+=m.body_mass[body]*jp.T
    required=W@np.hstack(force_columns);spring=W@d.qfrc_passive;gravity_bound=np.linalg.norm(m.opt.gravity)*np.linalg.norm(W@gravity_map,axis=1);limit=float(max(abs(m.actuator_forcerange[0])));reserve=.1;A=np.vstack([required[0],-required[0],required[1:]]);b=np.r_[limit-spring[0]-gravity_bound[0]-reserve,limit+spring[0]-gravity_bound[0]-reserve,-spring[1:]-gravity_bound[1:]]
    if np.min(b[:2])<=0:raise RuntimeError('No native gripper actuator budget')
    return dict(A=A.tolist(),b=b.tolist(),motor_limit_Nm=limit,gravity_reaction_bounds=gravity_bound.tolist(),spring_reactions=spring.tolist(),coupling_reserve_Nm=reserve,scope='rigid parallel grasp: native loop reactions eliminated, finite motor and unilateral coupler upper-stop reactions, worst-orientation gravity; soft dynamics/hardware remain independently tested')


@dataclass
class FullProblem:
    task_definition: FullTask
    directory: Path
    types: tuple
    assignment: tuple
    robots: tuple
    full_robots: tuple
    capabilities: tuple
    frames: tuple
    specs: tuple
    system: MechanicalSystem
    refine_enabled: bool=True
    @property
    def case(self):
        if not hasattr(self,'_case'):self._case=SimpleNamespace(path=self.task_definition.original.path,panel=self.task_definition.object,full_robots=self.full_robots,data={'arms':list(self.specs),'simulation':{'timestep':.001}})
        return self._case
    @classmethod
    def create(cls,task,directory,types,assignment=(0,1,2),*,refine_source=True):
        out=Path(directory);out.mkdir(parents=True,exist_ok=True)
        if len(types)!=len(task.features) or sorted(assignment)!=list(range(len(task.arms))):raise ValueError('One tool and injective arm assignment per functional group required')
        robots=[];full=[];caps=[];frames=[];specs=[]
        for i,(f,kind,hardware) in enumerate(zip(task.features,types,assignment)):
            if kind not in f.allowed_types:raise RuntimeError(f'{kind} has no accessible local feature at {f.id}')
            arm=task.arms[hardware];spec=dict(id=f'arm{i}',physical_arm=hardware,group_id=f.id,kind=kind,controller='torque',model='franka_panda',object_frame=f.frame,q_initial=task.seeds[hardware].tolist())
            if kind=='P':
                r,profile=mount_point(arm,out/f'point_{i}.urdf');spec.update(point_tool_profile=profile,locked_joints={},tool_frame='tip');cap=Capability(mu=.8,normal_max=40.);frame='tip';full.append(r)
            else:
                r,profile=compiled_gripper(task,arm,f,out/f'gripper{i}');tcp=f.surface-profile['minimum_inset_m']*f.normal if f.edge_pinch else f.tcp-.001*f.tool_rotation[:,2]
                profile['feature_approach_clearance_m']=0. if f.edge_pinch else .001
                Ttcp=np.eye(4);Ttcp[:3,:3]=f.tool_rotation;Ttcp[:3,3]=tcp;R=contact_basis(f.normal);Ti=np.eye(4);Ti[:3,:3]=R@np.diag([1.,-1.,-1.]);Ti[:3,3]=f.surface;Tti=np.linalg.inv(Ttcp)@Ti
                tree=ET.parse(r.path);ET.SubElement(tree.getroot(),'link',name='paper_interface');j=ET.SubElement(tree.getroot(),'joint',name='paper_interface_mount',type='fixed');ET.SubElement(j,'parent',link='tip');ET.SubElement(j,'child',link='paper_interface');ET.SubElement(j,'origin',xyz=' '.join(map(str,Tti[:3,3])),rpy=' '.join(map(str,Rotation.from_matrix(Tti[:3,:3]).as_euler('xyz'))));path=out/f'gripper_{i}_interface.urdf';ET.indent(tree);tree.write(path,encoding='utf-8',xml_declaration=True);r=Robot(path,base_pose=arm.base_pose);full.append(r)
                Tg=np.eye(4);Tg[:3,3]=profile['grasp_center_tool'];Tc=np.eye(4);Tc[:3,:3]=R;Tc[:3,3]=f.surface;Tcg=np.linalg.inv(Tc)@Ttcp@Tg
                cut=profile['actuator_linear_cut'];cap=FingerGrasp(profile['aperture_m'],profile['half_width'],profile['half_height'],profile['mu'],profile['closing_force_limit_N'],aperture_max=profile['aperture_max_m'],interface_from_grasp=Tcg,force_component_limits=np.full(3,profile['force_component_limit_N']),finger_linear_limits=(cut['A'],cut['b']),preferred_force_coordinates='scaled_tangent');spec.update(gripper_model='robotiq_2f85',gripper_profile=profile,locked_joints=profile['hold'],tool_frame='paper_interface');r=Robot(r.path,base_pose=arm.base_pose,locked_joints=profile['hold']);frame='paper_interface'
            spec['tool_quaternion_xyzw']=Rotation.from_matrix(f.tool_rotation).as_quat().tolist();robots.append(r);caps.append(cap);frames.append(frame);specs.append(spec)
        obj=task.object.symbolic()
        passive=None
        if task.id=='box':
            cfg=task.original.data
            passive=lambda q,v:ca.vertcat(ca.DM.zeros(obj.nv-1),cfg['damping']*v[-1]+cfg['frictionloss']*ca.tanh(v[-1]/.001))
        system=MechanicalSystem([r.symbolic() for r in robots],obj,object_passive_bias=passive)
        return cls(task,out,tuple(types),tuple(assignment),tuple(robots),tuple(full),tuple(caps),tuple(frames),tuple(specs),system,refine_source)

    def task_load(self,qobj,*,source_object=None):
        obj=self.task_definition.object if source_object is None else source_object;d=self.task_definition.id
        if d=='box':
            R=obj.fk(qobj,'base')[:3,:3];J=obj.jacobian(qobj,'base','world_aligned')[3:];E=np.zeros((obj.nv,4));E[:,:3]=J.T@R;E[-1,3]=1.;D=np.diag([.05,.05,.05,.001] if source_object is not None else [1.,1.,1.,.15])
        elif d=='assembly':
            E=np.vstack([np.eye(6)*.8,np.eye(6)*.2]);D=np.diag([.001]*3+[.05]*3 if source_object is not None else [.15]*3+[1.]*3)
        else:E=np.eye(6);D=np.diag([.001]*3+[.05]*3 if source_object is not None else [.15]*3+[1.]*3)
        return GeneralizedTaskLoad(-obj.gravity(qobj),E,D)

    def source(self,key):
        task=self.task_definition;qo=key['object_q'];obj=task.object
        if task.id=='panel':
            result=refine_source_and_transfer(task.original.path.parent/task.original.data['source_file'],enabled=self.refine_enabled,geometry_weight=0.,object_rotation=Rotation.from_quat(qo[3:7]).as_matrix());return dict(positions=np.array(result['target_positions_m']),normals=np.array(result['target_normals']),record=result)
        def scaled(part,name):
            masses={n:link.mass/task.scale**3 for n,link in part.tree.links.items() if link.mass>0}
            return source_urdf(SimpleNamespace(object=part),self.directory/f'source_{name}.urdf',task.scale,masses)
        if task.id=='assembly':src=RigidCollection({n:scaled(r,n) for n,r in obj.bodies.items()});qs=np.concatenate([np.r_[qo[s][:3]/task.scale,qo[s][3:]] for s in obj.q_slices.values()])
        else:src=scaled(obj,'box');qs=qo.copy();qs[:3]/=task.scale
        patches=[SurfacePatch.plane(f.surface/task.scale,f.normal,(.001,.001)) for f in task.features];maps=[link_reaction_map(src.symbolic(),qs,f.frame) for f in task.features];prior=source_prior(patches,self.task_load(qs,source_object=src),[Capability(mu=.8,normal_max=2.) for _ in patches],group_ids=[f.id for f in task.features],reaction_maps=maps,position_radius=.0005 if self.refine_enabled else 0.,normal_radius=0.,geometry_weight=0.,settings=SolverSettings(max_iterations=400,options={'expand':False}));transfer=scale_prior(prior,task.scale)
        return dict(positions=transfer['positions'],normals=transfer['normals'],record=dict(synthetic_input=True,network_inference_executed=False,source_refinement_enabled=self.refine_enabled,source_status=prior.source_status,source_certificate=prior.source_certificate,source_rho=prior.source_rho,source_failure=prior.failure_reason,source_geometry_fallback=not prior.source_certificate,source_forces_transferred=False,source_to_target_scale=task.scale))

    def keyframe(self,k,previous=None,*,environment=(),settings=None):
        key=self.task_definition.keyframes[k];qo=key['object_q']
        retreat=float(key.get('retreat_m',.06 if key['phase'] in {'released','release_leg'} else 0.))
        if previous is not None and (key.get('freeze_posture') or key.get('freeze_holding_posture')):
            from .full_static import fixed_posture_keyframe
            return fixed_posture_keyframe(self,k,previous,environment,retreat)
        if previous is not None and key['phase']==previous.phase and retreat==getattr(previous,'retreat_m',.06 if previous.phase in {'released','release_leg'} else 0.) and np.array_equal(qo,previous.q[self.system.object_q]) and set(c.name for c in environment)==set(previous.environment_reactions):
            # Identical task pose, transferred prior, force mode and environment
            # define the same robust realization problem. Preserve its solution
            # through a dwell instead of creating an arbitrary nullspace sweep
            # from a second numerically equivalent maximum-margin solve.
            certificate=deepcopy(previous);certificate.identical_keyframe_reuse=True;return certificate
        prior=self.source(key);interfaces=[];initial=[]
        for i,(f,r,cap,frame,spec,p,n) in enumerate(zip(self.task_definition.features,self.robots,self.capabilities,self.frames,self.specs,prior['positions'],prior['normals'])):
            p=f.surface if cap.kind=='G' else previous.positions[i] if previous is not None else p
            patch=SurfacePatch.fixed(p,n) if cap.kind=='G' or previous is not None else SurfacePatch.plane(p,n,(.012,.012));interface=Interface(f.id,i,frame,f.frame,patch,cap,np.array(prior['positions'][i]),contact_basis(n));interfaces.append(interface)
            T=self.task_definition.object.fk(qo,f.frame);goal=np.eye(4);goal[:3,3]=T[:3,3]+T[:3,:3]@p
            if cap.kind=='G':local_rotation=contact_basis(n)@np.diag([1.,-1.,-1.])
            else:local_rotation=Rotation.align_vectors(np.array([-n]),np.array([f.tool_rotation[:,2]]))[0].as_matrix()@f.tool_rotation
            goal[:3,:3]=T[:3,:3]@local_rotation
            released=key['phase']=='released' or key['phase']=='release_leg' and f.id=='leg_gripper';unload=key['phase']=='unload' or key['phase']=='unload_leg' and f.id=='leg_gripper'
            if released:goal[:3,3]+=T[:3,:3]@n*retreat
            seed=previous.q[self.system.q_slices[i]] if previous is not None else spec['q_initial'];initial.extend(release_ik(r,frame,goal,seed) if released else point_pose_ik(r,frame,goal[:3,3],T[:3,:3]@n,seed) if cap.kind=='P' else numerical_pose_ik(r,frame,goal,seed));interface.force_enabled=not unload
            if released:interfaces.pop()
        seed=np.r_[initial,qo];reference=seed.copy()
        for i,(f,spec,qs) in enumerate(zip(self.task_definition.features,self.specs,self.system.q_slices)):
            if key['phase']!='released' and not (key['phase']=='release_leg' and f.id=='leg_gripper'):reference[qs]=spec['q_initial']
        certificate=realize_system_keyframe(self.system,seed,qo,interfaces,self.task_load(qo),environment=environment,rho_min=.1,q_reference=reference,position_radius=.015,characteristic_length=.6,settings=settings or SolverSettings(tolerance=1e-7,max_iterations=800,options={'expand':False}))
        # Keep a stable group order, including unloaded/released zero reactions.
        M=len(self.task_definition.features);old=certificate.reactions;reactions=np.zeros((len(old),M,6));positions=np.array([f.surface for f in self.task_definition.features]);rotations=np.array([contact_basis(f.normal) for f in self.task_definition.features]);apertures=[getattr(c,'aperture_required',None) for c in self.capabilities]
        for i,s in enumerate(interfaces):j=next(j for j,f in enumerate(self.task_definition.features) if f.id==s.group_id);reactions[:,j]=old[:,i];positions[j]=certificate.positions[i];rotations[j]=certificate.rotations[i]
        certificate.positions=positions;certificate.rotations=rotations;certificate.reactions=reactions;certificate.apertures=tuple(apertures);certificate.source=prior['record'];certificate.interfaces=interfaces;certificate.phase=key['phase'];certificate.retreat_m=retreat;return certificate
