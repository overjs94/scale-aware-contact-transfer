"""Uploaded manuscript's rigid-panel experiment, independent of nominal replay.

Given functional groups are inputs. Source refinement, robust target realization
and interval optimization are executed explicitly; no nominal-path fallback is
accepted as an algorithm result. All dimensions and inertias come from URDFs.
"""
from dataclasses import dataclass
from pathlib import Path
import json
import xml.etree.ElementTree as ET
import casadi as ca
import numpy as np
from scipy.spatial.transform import Rotation, Slerp
from .panel_fixture import PanelCase, build_panel_scene
from .source_transfer import refine_source_and_transfer, load_source
from .contacts import Capability, SurfacePatch, contact_basis
from .finger_grasp import FingerGrasp
from .paper_free import native_finger_geometry
from .generalized_load import GeneralizedTaskLoad
from .generalized_conic import fixed_generalized_margin
from .paper_keyframe import Interface, realize_system_keyframe, relative_interface
from .system import MechanicalSystem
from .robot import Robot
from .contracted import numerical_pose_ik, pose_matrix
from .trajectory import Contact, TrajectoryResult, solve_cito
from .optimization import SolverSettings


def planar_patch(point, normal, axes, half):
    u=ca.SX.sym('panel_surface',len(half));p=ca.DM(point)+ca.DM(axes)@u
    return SurfacePatch(ca.Function('panel_p',[u],[p]),ca.Function('panel_R',[u],[contact_basis(normal)]),-np.asarray(half),np.asarray(half),np.zeros(len(half)))


@dataclass
class PaperPanelProblem:
    case: PanelCase
    directory: Path
    source: dict
    robots: tuple
    capabilities: tuple
    frames: tuple
    patches: tuple
    system: MechanicalSystem

    @classmethod
    def create(cls,directory='outputs/paper_panel',case_path=None,tool_types=('G','P','P'),assignment=(0,1,2)):
        case=PanelCase.load(case_path);out=Path(directory);out.mkdir(parents=True,exist_ok=True)
        if tool_types not in [('G','P','P'),('P','P','P')] or sorted(assignment)!=[0,1,2]:raise ValueError('Unsupported hardware vector or noninjective assignment')
        if tool_types[0]=='P':
            spec=case.data['arms'][0];spec.update(kind='P',urdf='point_arm.urdf',locked_joints={});base=case.robots[0].base_pose
            robot=Robot(case.path.parent/'point_arm.urdf',base_pose=base);case.robots=(robot,*case.robots[1:]);case.full_robots=(robot,*case.full_robots[1:])
        case.data['hardware_types']=list(tool_types);case.data['group_to_hardware_arm']=list(assignment)
        case.data['arms']=[case.data['arms'][i] for i in assignment];case.robots=tuple(case.robots[i] for i in assignment);case.full_robots=tuple(case.full_robots[i] for i in assignment)
        raw=load_source(case.path.parent/case.data['source_file'])
        for spec,group in zip(case.data['arms'],raw['groups']):
            spec['group_id']=group['group_id']
            if spec['kind']=='G' and group['group_id']!='edge_grip':raise ValueError('This grasp primitive requires the accessible panel edge')
        source=refine_source_and_transfer(case.path.parent/case.data['source_file'],geometry_weight=0.,object_rotation=pose_matrix(case.data['keyframes'][0]['object_pose'])[:3,:3])
        raw=load_source(case.path.parent/case.data['source_file']);robots=[];caps=[];frames=[];patches=[]
        for index,(spec,full,group,point,normal) in enumerate(zip(case.data['arms'],case.full_robots,raw['groups'],source['target_positions_m'],source['target_normals'])):
            p,n=np.asarray(point),np.asarray(normal);Rc=contact_basis(n)
            axes=np.asarray(group['tangent_axes']).T
            patches.append(planar_patch(p,n,axes,np.asarray(group['half_size'])*source['source_to_target_scale']))
            if spec['kind']=='P':
                robots.append(case.robots[index]);caps.append(Capability(mu=case.data['optimization']['point_mu'],normal_max=case.data['optimization']['point_normal_max']));frames.append(spec['tool_frame']);continue
            # One holding primitive is shared across all keyframes of a grasp.
            patches[-1]=SurfacePatch.fixed(p,n)
            Rt=Rotation.from_quat(spec['tool_quaternion_xyzw']).as_matrix()
            a0,center,hw,hh=native_finger_geometry(full)
            size=np.fromstring(ET.parse(case.panel.path).find('link/collision/geometry/box').get('size'),sep=' ')
            width=float(np.abs(Rt[:,1])@size)
            # Place a conservative pad rectangle wholly inside the panel's edge.
            inset=hh/2;tcp=p-inset*n;Tlt=np.eye(4);Tlt[:3,:3]=Rt;Tlt[:3,3]=tcp
            Tli=np.eye(4);Tli[:3,:3]=Rc@np.diag([1.,-1.,-1.]);Tli[:3,3]=p
            Tti=np.linalg.inv(Tlt)@Tli;Tlc=np.eye(4);Tlc[:3,:3]=Rc;Tlc[:3,3]=p
            Ttg=np.eye(4);Ttg[:3,3]=center;Tcg=np.linalg.inv(Tlc)@Tlt@Ttg
            root=ET.parse(full.path);ET.SubElement(root.getroot(),'link',name='paper_interface');joint=ET.SubElement(root.getroot(),'joint',name='paper_interface_mount',type='fixed')
            ET.SubElement(joint,'parent',link=spec['tool_frame']);ET.SubElement(joint,'child',link='paper_interface')
            ET.SubElement(joint,'origin',xyz=' '.join(map(str,Tti[:3,3])),rpy=' '.join(map(str,Rotation.from_matrix(Tti[:3,:3]).as_euler('xyz'))))
            path=out/f'interface{index}.urdf';ET.indent(root);root.write(path,encoding='utf-8',xml_declaration=True)
            qhold=(width-a0)/2;fingers=[full.joint_names.index(f'panda_finger_joint{k}') for k in [1,2]]
            if not max(full.lower[fingers])<=qhold<=min(full.upper[fingers]):raise ValueError('Panel cannot fit URDF gripper aperture')
            cap=FingerGrasp(width,hw,min(hh,inset+float(Rt[:,2]@(-n))*center[2])*.9,.8,18.,aperture_min=max(0,a0),aperture_max=2*min(full.upper[fingers])+a0,interface_from_grasp=Tcg)
            robot=Robot(path,base_pose=full.base_pose,locked_joints={full.joint_names[j]:qhold for j in fingers})
            robots.append(robot);caps.append(cap);frames.append('paper_interface')
        system=MechanicalSystem([r.symbolic() for r in robots],case.panel.symbolic())
        return cls(case,out,source,tuple(robots),tuple(caps),tuple(frames),tuple(patches),system)

    def interfaces(self,source=None):
        source=self.source if source is None else source
        return [Interface(spec['group_id'],i,frame,'panel',patch,cap,np.asarray(p),contact_basis(n)) for i,(spec,frame,patch,cap,p,n) in enumerate(zip(self.case.data['arms'],self.frames,self.patches,self.capabilities,source['target_positions_m'],source['target_normals']))]

    def task(self,pose):
        return GeneralizedTaskLoad(-self.case.panel.gravity(pose),np.eye(6),np.diag(self.case.data['optimization']['disturbance_scale']))

    def keyframe(self,key,settings=None,geometry_constraints=None):
        pose=np.array(key['object_pose']);T=pose_matrix(pose);initial=[]
        source=refine_source_and_transfer(self.case.path.parent/self.case.data['source_file'],geometry_weight=0.,object_rotation=T[:3,:3])
        for i,(r,spec,interface) in enumerate(zip(self.robots,self.case.data['arms'],self.interfaces())):
            point=np.asarray(interface.patch.position(interface.patch.initial)).ravel();target=np.eye(4);target[:3,3]=T[:3,3]+T[:3,:3]@point
            local=Rotation.from_quat(spec['tool_quaternion_xyzw']).as_matrix() if spec['kind']=='P' else contact_basis(self.source['target_normals'][i])@np.diag([1.,-1.,-1.]);target[:3,:3]=T[:3,:3]@local
            initial.extend(numerical_pose_ik(r,self.frames[i],target,spec['q_initial']))
        result=realize_system_keyframe(self.system,np.r_[initial,pose],pose,self.interfaces(source),self.task(pose),rho_min=.1,position_radius=.012,characteristic_length=.6,q_reference=np.r_[np.concatenate([s['q_initial'] for s in self.case.data['arms']]),pose],geometry_constraints=geometry_constraints,settings=settings or SolverSettings(options={'expand':False}))
        result.source=source
        return result

    def verify_keyframe(self,result):
        maps=[];torque_maps=[];closure=0.
        for spec,p,R in zip(self.interfaces(),result.positions,result.rotations):
            T,pw,Rw,J=relative_interface(self.system,ca.DM(result.q),spec,ca.DM(p),ca.DM(R));J=np.asarray(ca.DM(J));maps.append(-J.T[self.system.object_v,:]);torque_maps.append(np.asarray(self.system.B).T@J.T);closure=max(closure,float(ca.norm_2(T[:3,3]-pw)))
        b=np.asarray(ca.DM(self.system.bias(ca.DM(result.q),ca.DM.zeros(self.system.nv)))).ravel();check=fixed_generalized_margin(maps,self.task(result.q[self.system.object_q]),self.capabilities,torque_gravity=np.asarray(self.system.B).T@b,torque_maps=torque_maps,effort=self.system.effort)
        task=self.task(result.q[self.system.object_q]);balance=0.;torque_error=0.;cap_error=0.
        for vertex,reactions,tau in zip(task.vertices,result.reactions,result.torques):
            total=np.zeros(6);expected=[]
            for i,(robot,frame,cap,point,R,lam,qs) in enumerate(zip(self.robots,self.frames,self.capabilities,result.positions,result.rotations,reactions,self.system.q_slices)):
                # Independent NumPy wrench shift and native Pinocchio RNEA/J.
                force=R@lam[3:];moment=R@lam[:3]+np.cross(point,force);total+=np.r_[moment,force]
                Tobj=self.case.panel.fk(result.q[self.system.object_q],'panel');Tw=robot.fk(result.q[qs],frame);w=np.r_[Tobj[:3,:3]@R@lam[:3],Tobj[:3,:3]@force]
                expected.extend(robot.gravity(result.q[qs])-robot.jacobian(result.q[qs],frame,'world_aligned').T@w)
                if cap.kind=='P':cap_error=max(cap_error,-lam[5],lam[5]-cap.normal_max,np.linalg.norm(lam[3:5])-cap.mu*lam[5])
                else:
                    witness=cap.check(lam)
                    if not witness['feasible']:cap_error=max(cap_error,witness['primal_residual'])
            balance=max(balance,float(np.max(abs(total-np.asarray(task.required(result.rho,vertex)).ravel()))));torque_error=max(torque_error,float(np.max(abs(np.asarray(expected)-tau))))
        return dict(rho=check['rho'],primal_residual=check['primal_residual'],certificate=check['certificate'],closure_m=closure,native_equilibrium_residual=balance,native_torque_residual=torque_error,capability_violation=max(0.,cap_error))

    def contacts(self,left,steps,point_phase='maintained'):
        result=[]
        half=np.fromstring(ET.parse(self.case.panel.path).find('link/collision/geometry/box').get('size'),sep=' ')/2
        for i,(spec,cap,p,R) in enumerate(zip(self.case.data['arms'],self.capabilities,left.positions,left.rotations)):
            extent=np.abs(R[:,:2]).T@half;offset=R[:,:2].T@p
            result.append(Contact(spec['group_id'],p,R[:,2],cap,robot_index=i,frame=self.frames[i],object_frame='panel',rotation_object=R,attached_steps=tuple(range(steps)) if cap.kind=='G' else (),maintained_steps=tuple(range(steps)) if cap.kind=='P' else (),sticking_steps=tuple(range(steps)) if cap.kind=='P' and point_phase=='sticking' else (),project_to_surface=cap.kind=='P',align_normal=cap.kind=='P',tangent_bounds=(-extent-offset,extent-offset) if cap.kind=='P' else None))
        return result

    def transition(self,left,right,duration,*,steps=8,settings=None,validator=None,scene_constraints=None,point_phase='maintained',grasp_velocity_model='instantaneous',objective_mode='motion_cost'):
        contacts=self.contacts(left,steps,point_phase);poses=np.array([left.q[self.system.object_q],right.q[self.system.object_q]]);rot=Slerp([0,1],Rotation.from_quat(poses[:,3:]));Q=[]
        for j in range(steps+1):
            s=min(j/(steps-1),1.);f=10*s**3-15*s**4+6*s**5;pose=np.r_[(1-f)*poses[0,:3]+f*poses[1,:3],rot(f).as_quat()];T=pose_matrix(pose);qr=[]
            for i,(r,spec,frame,p) in enumerate(zip(self.robots,self.case.data['arms'],self.frames,left.positions)):
                point=p if spec['kind']=='G' else (1-f)*p+f*right.positions[i]
                target=np.eye(4);target[:3,3]=T[:3,3]+T[:3,:3]@point;target[:3,:3]=T[:3,:3]@(Rotation.from_quat(spec['tool_quaternion_xyzw']).as_matrix() if spec['kind']=='P' else left.rotations[i]@np.diag([1.,-1.,-1.]));qr.extend(numerical_pose_ik(r,frame,target,(1-f)*left.q[self.system.q_slices[i]]+f*right.q[self.system.q_slices[i]]))
            Q.append(np.r_[qr,pose])
        import pinocchio as pin
        Q=np.asarray(Q);Q[0]=left.q;Q[-2:]=right.q;dt=duration/steps;V=np.zeros((steps+1,self.system.nv))
        for j in range(1,steps+1):
            V[j,:self.system.object_v.start]=(Q[j,:self.system.object_q.start]-Q[j-1,:self.system.object_q.start])/dt
            T0=pose_matrix(Q[j-1,self.system.object_q]);T1=pose_matrix(Q[j,self.system.object_q]);difference=np.linalg.inv(T0)@T1;delta=pin.log6(pin.SE3(difference[:3,:3],difference[:3,3])).vector/dt;V[j,self.system.object_v]=np.r_[delta[3:],delta[:3]]
            constrained=[]
            for c in contacts:
                J=np.asarray(ca.DM(c.geometry(self.system,ca.DM(Q[j]))[1]));constrained.append(J if c.capability.kind=='G' else J[2:3])
            J=np.vstack(constrained);V[j]-=np.linalg.lstsq(J,J@V[j],rcond=1e-9)[0]
        V[[0,-1]]=0;U=np.zeros((steps,self.system.nu));impulses={c.name:[] for c in contacts}
        for j in range(steps):
            b=np.asarray(ca.DM(self.system.bias(ca.DM(Q[j]),ca.DM.zeros(self.system.nv)))).ravel();maps=[];tmap=[]
            for interface,p,R in zip(self.interfaces(),left.positions,left.rotations):
                J=np.asarray(ca.DM(relative_interface(self.system,ca.DM(Q[j]),interface,ca.DM(p),ca.DM(R))[-1]));maps.append(-J.T[self.system.object_v,:]);tmap.append(np.asarray(self.system.B).T@J.T)
            check=fixed_generalized_margin(maps,self.task(Q[j,self.system.object_q]),self.capabilities);lam=check['reactions'].mean(axis=0)
            if not check['certificate']:raise RuntimeError('No feasible static force initialization along interval')
            U[j]=np.asarray(self.system.B).T@b-sum(M@w for M,w in zip(tmap,lam))
            for c,w in zip(contacts,lam):impulses[c.name].append(dt*(w[3:] if c.capability.kind=='P' else w))
        initial=TrajectoryResult(Q,V,U,np.full(steps,dt),{k:np.asarray(v) for k,v in impulses.items()},0.,0.,0.,0.,0.,False,())
        desired=[pose_matrix(np.r_[(1-j/steps)*poses[0,:3]+j/steps*poses[1,:3],rot(j/steps).as_quat()]) for j in range(steps+1)]
        return solve_cito(self.system,left.q,right.q,contacts,steps=steps,duration=duration,dt_bounds=(duration/steps*.5,duration/steps*1.5),fixed_dt=False,initial=initial,desired_object_poses=desired,characteristic_length=.6,kinematic_tolerance=1e-5,grasp_velocity_model=grasp_velocity_model,objective_mode=objective_mode,friction_relaxations=(1e-3,1e-5,1e-8),velocity_relaxations=(1e-3,1e-5,0.),scene_constraints=scene_constraints,full_geometry_validator=validator,settings=settings or SolverSettings(max_iterations=1200,options={'expand':False}))
