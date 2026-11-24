"""Manuscript III-B/C adapters for free, internally articulated objects.

This is the explicit generalized-coordinate extension of the paper's single
rigid-body model. The given source anchors are SURFACE candidates from dense
patches; the native grasp TCP and its interface transform remain distinct.
"""
from pathlib import Path
from dataclasses import dataclass
import json,copy
import xml.etree.ElementTree as E
import numpy as np
import casadi as ca
from scipy.spatial.transform import Rotation
from .free_articulated import FreeCase,object_reference
from .robot import Robot
from .contacts import Capability,SurfacePatch,contact_basis
from .pipeline import source_prior,scale_prior
from .generalized_load import GeneralizedTaskLoad,link_reaction_map
from .generalized_conic import fixed_generalized_margin
from .paper_keyframe import Interface,realize_system_keyframe
from .finger_grasp import FingerGrasp
from .system import MechanicalSystem
from .urdf import origin
from .optimization import SolverSettings


def source_urdf(case,path,scale,masses):
 tree=E.parse(case.object.path)
 for node in tree.findall('.//origin'):
  if node.get('xyz') is not None:node.set('xyz',' '.join(map(str,np.fromstring(node.get('xyz'),sep=' ')/scale)))
 for box in tree.findall('.//box'):box.set('size',' '.join(map(str,np.fromstring(box.get('size'),sep=' ')/scale)))
 for link in tree.findall('link'):
  inertial=link.find('inertial')
  if inertial is None:continue
  old=float(inertial.find('mass').get('value'));mass=masses[link.get('name')];inertial.find('mass').set('value',str(mass))
  for key,value in list(inertial.find('inertia').attrib.items()):inertial.find('inertia').set(key,str(float(value)*mass/old/scale**2))
 for joint in tree.findall('joint'):
  if joint.get('type')=='prismatic':
   for name in ['lower','upper','velocity']:joint.find('limit').set(name,str(float(joint.find('limit').get(name))/scale))
 E.indent(tree);tree.write(path,encoding='utf-8',xml_declaration=True);return Robot(path,floating_base=True)


def native_finger_geometry(full):
 from .lie import inverse
 q=full.neutral();q[-2:]=0;Tcp=full.fk(q,'tip');samples=[];half=[]
 for name,sign in [('panda_leftfinger',1),('panda_rightfinger',-1)]:
  elements=E.parse(full.path).findall(f"link[@name='{name}']/collision");element=elements[-1];R,p=origin(element);size=np.fromstring(element.find('geometry/box').get('size'),sep=' ')/2;T=full.fk(q,name);pw=T[:3,3]+T[:3,:3]@p;pc=Tcp[:3,:3].T@(pw-Tcp[:3,3]);Rc=Tcp[:3,:3].T@T[:3,:3]@R;support=np.abs(Rc)@size;inner=pc.copy();inner[1]-=sign*support[1];samples.append(inner);half.append(support)
 a0=samples[0][1]-samples[1][1];center=(samples[0]+samples[1])/2;return a0,center,.9*min(h[0] for h in half),.9*min(h[2] for h in half)


def interface_robot(case,index,surface,normal,directory):
 spec=case.data['arms'][index];r=case.full[index];Rtcp=Rotation.from_quat(spec['tool_rotation_link_xyzw']).as_matrix();center=np.asarray(spec['point_link']);Rc=contact_basis(normal);Tlc=np.eye(4);Tlc[:3,:3]=Rc;Tlc[:3,3]=surface;Tlt=np.eye(4);Tlt[:3,:3]=Rtcp;Tlt[:3,3]=center;Tli=Tlc.copy();Tli[:3,:3]=Rc@np.diag([1.,-1.,-1.]);Tti=np.linalg.inv(Tlt)@Tli
 tree=E.parse(r.path);root=tree.getroot();E.SubElement(root,'link',name='paper_interface');j=E.SubElement(root,'joint',name='paper_interface_mount',type='fixed');E.SubElement(j,'parent',link='tip');E.SubElement(j,'child',link='paper_interface');E.SubElement(j,'origin',xyz=' '.join(map(str,Tti[:3,3])),rpy=' '.join(map(str,Rotation.from_matrix(Tti[:3,:3]).as_euler('xyz'))));path=directory/f'arm{index}.urdf';E.indent(tree);tree.write(path,encoding='utf-8',xml_declaration=True)
 a0,pad_center,hw,hh=native_finger_geometry(r);axis=Rtcp[:,1];feature=next(node for node in E.parse(case.object.path).findall(f"link[@name='{spec['link_id']}']/collision") if np.allclose(np.fromstring(node.find('origin').get('xyz'),sep=' '),center));half=np.fromstring(feature.find('geometry/box').get('size'),sep=' ')/2;width=float(2*np.dot(abs(axis),half));qhold=(width-a0)/2;finger_indices=[r.joint_names.index(f'panda_finger_joint{k}') for k in [1,2]];qlo=max(r.lower[finger_indices]);qhi=min(r.upper[finger_indices])
 if not qlo<=qhold<=qhi:raise ValueError('Handle aperture outside URDF finger limits')
 Ttg=np.eye(4);Ttg[:3,3]=pad_center;Tcg=np.linalg.inv(Tlc)@Tlt@Ttg;cap=FingerGrasp(width,hw,hh,.8,18.,aperture_min=max(0,2*qlo+a0),aperture_max=2*qhi+a0,interface_from_grasp=Tcg);numeric=Robot(path,base_pose=r.base_pose,locked_joints={'panda_finger_joint1':qhold,'panda_finger_joint2':qhold});return numeric,cap,Rc

@dataclass
class PaperFreeProblem:
 case: FreeCase
 directory: Path
 source_object: Robot
 robots: tuple
 capabilities: tuple
 surface_positions: np.ndarray
 surface_normals: np.ndarray
 rotations: tuple
 source_profile: dict
 target_profile: dict
 system: MechanicalSystem
 @classmethod
 def create(cls,id='free_box_lid',directory='outputs/paper_free'):
  case=FreeCase.load(id);directory=Path(directory);directory.mkdir(parents=True,exist_ok=True);scale=case.data['source_to_target_scale'];source_cfg=dict(masses={'base':.03125,'moving':.0078125},mu=.8,normal_max=2.,position_radius=.0005,chart_half=.001,force_reference=[.05,.05,.05],joint_reference=.05 if id=='free_portable_drawer' else .001);target_cfg=dict(force_reference=[1.,1.,1.],joint_reference=1. if id=='free_portable_drawer' else .15,rho_min=.1,position_radius=.02,characteristic_length=.5)
  source=source_urdf(case,directory/'source_object.urdf',scale,source_cfg['masses']);positions=[];normals=[];robots=[];caps=[];rotations=[]
  for i,g in enumerate(case.contraction['groups']):
   points=np.array(g['source_points_m']);center=points.mean(axis=0)
   if g['kind']=='G':
    sign=-1 if i==0 else 1;face=points[np.isclose(points[:,1],points[:,1].min() if sign<0 else points[:,1].max())];p=face[np.argmin(np.linalg.norm(face-center,axis=1))];n=np.array([0,sign,0.]);r,cap,R=interface_robot(case,i,p*scale,n,directory)
   else:p=points[np.argmin(np.linalg.norm(points-center,axis=1))];n=-np.asarray(g['inward_link']);r=case.robots[i];cap=Capability(mu=.8,normal_max=40.);R=contact_basis(n)
   positions.append(p);normals.append(n);robots.append(r);caps.append(cap);rotations.append(R)
  syms=[r.symbolic() for r in robots];obj=case.object.symbolic()
  def passive(q,v):return ca.vertcat(ca.DM.zeros(obj.nv-1),case.data['damping']*v[-1]+case.data['frictionloss']*ca.tanh(v[-1]/.001))
  system=MechanicalSystem(syms,obj,object_passive_bias=passive);return cls(case,directory,source,tuple(robots),tuple(caps),np.array(positions),np.array(normals),tuple(rotations),source_cfg,target_cfg,system)
 def configuration(self,t,source=False):
  q=object_reference(self.case,t)[0].copy()
  if source:
   q[:3]/=self.case.data['source_to_target_scale']
   if self.case.data['id']=='free_portable_drawer':q[-1]/=self.case.data['source_to_target_scale']
  return q
 def task(self,t,source=False):
  obj=self.source_object if source else self.case.object;q=self.configuration(t,source);R=obj.fk(q,'base')[:3,:3];J=obj.jacobian(q,'base','world_aligned')[3:];basis=np.zeros((obj.nv,4));basis[:,:3]=J.T@R;basis[-1,3]=1;cfg=self.source_profile if source else self.target_profile;return GeneralizedTaskLoad(-obj.gravity(q),basis,np.diag([*cfg['force_reference'],cfg['joint_reference']]))
 def source(self,t):
  q=self.configuration(t,True);sym=self.source_object.symbolic();maps=[link_reaction_map(sym,q,g['link_id']) for g in self.case.contraction['groups']];patches=[SurfacePatch.plane(p,n,(self.source_profile['chart_half'],)*2) for p,n in zip(self.surface_positions,self.surface_normals)];caps=[Capability(mu=self.source_profile['mu'],normal_max=self.source_profile['normal_max']) for _ in patches];prior=source_prior(patches,self.task(t,True),caps,group_ids=[g['group_id'] for g in self.case.contraction['groups']],reaction_maps=maps,position_radius=self.source_profile['position_radius'],normal_radius=0.,settings=SolverSettings(options={'expand':False,'ipopt.max_iter':400}));transferred=scale_prior(prior,self.case.data['source_to_target_scale']);validation=None
  if prior.source_certificate:
   matrices=[np.column_stack([np.asarray(f(ca.DM(p),ca.DM(contact_basis(n)),ca.DM.eye(6)[:,j])).ravel() for j in range(6)]) for f,p,n in zip(maps,prior.positions,prior.normals)];validation=fixed_generalized_margin(matrices,self.task(t,True),caps)
  return prior,transferred,validation
 def keyframe(self,t,q_initial=None,geometry_constraints=None):
  prior,transfer,source_check=self.source(t);qobj=self.configuration(t);interfaces=[]
  for i,(g,r,cap,p0,n,R) in enumerate(zip(self.case.contraction['groups'],self.robots,self.capabilities,transfer['positions'],transfer['normals'],self.rotations)):
   canonical=self.surface_positions[i]*self.case.data['source_to_target_scale'];patch=SurfacePatch.fixed(canonical,n) if cap.kind=='G' else SurfacePatch.plane(p0,n,(.015,.015));interfaces.append(Interface(g['group_id'],i,'paper_interface' if cap.kind=='G' else 'tip',g['link_id'],patch,cap,p0,contact_basis(n)))
  if q_initial is None:
   from .contracted import numerical_pose_ik
   initial=[]
   for i,(spec,r) in enumerate(zip(interfaces,self.robots)):
    p=np.array(spec.patch.position(spec.patch.initial)).ravel();T=self.case.object.fk(qobj,spec.object_frame);target=np.eye(4);target[:3,3]=T[:3,3]+T[:3,:3]@p;target[:3,:3]=T[:3,:3]@(Rotation.from_quat(self.case.data['arms'][i]['tool_rotation_link_xyzw']).as_matrix() if spec.capability.kind=='P' else np.array(spec.patch.rotation(spec.patch.initial))@np.diag([1.,-1.,-1.]));initial.extend(numerical_pose_ik(r,spec.robot_frame,target,self.case.data['arms'][i]['q_seed']))
   q_initial=np.r_[initial,qobj]
  result=realize_system_keyframe(self.system,q_initial,qobj,interfaces,self.task(t),rho_min=self.target_profile['rho_min'],q_reference=np.r_[np.concatenate([a['q_seed'] for a in self.case.data['arms']]),qobj],position_radius=self.target_profile['position_radius'],characteristic_length=self.target_profile['characteristic_length'],geometry_constraints=geometry_constraints,settings=SolverSettings(options={'expand':False,'ipopt.max_iter':500}));return result,interfaces,dict(source=prior,source_validation=source_check,transfer=transfer)
