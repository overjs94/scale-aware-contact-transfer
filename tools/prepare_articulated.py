"""Author three small passive one-DoF furniture testbeds; robot mechanics stay URDF-owned."""
import json,shutil
from pathlib import Path
import xml.etree.ElementTree as E
import numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[1];DEST=ROOT/'src/contact_transfer/scenarios/articulated'

def box(link,size,pos=(0,0,0),color='.65 .39 .18 1',name='wood'):
    for kind in ['visual','collision']:
        g=E.SubElement(link,kind);E.SubElement(g,'origin',xyz=' '.join(map(str,pos)));E.SubElement(E.SubElement(g,'geometry'),'box',size=' '.join(map(str,size)))
        if kind=='visual':E.SubElement(E.SubElement(g,'material',name=name),'color',rgba=color)
def inertia(link,mass,size,com):
    x,y,z=size;I=mass/12*np.array([y*y+z*z,x*x+z*z,x*x+y*y]);i=E.SubElement(link,'inertial');E.SubElement(i,'origin',xyz=' '.join(map(str,com)));E.SubElement(i,'mass',value=str(mass));E.SubElement(i,'inertia',ixx=str(I[0]),iyy=str(I[1]),izz=str(I[2]),ixy='0',ixz='0',iyz='0')
def main():
    DEST.mkdir(parents=True,exist_ok=True);base=ROOT/'src/contact_transfer/scenarios/panel_fixture'
    for n in ['point_arm.urdf','gripper_arm.urdf','panda.srdf','LICENSE.panda','LICENSE.authored','franka_panda_provenance.json']:shutil.copyfile(base/n,DEST/n)
    specs=[('hinged_door','revolute',[0,0,1],[0,0,.7],[.6,.04,.4],[.3,0,.2],[.5,-.02,.2],[0,1,0],0,np.pi/3,[.4,-.3,.25],1.5,.4,.05,'P'),
      ('folding_panel','revolute',[0,1,0],[0,0,.85],[.6,.4,.04],[.3,0,0],[.5,0,-.02],[0,0,1],np.deg2rad(75),0,[.45,-.6,0],0,.3,.02,'P'),
      ('drawer','prismatic',[-1,0,0],[0,0,.65],[.4,.35,.2],[.2,0,.1],[-.06,0,.15],[-1,0,0],0,.22,[-.65,-.35,.3],0,8.,2.,'G')]
    for id,kind,axis,pivot,size,com,contact,inward,start,end,robot_base,stiffness,damping,friction,tool in specs:
        root=E.Element('robot',name=id);base_link=E.SubElement(root,'link',name='fixture');moving=E.SubElement(root,'link',name='moving');inertia(moving,2.,size,com);box(moving,size,com)
        if id=='drawer':
            box(moving,[.04,.04,.08],contact,'.32 .35 .39 1','handle_metal')
            box(base_link,[.7,.5,.08],[.2,0,.59],'.23 .29 .34 1','fixture')
            box(base_link,[.04,.5,.3],[.43,0,.75],'.23 .29 .34 1','fixture')
        else:
            for y in [-.4,.4]:box(base_link,[.045,.045,pivot[2]],[-.12,y,pivot[2]/2],'.23 .29 .34 1','fixture')
        # Pedestal for the fixed robot when required by the workcell layout.
        if robot_base[2]>0:box(base_link,[.3,.3,robot_base[2]],[robot_base[0],robot_base[1],robot_base[2]/2],'.23 .29 .34 1','fixture')
        j=E.SubElement(root,'joint',name='object_joint',type=kind);E.SubElement(j,'parent',link='fixture');E.SubElement(j,'child',link='moving');E.SubElement(j,'origin',xyz=' '.join(map(str,pivot)));E.SubElement(j,'axis',xyz=' '.join(map(str,axis)));E.SubElement(j,'limit',lower='0',upper=str(1.5 if kind=='revolute' else .3),effort='0',velocity='2');E.SubElement(j,'dynamics',damping=str(damping),friction=str(friction))
        E.indent(root);E.ElementTree(root).write(DEST/(id+'.urdf'),encoding='utf-8',xml_declaration=True)
        if id=='hinged_door':Rtool=np.column_stack(([0,0,1],[1,0,0],[0,1,0]))
        elif id=='drawer':Rtool=Rotation.from_euler('y',90,degrees=True).as_matrix()
        else:Rtool=np.eye(3)
        path=DEST/(id+'.json');old=json.loads(path.read_text()) if path.exists() else {}
        d=dict(id=id,schema_version=1,units='SI',object_urdf=id+'.urdf',robot_urdf='gripper_arm.urdf' if tool=='G' else 'point_arm.urdf',tool=tool,contact_object=contact,inward_object=inward,tool_rotation_object_xyzw=old.get('tool_rotation_object_xyzw',Rotation.from_matrix(Rtool).as_quat().tolist()),initial_coordinate=start,target_coordinate=end,duration=14.,motion_start=2.,motion_end=11.,timestep=.001,sample_dt=.25,robot_base=robot_base,robot_yaw=float(np.arctan2(-robot_base[1],.2-robot_base[0])),q_seed=old.get('q_seed',[0,-.5,0,-2,0,1.7,0]),stiffness=stiffness,damping=damping,frictionloss=friction,normal_max_N=40.,force_mu=.8,contact_preload_m=(.0005 if id=="hinged_door" else .0002) if tool=='P' else 0.)
        path.write_text(json.dumps(d,indent=2)+'\n')
if __name__=='__main__':main()
