"""Native MuJoCo collision oracle for the paper panel's complete URDF scene.

Separate from NLP success: expanded candidate pairs constrain subsequent solves;
finer interpolation is sampled independently. No continuous certificate implied.
"""
import casadi as ca
import numpy as np
import xml.etree.ElementTree as ET
from .panel_fixture import build_panel_scene


class PanelGeometry:
    def __init__(self,problem,clearance=.001,tolerance=2e-5):
        import mujoco as mj
        self.mj=mj;self.problem=problem;self.clearance=clearance;self.tolerance=tolerance
        path=problem.directory/'geometry.xml';build_panel_scene(problem.case,path)
        self.model=mj.MjModel.from_xml_path(str(path));self.data=mj.MjData(self.model)
        self.model.geom_margin[:]=clearance;self.model.pair_margin[:]=clearance
        self.callbacks=[];self.pairs=set();self.last_violations=[]
        self.robot_addresses=[]
        for robot,spec in zip(problem.robots,problem.case.data['arms']):
            self.robot_addresses.append([self.model.jnt_qposadr[mj.mj_name2id(self.model,mj.mjtObj.mjOBJ_JOINT,spec['id']+'_'+name)] for name in robot.joint_names])
        self.object_address=self.model.jnt_qposadr[mj.mj_name2id(self.model,mj.mjtObj.mjOBJ_JOINT,'panel_free')]
        self.intended=set()
        for full,spec in zip(problem.case.full_robots,problem.case.data['arms']):
            links=['tool'] if spec['kind']=='P' else ['left_pad','right_pad'] if spec.get('gripper_profile') else ['panda_leftfinger','panda_rightfinger']
            for link in links:
                count=len(ET.parse(full.path).findall(f"link[@name='{link}']/collision"))
                for i in range(count) if spec.get('gripper_profile') else [count-1]:self.intended.add(spec['id']+'_'+link+'_collision_'+str(i))

    def set_state(self,q):
        q=np.asarray(q,float).ravel();m=self.model;d=self.data;mj=self.mj
        d.qpos[:]=m.qpos0
        for robot,spec,qs,adr in zip(self.problem.robots,self.problem.case.data['arms'],self.problem.system.q_slices,self.robot_addresses):
            d.qpos[adr]=q[qs]
            for name,value in robot.tree.locked.items():
                j=mj.mj_name2id(m,mj.mjtObj.mjOBJ_JOINT,spec['id']+'_'+name)
                if j>=0:d.qpos[m.jnt_qposadr[j]]=value
        pose=q[self.problem.system.object_q];d.qpos[self.object_address:self.object_address+7]=np.r_[pose[:3],pose[6],pose[3:6]]
        mj.mj_forward(m,d)

    def names(self,pair):return tuple(self.mj.mj_id2name(self.model,self.mj.mjtObj.mjOBJ_GEOM,int(g)) for g in pair)

    def margin(self,pair):
        names=self.names(pair)
        if 'panel_collision_0' in names:
            other=names[1-names.index('panel_collision_0')]
            if other in self.intended:return None
            if other.startswith('jig_collision_'):return 0.
        return self.clearance

    def discover(self,states):
        failures=[]
        for node,q in enumerate(states):
            self.set_state(q)
            for contact in self.data.contact:
                pair=tuple(sorted(map(int,contact.geom)));margin=self.margin(pair)
                if margin is not None and contact.dist<margin-self.tolerance:
                    self.pairs.add(pair);failures.append(dict(node=node,pair=self.names(pair),distance_m=float(contact.dist),required_m=margin))
        self.last_violations=failures
        return failures

    def constraint(self):
        if not self.pairs:return None
        owner=self;pairs=tuple(sorted(self.pairs))
        class Distance(ca.Callback):
            def __init__(self):ca.Callback.__init__(self);self.construct('panel_distances_'+str(len(owner.callbacks)),{'enable_fd':True,'fd_method':'central'})
            def get_n_in(self):return 1
            def get_n_out(self):return 1
            def get_sparsity_in(self,i):return ca.Sparsity.dense(owner.problem.system.nq,1)
            def get_sparsity_out(self,i):return ca.Sparsity.dense(len(pairs),1)
            def eval(self,args):
                owner.set_state(args[0]);values=[]
                for pair in pairs:
                    distance=owner.mj.mj_geomDistance(owner.model,owner.data,*pair,.05,None);values.append(distance-owner.margin(pair))
                return [ca.DM(values)]
        callback=Distance();self.callbacks.append(callback)
        return lambda q:callback(q)

    def validate(self,trajectory,substeps=10,object_interpolation='independent_se3'):
        from scipy.interpolate import CubicHermiteSpline
        from scipy.spatial.transform import Slerp,Rotation
        t=np.r_[0,np.cumsum(trajectory.dt)];sys=self.problem.system
        joints=CubicHermiteSpline(t,trajectory.q[:,:sys.object_q.start],trajectory.v[:,:sys.object_v.start])
        if object_interpolation not in {'independent_se3','holding_grasp'}:raise ValueError('Unknown reference interpolation')
        holding=next((i for i,c in enumerate(self.problem.capabilities) if c.kind=='G'),None)
        if object_interpolation=='holding_grasp' and holding is None:raise ValueError('Holding-grasp interpolation requires a gripper')
        if object_interpolation=='holding_grasp':
            robot=self.problem.robots[holding];frame=self.problem.frames[holding];qs=sys.q_slices[holding]
            initial=trajectory.q[0,sys.object_q];To=np.eye(4);To[:3,3]=initial[:3];To[:3,:3]=Rotation.from_quat(initial[3:]).as_matrix();tool_to_object=np.linalg.inv(robot.fk(trajectory.q[0,qs],frame))@To
            for q in trajectory.q:
                T=robot.fk(q[qs],frame)@tool_to_object;actual=q[sys.object_q]
                if max(np.linalg.norm(T[:3,3]-actual[:3]),(Rotation.from_matrix(T[:3,:3]).inv()*Rotation.from_quat(actual[3:])).magnitude())>2e-5:raise ValueError('Holding-grasp reference cannot interpolate a released or changing grasp')
        rot=Slerp(t,Rotation.from_quat(trajectory.q[:,sys.object_q.start+3:sys.object_q.start+7]));states=[]
        for i,h in enumerate(trajectory.dt):
            for s in np.linspace(0,1,substeps,endpoint=False):
                now=t[i]+s*h;qr=joints(now)
                if object_interpolation=='holding_grasp':
                    # Match CoupledEpisode.desired_object_pose: interpolate
                    # robot references and carry the held-object reference
                    # through FK. This does not prescribe simulated state.
                    To=robot.fk(qr[qs],frame)@tool_to_object;pose=np.r_[To[:3,3],Rotation.from_matrix(To[:3,:3]).as_quat()]
                else:
                    position=(1-s)*trajectory.q[i,sys.object_q.start:sys.object_q.start+3]+s*trajectory.q[i+1,sys.object_q.start:sys.object_q.start+3];pose=np.r_[position,rot(now).as_quat()]
                states.append(np.r_[qr,pose])
        states.append(trajectory.q[-1]);return not self.discover(states)
