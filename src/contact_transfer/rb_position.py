"""Standalone RB5 position-servo verification, not a panel execution result."""
import json
from pathlib import Path
import numpy as np
from .panel_fixture import PanelCase,build_panel_scene
from .contact_simulation import MujocoContactScene
from .control import rb_position_command
from .robot import Robot


def run_rb5_position_servo(output='outputs/rb5_position_servo',duration=2.):
    output=Path(output);output.mkdir(parents=True,exist_ok=True);case=PanelCase.load();spec=dict(case.data['arms'][1]);spec.update(id='rb5',model='rb5',urdf='rb5_point.urdf',controller='position',q_initial=[0.]*6,base_position=[1.5,0,0],base_rpy=[0,0,0],locked_joints={})
    T=np.eye(4);T[:3,3]=spec['base_position'];robot=Robot(case.path.parent/spec['urdf'],base_pose=T);case.data['arms']=[spec];case.robots=(robot,);case.full_robots=(robot,);build_panel_scene(case,output/'scene.xml')
    names=['rb5_'+n for n in robot.joint_names];motors=['rb5_motor_'+n for n in robot.joint_names];scene=MujocoContactScene(output/'scene.xml',names,motors,['panel'],actuator_modes=['position']*robot.n)
    cfg=case.data['simulation'];goal=np.array([.03,.015,-.015,.01,0,.02])
    def reference(t):
        s=np.clip(t/(duration*.75),0,1);f=10*s**3-15*s**4+6*s**5;fd=(30*s*s-60*s**3+30*s**4)/(duration*.75);fdd=(60*s-180*s*s+120*s**3)/(duration*.75)**2
        return goal*f,goal*fd,goal*fdd
    class Position:
        def command(self,q,v,qr,vr,ar):
            ff=robot.inverse_dynamics(qr,vr,ar);return np.clip(qr+(ff+cfg['position_kd']*vr)/cfg['position_kp'],robot.lower,robot.upper)
    trace=scene.run(Position(),reference,duration,case_id='rb5-850e-standalone-position',plan_id='small-upright-motion',initial_q=np.zeros(robot.n));trace.save(output/'execution.npz')
    error=float(np.max(abs(trace.q[-1]-goal)));report=dict(model='RB5-850E',scenario='standalone_position_servo',panel_task_executed=False,command_units='rad',torque_units='Nm',assumed_position_kp=cfg['position_kp'],assumed_position_kd=cfg['position_kd'],urdf_effort_limits_Nm=robot.effort.tolist(),final_joint_error_rad=error,maximum_actuator_torque_Nm=float(np.max(abs(trace.torque))),hardware_transmission_executed=False,execution_passed=bool(error<.002 and np.all(abs(trace.torque)<=robot.effort+1e-8)),last_command=rb_position_command(robot.joint_names,goal),limitations=['assumed inner PD; not vendor identified servo','small upright joint motion; not loaded panel carrying'])
    (output/'report.json').write_text(json.dumps(report,indent=2)+'\n');return report,trace
