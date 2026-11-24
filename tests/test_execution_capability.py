from types import SimpleNamespace as S
import numpy as np
from scipy.spatial.transform import Rotation
from contact_transfer.contacts import Capability
from contact_transfer.paper_execution import check_execution_wrenches


def test_native_impulse_is_checked_in_its_preceding_configuration():
    R=Rotation.from_euler('y',np.array([[0.],[-.1],[-.2]])).as_matrix()
    poses=np.array([np.r_[np.zeros(3),Rotation.from_matrix(r).as_quat()] for r in R])[:,None,:]
    loads=np.array([r@np.array([.8,0.,1.]) for r in R[:-1]])
    trace=S(metadata=dict(engine='crisp',timestep=.1,contact_settings=dict(force_interval='preceding native step')),time=np.array([0.,.1,.2]),object_pose=poses,contact_time=np.array([.1,.2]),contact_pair=np.array([['support_tool','panel_collision_0']]*2),contact_position=np.zeros((2,3)),contact_wrench=np.c_[np.zeros((2,3)),-loads])
    episode=S(preparation=.1,problem=S(case=S(data={'arms':[{'id':'support'}]}),capabilities=[Capability(mu=.8,normal_max=10)]),attached=lambda arm,t:True,phase=lambda t:(0,0,t),keyframes=[S(rotations=[np.eye(3)],positions=[np.zeros(3)])])
    assert check_execution_wrenches(episode,trace,sample_dt=.1)['passed']
    trace.metadata['engine']='mujoco'
    assert not check_execution_wrenches(episode,trace,sample_dt=.1)['passed']
