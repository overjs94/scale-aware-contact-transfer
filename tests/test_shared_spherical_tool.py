from pathlib import Path
import numpy as np
from contact_transfer.robot import Robot
from contact_transfer.spherical_tool import mount,specification


def test_three_full_tasks_share_physical_sphere_and_ee_transform(tmp_path):
    source=Path(__file__).parents[1]/'src/contact_transfer/scenarios';records=[]
    for scenario in ['table_assembly','free_articulated','panel_fixture']:
        original=Robot(source/scenario/'point_arm.urdf');robot,spec=mount(original,tmp_path/scenario/'point.urdf')
        records.append(spec);assert robot.joint_names==original.joint_names
        for q in [robot.neutral(),np.array([.1,-.4,.2,-1.5,.1,1.4,.2])]:
            T=np.linalg.inv(robot.fk(q,'panda_link8'))@robot.fk(q,'tip')
            np.testing.assert_allclose(T,specification()['ee_to_tip'],atol=1e-12)
            np.testing.assert_allclose(robot.fk(q,'panda_link8'),original.fk(q,'panda_link8'),atol=1e-12)
    assert len({r['urdf_sha256'] for r in records})==1
    assert records[0]['radius_m']==.025
    np.testing.assert_allclose(np.array(records[0]['ee_to_tip'])[:3,3],[0,0,.165],atol=1e-12)
