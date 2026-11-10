from pathlib import Path
import pytest


@pytest.fixture
def slider_urdf(tmp_path):
    links = ['<link name="base"/>']
    joints = []
    axes = ["1 0 0", "0 1 0", "0 0 1"]
    for i, axis in enumerate(axes):
        links.append(f'''<link name="link{i}"><inertial><origin xyz="0 0 0"/>
        <mass value="1"/><inertia ixx=".01" iyy=".01" izz=".01" ixy="0" ixz="0" iyz="0"/>
        </inertial></link>''')
        joints.append(f'''<joint name="joint{i}" type="prismatic">
        <parent link="{'base' if i == 0 else 'link'+str(i-1)}"/><child link="link{i}"/>
        <axis xyz="{axis}"/><limit lower="-2" upper="2" effort="100" velocity="2"/></joint>''')
    path = tmp_path/"slider.urdf"
    path.write_text('<robot name="slider">'+''.join(links+joints)+'</robot>')
    return path
