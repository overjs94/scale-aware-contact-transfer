import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from contact_transfer import Robot
from contact_transfer.embodiments import catalog, load


ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize("name", list(catalog()))
def test_full_visual_model_matches_recorded_mechanics(name):
    folder = ROOT/"web/models"/name
    meta = json.loads((folder/"model.json").read_text())
    source = catalog()[name]
    assert meta["commit"] == source["commit"]
    assert meta["source_file_sha256"] == source["source_file_sha256"]
    assert hashlib.sha256((folder/"robot.urdf").read_bytes()).hexdigest() == meta["urdf_sha256"]
    assert (folder/"LICENSE.upstream").read_bytes() == (ROOT/"src/contact_transfer/models"/name/"LICENSE.upstream").read_bytes()
    tree = ET.parse(folder/"robot.urdf")
    assert tree.findall("link/visual") and tree.findall("link/collision")
    for mesh in tree.findall(".//mesh"):
        assert mesh.get("filename").startswith("package://upstream/")
        url = meta["packages"]["upstream"]+"/"+mesh.get("filename").removeprefix("package://upstream/")
        assert url in meta["assets"]
    numeric = load(name)
    full = Robot(folder/"robot.urdf", floating_base=source["floating_base"], locked_joints=source["locked_joints"])
    data = json.loads((ROOT/"web/data"/(name+".json")).read_text())
    assert full.joint_names == numeric.joint_names == data["joint_names"]
    assert data["locked_joints"] == source["locked_joints"]
    for index in (0, len(data["q"])//2, len(data["q"])-1):
        q = np.array(data["q"][index])
        np.testing.assert_allclose(full.mass(q), numeric.mass(q), atol=1e-10)
        np.testing.assert_allclose(full.gravity(q), numeric.gravity(q), atol=1e-10)
        for link in data["link_names"]:
            np.testing.assert_allclose(full.fk(q, link), numeric.fk(q, link), atol=1e-10)
