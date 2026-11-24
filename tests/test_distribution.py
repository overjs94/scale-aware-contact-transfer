import hashlib
import json
from pathlib import Path
import subprocess
import sys

import numpy as np
import pytest

from contact_transfer.assets import asset_manifest,fetch_native_assets
from contact_transfer.full_recording import atomic_npz


def test_installed_cli_config_runs_without_a_source_checkout(tmp_path):
    result=subprocess.run([sys.executable,'-m','contact_transfer.cli','config','--task','assembly'],cwd=tmp_path,capture_output=True,text=True,check=True)
    config=json.loads(result.stdout)
    assert config['disturbance']['body_weights']==[.8,.2]
    assert config['source']['failure_policy']=='geometric_prior'


def test_asset_cache_requires_the_pinned_content_and_does_not_download_on_import(tmp_path,monkeypatch):
    manifest=asset_manifest()['components']['panda_meshes'];name=next(iter(manifest['files']))
    path=tmp_path/'panda_meshes'/name;path.parent.mkdir();path.write_bytes(b'corrupted mesh')
    def forbidden(*args,**kwargs):raise AssertionError('Do not download while checking an existing cache')
    monkeypatch.setattr('urllib.request.urlopen',forbidden)
    with pytest.raises(ValueError,match='hash mismatch'):
        fetch_native_assets(tmp_path,components=['panda_meshes'])


def test_interrupted_cache_write_preserves_previous_complete_archive(tmp_path,monkeypatch):
    path=tmp_path/'stage.npz';atomic_npz(path,value=np.arange(3));before=path.read_bytes()
    def interrupted(*args,**kwargs):raise OSError('disk write interrupted')
    monkeypatch.setattr(np,'savez_compressed',interrupted)
    with pytest.raises(OSError):atomic_npz(path,value=np.arange(4))
    assert path.read_bytes()==before
    assert list(tmp_path.iterdir())==[path]


def test_review_localization_preserves_large_float_values_and_boolean_types():
    import importlib.util
    spec=importlib.util.spec_from_file_location('review_export',Path('tools/export_review_viewer.py'))
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    original={'value':1.0044860745704145e18,'flags':[True,0.],'title':'Original'}
    localized={'value':1004486074570414500,'flags':[True,0],'title':'English'}
    assert original['value']!=localized['value']
    restored=module.preserve_numeric_leaves(original,localized)
    assert restored['value']==original['value'] and type(restored['value']) is float
    assert restored['flags']==original['flags'] and type(restored['flags'][0]) is bool
    assert restored['title']=='English'
    with pytest.raises(ValueError,match='array shape'):module.preserve_numeric_leaves([1,2],[1])
