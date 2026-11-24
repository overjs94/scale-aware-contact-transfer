import hashlib
import json
from importlib.metadata import metadata
from importlib.resources import files
from pathlib import Path
import subprocess
import sys
import numpy as np
import mujoco as mj
import contact_transfer as ct
from contact_transfer.configuration import load_task_configuration
from contact_transfer.full_scenario import FullTask,FullProblem
from contact_transfer.full_scene import build_scene
from contact_transfer.assets import fetch_native_assets
assert str(Path(ct.__file__).resolve()).startswith('/tmp/contact-transfer-research-wheel-check/site/')
assert metadata('scale-aware-contact-transfer')['License-Expression']=='Apache-2.0 AND BSD-3-Clause AND GPL-3.0-or-later'
assert ct.__version__=='0.2.0a2'
asset_check=fetch_native_assets()
profile=files('contact_transfer').joinpath('scenarios/panda_hybrid_control.json')
assert load_task_configuration('box',profile).execution.force_d[-1]==160.
assert load_task_configuration('panel').source.position_radius_m==.002
cli={}
for name,args in [('config',['config','--task','panel']),('demo',['demo-refinement']),('doctor',['doctor'])]:
 result=subprocess.run([sys.executable,'-m','contact_transfer.cli',*args],capture_output=True,text=True,check=True)
 cli[name]=json.loads(result.stdout)
task=FullTask.load('panel',Path('release-check/task'))
p=FullProblem.create(task,Path('release-check/planning'),'GPP')
scene=build_scene(p,Path('release-check/native.xml'))
m=mj.MjModel.from_xml_path(str(scene.path.resolve()));d=mj.MjData(m)
checks=[]
for r,spec in zip(p.full_robots,p.specs):
 q=np.clip(r.neutral(),r.lower+1e-5,r.upper-1e-5)
 dofs=[]
 for name,value in zip(r.joint_names,q):
  j=mj.mj_name2id(m,mj.mjtObj.mjOBJ_JOINT,spec['id']+'_'+name)
  assert j>=0
  d.qpos[m.jnt_qposadr[j]]=value;dofs.append(m.jnt_dofadr[j])
 mj.mj_forward(m,d)
 error=float(np.max(abs(d.qfrc_bias[dofs]-r.gravity(q))))
 assert error<2e-7,error
 checks.append(error)
report=dict(package_file=ct.__file__,version=ct.__version__,aggregate_spdx=metadata('scale-aware-contact-transfer')['License-Expression'],asset_files={k:v['files'] for k,v in asset_check.items()},cli_demo=cli['demo'],crisp=cli['doctor']['simulation']['crisp'],hybrid_profile_loaded=True,panel_source_radius_m=.002,native_scene_built=True,native_full_robot_gravity_max_error=checks,installed_without_checkout_import=True)
from contact_transfer.full_pipeline import source_solver_settings
source_checks={}
for name in ['box','assembly']:
 cfg=json.loads(Path(files('contact_transfer').joinpath(f'scenarios/certified_sources/{name}-configuration.json')).read_text())
 task=FullTask.load(name,Path('release-check')/name/'task',configuration=cfg)
 problem=FullProblem.create(task,Path('release-check')/name/'models','GPG',source_settings=source_solver_settings())
 witness=problem.source(task.keyframes[0])['record']
 assert witness['source_certificate'] and witness['source_site_count']==19
 source_checks[name]=dict(certified=True,rho=witness['source_rho'],site_count=19)
report['installed_grouped_source_checks']=source_checks
print(json.dumps(report,indent=2))
