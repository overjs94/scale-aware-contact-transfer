"""Build our API client against a separately installed CRISP SDK and test it."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess

import numpy as np


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sdk',required=True,help='Unmodified SDK root with lib/cmake/crisp')
    parser.add_argument('--eigen-cmake',help='Directory containing Eigen3Config.cmake')
    parser.add_argument('--cmake',default='cmake')
    parser.add_argument('--build',default='.cache/crisp-adapter-build')
    parser.add_argument('--output',default='outputs/crisp-probe.json')
    args=parser.parse_args();sdk=Path(args.sdk).resolve();build=Path(args.build).resolve()
    if not (sdk/'LICENSE').is_file():parser.error('SDK LICENSE must be retained')
    source=Path(__file__).resolve().parents[1]/'backends/crisp'
    command=[args.cmake,'-S',str(source),'-B',str(build),'-Dcrisp_DIR='+str(sdk/'lib/cmake/crisp'),'-DCMAKE_BUILD_TYPE=Release']
    if args.eigen_cmake:command+=['-DEigen3_DIR='+str(Path(args.eigen_cmake).resolve())]
    subprocess.run(command,check=True);subprocess.run([args.cmake,'--build',str(build),'--parallel','2'],check=True)
    # Exercise the actual IPC bridge, including ground's negative owner ID,
    # one native step and clean SDK destruction, without robot mesh downloads.
    smoke=build/'bridge-smoke';smoke.mkdir(exist_ok=True)
    urdf=smoke/'probe.urdf';urdf.write_text('<robot name="probe"><link name="probe"><inertial><mass value="0.1"/><inertia ixx="0.000016" iyy="0.000016" izz="0.000016" ixy="0" ixz="0" iyz="0"/></inertial><collision><geometry><sphere radius="0.02"/></geometry></collision></link></robot>')
    manifest=smoke/'scene.txt';manifest.write_text(f'canal .001 .5 1\n{urdf} 0 0 0 .2 0 0 0 1\n0\n0\nprobe\n')
    result=subprocess.run([str(build/'crisp_scene_bridge'),str(manifest)],input='step\nquit\n',check=True,text=True,capture_output=True,timeout=30)
    states=[json.loads(line[6:]) for line in result.stdout.splitlines() if line.startswith('STATE ')]
    bridge_passed=len(states)==2 and np.isclose(states[1]['time'],.001) and np.allclose(states[0]['object_pose'],[0,0,.2,0,0,0,1])
    inclined=subprocess.run([str(build/'crisp_frame_probe')],check=True,text=True,capture_output=True)
    frame_check=json.loads(inclined.stdout.strip().splitlines()[-1])
    records=[]
    for solver in ['canal','sub_admm']:
        for dt in [.001,.0005]:
            result=subprocess.run([str(build/'crisp_contact_probe'),solver,str(dt)],check=True,text=True,capture_output=True)
            record=json.loads(result.stdout.strip().splitlines()[-1]);raw=np.array(record['raw_support_world']);force=-raw/dt
            passed=bool(abs(record['height_m']-.02)<1e-6 and record['contact_fraction']>.99 and record['maximum_solver_residual']<1e-7 and np.allclose(force,[0,0,record['weight_N']],atol=1e-7) and record['pair']==[0,1] and np.allclose(record['normal'],[0,0,-1]))
            record.update(force_on_sphere_world_N=force.tolist(),passed=passed);records.append(record)
    report=dict(scope='ground/sphere settling, inclined contact frame/impulse balance and bridge smoke tests; not multi-arm validation or PMI proof',sdk_license_sha256=hashlib.sha256((sdk/'LICENSE').read_bytes()).hexdigest(),sdk_redistributed=False,bridge_initialization_step_and_clean_exit_passed=bool(bridge_passed),inclined_contact=frame_check,records=records,passed=bridge_passed and frame_check['passed'] and all(r['passed'] for r in records))
    output=Path(args.output);output.parent.mkdir(parents=True,exist_ok=True);output.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
    if not report['passed']:raise SystemExit(2)


if __name__=='__main__':main()
