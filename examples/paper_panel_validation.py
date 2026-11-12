"""Execute manuscript source/target checks and a first carry CITO diagnostic.

This is a diagnostic entry point, not the completed panel/fixture/release runner.
Failures persist their reports and exit nonzero; no nominal replay is substituted.
"""
import argparse
import hashlib
import json
from pathlib import Path
import time
import numpy as np
from contact_transfer.paper_panel import PaperPanelProblem
from contact_transfer.paper_geometry import PanelGeometry
from contact_transfer.paper_validation import validate_rigid_trajectory
from contact_transfer.optimization import SolverSettings


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',default='outputs/paper_panel_validation')
    parser.add_argument('--types',choices=['GPP','PPP'],default='GPP')
    parser.add_argument('--assignment',nargs=3,type=int,default=[0,1,2])
    parser.add_argument('--steps',type=int,default=8)
    parser.add_argument('--iterations',type=int,default=700)
    parser.add_argument('--objective',choices=['motion_cost','feasibility'],default='motion_cost')
    parser.add_argument('--grasp-velocity-model',choices=['instantaneous','pose_difference'],default='instantaneous')
    parser.add_argument('--static-only',action='store_true')
    args=parser.parse_args();out=Path(args.output);out.mkdir(parents=True,exist_ok=True)
    report=dict(scope='rigid panel carry/seat keyframes and FIRST lift interval diagnostic; fixture load transfer and release not yet connected',types=args.types,assignment=args.assignment,steps=args.steps,objective_mode=args.objective,grasp_velocity_model=args.grasp_velocity_model,algorithm_validation_passed=False,full_paper_pipeline_executed=False,sections={'III-B':'pending','III-C':'pending','III-D':'not executed: one specified candidate','IV':'pending'},keyframes=[],failure=None)
    def save(): (out/'validation-report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    start=time.perf_counter();save()
    try:
        problem=PaperPanelProblem.create(out,tool_types=tuple(args.types),assignment=tuple(args.assignment));geometry=PanelGeometry(problem);keyframes=[]
        report['gripper_interfaces']=[dict(kind=c.kind,aperture_m=c.aperture_required,mu=c.mu,closing_force_per_finger_N=c.closing_force_per_finger,source='declared simulation capability, native URDF pad geometry; hardware calibration and execution controller validation pending') for c in problem.capabilities if hasattr(c,'aperture_required')]
        report['urdf_sha256']={str(r.path.name):hashlib.sha256(r.path.read_bytes()).hexdigest() for r in [*problem.robots,problem.case.panel]}
        for index in [0,2,3,4,5]:
            key=problem.case.data['keyframes'][index];result=problem.keyframe(key);native=problem.verify_keyframe(result);collisions=geometry.discover([result.q])
            if not native['certificate'] or max(native[k] for k in ['native_equilibrium_residual','native_torque_residual','capability_violation','closure_m'])>2e-6:raise RuntimeError('Independent keyframe verification failed')
            if collisions:raise RuntimeError('Full native collision check rejected keyframe: '+json.dumps(collisions))
            record=dict(id=key['id'],time=key['time'],source=result.source,rho=result.rho,objective=result.objective,primal_residual=result.primal_residual,independent=native,sampled_native_geometry_passed=True)
            report['keyframes'].append(record);keyframes.append(result)
            np.savez_compressed(out/f'keyframe{index}.npz',q=result.q,positions=result.positions,rotations=result.rotations,reactions=result.reactions,torques=result.torques,rho=result.rho,objective=result.objective)
            print(json.dumps(dict(keyframe=key['id'],rho=result.rho,independent=native)),flush=True);save()
        report['sections']['III-B']='executed and independently checked at five source orientations';report['sections']['III-C']='executed and independently checked at five target keyframes';save()
        if not args.static_only:
            report['sections']['IV']='first lift interval running';save()
            settings=SolverSettings(max_iterations=args.iterations,diagnostics_path=str(out/'ipopt-failure.json'),options={'expand':False})
            result=problem.transition(keyframes[0],keyframes[1],7.,steps=args.steps,settings=settings,objective_mode=args.objective,grasp_velocity_model=args.grasp_velocity_model)
            independent=validate_rigid_trajectory(problem,result,problem.contacts(keyframes[0],args.steps));geometric=geometry.validate(result)
            np.savez_compressed(out/'first-interval.npz',q=result.q,v=result.v,tau=result.tau,dt=result.dt,**result.impulses)
            report['interval']=dict(objective=result.objective,primal_residual=result.primal_residual,grasp_velocity_model=result.grasp_velocity_model,grasp_velocity_residual=result.gripper_velocity_residual,instantaneous_grasp_velocity_residual=result.instantaneous_grasp_velocity_residual,independent=independent,sampled_native_geometry_passed=geometric,continuation=result.continuation)
            report['sections']['IV']='first interval returned; remaining intervals and execution not verified'
            if not independent['passed'] or not geometric:raise RuntimeError('Independent interval validation failed')
        report['elapsed_s']=time.perf_counter()-start;save()
    except (RuntimeError,ValueError) as error:
        report['failure']=str(error);report['elapsed_s']=time.perf_counter()-start;save();print(json.dumps(report,indent=2),flush=True);raise SystemExit(2)
    print(json.dumps(report,indent=2),flush=True)


if __name__=='__main__':main()
