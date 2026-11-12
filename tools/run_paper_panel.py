"""Optimize and execute the FIRST coupled 3-Panda panel lift, with engine choice.

This is not the full reorientation/jig/release or III-D candidate comparison.
Literal manuscript integration remains selectable and failed solves stay failed.
"""
import argparse
import json
from pathlib import Path
from contact_transfer.optimization import SolverSettings
from contact_transfer.paper_panel import PaperPanelProblem
from contact_transfer.paper_execution import CoupledEpisode,run_coupled_episode
from contact_transfer.paper_recording import save_lift,load_lift
from contact_transfer.paper_validation import validate_rigid_trajectory
from contact_transfer.paper_geometry import PanelGeometry


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',default='outputs/coupled_panel_lift')
    parser.add_argument('--engine',choices=['plan','mujoco','crisp'],default='mujoco')
    parser.add_argument('--configuration-update',choices=['independent_se3','independent_midpoint','grasp_consistent'],default='independent_midpoint')
    parser.add_argument('--steps',type=int,default=12)
    parser.add_argument('--iterations',type=int,default=1500)
    parser.add_argument('--solver-tolerance',type=float,default=1e-8,help='IPOPT tolerance; independent mechanics/contact acceptance is checked separately')
    parser.add_argument('--timestep',type=float,choices=[.001,.0005],default=.001)
    parser.add_argument('--control-mode',choices=['hybrid_feedback','hybrid_feedforward','impedance_feedforward'],default='hybrid_feedback')
    parser.add_argument('--gripper',choices=['panda_hand','robotiq_2f85'],default='panda_hand')
    parser.add_argument('--viewer',help='Export a passed measured lift to this static viewer directory')
    parser.add_argument('--jaw-force-fraction',type=float,help='Commanded squeeze fraction: default .95 for maximum-jaw force feedback, .7 for mean-jaw force feedforward')
    parser.add_argument('--replay',help='Reload a plan bundle; revalidate mechanics, capability, endpoints and sampled geometry')
    parser.add_argument('--crisp-binary',default='.cache/crisp-adapter-build/crisp_scene_bridge')
    parser.add_argument('--crisp-solver',choices=['canal','sub_admm'],default='canal')
    args=parser.parse_args();out=Path(args.output);out.mkdir(parents=True,exist_ok=True);problem=PaperPanelProblem.create(out/'model',gripper_model=args.gripper)
    # Simulation step is deliberately independent of the optimized time grid.
    problem.case.data['simulation']['timestep']=args.timestep
    if args.replay:
        episode=load_lift(args.replay,problem);save_lift(out/'plan.npz',problem,episode.keyframes,episode.trajectories[0],episode.contacts_by_interval[0])
    else:
        keys=[]
        for i in [0,2]:
            print(f'Optimizing target keyframe {i}',flush=True)
            key=problem.keyframe(problem.case.data['keyframes'][i],shared_points=keys[0].positions if keys else None);keys.append(key)
            print(f'Keyframe {i}: rho={key.rho:.6g}',flush=True)
        keys=tuple(keys)
        settings=SolverSettings(max_iterations=args.iterations,tolerance=args.solver_tolerance,options={'expand':False,'ipopt.acceptable_tol':args.solver_tolerance,'ipopt.acceptable_obj_change_tol':args.solver_tolerance},diagnostics_path=str(out/'ipopt-failure.json'))
        print('Optimizing coupled lift interval',flush=True);result=problem.transition(*keys,7.,steps=args.steps,point_phase='sticking',configuration_update=args.configuration_update,settings=settings)
        contacts=problem.contacts(keys[0],len(result.dt),'sticking');independent=validate_rigid_trajectory(problem,result,contacts);geometry=PanelGeometry(problem);geometric=geometry.validate(result,object_interpolation='holding_grasp')
        save_lift(out/'candidate-plan.npz',problem,keys,result,contacts)
        (out/'candidate-validation.json').write_text(json.dumps(dict(independent=independent,sampled_geometry_passed=geometric,geometry_violations=geometry.last_violations,accepted=independent['passed'] and geometric),indent=2)+'\n')
        if not independent['passed'] or not geometric:raise RuntimeError('Independent plan validation rejected the interval; see candidate-validation.json')
        episode=CoupledEpisode(problem,keys,(result,),contacts_by_interval=(contacts,));save_lift(out/'plan.npz',problem,keys,result,contacts)
    episode.control_mode=args.control_mode;episode.jaw_force_fraction=args.jaw_force_fraction if args.jaw_force_fraction is not None else (.95 if args.control_mode=='hybrid_feedback' else .7)
    if not 0<episode.jaw_force_fraction<=1:parser.error('jaw-force-fraction must be in (0,1]')
    report=dict(scope='first lift only',configuration_update=episode.trajectories[0].configuration_update,reference_interpolation='holding_grasp',sampled_execution_reference_geometry_passed=True,independent=validate_rigid_trajectory(problem,episode.trajectories[0],episode.contacts_by_interval[0]),full_paper_pipeline_executed=False)
    (out/'plan-report.json').write_text(json.dumps(report,indent=2)+'\n')
    if args.engine=='plan':print(json.dumps(report,indent=2));return
    if args.engine=='mujoco':trace,report,controller=run_coupled_episode(episode,out/'mujoco')
    else:
        from contact_transfer.crisp_simulation import run_crisp_episode
        trace,report,controller=run_crisp_episode(episode,out/'crisp',binary=args.crisp_binary,solver=args.crisp_solver,progress=lambda t,d:print(f'CRISP {t:.3f}/{d:.3f} s',flush=True))
    print(json.dumps(report,indent=2))
    if not report['execution_passed']:raise SystemExit(2)
    if args.viewer:
        from contact_transfer.paper_view import export_coupled_lift
        export_coupled_lift(episode,trace,report,args.viewer)


if __name__=='__main__':main()
