"""Machine-readable manuscript scope and native finite-step equation checks."""
import numpy as np
from scipy.spatial.transform import Rotation

from .multi_object import RigidCollection


def require_literal_task(task):
    cfg=task.configuration
    if cfg.formulation_mode!='manuscript_literal':
        return
    if isinstance(task.object,RigidCollection) or task.object.n:
        raise ValueError('Manuscript literal mode requires one free rigid manipulated object')
    if cfg.source.representation!='single_point':
        raise ValueError('Functional source sites are an explicitly declared extension')
    if cfg.transition.configuration_update!='independent_se3':
        raise ValueError('Manuscript literal mode requires independent_se3 post-step integration')
    if not cfg.target.robustness_enabled:
        raise ValueError('Nominal-only target is an ablation, not the robust manuscript problem')


def literal_integration(problem,trajectory,tolerance=2e-5):
    """Post-step Eq. (17) integration, with native mechanics even for variants."""
    obj=problem.task_definition.object;sys=problem.system
    error=dict(joint_coordinates=0.,object_translation_m=0.,object_rotation_rad=0.,object_internal_coordinates=0.)
    roots=obj.q_slices.values() if isinstance(obj,RigidCollection) else [slice(0,obj.nq)]
    for q,qn,vn,h in zip(trajectory.q[:-1],trajectory.q[1:],trajectory.v[1:],trajectory.dt):
        for robot,qs,vs in zip(problem.robots,sys.q_slices,sys.v_slices):
            prediction=robot.integrate(q[qs],h*vn[vs]);error['joint_coordinates']=max(error['joint_coordinates'],float(np.max(abs(qn[qs]-prediction))))
        actual=qn[sys.object_q];prediction=obj.integrate(q[sys.object_q],h*vn[sys.object_v])
        for root in roots:
            a,b=actual[root],prediction[root]
            error['object_translation_m']=max(error['object_translation_m'],float(np.max(abs(a[:3]-b[:3]))))
            error['object_rotation_rad']=max(error['object_rotation_rad'],float((Rotation.from_quat(a[3:7]).inv()*Rotation.from_quat(b[3:7])).magnitude()))
            if len(a)>7:error['object_internal_coordinates']=max(error['object_internal_coordinates'],float(np.max(abs(a[7:]-b[7:]))))
    return dict(passed=max(error.values())<=tolerance,tolerance=tolerance,residuals=error,
                reference='independent native post-step joint and SE(3) integration',
                optimized_configuration_update=trajectory.configuration_update,
                not_a_continuous_certificate=True)


def audit_episode(episode):
    p=episode.problem;task=p.task_definition;cfg=task.configuration
    sources=[key.source for key in episode.keyframes]
    source_pass=all(s.get('source_certificate',False) for s in sources)
    expanded=any(s.get('source_model_extension',False) for s in sources)
    rigid=not isinstance(task.object,RigidCollection) and task.object.n==0
    checks=[literal_integration(p,r) for r in episode.trajectories]
    intervals=[r.objective_mode for r in episode.trajectories]
    source_refined=bool(p.refine_enabled and all(s.get('source_refinement_enabled',False) for s in sources))
    transitions_optimized=all(method in {'motion_cost','stationary_contact_conic_QP','stationary_objects_release_NLP'} for method in intervals)
    stages=source_pass and source_refined and cfg.target.robustness_enabled and transitions_optimized and all(r.full_geometry_checked for r in episode.trajectories)
    literal=stages and rigid and not expanded and all(c['passed'] for c in checks)
    phases={k['phase'] for k in task.keyframes}
    inactive=[list(pair) for pair in cfg.transition.terminal_support_transitions if any(value not in phases for value in pair)]
    environment_nodes=[{c.name:None if c.nodes is None else list(c.nodes) for c in contacts if c.robot_index is None}
        for contacts in episode.contacts_by_interval]
    restrictions=dict(gripper_geometry=cfg.target.gripper_geometry,point_geometry=cfg.target.point_geometry,
        normal_geometry=cfg.target.normal_geometry,posture_reference=task.posture_reference_rule,
        posture_trust_radius_rad=task.posture_trust_radius_rad,assignment='fixed_through_sequence',
        point_mode='sticking',stationary_solver=cfg.transition.stationary_solver,
        fixture_activation_distance_m=cfg.transition.support_activation_distance_m,
        fixture_terminal_transitions=cfg.transition.terminal_support_transitions,
        terminal_rule_reference='contact phase names, not keyframe IDs',
        inactive_terminal_phase_rules=inactive,effective_environment_nodes=environment_nodes,
        phase_modes=cfg.feature_phase_modes,
        frozen_keyframes=[k['id'] for k in task.keyframes if k.get('freeze_posture') or k.get('freeze_holding_posture')])
    return dict(schema_version=1,source_certificates_complete=source_pass,
        source_site_model_extension=expanded,single_rigid_object_domain=rigid,
        target_robustness_enabled=cfg.target.robustness_enabled,
        fixed_E_D=True,source_forces_transferred=False,
        literal_integration=checks,restrictions=restrictions,
        unrestricted_problem_optimality_claimed=False,
        interval_optimization_methods=intervals,
        source_refinement_executed_or_equivalently_reused=source_refined,
        transition_optimization_complete=transitions_optimized,
        full_framework_stage_execution=bool(stages),
        manuscript_literal_equations_passed=bool(literal),
        full_paper_pipeline_executed=bool(literal),
        learned_inference_executed=False,continuous_geometry_certificate=False)
