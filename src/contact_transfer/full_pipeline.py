"""Complete-task candidate search, accepted stage recording and controlled ablations."""
from itertools import permutations
from pathlib import Path
from time import perf_counter
import numpy as np
from scipy.spatial.transform import Rotation
from .full_scenario import FullTask,FullProblem,polish_keyframe
from .full_scene import build_scene,support_contacts
from .full_validation import Geometry,numeric_contact,support_patch_contains,validate_keyframe,validate_interval
from .full_trajectory import interval_contacts,solve_interval,initial_interval,phase_mode
from .full_execution import FullEpisode,reference_geometry,run_full_episode
from .full_assessment import assess
from .full_recording import fingerprint,save_key,load_key,save_interval,load_interval,write_json
from .contracted import numerical_pose_ik
from .optimization import SolverSettings
from .sequence import choose_sequence


ABLATIONS=('full','no_source_refinement','fixed_contact_types','fixed_path','no_arm_force_feedforward','contact_disabled','external_push')


def scenario_solver_settings(max_iterations=1000,verbose=False):
    # Separate objective/KKT convergence from physical primal feasibility.
    # Degenerate contact witnesses can have nonunique dual multipliers; an
    # acceptable solve must still meet 1e-6 unscaled constraint violation and
    # every final independent unrelaxed dynamics/contact gate.
    return SolverSettings(tolerance=1e-6,max_iterations=max_iterations,verbose=verbose,options={'expand':False,'ipopt.mu_strategy':'adaptive','ipopt.mu_init':1e-5,'ipopt.bound_push':1e-8,'ipopt.bound_frac':1e-8,'ipopt.slack_bound_push':1e-8,'ipopt.slack_bound_frac':1e-8,'ipopt.acceptable_tol':1e-4,'ipopt.acceptable_constr_viol_tol':1e-6,'ipopt.acceptable_compl_inf_tol':1e-4,'ipopt.acceptable_dual_inf_tol':1e-3,'ipopt.acceptable_obj_change_tol':1e-6,'ipopt.acceptable_iter':8})


class FullPlanner:
    def __init__(self,task,directory,*,steps=16,refine_source=True,settings=None,progress=print,keyframe_archive=None):
        self.task=task;self.directory=Path(directory);self.directory.mkdir(parents=True,exist_ok=True);self.steps=steps;self.refine_source=refine_source;self.settings=settings or scenario_solver_settings();self.progress=progress;self.problems={};self.identities={};self.scenes={};self.ledger=[];self.timings=[];self.keyframe_archive=None if keyframe_archive is None else Path(keyframe_archive)
    def problem(self,types,assignment):
        index=(tuple(types),tuple(assignment))
        if index not in self.problems:
            folder=self.directory/(''.join(types)+'_'+''.join(map(str,assignment)));p=FullProblem.create(self.task,folder,types,assignment,refine_source=self.refine_source);self.problems[index]=p;self.identities[index]=fingerprint(p,self.settings,self.steps);self.scenes[index]=build_scene(p,folder/'planning-scene.xml');self.preflight(p)
        return self.problems[index]
    def preflight(self,p):
        """Local IK screening only: retain numerical rejection, no global infeasibility claim."""
        # Each initially independent assembly body must balance its complete
        # six-dimensional task-load image. One/two unilateral point sites have
        # wrench-map rank at most three/five, for every choice of positions.
        # This structural rejection precedes the local numerical IK screen.
        from .multi_object import RigidCollection
        if isinstance(self.task.object,RigidCollection):
            for name in self.task.object.names:
                kinds=[kind for f,kind in zip(self.task.features,p.types) if f.frame==name];rank=6 if 'G' in kinds or len(kinds)>=3 else 5 if len(kinds)==2 else 3*len(kinds)
                if rank<6:raise RuntimeError(f'Structurally insufficient contact-type wrench span on free {name}: maximum rank {rank}<6 before assembly')
        for key in self.task.keyframes:
            for i,(f,r,frame,spec) in enumerate(zip(self.task.features,p.robots,p.frames,p.specs)):
                T=self.task.object.fk(key['object_q'],f.frame);goal=np.eye(4);goal[:3,3]=T[:3,3]+T[:3,:3]@f.surface
                from .contacts import contact_basis
                goal[:3,:3]=T[:3,:3]@(contact_basis(f.normal)@np.diag([1.,-1.,-1.]) if p.types[i]=='G' else Rotation.align_vectors(np.array([-f.normal]),np.array([f.tool_rotation[:,2]]))[0].as_matrix()@f.tool_rotation)
                numerical_pose_ik(r,frame,goal,spec['q_initial'])
    def environment(self,p,k):
        q=np.r_[np.zeros(p.system.object_q.start),self.task.keyframes[k]['object_q']]
        return tuple(c for c in support_contacts(p,self.task.keyframes[k]['phase'],self.steps) if abs(numeric_contact(p,c,q)[0])<=2e-5 and support_patch_contains(p,c,q))
    def keyframe(self,k,types,assignment,previous):
        p=self.problem(types,assignment);identity=self.identities[(tuple(types),tuple(assignment))];env=self.environment(p,k);path=p.directory/f'key{k}.npz';start=perf_counter();cached=False
        try:key=load_key(path,identity);cached=True
        except (FileNotFoundError,ValueError):
            self.progress(f'{self.task.id} {types}/{assignment}: key {k} {self.task.keyframes[k]["id"]}',flush=True);self.settings.diagnostics_path=str(p.directory/f'key{k}-failure.json');key=self.archived_key(p,k,env,previous)
            if key is None:key=p.keyframe(k,previous,environment=env,settings=self.settings)
            else:cached='independently_revalidated_archive'
        key=polish_keyframe(p,key);check=validate_keyframe(p,key,env);geometry=Geometry(p,self.scenes[(tuple(types),tuple(assignment))]);violations=geometry.discover([key.q]);write_json(p.directory/f'key{k}-validation.json',dict(independent=check,geometry_violations=violations,source=key.source,endpoint_polish_max_rad=key.endpoint_polish_max_rad))
        if not check['passed'] or violations:
            np.savez_compressed(p.directory/f'key{k}-unaccepted.npz',q=key.q,positions=key.positions,rotations=key.rotations,reactions=key.reactions,torques=key.torques,rho=key.rho,accepted=False);raise RuntimeError(f'Key {k} independent/geometry rejected; see {p.directory}')
        if cached is not True:save_key(path,key,identity)
        self.timings.append(dict(types=types,assignment=assignment,stage='keyframe',index=k,cache_hit=cached,wall_time_s=perf_counter()-start));return key
    def archived_key(self,p,k,environment,previous):
        """Explicit reuse of earlier optimized keyframes; no archive is trusted as a certificate.

        A caller must opt in. Require byte-identical mechanical URDFs, matching
        source mode/task goal, current trust-region/force-mode feasibility and a
        fresh native/Clarabel/geometry gate. Moving intervals are recomputed.
        This is recorded as cache reuse, never a new NLP timing measurement.
        """
        if self.keyframe_archive is None:return None
        import json,hashlib
        folder=self.keyframe_archive/p.directory.name;path=folder/f'key{k}.npz'
        if not path.exists():return None
        for robot in p.full_robots:
            other=folder/robot.path.name
            if not other.exists() or other.read_bytes()!=robot.path.read_bytes():raise RuntimeError('Archive mechanical URDF mismatch')
        with np.load(path,allow_pickle=False) as a:old_identity=json.loads(str(a['metadata_json']))['identity']
        key=load_key(path,old_identity);goal=self.task.keyframes[k]
        if key.phase!=goal['phase'] or np.max(abs(key.q[p.system.object_q]-goal['object_q']))>2e-5 or key.source.get('source_refinement_enabled')!=self.refine_source:raise RuntimeError('Archive task/source mode mismatch')
        if set(key.environment_reactions)!=set(c.name for c in environment):raise RuntimeError('Archive support-candidate mismatch')
        if previous is not None and goal['phase']==previous.phase and np.array_equal(goal['object_q'],previous.q[p.system.object_q]) and set(c.name for c in environment)==set(previous.environment_reactions):return p.keyframe(k,previous,environment=environment,settings=self.settings)
        prior=p.source(goal);objective=-key.rho;active={s.group_id:s for s in key.interfaces}
        for i,(f,cap,spec,r,qs) in enumerate(zip(self.task.features,p.capabilities,p.specs,p.robots,p.system.q_slices)):
            mode=phase_mode(goal['phase'],f)
            if (f.id in active)!=(mode!='released') or f.id in active and active[f.id].force_enabled!=(mode=='holding'):raise RuntimeError('Archive contact force mode mismatch')
            if mode=='released':continue
            delta=key.positions[i]-prior['positions'][i];normal=1-key.rotations[i,:,2]@prior['normals'][i]
            if np.linalg.norm(delta)>.015+1e-8 or normal>.1+1e-8:raise RuntimeError('Archive leaves current transferred-prior trust region')
            if previous is not None and np.max(abs(key.positions[i]-previous.positions[i]))>2e-5:raise RuntimeError('Archive changes held material contact')
            span=np.maximum(r.upper-r.lower,1e-3);objective+=np.sum(delta**2)/.6**2+normal+.01*np.sum(((key.q[qs]-spec['q_initial'])/span)**2)
        key.objective=float(objective);key.source=prior['record'];key.archive_provenance=dict(path=str(path),sha256=hashlib.sha256(path.read_bytes()).hexdigest(),original_formulation_identity=old_identity,mechanical_URDFs_byte_identical=True,source_and_trust_region_rechecked=True);write_json(p.directory/f'key{k}-archive.json',key.archive_provenance);return key
    def transition(self,k,left,right,types,assignments):
        assignment=assignments[k]
        if any(tuple(a)!=tuple(assignment) for a in assignments):raise RuntimeError('Online physical arm/tool reassignment is not supported')
        p=self.problem(types,assignment);identity=self.identities[(tuple(types),tuple(assignment))];path=p.directory/f'interval{k}.npz';contacts=interval_contacts(p,k,left,self.steps);start=perf_counter();cached=False
        try:r=load_interval(path,identity);cached=True
        except (FileNotFoundError,ValueError):
            self.progress(f'{self.task.id} {types}/{assignment}: CITO interval {k}',flush=True);self.settings.diagnostics_path=str(p.directory/f'interval{k}-failure.json');r,contacts,_=solve_interval(p,k,left,right,self.settings,steps=self.steps)
        independent=validate_interval(p,r,contacts)
        if not independent['passed']:raise RuntimeError('Cached/fresh CITO failed native independent checks')
        local=FullEpisode(p,(left,right),(r,),(contacts,));geometry=reference_geometry(local,self.scenes[(tuple(types),tuple(assignment))]);write_json(p.directory/f'interval{k}-validation.json',dict(independent=independent,reference_geometry=geometry))
        if not geometry['passed']:raise RuntimeError(f'Interval {k} reference geometry rejected; see {p.directory}')
        r.full_geometry_checked=True
        if not cached:save_interval(path,r,identity)
        self.timings.append(dict(types=types,assignment=assignment,stage='transition',index=k,cache_hit=cached,wall_time_s=perf_counter()-start));return r
    def search(self,*,fixed=False):
        start=perf_counter();given=tuple('GPP' if self.task.id=='panel' else 'GPG');allowed=[(kind,) for kind in given] if fixed else [tuple(sorted(f.allowed_types,reverse=True)) for f in self.task.features];assignments=[(0,1,2)] if fixed else list(permutations(range(len(self.task.arms))));sequences=[tuple(a for _ in self.task.keyframes) for a in assignments]
        def key(k,types,assignment,previous):
            try:return self.keyframe(k,types,assignment,previous)
            except (RuntimeError,ValueError) as error:
                self.ledger.append(dict(types=types,assignment=assignment,keyframe=k,accepted=False,reason=str(error)));write_json(self.directory/'candidate-ledger.json',self.ledger);raise RuntimeError(str(error)) from error
        def transition(k,left,right,types,assignments):
            try:return self.transition(k,left,right,types,assignments)
            except (RuntimeError,ValueError) as error:
                self.ledger.append(dict(types=types,assignment=assignments[0],interval=k,accepted=False,reason=str(error)));write_json(self.directory/'candidate-ledger.json',self.ledger);raise RuntimeError(str(error)) from error
        try:result=choose_sequence(allowed,sequences,key,transition)
        finally:write_json(self.directory/'stage-timings.json',self.timings)
        p=self.problem(result.best.types,result.best.assignments[0]);contacts=tuple(interval_contacts(p,k,left,len(r.dt)) for k,(left,r) in enumerate(zip(result.best.keyframes,result.best.transitions)));episode=FullEpisode(p,result.best.keyframes,result.best.transitions,contacts);report=dict(selection_executed=not fixed,selection_method='exhaustive configured hardware-compatible P/G and fixed-through-sequence injective assignment candidates',candidate_count=int(np.prod([len(a) for a in allowed]))*len(sequences),accepted_candidates=[dict(types=c.types,assignment=c.assignments[0],objective=c.objective) for c in result.candidates],rejections=result.failures,selected_types=result.best.types,selected_assignment=result.best.assignments[0],objective=result.best.objective,objective_definition='sum keyframe geometry/posture minus robustness plus sum interval motion costs; no type penalty',planning_wall_time_s=perf_counter()-start,source_refinement_enabled=self.refine_source,source_certificates=[k.source for k in result.best.keyframes],all_keyframes_optimized=True,all_intervals_cito_optimized=True,keyframes=len(episode.keyframes),intervals=len(episode.trajectories),restricted_contact_candidates=True,sampled_geometry=True,continuous_certificate=False,configuration_update=[r.configuration_update for r in episode.trajectories],manuscript_literal_equations_passed=False,extension='free articulated object' if self.task.id=='box' else 'two independent floating bodies and equal/opposite inter-object contacts' if self.task.id=='assembly' else None,full_paper_pipeline_executed=False);write_json(self.directory/'selection.json',report);return episode,report


def run_task(task_id,directory,*,variants=ABLATIONS,steps=16,settings=None,progress=print,web_directory=None,keyframe_archive=None):
    out=Path(directory);task=FullTask.load(task_id,out/'task');results={};full_episode=None;full_selection=None;recordings=[]
    for variant in variants:
        folder=out/variant;folder.mkdir(parents=True,exist_ok=True);start=perf_counter()
        try:
            if variant in {'full','no_source_refinement','fixed_contact_types'}:
                archive=None if keyframe_archive is None else Path(keyframe_archive)/task_id/variant/'planning';planner=FullPlanner(task,folder/'planning',steps=steps,settings=settings,refine_source=variant!='no_source_refinement',progress=progress,keyframe_archive=archive);episode,selection=planner.search(fixed=variant=='fixed_contact_types')
                if variant=='full':full_episode,full_selection=episode,selection
            else:
                if full_episode is None:raise RuntimeError('This controlled ablation requires an accepted complete full-method plan')
                episode=full_episode;selection=full_selection
                if variant=='fixed_path':
                    rs=[];cs=[]
                    for k,(left,right) in enumerate(zip(episode.keyframes[:-1],episode.keyframes[1:])):
                        contacts=interval_contacts(episode.problem,k,left,steps);r=initial_interval(episode.problem,k,left,right,steps,task.keyframes[k+1]['time']-task.keyframes[k]['time'],contacts);r.objective_mode='fixed_path_force_allocation';rs.append(r);cs.append(contacts)
                    episode=FullEpisode(episode.problem,episode.keyframes,rs,cs)
                elif variant=='no_arm_force_feedforward':episode=FullEpisode(episode.problem,episode.keyframes,episode.trajectories,episode.contacts_by_interval,force_feedforward=False)
            disturbances=()
            if variant=='external_push':
                from .perturbations import WrenchPulse
                roots=build_scene(episode.problem,folder/'force-target-scene.xml').object_roots;disturbances=(WrenchPulse(next(iter(roots.values())),episode.preparation+episode.time[-1]*.45,1.,(5.,0.,0.)),)
            trace,controller,scene,geometry=run_full_episode(episode,folder/'native',contacts_enabled=variant!='contact_disabled',allow_reduced_baseline=variant=='fixed_path',disturbances=disturbances,progress=lambda t,total:progress(f'{task_id}/{variant}: native {t:.0f}/{total:.1f}s',flush=True) if int(t)%5==0 else None);report=assess(episode,trace,controller,scene,geometry=geometry);report.update(variant=variant,selection=selection,full_variant_pipeline_executed=variant=='full',all_intervals_cito_optimized=variant!='fixed_path',fixed_path_integration_residuals=[validate_interval(episode.problem,r,c) for r,c in zip(episode.trajectories,episode.contacts_by_interval)] if variant=='fixed_path' else None);recordings.append((episode,trace,report,scene,variant))
        except (RuntimeError,ValueError) as error:report=dict(variant=variant,execution_passed=False,full_variant_pipeline_executed=False,failure=str(error))
        report['trial_wall_time_s']=perf_counter()-start;write_json(folder/'result.json',report);results[variant]=report;write_json(out/'ablation-results.json',dict(task=task_id,variants=results,nominal_trials_per_variant=1,statistical_success_rate_claimed=False))
    if web_directory is not None:
        from .full_view import export_full
        for episode,trace,report,scene,variant in recordings:export_full(episode,trace,report,scene,web_directory,variant=variant,ablation_results=results)
    return results
