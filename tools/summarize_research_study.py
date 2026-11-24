"""Publish measured current-study tables without mixing historical trials."""
import argparse
import json
from pathlib import Path

from contact_transfer.full_recording import write_json


def read(path):return json.loads(path.read_text())


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--root',type=Path,default=Path('outputs/research-study'));parser.add_argument('--allow-incomplete',action='store_true');args=parser.parse_args();root=args.root
    output=dict(schema_version=1,release='0.2.0a2',tasks={},contact_types={},target_probes={},full_load={},source_margin=[])
    lines=['# Measured research-study results — 24 November 2026','','One trial per condition. Source model extensions and literal-equation failures are described in [the validation report](research-validation-2026-11-24.md).','']
    count=0
    for task in ['panel','box','assembly']:
        folder=root/'full-common'/task;output['tasks'][task]={};lines.extend([f'## {task.title()} ablations','','| Condition | Native checks | Peak position error | Task goal |','|---|---|---:|---|'])
        for variant in ['full','no_source_refinement','fixed_contact_types','no_target_robustness','fixed_path','external_push']:
            path=folder/variant/'result.json'
            if not path.exists():
                if args.allow_incomplete:continue
                raise RuntimeError(f'Missing completed condition: {path}')
            d=read(path);maximum=max((x['maximum_position_error_m'] for x in d.get('object_tracking',{}).values()),default=None)
            output['tasks'][task][variant]=dict(execution_passed=d['execution_passed'],task_passed=d.get('task_passed'),maximum_position_error_m=maximum,sequence_objective=d.get('selection',{}).get('objective'),source_certificates_complete=d.get('equation_contract',{}).get('source_certificates_complete'),source_model_extension=d.get('equation_contract',{}).get('source_site_model_extension'))
            lines.append(f'| {variant} | {"PASS" if d["execution_passed"] else "FAIL"} | {1000*maximum:.3f} mm | {d.get("task_passed")} |' if maximum is not None else f'| {variant} | REJECTED | — | — |')
            count+=bool(d.get('new_physical_trial_executed'))
        alternatives=[]
        for path in sorted((root/'types-v1'/task).glob('*/result.json')):
            d=read(path);s=d.get('selection',{});alternatives.append(dict(forced_types=d['forced_types'],comparison_stage=d.get('comparison_stage'),execution_passed=d['execution_passed'],new_physical_trial_executed=d['new_physical_trial_executed'],selection=dict(objective=s.get('objective')),failure=d.get('failure'),candidate_rejections=d.get('candidate_rejections'),object_tracking=d.get('object_tracking')));count+=bool(d['new_physical_trial_executed'])
        output['contact_types'][task]=alternatives
        probe=root/'probes-v1'/task/'summary.json';loaded=root/'full-load-v1'/task/'summary.json'
        if probe.exists():
            d=read(probe);expected=2*len(d['protocol']['cases'])
            for trial in d['results']:
                dataset=f'research_probe_{task}_{trial["mode"]}'+('_opposite' if trial['case']=='opposite_interior' else '')
                if trial['case'] in {'on_interior','opposite_interior'} and Path('web/data',dataset+'.json').exists():trial['recording_id']=dataset
            if not args.allow_incomplete and len(d['results'])!=expected:raise RuntimeError(f'Incomplete probe {task}')
            output['target_probes'][task]=d;count+=len(d['results'])
        if loaded.exists():
            d=read(loaded)
            if not args.allow_incomplete and len(d['results'])!=4:raise RuntimeError(f'Incomplete full-load task {task}')
            output['full_load'][task]=d;count+=len(d['results'])
        elif not args.allow_incomplete:raise RuntimeError(f'Missing full-load task {task}')
    lines.extend(['','The full baseline and nominal-only target use different objectives; their raw objective values are not treated as a like-for-like cost comparison. Fixed types equal the selected candidate. Fixed path is a prescribed-path allocation baseline.','','## Forced contact types','','| Task / types | Stage | Native outcome | Full objective | Evidence |','|---|---|---|---:|---|'])
    for task,rows in output['contact_types'].items():
        for d in rows:
            stage=d['comparison_stage'];outcome=('PASS' if d['execution_passed'] else 'FAIL') if d['new_physical_trial_executed'] else 'Not executed';cost=d['selection']['objective'];evidence=d['failure'] or 'Same objective, assignment, source and controller; see the native report'
            if d.get('candidate_rejections'):evidence+='; '+str(d['candidate_rejections'][-1].get('reason',''))
            lines.append(f'| {task} / {"/".join(d["forced_types"])} | {stage} | {outcome} | {cost if cost is not None else "—"} | {evidence} |')
    lines.extend(['','Local IK / trust-region rejection is not global infeasibility. The assembly rank obstruction concerns the required free-body wrench image.','','## Mid-task matched probes','','Certified fixed-pose margin lower bounds are reported below. A failed maximum-margin solve followed by rho=0 feasibility does not establish zero physical capacity.','','| Task | ON lower bound | OFF lower bound | ON / OFF challenge outcome |','|---|---:|---:|---|'])
    for task,d in output['target_probes'].items():
        protocol=d['protocol'];pair=[r for r in d['results'] if r['case']=='on_interior'];status=' / '.join(r['outcome'] for r in pair)
        lines.append(f'| {task} | {protocol["rho_on_optimized_and_independently_checked"]:.6g} | {protocol["rho_off_independently_checked"]:.6g} | {status} |')
    lines.extend(['','## Full tasks with a mid-task load interval','','The six-second inserted hold has matched zero-load controls. The 80% ON load is checked at the optimized reference pose. Both tasks then continue to their original goals; this is a measured closed-loop test, not a dynamic certificate.','','| Task | Robustness | Pause load | Native checks | Hold peak position error | Recovered during hold |','|---|---|---|---|---:|---|'])
    for task,d in output['full_load'].items():
        for r in d['results']:
            a=r['hold_response'];lines.append(f'| {task} | {r["mode"].upper()} | {r["case"]} | {"PASS" if r["execution_passed"] else "FAIL"} | {1000*a["hold_peak_position_error_m"]:.3f} mm | {a["hold_recovered"]} |')
    lines.extend(['','The box opposite-direction probe fails ON tracking despite an inside-box 80% load; contact and force/torque admissibility still pass. This retained counterexample limits a closed-loop robustness claim and motivates measured-load redistribution/control qualification.','','## Source-refinement robustness contrast','','| Load | Refinement | Load / own static margin | Native checks | Measured outcome | Tail position error |','|---|---|---:|---|---|---:|'])
    for mode in ['on','off']:
        for case in ['nominal','common_up1','slow_up4']:
            d=read(root/'source-margin-v1'/mode/case/'summary.json');output['source_margin'].append(d);count+=1
            lines.append(f'| {case} | {mode.upper()} | {d["initial_certificate"]["load_to_margin_ratio"]:.3f} | {"PASS" if d["execution_passed"] else "FAIL"} | {d["outcome"]} | {1000*d["response"]["tail_position_error_m"]:.3f} mm |')
    lines.extend(['','![Matched source robustness response](images/research-source-margin-response.svg)','','The primary 4 N load is 83.63% of the ON certified static box and 132.69% of the OFF certified lower-bound box. ON recovers; OFF retains a finite-time loss of support. Both pass no-load and common 1 N controls. The source protocol is an exploratory scene/rate design whose historical alternatives remain available; it is not a statistical or asymptotic-stability result.',''])
    output['new_native_trials_counted']=count;write_json(root/'study-summary.json',output);write_json(Path('web/data/research-study-summary.json'),output)
    Path('docs/research-study-results-2026-11-24.md').write_text('\n'.join(lines));print(count,'new physical trials counted')


if __name__=='__main__':main()
