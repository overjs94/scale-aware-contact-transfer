#!/usr/bin/env python3
"""Verify matched native trials and publish all sustained-load outcomes."""
import argparse
import hashlib
import json
from pathlib import Path
from contact_transfer.margin_hold_study import CASES,EVALUATION_CASES
from contact_transfer.full_recording import write_json


def summarize(root,flat_root,web,high_root=None):
    rows=[];identities={};settings=set()
    for mode in ['on','off']:
        pairs=set()
        for case in EVALUATION_CASES:
            path=root/mode/case/'summary.json';row=json.loads(path.read_text())
            raw=root/mode/case/'native/execution.npz'
            assert row['native_trace_sha256']==hashlib.sha256(raw.read_bytes()).hexdigest()
            recording=json.loads((web/'data'/(row['recording_id']+'.json')).read_text())
            row['external_disturbances']=recording['external_disturbances']
            assert row['native_plan_id']==recording['provenance']['plan_id']
            assert row['native_case_id']==recording['provenance']['case_id']
            if mode=='on':assert row['initial_certificate']['inside_initial_static_box']
            pairs.add((row['native_case_id'],row['native_plan_id']))
            settings.add(json.dumps(row['controller_configuration'],sort_keys=True));rows.append(row)
        assert len(pairs)==1,'A condition changed the native plan'
        identities[mode]=list(pairs.pop())
    assert len(settings)==1,'ON and OFF controller settings differ'
    assert identities['on'][0]==identities['off'][0],'Physical cases differ'
    for case in EVALUATION_CASES:
        on,off=[next(r for r in rows if r['case']==case and r['robustness_enabled']==enabled) for enabled in [True,False]]
        a,b=[json.loads((web/'data'/(r['recording_id']+'.json')).read_text()) for r in [on,off]]
        assert a['external_disturbances']==b['external_disturbances']
        assert on['response']['time_s']==off['response']['time_s']
        assert on['response']['force_world_N']==off['response']['force_world_N']
        if case in ['common_up1','common_up2']:assert off['initial_certificate']['inside_initial_static_box']
    historical=[]
    for path in sorted(flat_root.glob('*/sustained_up4/summary.json')):
        historical.append(json.loads(path.read_text()))
    tall=[r for r in rows if r['case']=='sustained_up4']
    compact=[json.loads(path.read_text()) for path in sorted((root.parent/'margin_compact_v1').glob('*/*/summary.json'))]
    on,off=[next(r for r in rows if r['case']=='slow_up4' and r['robustness_enabled']==v) for v in [True,False]]
    protocol=json.loads((root/'protocol.json').read_text());protocol['protocol']['cases']={k:CASES[k] for k in EVALUATION_CASES};protocol['protocol']['primary_comparison']='slow_up4'
    result=dict(schema_version=1,protocol=protocol,trials=rows,
        matched_plan_ids=identities,controller_settings_equal=True,physical_case_ids_equal=True,
        primary_comparison=dict(case='slow_up4',on_bounded_tracking=on['outcome']=='bounded_tracking',
            off_persistent_loss_of_support=off['outcome']=='persistent_loss_of_support',
            demonstrated=on['outcome']=='bounded_tracking' and off['outcome']=='persistent_loss_of_support',
            mathematical_divergence_claimed=False),
        flat_panel_exploratory_trials=historical,
        tall_payload_exploratory_trials=tall,
        compact_payload_exploratory_trials=compact,
        exploratory_design=True,
        design_history='Exploratory design. Flat panel recovered; fast loading of a tall payload lost support in both modes. Compact payload trials are retained. Slow loading of the tall payload separates ON/OFF; all tested rates and object variants are reported.',
        scope='Initial-pose static certificate plus finite-time native response; no asymptotic stability theorem or statistical success-rate claim')
    write_json(web/'data/margin-hold-comparison.json',result)
    write_json(Path('docs/margin-hold-results.json'),result)
    print(json.dumps(dict(primary=result['primary_comparison'],trials=len(rows),flat_exploratory=len(historical))))
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--root',type=Path,default=Path('outputs/margin_platform_v1'))
    p.add_argument('--flat-root',type=Path,default=Path('outputs/margin_hold_v1'))
    p.add_argument('--web',type=Path,default=Path('web'));a=p.parse_args();summarize(a.root,a.flat_root,a.web)
