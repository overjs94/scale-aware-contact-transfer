"""Isolated optimization/simulation worker used by the local web API."""
import json
from pathlib import Path
import sys
from contact_transfer.three_arm import run_three_arm
from contact_transfer.three_arm_view import export_three_arm_view

root = Path(sys.argv[1]).resolve()
def status(stage, **extra):
    tmp = root/'status.tmp'
    tmp.write_text(json.dumps({'state':'running','stage':stage, **extra}))
    tmp.replace(root/'status.json')
try:
    status('optimization')
    case_id=json.loads((root/'case.json').read_text()).get('id')
    if case_id in {'free_box_lid','free_folding_object','free_portable_drawer'}:
        from contact_transfer.free_articulated import run_free
        from contact_transfer.free_articulated_view import export_free
        duration=json.loads((root/'case.json').read_text())['duration']
        plan,trace,report,flags=run_free(case_id,root/'result',case_path=root/'case.json',progress=lambda t,d:status('simulation',time_s=t,duration_s=duration))
        status('export')
        data=export_free(plan,trace,report,flags,root/'viewer')
    elif case_id in {'hinged_door','folding_panel','drawer'}:
        from contact_transfer.articulated import run_articulated
        from contact_transfer.articulated_view import export_articulated_view
        duration=json.loads((root/'case.json').read_text())['duration']
        plan,trace,report,flags=run_articulated(case_id,root/'result',case_path=root/'case.json',progress=lambda t,d:status('simulation',time_s=t,duration_s=duration))
        status('export')
        data=export_articulated_view(plan,trace,report,flags,root/'viewer')
    elif case_id=='mobile_panel':
        from contact_transfer.mobile_panel import run_mobile
        from contact_transfer.mobile_view import export_mobile_view
        plan,trace,report,flags=run_mobile(root/'result',case_path=root/'case.json',progress=status)
        status('export')
        data=export_mobile_view(plan,trace,report,flags,root/'viewer')
    elif case_id=='panel_fixture':
        from contact_transfer.panel_fixture import run_panel
        from contact_transfer.panel_view import export_panel_view
        plan,trace,report,flags=run_panel(root/'result',case_path=root/'case.json',progress=status)
        status('export')
        data=export_panel_view(plan,trace,report,flags,root/'viewer')
    elif case_id=='table_assembly':
        from contact_transfer.table_assembly import run_table_assembly
        from contact_transfer.assembly_view import export_assembly_view
        plan,trace,report,flags=run_table_assembly(root/'result',case_path=root/'case.json',progress=status)
        status('export')
        data=export_assembly_view(plan,trace,report,flags,root/'viewer')
    else:
        plan, trace, seal, report = run_three_arm(root/'result', case_path=root/'case.json', progress=status)
        status('export')
        data = export_three_arm_view(plan, trace, report, root/'viewer')
    for spec in data['instances']:
        spec['manifest'] = '/api/experiments/'+root.name+'/assets/'+spec['manifest'].removeprefix('./')
    (root/'viewer/result.json').write_text(json.dumps(data, separators=(',',':'),allow_nan=False))
    status('complete', state='completed', report=report, recording='/api/experiments/'+root.name+'/assets/result.json')
except Exception as error:
    status('failed',state='failed',error=type(error).__name__+': '+str(error))
    raise
