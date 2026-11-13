"""Reproduce the primary single-panel baseline and its URDF/mesh recording."""
from contact_transfer.panel_fixture import run_panel
from contact_transfer.panel_view import export_panel_view
if __name__=='__main__':
    plan,trace,report,flags=run_panel(progress=lambda stage,**kw:print(stage,kw,flush=True))
    export_panel_view(plan,trace,report,flags,'web')
    print('execution_passed:',report['execution_passed'])
    if not report['execution_passed']:raise SystemExit(1)
