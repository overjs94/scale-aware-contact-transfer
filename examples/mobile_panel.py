"""Recompute wheel-driven RBY1/Panda handover, then export measured playback."""
from contact_transfer.mobile_panel import run_mobile
from contact_transfer.mobile_view import export_mobile_view
if __name__=='__main__':
    plan,trace,report,flags=run_mobile(progress=lambda stage,**kw:print(stage,kw,flush=True))
    export_mobile_view(plan,trace,report,flags,'web')
    print({k:report[k] for k in ['execution_passed','base_travel_m','workspace_comparison','jig_support_final_N']})
    if not report['execution_passed']:raise SystemExit(1)
