"""Run three passive-object tasks and contact-disabled controls, export measured playback."""
import argparse
import json
from pathlib import Path
from contact_transfer.articulated import ArticulatedCase,plan_articulated,run_articulated
from contact_transfer.articulated_view import export_articulated_view

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',default='outputs/articulated_suite');parser.add_argument('--viewer',default='web');args=parser.parse_args()
    root=Path(args.output);root.mkdir(parents=True,exist_ok=True);results={}
    for id in ['hinged_door','folding_panel','drawer']:
        plan,trace,report,flags=run_articulated(id,root/id)
        export_articulated_view(plan,trace,report,flags,args.viewer)
        _,_,control,_=run_articulated(id,root/(id+'_no_contact'),plan=plan,contacts_enabled=False)
        results[id]=dict(baseline=report,contact_disabled=control)
        print(id,report['execution_passed'],control['execution_passed'],flush=True)
    case=ArticulatedCase.load('hinged_door');case.data['timestep']/=2
    _,_,report,_=run_articulated('hinged_door',root/'door_half_dt',plan=plan_articulated(case));results['door_half_dt']=report
    (root/'summary.json').write_text(json.dumps(results,indent=2)+'\n')
    assert all(results[id]['baseline']['execution_passed'] and not results[id]['contact_disabled']['execution_passed'] for id in ['hinged_door','folding_panel','drawer'])
    assert report['execution_passed']
if __name__=='__main__':main()
