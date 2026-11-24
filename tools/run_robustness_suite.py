"""Same saved plan/controller, predefined external pulses; retain successes and failures."""
import argparse,json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from contact_transfer.mobile_panel import run_mobile
from contact_transfer.mobile_view import load_saved_mobile,export_mobile_view
from contact_transfer.perturbations import WrenchPulse


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--plan',default='outputs/mobile_panel');parser.add_argument('--output',default='outputs/robustness');parser.add_argument('--amplitudes',nargs='+',type=float,default=[2,8,25]);args=parser.parse_args()
    plan=load_saved_mobile(args.plan);root=Path(args.output);root.mkdir(parents=True,exist_ok=True);rows=[]
    for amp in args.amplitudes:
        pulse=WrenchPulse('panel',16.,.5,(0.,amp,0.));out=root/f'push_{amp:g}N'
        p,t,r,f=run_mobile(out,plan=plan,disturbances=[pulse],progress=lambda stage,**kw:print(amp,stage,kw,flush=True))
        desired=np.array([__import__('contact_transfer.mobile_panel',fromlist=['object_reference']).object_reference(p.case,float(tm))[0] for tm in t.time]);err=np.linalg.norm(t.object_pose[:,0,:3]-desired[:,:3],axis=1)
        window=(t.time>=16)&(t.time<=20);settled=(t.time>=18)&(t.time<=20)
        recovery=bool(np.max(err[settled])<.015 and np.all(f[window,:2]))
        r['external_disturbances']=[pulse.to_dict()];r['robustness']=dict(force_peak_N=amp,pulse_start_s=16.,pulse_duration_s=.5,impulse_Ns=amp*.25,peak_pose_error_m=float(err[window].max()),post_pulse_error_m=float(err[settled].max()),recovery_passed=recovery,all_task_checks_passed=r['execution_passed'],scope='one deterministic world-Y pulse at COM; same plan/gains, not a robustness certificate')
        (out/'report.json').write_text(json.dumps(r,indent=2)+'\n');rows.append(r['robustness']);print('RESULT',rows[-1],flush=True)
        data=export_mobile_view(p,t,r,f,root/'viewer');data.update(id=f'mobile_push_{amp:g}',title=f'외력 {amp:g} N · RBY1 + Panda',external_disturbances=[pulse.to_dict()]);data['description']+=' Prespecified external world-Y sin-squared force pulse at panel COM, 16–16.5 s. Measured recovery and capability checks are reported separately.'
        target=Path('web/data')/(data['id']+'.json');target.write_text(json.dumps(data,separators=(',',':'),allow_nan=False))
        index=Path('web/data/index.json');catalog=json.loads(index.read_text());catalog['models']=[m for m in catalog['models'] if m['id']!=data['id']]+[{k:data[k] for k in ['id','title','engine','dof','floating_base']}];index.write_text(json.dumps(catalog,indent=2)+'\n')
    (root/'summary.json').write_text(json.dumps(dict(protocol='predefined amplitude sweep, world-Y, same saved plan and gains',trials=rows),indent=2)+'\n')
if __name__=='__main__':main()
