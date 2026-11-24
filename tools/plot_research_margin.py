"""Plot all recorded source-margin challenge signals with a common native clock."""
import json
from pathlib import Path
import shutil
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


if __name__=='__main__':
    root=Path('outputs/research-study/source-margin-v1');rows={mode:json.loads((root/mode/'slow_up4/summary.json').read_text()) for mode in ['on','off']}
    plt.rcParams.update({'svg.fonttype':'none','font.size':10})
    fig,axes=plt.subplots(2,2,figsize=(10,6.5),layout='constrained')
    a=rows['on']['response'];t=np.asarray(a['time_s']);axes[0,0].plot(t,np.linalg.norm(a['force_world_N'],axis=1),color='#546675',lw=2)
    axes[0,0].set_ylabel('Applied force magnitude (N)')
    for mode,color in [('on','#148576'),('off','#ca733d')]:
        d=rows[mode]['response'];t=np.asarray(d['time_s']);label='Source robustness '+mode.upper()
        axes[0,1].plot(t,np.rad2deg(d['rotation_error_rad']),color=color,lw=2,label=label)
        axes[1,0].plot(t,d['height_m'],color=color,lw=2,label=label)
        axes[1,1].step(t,d['supporting_contacts'],where='post',color=color,lw=1.5,label=label)
    axes[0,1].set_ylabel('Orientation error (deg)');axes[0,1].axhline(np.rad2deg(.06),color='.5',ls='--',lw=1);axes[0,1].legend(fontsize=8)
    axes[1,0].set_ylabel('Platform-origin height (m)');axes[1,0].axhline(.52,color='.5',ls='--',lw=1)
    axes[1,1].set_ylabel('Load-carrying tool contacts');axes[1,1].set_yticks([0,1,2,3]);axes[1,1].set_ylim(-.1,3.1)
    for ax in axes.flat:
        ax.axvspan(4,12,color='#f5dbab',alpha=.3);ax.axvspan(12,16,color='#ccdfed',alpha=.25)
        ax.set_xlabel('Native execution time (s)');ax.set_xlim(0,16);ax.grid(alpha=.2)
    ratio=rows['on']['initial_certificate']['load_to_margin_ratio']
    fig.suptitle(f'Matched 4 N load at {100*ratio:.1f}% of the ON static margin\nSame scene, controller and applied wrench; source refinement alone is ablated',fontsize=12)
    out=Path('docs/images/research-source-margin-response.svg');fig.savefig(out);plt.close(fig)
    out.write_text('\n'.join(line.rstrip() for line in out.read_text().splitlines())+'\n')
    shutil.copyfile(out,'web/data/research-source-margin-response.svg')
