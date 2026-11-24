#!/usr/bin/env python3
"""Time-aligned scientific plots, including recovered and failed explorations."""
import json
from pathlib import Path
import shutil
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

if __name__=='__main__':
    data=json.loads(Path('docs/margin-hold-results.json').read_text())
    case=data['primary_comparison']['case'];rows=[r for r in data['trials'] if r['case']==case]
    plt.rcParams.update({'svg.fonttype':'none','font.size':10})
    fig,axes=plt.subplots(2,2,figsize=(11,6.7),layout='constrained')
    a=next(r for r in rows if r['robustness_enabled'])['response'];t=np.array(a['time_s'])
    axes[0,0].plot(t,np.linalg.norm(a['force_world_N'],axis=1),color='#526377',lw=2,label='Same applied force')
    axes[0,0].set_ylabel('External force magnitude (N)')
    for row in rows:
        a=row['response'];t=np.array(a['time_s']);label='Refinement ON' if row['robustness_enabled'] else 'Refinement OFF';color='#148576' if row['robustness_enabled'] else '#ca733d'
        axes[0,1].plot(t,np.rad2deg(a['rotation_error_rad']),color=color,label=label,lw=2)
        axes[1,0].plot(t,a['height_m'],color=color,label=label,lw=2)
        axes[1,1].step(t,a['supporting_contacts'],where='post',color=color,label=label,lw=1.5)
    axes[0,1].axhline(np.rad2deg(.06),color='.5',lw=1,ls='--',label='Tracking limit')
    axes[0,1].set_ylabel('Orientation error (deg)');axes[0,1].legend(fontsize=8)
    axes[1,0].axhline(.52,color='.5',lw=1,ls='--');axes[1,0].set_ylabel('Platform-origin height (m)')
    axes[1,1].set_ylabel('Tool contacts carrying load');axes[1,1].set_yticks([0,1,2,3]);axes[1,1].set_ylim(-.15,3.15)
    for ax in axes.flat:
        ax.axvspan(4,a['force_end_s'],color='#f5dbab',alpha=.25)
        ax.axvspan(a['force_end_s'],t[-1],color='#ccdfed',alpha=.2)
        ax.set_xlabel('Native execution time (s)');ax.grid(alpha=.2);ax.set_xlim(0,t[-1])
    on=next(r for r in rows if r['robustness_enabled'])
    fig.suptitle(f"Matched load at {100*on['initial_certificate']['load_to_margin_ratio']:.1f}% of the ON static margin\nSame controller; load removal and recovery are recorded",fontsize=12)
    out=Path('docs/images');out.mkdir(exist_ok=True);fig.savefig(out/'margin-hold-response.svg');fig.savefig(out/'margin-hold-response.png',dpi=160);plt.close(fig)
    fig,axes=plt.subplots(1,2,figsize=(11,4),layout='constrained')
    for name,records in [('Flat panel',data['flat_panel_exploratory_trials']),('Tall payload',data['tall_payload_exploratory_trials'])]:
        for row in records:
            a=row['response'];color='#148576' if row['robustness_enabled'] else '#ca733d';ax=axes[0 if name=='Flat panel' else 1]
            ax.plot(a['time_s'],np.rad2deg(a['rotation_error_rad']),color=color,label='ON' if row['robustness_enabled'] else 'OFF');ax.set_title(name);ax.set_xlabel('Time (s)');ax.set_ylabel('Orientation error (deg)');ax.grid(alpha=.2)
    for ax in axes:ax.legend();ax.axvspan(4,9,color='#f5dbab',alpha=.25)
    fig.suptitle('Exploratory pairs retained: recovered transients and shared failures',fontsize=12);fig.savefig(out/'margin-hold-exploratory.svg');plt.close(fig)
    for name in ['margin-hold-response.svg','margin-hold-exploratory.svg']:
        path=out/name;path.write_text('\n'.join(line.rstrip() for line in path.read_text().splitlines())+'\n')
        shutil.copyfile(path,Path('web/data')/name)
