"""Standalone measured-response figure; no synthetic response curves."""
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def main():
    old=json.loads(Path('outputs/research-study/probes-v1/box/on/opposite_interior/result.json').read_text())['robustness_probe']['response']
    cases=[('Original · fixed 24 N',old,'#bc4349')]
    for mode,label,color,root in [('preload','Preload only · 30 N','#d08d2d','controller-study'),('allocation','Allocation · fixed 24 N','#3879b5','controller-final'),('adaptive','Allocation · adaptive preload','#267d60','controller-final')]:
        record=json.loads((Path('outputs')/root/'box'/mode/'opposite/result.json').read_text());cases.append((label,record['response'],color))
    fig,(ax,force)=plt.subplots(2,1,figsize=(9,5.7),sharex=True,gridspec_kw={'height_ratios':[3,1]},layout='constrained')
    for label,response,color in cases:ax.plot(response['time_s'],1000*np.array(response['position_error_m']),label=label,color=color,lw=1.8)
    ax.axhline(8,color='#444',ls='--',lw=1,label='Shared tracking limit · 8 mm');ax.set(ylabel='Maximum object-frame position error [mm]',title='Free articulated box · same 14.755 N downward load');ax.legend(frameon=False,ncols=2,fontsize=9);ax.grid(alpha=.2)
    t=np.linspace(0,9,901);elapsed=t-4;edge=np.minimum(elapsed,3-elapsed);scale=np.where((elapsed>=0)&(elapsed<=3),np.where(edge<.5,np.sin(.5*np.pi*edge/.5)**2,1),0)
    force.fill_between(t,0,14.7545885277*scale,color='#637789',alpha=.5);force.set(ylabel='Applied load [N]',xlabel='Simulation time [s]',xlim=(0,9));force.grid(alpha=.2)
    folder=Path('docs/images');folder.mkdir(exist_ok=True);fig.savefig(folder/'controller-allocation-response.svg');fig.savefig(folder/'controller-allocation-response.png',dpi=180);fig.savefig('web/data/controller-allocation-response.svg');plt.close(fig)


if __name__=='__main__':main()
