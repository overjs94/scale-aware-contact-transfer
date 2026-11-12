#!/usr/bin/env python3
"""Standalone figures from the complete published comparison ledger."""
import json,shutil
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

if __name__=='__main__':
 data=json.loads(Path('docs/algorithm-comparison-results.json').read_text());out=Path('docs/images');out.mkdir(exist_ok=True);plt.rcParams.update({'svg.fonttype':'none','font.size':10})
 fig,axes=plt.subplots(1,2,figsize=(10,3.8),layout='constrained')
 for enabled,color,label in [(True,'#148576','Polytope ON'),(False,'#c36c32','Nominal only')]:
  rows=sorted((r for r in data['source_robustness']['trials'] if r['robustness_enabled']==enabled),key=lambda r:r['force_N']);x=[r['force_N'] for r in rows]
  axes[0].plot(x,[np.degrees(r['maximum_rotation_error_rad']) for r in rows],'o-',color=color,label=label);axes[1].plot(x,[100*r['minimum_contact_fraction'] for r in rows],'o-',color=color,label=label)
 axes[0].set_yscale('log');axes[0].axhline(np.degrees(.06),color='.4',ls='--',lw=1,label='Pass threshold');axes[0].set_ylabel('Maximum orientation error (deg)');axes[0].legend(fontsize=8)
 axes[1].axhline(99,color='.4',ls='--',lw=1);axes[1].set_ylabel('Minimum loaded-contact retention (%)');axes[1].set_ylim(0,102)
 for ax in axes:ax.set_xlabel('Signed force (N): negative = upward');ax.set_xticks([-12,-8,-4,0,4,8,12]);ax.grid(alpha=.2)
 fig.suptitle('Same three-arm controller and edge pulse; one trial per condition');fig.savefig(out/'source-robustness-comparison.svg');plt.close(fig)
 fig,axes=plt.subplots(1,2,figsize=(10,3.8),layout='constrained');names=[];delta=[];selected_error=[];other_error=[]
 baseline=json.loads(Path('docs/full-scenario-results.json').read_text())['cases']
 for task in ('panel','box'):
  group=data['contact_types'][task];other=group['counterfactuals'][0];names.append(task);delta.append(other['sequence_objective']-group['selected_sequence_objective']);selected_error.append(1000*baseline[task]['full']['maximum_body_position_error_m']);other_error.append(1000*other['maximum_position_error_m'])
 axes[0].bar(names,delta,color=['#c36c32','#aa5b78']);axes[0].set_ylabel('Alternative minus selected sequence cost');axes[0].set_title('Higher cost is worse')
 x=np.arange(len(names));axes[1].bar(x-.18,selected_error,.36,label='Selected',color='#148576');axes[1].bar(x+.18,other_error,.36,label='Forced alternative',color='#c36c32');axes[1].axhline(8,color='.4',ls='--',lw=1);axes[1].set_xticks(x,names);axes[1].set_ylabel('Maximum body position error (mm)');axes[1].legend(fontsize=8)
 fig.suptitle('Panel PPP and box PPG: complete optimized plans, fresh native executions');fig.savefig(out/'type-counterfactual-comparison.svg');plt.close(fig)
 for name in ('source-robustness-comparison.svg','type-counterfactual-comparison.svg'):shutil.copyfile(out/name,Path('web/data')/name)
