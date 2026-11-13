let request,token=0;
const node=(tag,text)=>{const e=document.createElement(tag);if(text!==undefined)e.textContent=text;return e;};
function table(parent,headers){const t=node('table'),h=node('tr');headers.forEach(v=>h.append(node('th',v)));const head=node('thead');head.append(h);t.append(head,node('tbody'));parent.append(t);return t;}
function row(t,values){const r=node('tr');values.forEach(v=>{const d=node('td');typeof v==='string'?d.textContent=v:d.append(v);r.append(d);});t.tBodies[0].append(r);return r;}
function note(parent,text){const p=node('p',text);p.className='fine';parent.append(p);}
function link(id,text){const a=node('a',text+' ↗');a.href='?model='+id+'#algorithm-comparison-results';return a;}
const number=(v,digits=3)=>v===null||v===undefined?'—':Number(v).toFixed(digits);

export async function showAlgorithmComparisons(data){
 const current=++token;let root=document.getElementById('algorithm-comparison-results');
 if(!root){root=node('div');root.id='algorithm-comparison-results';document.getElementById('experiment-results').append(root);}
 root.hidden=!data.full_task_recording&&!data.comparison_recording;if(root.hidden)return;
 try{
  request??=fetch(new URL('./data/algorithm-comparisons.json',document.baseURI)).then(r=>{if(!r.ok)throw Error('No completed comparison evidence');return r.json();});
  const study=await request;if(current!==token)return;root.replaceChildren();
  const task=data.scenario?.startsWith('full_')?data.scenario.slice(5):data.id.startsWith('counterfactual_')?data.id.split('_')[1]:null;
  if(task&&study.contact_types?.[task]){
   const group=study.contact_types[task];root.append(node('h4','Alternative contact types: complete-sequence comparison'));
   const t=table(root,['Contact type','Stage','Execution','Sequence objective','Maximum position error / evidence']);
   row(t,[group.selected_types.join('/'),'Selected method','PASS',number(group.selected_sequence_objective),link('full_'+task,'Replay selected method')]);
   for(const v of group.counterfactuals||[]){const detail=v.failure||`${number(1000*v.maximum_position_error_m,2)} mm · ${v.failed_checks.join(', ')||'All criteria passed'}`;row(t,[v.types.join('/'),v.stage==='planning'?'Rejected during planning':'Fresh native physical execution',v.execution_passed?'PASS':'FAIL',number(v.sequence_objective),v.recording_id?link(v.recording_id,detail):detail]);}
   note(root,'Source refinement, masses, friction, interface limits, arm assignment, and controller settings are matched while the contact type is forced. Planning rejection is distinct from physical execution failure; local NLP failure is not proof of global infeasibility.');
   const typeFigure=node('img');typeFigure.src='./data/type-counterfactual-comparison.svg';typeFigure.alt='Sequence-objective increase and maximum position error for panel PPP and box PPG';typeFigure.className='comparison-figure';root.append(typeFigure);
   root.append(node('h4','Independent interface capacity at fixed material contacts and object pose'));
   const c=table(root,['Contact type','Maximum ρ','Frozen-keyframe objective','Minimum joint-torque headroom at common ρ = 0.1','Outcome']);
   for(const v of group.rows)row(c,[v.types.join('/'),number(v.rho),number(v.frozen_keyframe_cost),v.common_rho_metrics?number(v.common_rho_metrics.minimum_joint_torque_margin_Nm,2)+' Nm':'—',v.failure||`${v.planning_passed?'PASS':'FAIL'} · ${v.solver_status}`]);
   note(root,'Material contact sites and object pose are fixed. IK, reactions, and torques are recomputed for each physical interface. Larger ρ and smaller objective values are preferred. Frozen-keyframe and complete-sequence objectives are distinct quantities.');
  }
  if(study.source_robustness){
   const group=study.source_robustness;root.append(node('h4','Stationary panel diagnostic · box-vertex robustness ON/OFF'));
   note(root,'Three Panda arms support a free panel with identical spherical tools. ON enforces eight vertices of the prescribed task-force disturbance box; OFF enforces nominal gravity equilibrium. Contact positions alone are transferred at scale four; target posture, reactions, and torques are recomputed. Target material contacts are fixed to isolate source refinement.');
   const t=table(root,['External force / direction','Polytope','Independent source / target ρ','Outcome','Maximum position / orientation error','Loaded-contact retention','Replay']);
   for(const v of group.trials){const direction=v.force_N===0?'None':`${Math.abs(v.force_N)} N · ${v.force_N<0?'upward':'downward'}`;const tr=row(t,[direction,v.robustness_enabled?'ON':'OFF',`${number(v.source_rho)} / ${number(v.target_rho)}`,v.execution_passed?'PASS':'FAIL',`${number(1000*v.maximum_position_error_m,2)} mm / ${number(v.maximum_rotation_error_rad)} rad`,number(100*v.minimum_contact_fraction,1)+'%',link(v.recording_id,'Replay force and contacts')]);tr.dataset.robustness=String(v.robustness_enabled);tr.dataset.force=String(v.force_N);tr.dataset.passed=String(v.execution_passed);}
   note(root,'One trial per force condition. Common criteria are 8 mm position error, 0.06 rad orientation error, and 99% required contact retention. The same edge receives a one-second sin² pulse at 4–5 s. No stability guarantee is made outside the certified static box. Static wrench feasibility is not a closed-loop stability proof.');
   const forceFigure=node('img');forceFigure.src='./data/source-robustness-comparison.svg';forceFigure.alt='Maximum orientation error and contact retention for every matched source-robustness condition';forceFigure.className='comparison-figure';root.append(forceFigure);
   note(root,`The evaluation set comprises three-axis forces at the declared application point. Common force-box D = ${group.protocol.target_force_box_scales_N.join(', ')} N · application point = [${group.protocol.force_application_point_body_m.join(', ')}] m · shared source geometry weight = ${group.protocol.geometry_weight}. Fixed E, D, controller settings, and plan IDs are recorded; the controller is not retuned across force conditions.`);
  }
  note(root,'Implementation scope: contracted groups are supplied; learned inference is not executed. Complete-motion records use restricted contact candidates and midpoint variants. These controlled diagnostics do not establish literal execution of every manuscript equation.');
 }catch{if(current===token)root.hidden=true;request=undefined;}
}
