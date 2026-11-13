// Summaries of saved evidence. Never infer success from animation or solver status.
const labels={full:'Selected method',no_source_refinement:'Without source refinement',fixed_contact_types:'Fixed contact types',fixed_path:'Fixed path',no_arm_force_feedforward:'Without arm reaction feedforward',contact_disabled:'Contact disabled · negative control',external_push:'External disturbance · peak 5 N'};
const maxError=r=>Math.max(0,...Object.values(r.object_tracking||{}).map(e=>e.maximum_position_error_m));
const minContact=r=>Object.values(r.maintained_contact_fraction||{}).filter(v=>v!==null);
export function failureReasons(r){
  const reasons=[];
  if(r.failure)reasons.push('Planning/execution error: '+r.failure);
  if(r.tracking_passed===false){if(maxError(r)>.008)reasons.push(`Position error ${(maxError(r)*1000).toFixed(2)} mm / limit 8 mm`);const angle=Math.max(0,...Object.values(r.object_tracking||{}).map(e=>e.maximum_rotation_error_rad));if(angle>.06)reasons.push(`Orientation error ${angle.toFixed(3)} rad / 0.060 rad`);}
  for(const [id,j] of Object.entries(r.jaw_force||{}))if(!j.passed){const peak=Math.max(...(j.all_time_maximum_per_jaw_N||j.maximum_per_jaw_N));reasons.push(peak>j.limit_N?`${id}: jaw ${peak.toFixed(2)} N / ${j.limit_N} N`:`${id}: insufficient two-jaw contact retention`);}
  if(r.contact_retention_passed===false)reasons.push(`Required loaded-contact retention ${(100*Math.min(...minContact(r))).toFixed(1)}% / 99%`);
  if(r.sampled_interface_wrench_capability?.passed===false)reasons.push('Measured interface wrench outside its admissible set');
  if(r.task_passed===false)reasons.push('Task pose, assembly, or placement criterion not met');
  if(r.minimum_actuator_margin_Nm< -1e-8)reasons.push('Joint torque limit exceeded');
  if(r.reference_geometry?.passed===false)reasons.push('Reference-geometry check failed');
  if(Object.keys(r.unexpected_loaded_contact_penetrations_m||{}).length)reasons.push('Unintended loaded contact');
  return reasons;
}
const el=(tag,text)=>{const n=document.createElement(tag);if(text!==undefined)n.textContent=text;return n;};
function table(headers){const t=el('table'),h=el('tr');for(const s of headers)h.append(el('th',s));const head=el('thead');head.append(h);t.append(head,el('tbody'));return t;}
function row(t,values){const r=el('tr');for(const v of values){const c=el('td');typeof v==='string'?c.textContent=v:c.append(v);r.append(c);}t.tBodies[0].append(r);return r;}
function note(root,text){const p=el('p',text);p.className='fine';root.append(p);}
function reasonText(s){
 if(s.includes('Dominated cost bound'))return 'Pruned by a cost bound conditional on the realized keyframes; no physical infeasibility claim';
 if(s.includes('IK failed')||s.includes('IK target')||s.includes('unreachable'))return 'Local IK failed from the supplied seed; no global reachability claim';
 if(s.includes('Structurally insufficient'))return 'Insufficient rank for the free body’s six-dimensional task-wrench image';
 return s;
}
export function renderFullResults(data,root){
 root.replaceChildren();const variants=data.report.ablation_results||{},task=data.scenario.replace('full_','');
 root.append(el('h4',`Ablation study · ${Object.keys(variants).length} conditions`));
 const t=table(['Condition / replay','Outcome','Maximum position error','Loaded-contact retention','Failed check / comparison scope']);root.append(t);
 for(const [name,r] of Object.entries(variants)){
  const a=el('a',`${labels[name]||name} ↗`);a.href=`?model=full_${task}${name==='full'?'':'_'+name}#ablation-results`;a.title=name;
  const f=minContact(r),reasons=failureReasons(r),same=r.selection?.continuous_plan_reused;
  const detail=reasons.length?reasons.join(' · '):same?'Same continuous plan reused; fresh physical execution':name==='full'?'All execution criteria passed':'Execution criteria also passed in this condition';
  const tr=row(t,[a,r.execution_passed?'PASS':'FAIL',r.object_tracking?`${(1000*maxError(r)).toFixed(2)} mm`:'—',f.length?`${(100*Math.min(...f)).toFixed(1)}%`:'—',detail]);tr.dataset.variant=name;tr.dataset.passed=String(r.execution_passed);
 }
 note(root,'One trial per condition. FAIL denotes at least one unmet criterion. Disabling contact is a negative control; the disturbance case is a stress test. P retention is evaluated where the stored planned normal reaction exceeds 0.1 N.');
 const s=variants.full?.selection||data.report.selection;const heading=el('h4','Sequence-wide contact-type and arm-assignment selection');heading.id='selection-results';root.append(heading);
 const tool=data.provenance.canonical_point_tool;
 if(tool){const length=Math.hypot(...tool.ee_to_tip.slice(0,3).map(row=>row[3]));note(root,`P tool, end effector → contact point: ${(1000*length).toFixed(0)} mm · sphere radius ${(1000*tool.radius_m).toFixed(0)} mm. This is the physical URDF mount length. Minimum-length tool design has not been optimized.`);}
 const patterns=[...new Set([...(s.accepted_candidates||[]),...(s.pruned_candidates||[]),...(s.rejections||[])].map(c=>c.types.join('/')))];
 note(root,`G = bounded gripper-mediated contact; P = unilateral point contact with the common spherical tool. Allowed types: ${patterns.join(', ')} · including arm assignments: ${s.candidate_count} candidates. The second support is restricted to P. The objective contains no tool-count or type penalty; each selected type is fixed across the complete sequence.`);
 const fixed=variants.fixed_contact_types?.selection;
 if(fixed?.selected_types?.join('')===s.selected_types.join('')&&fixed.selected_assignment?.join('')===s.selected_assignment.join(''))note(root,`Selected types ${s.selected_types.join('/')} · arms ${s.selected_assignment.join('/')} match the supplied fixed-type baseline. This ablation alone does not demonstrate a type-selection benefit. Online tool changes or reassignment are not implemented.`);
 const candidates=table(['Contact type','Physical arm assignment','Result','Objective / lower bound','Evidence']);root.append(candidates);
 for(const c of s.accepted_candidates||[]){const chosen=c.types.join('')===s.selected_types.join('')&&c.assignment.join('')===s.selected_assignment.join('');row(candidates,[c.types.join('/'),c.assignment.join('/'),chosen?'Selected':'Plan accepted',c.objective.toFixed(3),chosen?'Selected continuous plan and execution passed':'Continuous plan accepted; separate physical execution not verified']);}
 for(const c of s.pruned_candidates||[])row(candidates,[c.types.join('/'),[...new Set(c.assignments.map(a=>a.join('/')))].join(' → '),'Pruned',c.sequence_cost_lower_bound.toFixed(3),reasonText(c.reason)]);
 for(const c of s.rejections||[])row(candidates,[c.types.join('/'),[...new Set(c.assignments.map(a=>a.join('/')))].join(' → '),'Rejected','—',reasonText(c.reason)]);
 note(root,'Lower objective values are preferred. Bounds are conditional on the realized keyframe solutions, not global IK optimality certificates. The current objective favors the larger admissible wrench sets of G, yielding the same types as the supplied baseline.');
}
