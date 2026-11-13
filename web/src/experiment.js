import { categories, experimentPresets, libraryEntry } from './library.js';
import { renderFullResults } from './full-results.js';
import { showAlgorithmComparisons } from './algorithm-comparisons.js';
import { englishRecording } from './presentation-en.js';
const $=id=>document.getElementById(id);
let preset,job,timer,connected=false,presetId,presetToken=0;
const state=text=>{$('experiment-state').textContent=text;};
const stages={queued:'Queued',source_refinement_and_optimization:'Source contact refinement · target load and torque optimization',optimization:'SE(3) IK path · IPOPT reaction/torque optimization',simulation:'MuJoCo dynamic execution',validation:'Contact, insertion, and release checks',export:'Exporting recording',complete:'Computation complete',failed:'Failed',cancelled:'Cancelled'};
const articulated=()=>['hinged_door','folding_panel','drawer','free_box_lid','free_folding_object','free_portable_drawer'].includes(preset?.id);
const assembly=()=>preset?.id==='table_assembly';
const panel=()=>['panel_fixture','mobile_panel'].includes(preset?.id);
const mobile=()=>preset?.id==='mobile_panel';
const names={grip_and_support:'Tabletop grasp and spherical support',grasp_settled:'Maintain grasp',lift_tabletop:'Lift tabletop and leg',reorient_tabletop:'Reorient tabletop by 12°',approach_socket:'Approach socket',align_leg:'Align peg and socket',insert_leg:'Insert leg',assembled_and_released:'Release gripper and retain assembly'};
function articulatedFields(){
  const free=preset.id.startsWith('free_');const angular=!preset.id.includes('drawer');$('keyframe-inputs').replaceChildren();
  $('preset-tag').textContent=free?'PANDA × 3 · FREE ARTICULATED':'COMPONENT · SINGLE ARM';$('builder-title').textContent={hinged_door:'Door opening',folding_panel:'Hinged-panel unfolding',drawer:'Drawer opening',free_box_lid:'Free box · base grasp/support and lid opening',free_folding_object:'Free folding panel · support and unfolding',free_portable_drawer:'Free cabinet · support and drawer opening'}[preset.id];
  $('builder-note').textContent=free?'Free base and internal joint · G / P / G · dense contacts → representative contacts → three-arm realization':`Passive ${angular?'hinge':'rail'} · no object actuators · ${preset.tool==='G'?'native gripper':'spherical point tool'}`;
  $('tool-chips').replaceChildren();const chip=document.createElement('span');chip.textContent=free?'⋈ Base grasp · ● Base support · ⋈ Internal-link manipulation':preset.tool==='G'?'⋈ Gripper handle grasp':'● Spherical point contact';$('tool-chips').append(chip);
  $('mass-control').hidden=true;$('panel-mass').disabled=true;$('source-refinement-control').hidden=true;$('duration-control').hidden=false;$('assembly-duration').disabled=false;$('assembly-duration').min=12;
  $('planner-description').textContent=free?'Supplied link/functional groups of dense small-scale contacts are contracted into representative contacts. Positions alone are transferred at scale four; arm poses, reactions, torques, and free-base/internal-joint dynamics are matched. Robot contacts support the free body without a world-fixed joint, object actuator, or weld. Group IDs, assignment, and path are synthetic inputs.':'Optimize reactions and arm torques with passive-joint inertia, gravity, spring, and friction. Object poses are not prescribed and object joints are unactuated. Contacts, initial grasps, and joint paths are supplied; learned articulated contact contraction is not executed.';
  const scale=Number($('assembly-duration').value)/preset.duration;
  for(const [label,time,q] of [['Contact',0,preset.initial_coordinate],...(free?[['Base lift',preset.lift_end,preset.initial_coordinate]]:[]),['Start',preset.motion_start,preset.initial_coordinate],['Intermediate',(preset.motion_start+preset.motion_end)/2,(preset.initial_coordinate+preset.target_coordinate)/2],['Target',preset.motion_end,preset.target_coordinate],['Retained',preset.duration,preset.target_coordinate]]){
    const row=document.createElement('div');row.className='assembly-key';for(const text of [label,`${(time*scale).toFixed(1)} s`,`${(q*(angular?180/Math.PI:1000)).toFixed(1)} ${angular?'deg':'mm'}`]){const el=document.createElement('span');el.textContent=text;row.append(el);}$('keyframe-inputs').append(row);
  }
  $('keyframe-help').textContent='Initial contact → motion → hold · measured object-joint coordinates are replayed.';
  if(!free||$('contacts-json').dataset.preset!==preset.id){$('contacts-json').dataset.preset=preset.id;$('contacts-json').value=JSON.stringify(free?preset.dense_source_groups:{kind:preset.tool,link:'moving',point_link:preset.contact_object,inward_link:preset.inward_object},null,2);}$('contacts-json').readOnly=!free;$('contacts-file').disabled=!free;
}
function fields(){
  if(articulated())return articulatedFields();
  const isAssembly=assembly();const isPanel=panel();const detailed=isAssembly||isPanel;$('keyframe-inputs').replaceChildren();
  $('preset-tag').textContent=mobile()?'RBY1 + PANDA':'PANDA × 3';
  $('builder-title').textContent=mobile()?'Workspace extension · RBY1 + Panda':isPanel?'Panel transport, fixture placement, and release':isAssembly?'Tabletop–leg assembly':'Three Panda arms · panel transport';
  $('builder-note').textContent=mobile()?'Differential-drive mobile dual arm + fixed Panda · native base/object dynamics':isPanel?'Panda × 3 · G / P / P · one free panel and a fixed fixture':isAssembly?'Panda × 3 · 21 arm + 4 finger coordinates · two free objects':'Panda × 3 · 21 arm + 2 finger coordinates · one free object';
  $('tool-chips').replaceChildren();for(const title of mobile()?['⋈ RBY1 gripper','● RBY1 sphere support','● Fixed Panda support']:isPanel?['⋈ Edge gripper','● Left sphere support','● Right sphere support']:isAssembly?['⋈ Tabletop gripper','● Sphere support','⋈ Leg gripper']:['● Sphere · point','⋈ Panda hand','⊙ Vacuum seal']){const chip=document.createElement('span');chip.textContent=title;$('tool-chips').append(chip);}
  $('mass-control').hidden=detailed;$('panel-mass').disabled=detailed;
  $('source-refinement-control').hidden=!isPanel;
  $('duration-control').hidden=!detailed;$('assembly-duration').disabled=!detailed;
  $('assembly-duration').min=mobile()?30:24;
  $('planner-description').textContent=mobile()?'The fixed Panda transfers initial support and retreats. RBY1 drives 1.4 m and places/releases the panel on a fixture beyond fixed-base reach. Base and panel move through native dynamics. The straight path, modes, and initial grasp are supplied.':isPanel?'First refine synthetic contracted contacts at one-quarter scale. Transfer positions alone at scale four, then recompute reactions and torques for the 2 kg panel. Initial grasps and the SE(3) path are supplied. Fixture placement at 22 s → load transfer → all interfaces released at 26 s. No learned inference or hardware execution is claimed.':isAssembly?'Grasp/support and reorient a 2 kg tabletop, then insert a 0.4 kg leg. At 28 s the leg gripper opens and socket-contact retention is checked. Changing duration recomputes the path and control schedule together.':'Optimize reactions and torques on an IK contact path with fixed orientations. A separate Cartesian example evaluates restricted CITO.';
  $('keyframe-help').textContent=isPanel?`${preset.keyframes.length} keyframes · source refinement and target validation are recorded separately`:isAssembly?'Eight default keyframes · tabletop height/pitch and leg height · retimed with the requested duration':'Position in meters · time in seconds · fixed orientation xyzw = [0, 0, 0, 1]';
  if(detailed){
    const scale=Number($('assembly-duration').value)/(preset.simulation?.duration||preset.duration);
    for(const [i,k] of preset.keyframes.entries()){
      const row=document.createElement('div');row.className='assembly-key';
      const pose=isPanel?k.object_pose:k.tabletop;const pitch=2*Math.atan2(pose[4],pose[6])*180/Math.PI;
      for(const text of [`0${i+1}`,`${(k.time*scale).toFixed(1)} s`,names[k.id]||k.id,isPanel?`panel ${pose[2].toFixed(2)} m / ${pitch.toFixed(0)}°`:`top ${pose[2].toFixed(2)} m / ${pitch.toFixed(0)}° · leg ${k.leg[2].toFixed(2)} m`]){const el=document.createElement('span');el.textContent=text;row.append(el);}
      $('keyframe-inputs').append(row);
    }
    if(!isPanel)$('contacts-json').value=JSON.stringify(preset.arms.map(a=>({arm_id:a.id,kind:a.kind,object:a.object,point_object:a.point_object,normal_object:a.normal_object})),null,2);
  }else{
    for(const [i,k] of preset.keyframes.entries()){
      const row=document.createElement('div');row.className='keyframe-input';const name=document.createElement('span');name.textContent=`0${i+1} ${k.id}`;row.append(name);
      for(const [name,value] of [['time',k.time],['x',k.object_pose[0]],['y',k.object_pose[1]],['z',k.object_pose[2]]]){
        const label=document.createElement('label');label.textContent=name==='time'?'t · s':name+' · m';const input=document.createElement('input');input.type='number';input.step='.01';input.value=value;input.required=true;input.dataset.key=i;input.dataset.field=name;input.setAttribute('aria-label',`${k.id} ${name}`);input.min=name==='time'?0:-.9;input.max=name==='time'?12:1.2;label.append(input);row.append(label);
      }$('keyframe-inputs').append(row);
    }
    $('contacts-json').value=JSON.stringify(preset.contracted_contacts,null,2);$('panel-mass').value=preset.object.mass;
  }
  $('contacts-json').readOnly=isAssembly;$('contacts-file').disabled=isAssembly;
}
function report(data){
  data=englishRecording(data);
  const r=data.report;if(!r)return;$('experiment-results').hidden=false;
  showAlgorithmComparisons(data);
  let comparison=$('ablation-results');
  if(!comparison){comparison=document.createElement('div');comparison.id='ablation-results';$('experiment-results').append(comparison);}
  comparison.hidden=!data.full_task_recording;comparison.replaceChildren();
  if(data.comparison_recording){
    const errors=Object.values(r.object_tracking);$('result-tracking').textContent=`${r.tracking_passed?'PASS':'FAIL'} · ${(1000*Math.max(...errors.map(e=>e.maximum_position_error_m))).toFixed(2)} mm`;
    $('result-forces').textContent=r.sampled_interface_wrench_capability.passed&&Object.values(r.jaw_force).every(j=>j.passed)&&r.minimum_actuator_margin_Nm>=-1e-8?'PASS':'FAIL';
    $('result-seal-label').textContent='STABILITY / CONTACT';const fractions=Object.values(r.maintained_contact_fraction).filter(v=>v!==null);$('result-seal').textContent=`${r.execution_passed?'PASS':'FAIL'} · ${(100*Math.min(...fractions)).toFixed(1)}% contact`;
    $('result-residual').textContent=r.source?`ρ ${r.source.source_rho_after.toFixed(3)}`:r.selection.selected_types.join(' / ');
    $('source-result').hidden=false;$('source-result').textContent=r.source?`Source polytope ${r.source.robustness_enabled?'ON · 8 box vertices':'OFF · nominal equilibrium only'} · independent common-box margin ρ ${r.source.source_rho_after.toFixed(3)} · target ρ ${r.source.target_rho.toFixed(3)} · target posture, reactions, and torques recomputed at fixed contact sites`:`Forced contact type ${r.selection.selected_types.join(' / ')} · sequence objective ${r.selection.objective.toFixed(3)} · native execution ${r.execution_passed?'PASS':'FAIL'}`;
    for(const id of ['workspace-result','robustness-result'])$(id).hidden=true;$('report-json').textContent=JSON.stringify(r,null,2);return;
  }
  if(data.full_task_recording){
    const errors=Object.values(r.object_tracking);$('result-tracking').textContent=`${r.tracking_passed?'PASS':'FAIL'} · ${(1000*Math.max(...errors.map(e=>e.maximum_position_error_m))).toFixed(2)} mm`;
    $('result-forces').textContent=r.sampled_interface_wrench_capability.passed&&Object.values(r.jaw_force).every(j=>j.passed)&&r.minimum_actuator_margin_Nm>=-1e-8?'PASS':'FAIL';$('result-seal-label').textContent='TASK / LOADED CONTACT';$('result-seal').textContent=`${r.task_passed&&r.contact_retention_passed?'PASS':'FAIL'} · ${(100*Math.min(...Object.values(r.maintained_contact_fraction).filter(v=>v!==null))).toFixed(1)}% contact`;
    $('result-residual').textContent=r.selection.selected_types.join(' / ');$('source-result').hidden=false;$('source-result').textContent=`SELECTION · ${r.selection.accepted_candidates.length} accepted / ${r.selection.candidate_count} candidates · ${(r.selection.pruned_candidates||[]).length} cost-bound pruned · physical arms ${r.selection.selected_assignment.join(', ')} · ${r.selection.keyframes} keyframes / ${r.selection.intervals} transitions / ${(r.selection.interval_methods||[]).filter(m=>m==='motion_cost').length} moving-object CITO · source certificates ${r.selection.source_certificates.filter(s=>s.source_certificate??s.source_refinement_passed??false).length}/${r.selection.keyframes}`;
    for(const id of ['workspace-result','robustness-result'])$(id).hidden=true;
    renderFullResults(data,comparison);
    $('report-json').textContent=JSON.stringify(r,null,2);return;
  }
  if(['robotiq_clamp','coupled_panel_lift'].includes(data.scenario)){
    for(const id of ['source-result','workspace-result','robustness-result'])$(id).hidden=true;
    const component=data.scenario==='robotiq_clamp';
    $('result-tracking').textContent=component?(r.mechanics.passed?'PASS · mechanics':'FAIL'):(r.tracking_and_contact_passed?'PASS':'FAIL')+` · ${(r.maximum_position_error_m*1000).toFixed(3)} mm`;
    $('result-forces').textContent=component?`${Math.max(...r.maximum_jaw_force_N).toFixed(2)} N / jaw`:(r.physical_finger_force_limits_passed&&r.sampled_interface_wrench_capability.passed?'PASS':'FAIL');
    $('result-seal-label').textContent=component?'NATIVE GRIPPER / FREE BLOCK':'MULTI-ARM / FIRST LIFT';
    $('result-seal').textContent=component?`${r.native_internal_equalities} internal constraints · no object weld`:`${(100*Math.min(...Object.values(r.maintained_contact_fraction).filter(v=>v!==null))).toFixed(1)}% contact`;
    $('result-residual').textContent=component?Math.max(...Object.values(r.mechanics.residuals)).toExponential(2):r.independent_plan.residuals.momentum.toExponential(2);
    $('report-json').textContent=JSON.stringify(r,null,2);return;
  }
  if(r.articulated_object||r.free_articulated_object){
    const angular=r.coordinate_units==='rad',factor=angular?180/Math.PI:1000,unit=angular?'deg':'mm';
    $('result-tracking').textContent=`${r.execution_passed?'PASS':'FAIL'} · ${(r.final_error*factor).toFixed(3)} ${unit}`;
    $('result-forces').textContent=`${(r.peak_contact_force_N||Math.max(...r.peak_contact_force_per_arm_N)).toFixed(2)} N · peak`;
    $('result-seal-label').textContent=r.free_articulated_object?'FREE BODY / 3-ARM CONTACT':'PASSIVE JOINT / CONTACT';$('result-seal').textContent=`0 actuators · ${(100*(Array.isArray(r.contact_fraction)?Math.min(...r.contact_fraction):r.contact_fraction)).toFixed(1)}% contact`;
    for(const id of ['source-result','workspace-result','robustness-result'])$(id).hidden=true;
    if(r.contraction){$('source-result').hidden=false;$('source-result').textContent=(r.full_paper_pipeline_executed?'FULL PIPELINE · ':'NOMINAL BASELINE · Complete Sec. III-B–IV pipeline not executed · ')+`CONTRACTION · ${r.contraction.dense_sample_count} dense samples → ${r.contraction.contracted_contact_count} link-aware anchors → ${r.arm_count} arms · ×${r.contraction.source_to_target_scale} geometry transfer · synthetic group IDs`;}$('result-residual').textContent=r.primal_residual.toExponential(2);$('report-json').textContent=JSON.stringify(r,null,2);return;
  }
  const isAssembly=r.scenario==='table_assembly';const isPanel=r.scenario==='panel_fixture';const isMobile=r.scenario==='mobile_panel';const errors=isPanel?r.keyframe_errors.map(k=>k.panel.translation_m):isAssembly?r.keyframe_errors.flatMap(k=>[k.tabletop.translation_m,...(k.time<=(r.leg_release_time_s||28)?[k.leg.translation_m]:[])]):r.keyframe_errors.map(k=>k.translation_m);
  $('result-tracking').textContent=(r.tracking_passed?'PASS':'FAIL')+' · '+Math.max(...errors.map(v=>v*1000)).toFixed(2)+' mm';
  $('result-forces').textContent=isMobile?(r.force_envelopes_passed?'PASS':'FAIL'):r.force_envelopes_passed?'PASS':'FAIL';
  $('result-seal-label').textContent=isPanel||isMobile?'JIG SUPPORT / RELEASE':isAssembly?'ASSEMBLY / RELEASE':'SUCTION SEAL';
  $('result-seal').textContent=isPanel||isMobile?(r.fixture_supported&&r.all_tools_released?`SUPPORTED · ${r.jig_support_final_N.toFixed(2)} N`:'FAIL'):isAssembly?(r.assembly_seated&&r.leg_released?`SEATED · ${(r.insertion_depth_m*1000).toFixed(1)} mm`:'FAIL / not seated'):(r.seal_retained_before_vent?'Retained':'Lost');
  $('workspace-result').hidden=!isMobile;
  if(isMobile){const p=r.workspace_comparison.panda,g=r.workspace_comparison.tip_right;$('workspace-result').textContent=`WORKSPACE · Target contact: fixed Panda ${p.fixed_base_target_distance_m.toFixed(3)} m > URDF reach bound ${p.URDF_chain_radius_bound_m.toFixed(3)} m · RBY1 (same torso) ${g.fixed_base_target_distance_m.toFixed(3)} m > ${g.URDF_chain_radius_bound_m.toFixed(3)} m · measured base travel ${r.base_travel_m.toFixed(3)} m`;}
  $('source-result').hidden=!(isPanel||isMobile);
  if(isPanel||isMobile)$('source-result').textContent=`SOURCE · ${r.source.synthetic_input?'Synthetic input':'Supplied contracted contacts'} · refinement ${r.source.source_refinement_enabled?'ON':'OFF'} · ρ ${r.source.source_rho_before.toFixed(3)} → ${r.source.source_rho_after.toFixed(3)} · target ${r.target_validation?.length?'minimum ρ '+Math.min(...r.target_validation.map(v=>v.rho)).toFixed(3):'nominal coupled-torque check'} · learned inference not executed`;
  $('robustness-result').hidden=!r.robustness;
  if(r.robustness)$('robustness-result').textContent=`External force ${r.robustness.force_peak_N} N · pose recovery ${r.robustness.recovery_passed?'PASS':'FAIL'} · complete checks including interface admissibility ${r.robustness.all_task_checks_passed?'PASS':'FAIL'} · maximum error ${(r.robustness.peak_pose_error_m*1000).toFixed(2)} mm`;
  $('result-residual').textContent=r.primal_residual.toExponential(2);$('report-json').textContent=JSON.stringify(r,null,2);
}
async function request(path,options={}){const target=path.startsWith('/api/')?new URL('.'+path,document.baseURI):path;const response=await fetch(target,options);const body=await response.json();if(!response.ok)throw Error(body.error||'Computation-backend error');return body;}
function finish(){clearTimeout(timer);job=null;$('experiment-run').disabled=!connected;$('experiment-cancel').disabled=true;$('scenario-preset').disabled=false;}
async function poll(){const active=job;if(!active)return;try{
  const result=await request('/api/experiments/'+active);if(job!==active)return;
  state((stages[result.stage]||result.stage)+(result.time_s!==undefined?` · ${result.time_s.toFixed(1)} / ${result.duration_s.toFixed(1)} s`:''));
  if(result.state==='completed'){const data=await request(result.recording);for(const instance of data.instances)instance.manifest=new URL(instance.manifest,location.href).href;window.dispatchEvent(new CustomEvent('experiment-result',{detail:data}));report(data);finish();}
  else if(['failed','cancelled'].includes(result.state)){state(result.error||stages[result.stage]);finish();}else timer=setTimeout(poll,1000);
}catch(error){if(job===active){state(error.message);finish();}}}
$('experiment-form').onsubmit=async event=>{event.preventDefault();try{
  let payload;
  if(articulated())payload={preset:preset.id,duration:Number($('assembly-duration').value),...(preset.id.startsWith('free_')?{dense_source_groups:JSON.parse($('contacts-json').value)}:{})};
  else if(panel())payload={preset:mobile()?'mobile_panel':'panel_fixture',duration:Number($('assembly-duration').value),refine_source:$('source-refinement').checked,contracted_source:JSON.parse($('contacts-json').value)};
  else if(assembly())payload={preset:'table_assembly',duration:Number($('assembly-duration').value)};
  else{const keyframes=structuredClone(preset.keyframes);document.querySelectorAll('#keyframe-inputs input').forEach(input=>{const k=keyframes[Number(input.dataset.key)],v=Number(input.value);if(input.dataset.field==='time')k.time=v;else k.object_pose[['x','y','z'].indexOf(input.dataset.field)]=v;});payload={preset:'three_franka',mass:Number($('panel-mass').value),keyframes,contacts:JSON.parse($('contacts-json').value)};}
  $('experiment-run').disabled=true;$('scenario-preset').disabled=true;state('Submitting computation…');const result=await request('/api/experiments',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(payload)});job=result.id;$('experiment-cancel').disabled=false;await poll();
}catch(error){state(error.message);finish();}};
$('experiment-cancel').onclick=async()=>{if(job){try{await request('/api/experiments/'+job+'/cancel',{method:'POST'});state('Cancelled');finish();}catch(error){state(error.message);}}};
$('experiment-reset').onclick=()=>{$('contacts-json').dataset.preset='';$('assembly-duration').value=articulated()?preset.duration:mobile()?36:30;if(panel())$('contacts-json').value=JSON.stringify(preset.contracted_source,null,2);fields();};
$('assembly-duration').oninput=()=>{if((articulated()||assembly()||panel())&&$('assembly-duration').checkValidity())fields();};
$('contacts-file').onchange=async event=>{try{const file=event.target.files[0];if(!file)return;if(file.size>32768)throw Error('Use a JSON file no larger than 32 KB.');const body=JSON.parse(await file.text());$('contacts-json').value=JSON.stringify(body.dense_source_groups||body.contracted_contacts||body,null,2);state('Contact JSON loaded. Surface and normal constraints are checked during computation.');}catch(error){state(error.message);}};
function showRecordingExperiment(data){
  const id=libraryEntry(data).preset;
  document.querySelector('.experiment-builder').hidden=!id&&!data.full_task_recording&&!data.comparison_recording;
  $('experiment-form').hidden=!id;
  if(data.full_task_recording||data.comparison_recording){$('builder-title').textContent=data.title;$('builder-note').textContent='Measured planning/execution outcomes and controlled comparisons';$('preset-tag').textContent=data.report.selection?.selected_types.join(' / ')||'SOURCE ROBUSTNESS';$('source-refinement-control').hidden=true;}
  $('experiment-results').hidden=!data.report;
  if(id){$('scenario-preset').value=id;if(presetId!==id)selectPreset(id).catch(error=>state(error.message));}
  report(data);
}
window.addEventListener('recording-selected',event=>showRecordingExperiment(event.detail));
async function selectPreset(id){
  const token=++presetToken;presetId=id;
  const isArticulated=['hinged_door','folding_panel','drawer','free_box_lid','free_folding_object','free_portable_drawer'].includes(id);
  const url=isArticulated?`./data/${id}-preset.json`:id==='mobile_panel'?'./data/mobile-preset.json':id==='panel_fixture'?'./data/panel-preset.json':id==='table_assembly'?'./data/assembly-preset.json':'./data/franka-preset.json';
  const next=await(await fetch(url)).json();
  if(['panel_fixture','mobile_panel'].includes(id)){
    next.contracted_source=await(await fetch(id==='mobile_panel'?'./data/mobile-source.json':'./data/panel-source.json')).json();
  }
  if(token!==presetToken)return;
  preset=next;$('assembly-duration').value=isArticulated?preset.duration:id==='mobile_panel'?36:30;
  if(next.contracted_source)$('contacts-json').value=JSON.stringify(next.contracted_source,null,2);
  fields();
}
$('scenario-preset').onchange=async()=>{try{await selectPreset($('scenario-preset').value);window.dispatchEvent(new CustomEvent('experiment-preview',{detail:$('scenario-preset').value}));}catch(error){state(error.message);}};
export async function initializeExperiment(){
  for(const category of categories){
    const entries=experimentPresets().filter(entry=>entry.category===category.id);if(!entries.length)continue;
    const group=document.createElement('optgroup');group.label=category.title;
    for(const entry of entries){const option=document.createElement('option');option.value=entry.id;option.textContent=entry.title;group.append(option);}
    $('scenario-preset').append(group);
  }
  const model=new URLSearchParams(location.search).get('model')||'free_box_lid';const id=libraryEntry({id:model}).preset;
  document.querySelector('.experiment-builder').hidden=!id;
  if(id){$('scenario-preset').value=id;await selectPreset(id);}
  try{const health=await request('/api/health');if(health.ready){if(health.experiments_ready===false){state('Panda collision meshes required · python tools/fetch_panda_meshes.py');return;}connected=true;$('experiment-run').disabled=false;state('Python connected · source/target optimization and contact-dynamics execution available');return;}}catch{}
  state('GitHub Pages replays measured execution records. New computations require the Python backend: .venv/bin/python web/server.py');
}
