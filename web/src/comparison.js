import {disturbanceScale} from './disturbance.js';
const $=id=>document.getElementById(id);
const labels={bounded_tracking:'Bounded tracking',persistent_loss_of_support:'Persistent loss of support',transient_failure_recovered:'Transient failure · recovered',tracking_failure:'Tracking failure'};
let study,selected,ready=new Set(),playing=false,time=0,last=performance.now();
const frames=['on','off'].map(mode=>$(mode+'-view'));
const get=(mode,caseId=selected)=>study.trials.find(row=>row.case===caseId&&row.robustness_enabled===(mode==='on'));
function seek(){
 for(const frame of frames)frame.contentWindow?.postMessage({type:'contact-transfer-seek',time_s:time},location.origin);
 $('time').value=time;$('time-label').textContent=time.toFixed(2)+' s';
 const data=get('on').response,times=data.time_s;let i=0;while(i<times.length-1&&times[i]<time)i++;
 const vector=[0,0,0];for(const pulse of get('on').external_disturbances)for(let axis=0;axis<3;axis++)vector[axis]+=pulse.force_world_N[axis]*disturbanceScale(pulse,time);
 const force=Math.hypot(...vector);$('force').textContent=force>.001?`Applied external force: ${force.toFixed(2)} N · identical in both recordings`:'No applied external force';
 for(const mode of ['on','off']){const response=get(mode).response;$(`${mode}-measured`).textContent=`Measured tilt ${(response.rotation_error_rad[i]*180/Math.PI).toFixed(2)}° · height ${response.height_m[i].toFixed(3)} m · ${response.supporting_contacts[i]} / 3 tool contacts`;}
}
function choose(id){
 selected=id;playing=false;time=0;ready.clear();$('play').disabled=true;$('play').textContent='Play both';
 for(const mode of ['on','off']){const r=get(mode),c=r.initial_certificate;$(`${mode}-outcome`).textContent=labels[r.outcome]||r.outcome;$(`${mode}-certificate`).textContent=`Initial static box: ${c.inside_initial_static_box?'IN':'OUT'} · peak / certified margin ${(100*c.load_to_margin_ratio).toFixed(1)}% · vertical bound ±${c.vertical_bound_N.toFixed(3)} N`;$(`${mode}-view`).src=`./?model=${encodeURIComponent(r.recording_id)}&embed=comparison`;}
 const r=get('on');$('time').max=r.response.time_s.at(-1);$('protocol').textContent=`${study.protocol.protocol.cases[id].title}. ON/OFF have matching native physical-case IDs, controller settings, and applied-force histories. The ON load is inside its initial certified box.`;
 $('time-label').textContent='0.00 s';$('time').value='0';
 const url=new URL(location.href);url.searchParams.set('case',id);history.replaceState(null,'',url);
}
window.addEventListener('message',event=>{
 if(event.origin!==location.origin||event.data?.type!=='contact-transfer-ready')return;
 const mode=['on','off'].find(mode=>event.source===$(`${mode}-view`).contentWindow&&event.data.id===get(mode).recording_id);
 if(!mode)return;ready.add(mode);if(ready.size===2){$('play').disabled=false;seek();}
});
$('condition').onchange=e=>choose(e.target.value);
$('play').onclick=()=>{playing=!playing;$('play').textContent=playing?'Pause both':'Play both';};
$('time').oninput=e=>{playing=false;$('play').textContent='Play both';time=Number(e.target.value);seek();};
$('load-event').onclick=()=>{playing=false;const pulse=get('on').external_disturbances[0];time=pulse?pulse.start+(pulse.profile==='smooth_hold'?pulse.ramp_duration_s:pulse.duration/2):4;$('play').textContent='Play both';seek();};
$('recovery-event').onclick=()=>{playing=false;time=get('on').response.force_end_s+1;$('play').textContent='Play both';seek();};
function animate(now){const dt=Math.min(.05,(now-last)/1000);last=now;if(playing){time=Math.min(Number($('time').max),time+dt);if(time>=Number($('time').max)){playing=false;$('play').textContent='Play both';}seek();}requestAnimationFrame(animate);}requestAnimationFrame(animate);
async function init(){
 const response=await fetch('./data/margin-hold-comparison.json');if(!response.ok)throw Error('Matched simulation results are not available');study=await response.json();
 for(const [id,cfg] of Object.entries(study.protocol.protocol.cases))$('condition').append(new Option(cfg.title,id));
 for(const r of study.trials){const tr=document.createElement('tr');tr.dataset.passed=String(r.execution_passed);for(const value of [study.protocol.protocol.cases[r.case].title,r.robustness_enabled?'ON':'OFF',r.initial_certificate.load_to_margin_ratio.toFixed(3),r.initial_certificate.inside_initial_static_box?'IN':'OUT',r.execution_passed?'PASS':'FAIL',labels[r.outcome]||r.outcome]){const td=document.createElement('td');td.textContent=value;tr.append(td);}$('results').append(tr);}
 $('exploratory-note').textContent=study.design_history;
 for(const r of [...study.flat_panel_exploratory_trials,...study.compact_payload_exploratory_trials]){const tr=document.createElement('tr');for(const value of [`${r.object_variant} · ${r.case}`,r.robustness_enabled?'ON':'OFF',r.execution_passed?'PASS':'FAIL',labels[r.outcome]||r.outcome]){const td=document.createElement('td');td.textContent=value;tr.append(td);}const td=document.createElement('td'),a=document.createElement('a');a.textContent='Replay';a.href=`./?model=${encodeURIComponent(r.recording_id)}`;td.append(a);tr.append(td);$('exploratory-results').append(tr);}
 const requested=new URLSearchParams(location.search).get('case');selected=requested&&study.protocol.protocol.cases[requested]?requested:study.protocol.protocol.primary_comparison;$('condition').value=selected;choose(selected);
}
init().catch(error=>{$('error').textContent=error.message;});
