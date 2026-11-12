import { categories, experimentPresets, libraryEntry } from './library.js';
const $=id=>document.getElementById(id);
let preset,job,timer,connected=false,presetId,presetToken=0;
const state=text=>{$('experiment-state').textContent=text;};
const stages={queued:'대기',source_refinement_and_optimization:'소형 접촉점 refinement · target 하중/토크 최적화',optimization:'SE(3) IK 경로 · IPOPT 접촉력/토크 최적화',simulation:'MuJoCo 동역학 실행',validation:'접촉 · 삽입 · 해제 검증',export:'기록 내보내기',complete:'계산 완료',failed:'실패',cancelled:'취소됨'};
const articulated=()=>['hinged_door','folding_panel','drawer','free_box_lid','free_folding_object','free_portable_drawer'].includes(preset?.id);
const assembly=()=>preset?.id==='table_assembly';
const panel=()=>['panel_fixture','mobile_panel'].includes(preset?.id);
const mobile=()=>preset?.id==='mobile_panel';
const names={grip_and_support:'상판 파지·구형 지지',grasp_settled:'파지 유지',lift_tabletop:'상판·다리 들어 올리기',reorient_tabletop:'상판 12° 재정렬',approach_socket:'다리 접근',align_leg:'peg / socket 정렬',insert_leg:'다리 삽입',assembled_and_released:'gripper 해제·조립 유지'};
function articulatedFields(){
  const free=preset.id.startsWith('free_');const angular=!preset.id.includes('drawer');$('keyframe-inputs').replaceChildren();
  $('preset-tag').textContent=free?'PANDA × 3 · FREE ARTICULATED':'COMPONENT · SINGLE ARM';$('builder-title').textContent={hinged_door:'문 열기',folding_panel:'접이식 상판 펼치기',drawer:'서랍 당기기',free_box_lid:'자유 박스 · grip/support + 뚜껑 열기',free_folding_object:'자유 접이식 패널 · 지지 + 날개 펼치기',free_portable_drawer:'자유 수납함 · 지지 + 서랍 당기기'}[preset.id];
  $('builder-note').textContent=free?'세계에 고정되지 않은 본체 + 내부 관절 · G / P / G · dense contact → contraction → 3-arm realization':`수동 ${angular?'hinge':'rail'} · 물체 actuator 0 · ${preset.tool==='G'?'native gripper':'spherical point tool'}`;
  $('tool-chips').replaceChildren();const chip=document.createElement('span');chip.textContent=free?'⋈ 본체 파지 · ● 본체 지지 · ⋈ 내부 링크 조작':preset.tool==='G'?'⋈ Gripper handle grasp':'● Spherical point contact';$('tool-chips').append(chip);
  $('mass-control').hidden=true;$('panel-mass').disabled=true;$('source-refinement-control').hidden=true;$('duration-control').hidden=false;$('assembly-duration').disabled=false;$('assembly-duration').min=12;
  $('planner-description').textContent=free?'소형 입력의 링크·기능별 dense contact를 대표 접촉으로 contraction합니다. 위치만 4배 전달하고 세 팔의 자세·접촉력·토크와 자유 본체+내부 관절의 동역학을 공동으로 맞춥니다. 본체는 공중에서 로봇 접촉으로 지지하며 world 고정 관절·물체 actuator·weld가 없습니다. Group ID와 assignment/path는 supplied synthetic 입력입니다.':'물체의 수동 관절 관성·중력·스프링·마찰과 접촉력·팔 토크를 최적화합니다. 물체 pose를 강제하거나 actuator로 움직이지 않습니다. 접촉점·초기 파지·관절 경로는 주어진 조건이며, articulated contraction 학습은 아직 포함하지 않습니다.';
  const scale=Number($('assembly-duration').value)/preset.duration;
  for(const [label,time,q] of [['접촉',0,preset.initial_coordinate],...(free?[['본체 lift',preset.lift_end,preset.initial_coordinate]]:[]),['시작',preset.motion_start,preset.initial_coordinate],['중간',(preset.motion_start+preset.motion_end)/2,(preset.initial_coordinate+preset.target_coordinate)/2],['목표',preset.motion_end,preset.target_coordinate],['유지',preset.duration,preset.target_coordinate]]){
    const row=document.createElement('div');row.className='assembly-key';for(const text of [label,`${(time*scale).toFixed(1)} s`,`${(q*(angular?180/Math.PI:1000)).toFixed(1)} ${angular?'deg':'mm'}`]){const el=document.createElement('span');el.textContent=text;row.append(el);}$('keyframe-inputs').append(row);
  }
  $('keyframe-help').textContent='초기 접촉 → 이동 → 유지 · 물체 관절의 측정값을 재생합니다.';
  if(!free||$('contacts-json').dataset.preset!==preset.id){$('contacts-json').dataset.preset=preset.id;$('contacts-json').value=JSON.stringify(free?preset.dense_source_groups:{kind:preset.tool,link:'moving',point_link:preset.contact_object,inward_link:preset.inward_object},null,2);}$('contacts-json').readOnly=!free;$('contacts-file').disabled=!free;
}
function fields(){
  if(articulated())return articulatedFields();
  const isAssembly=assembly();const isPanel=panel();const detailed=isAssembly||isPanel;$('keyframe-inputs').replaceChildren();
  $('preset-tag').textContent=mobile()?'RBY1 + PANDA':'PANDA × 3';
  $('builder-title').textContent=mobile()?'작업영역 확장 · RBY1 + Panda':isPanel?'본 실험 · 패널 → 지그 → 릴리스':isAssembly?'책상 상판·다리 조립':'Three Franka · 패널 운반';
  $('builder-note').textContent=mobile()?'차동구동 모바일 듀얼암 + 고정 Panda · 실제 베이스·물체 동역학':isPanel?'Panda × 3 · G / P / P · 자유 패널 1개 · 고정 지그':isAssembly?'실제 Panda × 3 · arm 21 + finger 4 DoF · 자유 물체 2개':'실제 Panda × 3 · arm 21 + finger 2 DoF · 자유 물체 1개';
  $('tool-chips').replaceChildren();for(const title of mobile()?['⋈ RBY1 gripper','● RBY1 sphere support','● Fixed Panda support']:isPanel?['⋈ Edge gripper','● Left sphere support','● Right sphere support']:isAssembly?['⋈ 상판 gripper','● Sphere support','⋈ 다리 gripper']:['● Sphere · point','⋈ Panda hand','⊙ Vacuum seal']){const chip=document.createElement('span');chip.textContent=title;$('tool-chips').append(chip);}
  $('mass-control').hidden=detailed;$('panel-mass').disabled=detailed;
  $('source-refinement-control').hidden=!isPanel;
  $('duration-control').hidden=!detailed;$('assembly-duration').disabled=!detailed;
  $('assembly-duration').min=mobile()?30:24;
  $('planner-description').textContent=mobile()?'고정 Panda가 초기 지지를 넘긴 뒤 통로 밖으로 물러납니다. RBY1은 바퀴를 구동해 1.4 m 이동하고 고정 베이스의 도달 범위 밖에서 패널을 지그에 안착·해제합니다. 베이스와 패널은 자유 강체이며 위치를 강제하지 않습니다. 직선 경로·모드와 초기 파지는 주어집니다.':isPanel?'1/4 스케일의 synthetic contracted contact 위치를 먼저 최적화합니다. 위치만 4배 전달하고 2 kg 패널의 접촉력·토크를 다시 풉니다. 초기 파지와 SE(3) 경로는 주어집니다. 22 s 지그 안착 → 하중 전달 → 26 s 전체 공구 해제. 학습 추론·실제 로봇 실행은 포함하지 않습니다.':isAssembly?'2 kg 상판을 grip & support해 자세를 바꾸고 0.4 kg 다리를 삽입합니다. 28 s에 다리 gripper를 열어 자유 물체의 socket 접촉 유지를 검사합니다. 시간 변경 시 전체 경로와 제어 모드를 함께 재계산합니다.':'고정 자세의 IK 접촉 경로에서 접촉력·토크를 최적화합니다. 전체 CITO 탐색은 별도 Cartesian 예제입니다.';
  $('keyframe-help').textContent=isPanel?`${preset.keyframes.length}개 keyframe · 소형 source refinement와 target 검증을 따로 기록`:isAssembly?'기본 8개 keyframe · 상판의 z / pitch와 다리 z 표시 · 입력 시간에 비례해 재계산':'위치 m · 시간 s · 고정 자세 xyzw = [0, 0, 0, 1]';
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
  const r=data.report;if(!r)return;$('experiment-results').hidden=false;
  if(r.articulated_object||r.free_articulated_object){
    const angular=r.coordinate_units==='rad',factor=angular?180/Math.PI:1000,unit=angular?'deg':'mm';
    $('result-tracking').textContent=`${r.execution_passed?'PASS':'FAIL'} · ${(r.final_error*factor).toFixed(3)} ${unit}`;
    $('result-forces').textContent=`${(r.peak_contact_force_N||Math.max(...r.peak_contact_force_per_arm_N)).toFixed(2)} N · peak`;
    $('result-seal-label').textContent=r.free_articulated_object?'FREE BODY / 3-ARM CONTACT':'PASSIVE JOINT / CONTACT';$('result-seal').textContent=`0 actuators · ${(100*(Array.isArray(r.contact_fraction)?Math.min(...r.contact_fraction):r.contact_fraction)).toFixed(1)}% contact`;
    for(const id of ['source-result','workspace-result','robustness-result'])$(id).hidden=true;
    if(r.contraction){$('source-result').hidden=false;$('source-result').textContent=(r.full_paper_pipeline_executed?'FULL PIPELINE · ':'NOMINAL BASELINE · Sec. III-B–IV full pipeline 미실행 · ')+`CONTRACTION · ${r.contraction.dense_sample_count} dense samples → ${r.contraction.contracted_contact_count} link-aware anchors → ${r.arm_count} arms · ×${r.contraction.source_to_target_scale} geometry transfer · synthetic group IDs`;}$('result-residual').textContent=r.primal_residual.toExponential(2);$('report-json').textContent=JSON.stringify(r,null,2);return;
  }
  const isAssembly=r.scenario==='table_assembly';const isPanel=r.scenario==='panel_fixture';const isMobile=r.scenario==='mobile_panel';const errors=isPanel?r.keyframe_errors.map(k=>k.panel.translation_m):isAssembly?r.keyframe_errors.flatMap(k=>[k.tabletop.translation_m,...(k.time<=(r.leg_release_time_s||28)?[k.leg.translation_m]:[])]):r.keyframe_errors.map(k=>k.translation_m);
  $('result-tracking').textContent=(r.tracking_passed?'PASS':'FAIL')+' · '+Math.max(...errors.map(v=>v*1000)).toFixed(2)+' mm';
  $('result-forces').textContent=isMobile?(r.force_envelopes_passed?'PASS':'FAIL'):r.force_envelopes_passed?'PASS':'FAIL';
  $('result-seal-label').textContent=isPanel||isMobile?'JIG SUPPORT / RELEASE':isAssembly?'ASSEMBLY / RELEASE':'SUCTION SEAL';
  $('result-seal').textContent=isPanel||isMobile?(r.fixture_supported&&r.all_tools_released?`SUPPORTED · ${r.jig_support_final_N.toFixed(2)} N`:'FAIL'):isAssembly?(r.assembly_seated&&r.leg_released?`SEATED · ${(r.insertion_depth_m*1000).toFixed(1)} mm`:'FAIL / 미조립'):(r.seal_retained_before_vent?'유지':'이탈');
  $('workspace-result').hidden=!isMobile;
  if(isMobile){const p=r.workspace_comparison.panda,g=r.workspace_comparison.tip_right;$('workspace-result').textContent=`WORKSPACE · 목적 접촉점: 고정 Panda ${p.fixed_base_target_distance_m.toFixed(3)} m > 도달 상한 ${p.URDF_chain_radius_bound_m.toFixed(3)} m · RBY1 (동일 torso) ${g.fixed_base_target_distance_m.toFixed(3)} m > ${g.URDF_chain_radius_bound_m.toFixed(3)} m · 실제 베이스 이동 ${r.base_travel_m.toFixed(3)} m`;}
  $('source-result').hidden=!(isPanel||isMobile);
  if(isPanel||isMobile)$('source-result').textContent=`SOURCE · ${r.source.synthetic_input?'synthetic 입력':'외부 contracted 입력'} · refinement ${r.source.source_refinement_enabled?'ON':'OFF'} · ρ ${r.source.source_rho_before.toFixed(3)} → ${r.source.source_rho_after.toFixed(3)} · target ${r.target_validation?.length?'최소 ρ '+Math.min(...r.target_validation.map(v=>v.rho)).toFixed(3):'nominal 공동 토크 검증'} · 학습 추론 미실행`;
  $('robustness-result').hidden=!r.robustness;
  if(r.robustness)$('robustness-result').textContent=`외력 ${r.robustness.force_peak_N} N · 자세 회복 ${r.robustness.recovery_passed?'PASS':'FAIL'} · 접촉 허용영역 포함 전체 검사 ${r.robustness.all_task_checks_passed?'PASS':'FAIL'} · 최대 오차 ${(r.robustness.peak_pose_error_m*1000).toFixed(2)} mm`;
  $('result-residual').textContent=r.primal_residual.toExponential(2);$('report-json').textContent=JSON.stringify(r,null,2);
}
async function request(path,options={}){const target=path.startsWith('/api/')?new URL('.'+path,document.baseURI):path;const response=await fetch(target,options);const body=await response.json();if(!response.ok)throw Error(body.error||'계산 서버 오류');return body;}
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
  $('experiment-run').disabled=true;$('scenario-preset').disabled=true;state('작업 제출 중…');const result=await request('/api/experiments',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(payload)});job=result.id;$('experiment-cancel').disabled=false;await poll();
}catch(error){state(error.message);finish();}};
$('experiment-cancel').onclick=async()=>{if(job){try{await request('/api/experiments/'+job+'/cancel',{method:'POST'});state('취소됨');finish();}catch(error){state(error.message);}}};
$('experiment-reset').onclick=()=>{$('contacts-json').dataset.preset='';$('assembly-duration').value=articulated()?preset.duration:mobile()?36:30;if(panel())$('contacts-json').value=JSON.stringify(preset.contracted_source,null,2);fields();};
$('assembly-duration').oninput=()=>{if((articulated()||assembly()||panel())&&$('assembly-duration').checkValidity())fields();};
$('contacts-file').onchange=async event=>{try{const file=event.target.files[0];if(!file)return;if(file.size>32768)throw Error('32 KB 이하 JSON을 사용하세요.');const body=JSON.parse(await file.text());$('contacts-json').value=JSON.stringify(body.dense_source_groups||body.contracted_contacts||body,null,2);state('접촉 JSON을 불러왔습니다. 실행 시 표면·법선을 검사합니다.');}catch(error){state(error.message);}};
function showRecordingExperiment(data){
  const id=libraryEntry(data).preset;
  document.querySelector('.experiment-builder').hidden=!id;
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
  try{const health=await request('/api/health');if(health.ready){if(health.experiments_ready===false){state('Panda 충돌 mesh 준비 필요 · python tools/fetch_panda_meshes.py');return;}connected=true;$('experiment-run').disabled=false;state('Python 연결됨 · source/target 최적화와 접촉 동역학 실행 가능');return;}}catch{}
  state('GitHub Pages: 실제 실행 기록 재생 · 새 계산은 Python 백엔드에서 실행합니다. 서버: .venv/bin/python web/server.py');
}
