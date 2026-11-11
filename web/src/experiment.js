const $ = id => document.getElementById(id);
let preset, job, timer;
const state = text => { $('experiment-state').textContent=text; };
const stages={queued:'대기',optimization:'IK 경로 · IPOPT 접촉력 최적화',simulation:'MuJoCo 동역학 실행',export:'기록 내보내기',complete:'계산 완료',failed:'실패',cancelled:'취소됨'};
function fields() {
  $('keyframe-inputs').replaceChildren();
  for(const [i,k] of preset.keyframes.entries()) {
    const row=document.createElement('div');row.className='keyframe-input';
    row.innerHTML=`<span>0${i+1} <b>${k.id}</b></span>`;
    for(const [name,value] of [['time',k.time],['x',k.object_pose[0]],['y',k.object_pose[1]],['z',k.object_pose[2]]]) {
      const label=document.createElement('label');label.textContent=name==='time'?'t · s':name+' · m';
      const input=document.createElement('input');input.type='number';input.step='.01';input.value=value;input.required=true;input.dataset.key=i;input.dataset.field=name;input.setAttribute('aria-label',`${k.id} ${name}`);
      if(name==='time'){input.min=0;input.max=12;}else{input.min=-.9;input.max=1.2;}
      label.append(input);row.append(label);
    }
    $('keyframe-inputs').append(row);
  }
  $('contacts-json').value=JSON.stringify(preset.contracted_contacts,null,2);
  $('panel-mass').value=preset.object.mass;
}
function report(data) {
  const r=data.report;if(!r)return;
  $('experiment-results').hidden=false;
  $('result-tracking').textContent=(r.tracking_passed?'PASS':'FAIL')+' · '+Math.max(...r.keyframe_errors.map(k=>k.translation_m*1000)).toFixed(2)+' mm';
  $('result-forces').textContent=r.force_envelopes_passed?'PASS':'FAIL';
  $('result-seal').textContent=r.seal_retained_before_vent?'유지':'이탈';
  $('result-residual').textContent=r.primal_residual.toExponential(2);
  $('report-json').textContent=JSON.stringify(r,null,2);
}
async function request(path,options={}) {
  const response=await fetch(path,options);const body=await response.json();
  if(!response.ok)throw Error(body.error||'계산 서버 오류');return body;
}
function finish(){clearTimeout(timer);job=null;$('experiment-run').disabled=false;$('experiment-cancel').disabled=true;}
async function poll(){
  const active=job;if(!active)return;
  try{
    const result=await request('/api/experiments/'+active);
    if(job!==active)return;
    state(stages[result.stage]||result.stage);
    if(result.state==='completed'){
      const data=await request(result.recording);
      for(const instance of data.instances)instance.manifest=new URL(instance.manifest,location.href).href;
      window.dispatchEvent(new CustomEvent('experiment-result',{detail:data}));report(data);finish();
    }else if(['failed','cancelled'].includes(result.state)){state(result.error||stages[result.stage]);finish();}
    else timer=setTimeout(poll,1200);
  }catch(error){if(job===active){state(error.message);finish();}}
}
$('experiment-form').onsubmit=async event=>{
  event.preventDefault();
  try{
    const keyframes=structuredClone(preset.keyframes);
    document.querySelectorAll('#keyframe-inputs input').forEach(input=>{
      const k=keyframes[Number(input.dataset.key)],v=Number(input.value);
      if(input.dataset.field==='time')k.time=v;else k.object_pose[['x','y','z'].indexOf(input.dataset.field)]=v;
    });
    const contacts=JSON.parse($('contacts-json').value);
    $('experiment-run').disabled=true;state('작업 제출 중…');
    const result=await request('/api/experiments',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({preset:'three_franka',mass:Number($('panel-mass').value),keyframes,contacts})});
    job=result.id;$('experiment-cancel').disabled=false;await poll();
  }catch(error){state(error.message);finish();}
};
$('experiment-cancel').onclick=async()=>{if(job){try{await request('/api/experiments/'+job+'/cancel',{method:'POST'});state('취소됨');finish();}catch(error){state(error.message);}}};
$('experiment-reset').onclick=fields;
$('contacts-file').onchange=async event=>{
  try{
    const file=event.target.files[0];if(!file)return;if(file.size>32768)throw Error('32 KB 이하 JSON을 사용하세요.');
    const body=JSON.parse(await file.text());$('contacts-json').value=JSON.stringify(body.contracted_contacts||body,null,2);state('접촉 JSON을 불러왔습니다. 실행 시 표면·법선을 검사합니다.');
  }catch(error){state(error.message);}
};
window.addEventListener('recording-selected',event=>report(event.detail));
export async function initializeExperiment(){
  preset=await (await fetch('./data/franka-preset.json')).json();fields();
  try{const health=await request('/api/health');if(health.ready){
    if(health.experiments_ready===false){state('Panda 충돌 mesh 준비 필요 · python tools/fetch_panda_meshes.py');return;}
    $('experiment-run').disabled=false;state('Python 연결됨 · 실제 최적화와 동역학 실행 가능');return;
  }}catch{}
  state('기록 재생 모드 · 새 계산: 저장소에서 .venv/bin/python web/server.py 실행 후 localhost:8000 접속');
}
