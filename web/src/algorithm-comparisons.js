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
   const group=study.contact_types[task];root.append(node('h4','다른 접촉 유형: 같은 과제의 전체 시퀀스 비교'));
   const t=table(root,['유형','결과 단계','실행','시퀀스 비용','최대 위치 오차 / 근거']);
   row(t,[group.selected_types.join('/'),'선택된 원방법','PASS',number(group.selected_sequence_objective),link('full_'+task,'원방법 재생')]);
   for(const v of group.counterfactuals||[]){const detail=v.failure||`${number(1000*v.maximum_position_error_m,2)} mm · ${v.failed_checks.join(', ')||'전체 기준 통과'}`;row(t,[v.types.join('/'),v.stage==='planning'?'계획 중 탈락':'새 native 물리 실행',v.execution_passed?'PASS':'FAIL',number(v.sequence_objective),v.recording_id?link(v.recording_id,detail):detail]);}
   note(root,'Source·질량·마찰·공구 한계·팔 배정·제어 설정을 맞추고 유형을 강제로 바꿉니다. 계획 실패는 실제 실행 실패와 구분하며, 로컬 NLP 실패가 전역 불가능을 증명하지는 않습니다.');
   const typeFigure=node('img');typeFigure.src='./data/type-counterfactual-comparison.svg';typeFigure.alt='패널 PPP와 박스 PPG의 시퀀스 비용 증가 및 최대 위치 오차 비교';typeFigure.className='comparison-figure';root.append(typeFigure);
   root.append(node('h4','같은 접촉점·물체 자세에서 독립 검산한 유형별 용량'));
   const c=table(root,['유형','최대 ρ','고정 keyframe 비용','공통 ρ=0.1에서 관절 토크 여유','판정']);
   for(const v of group.rows)row(c,[v.types.join('/'),number(v.rho),number(v.frozen_keyframe_cost),v.common_rho_metrics?number(v.common_rho_metrics.minimum_joint_torque_margin_Nm,2)+' Nm':'—',v.failure||`${v.planning_passed?'PASS':'FAIL'} · ${v.solver_status}`]);
   note(root,'여기서는 접촉 물질점과 물체 자세를 고정하고, 실제 공구에 맞춰 IK·반력·토크를 다시 계산합니다. ρ가 클수록, 비용이 작을수록 좋습니다. 이 표의 keyframe 비용과 위 시퀀스 비용은 별개의 값입니다.');
  }
  if(study.source_robustness){
   const group=study.source_robustness;root.append(node('h4','별도 패널 지지 실험 · Source polytope ON/OFF'));
   note(root,'세 Panda가 같은 spherical tip으로 자유 패널을 지지합니다. ON은 8개 box vertex, OFF는 명목 중력 평형만으로 소형 접촉점을 최적화합니다. 접촉 위치만 4배 전달하고 target 자세·반력·토크를 다시 풉니다. 효과 분리를 위해 target 접촉점은 전달된 위치에 고정합니다.');
   const t=table(root,['외력 · 위/아래','Polytope','독립 source / target ρ','판정','최대 위치 / 회전 오차','하중 접촉 유지','재생']);
   for(const v of group.trials){const direction=v.force_N===0?'없음':`${Math.abs(v.force_N)} N · ${v.force_N<0?'위':'아래'}`;const tr=row(t,[direction,v.robustness_enabled?'ON':'OFF',`${number(v.source_rho)} / ${number(v.target_rho)}`,v.execution_passed?'PASS':'FAIL',`${number(1000*v.maximum_position_error_m,2)} mm / ${number(v.maximum_rotation_error_rad)} rad`,number(100*v.minimum_contact_fraction,1)+'%',link(v.recording_id,'외력·접촉 재생')]);tr.dataset.robustness=String(v.robustness_enabled);tr.dataset.force=String(v.force_N);tr.dataset.passed=String(v.execution_passed);}
   note(root,'각 힘 조건은 1회 실행입니다. 위치 8 mm, 회전 0.06 rad, 필요한 접촉 유지 99%의 같은 기준을 적용합니다. 외력은 같은 가장자리·4–5 s에 작용하는 1 s sin² pulse입니다. ON의 인증 박스 밖에서는 안정성을 보장하지 않습니다. 정적 wrench 가능성은 폐루프 동역학의 안정성 증명이 아닙니다.');
   const forceFigure=node('img');forceFigure.src='./data/source-robustness-comparison.svg';forceFigure.alt='같은 외력에서 source polytope ON/OFF의 최대 회전 오차와 접촉 유지율 전체 비교';forceFigure.className='comparison-figure';root.append(forceFigure);
   note(root,`검증 집합은 지정한 작용점의 3축 힘입니다. 공통 force-box D = ${group.protocol.target_force_box_scales_N.join(', ')} N · 작용점 = [${group.protocol.force_application_point_body_m.join(', ')}] m · source 공통 geometry weight = ${group.protocol.geometry_weight}. 고정 E,D·제어기·계획 ID를 기록하며 힘 조건마다 제어기를 조정하지 않습니다.`);
  }
  note(root,'구현 범위: contracted groups는 주어진 입력이며 학습 추론을 실행하지 않습니다. 기존 전체 동작은 제한된 접촉 후보와 midpoint 변형을 사용합니다. 이 분리 실험을 원고 전체 알고리즘의 완전한 검증으로 해석하지 않습니다.');
 }catch{if(current===token)root.hidden=true;request=undefined;}
}
