// Summaries of saved evidence. Never infer success from animation or solver status.
const labels={full:'원방법',no_source_refinement:'Source refinement 제거',fixed_contact_types:'접촉 유형 고정',fixed_path:'경로 고정',no_arm_force_feedforward:'팔 힘 feedforward 제거',contact_disabled:'접촉 비활성화 · 음성 대조군',external_push:'외란 · 최대 5 N'};
const maxError=r=>Math.max(0,...Object.values(r.object_tracking||{}).map(e=>e.maximum_position_error_m));
const minContact=r=>Object.values(r.maintained_contact_fraction||{}).filter(v=>v!==null);
export function failureReasons(r){
  const reasons=[];
  if(r.failure)reasons.push('계획/실행 오류: '+r.failure);
  if(r.tracking_passed===false){if(maxError(r)>.008)reasons.push(`추종 오차 ${(maxError(r)*1000).toFixed(2)} mm / 허용 8 mm`);const angle=Math.max(0,...Object.values(r.object_tracking||{}).map(e=>e.maximum_rotation_error_rad));if(angle>.06)reasons.push(`자세 오차 ${angle.toFixed(3)} rad / 0.060 rad`);}
  for(const [id,j] of Object.entries(r.jaw_force||{}))if(!j.passed){const peak=Math.max(...(j.all_time_maximum_per_jaw_N||j.maximum_per_jaw_N));reasons.push(peak>j.limit_N?`${id}: jaw ${peak.toFixed(2)} N / ${j.limit_N} N`:`${id}: 양쪽 jaw 접촉 유지 부족`);}
  if(r.contact_retention_passed===false)reasons.push(`계획 하중 접촉 ${(100*Math.min(...minContact(r))).toFixed(1)}% / 99%`);
  if(r.sampled_interface_wrench_capability?.passed===false)reasons.push('측정 wrench 허용영역 위반');
  if(r.task_passed===false)reasons.push('목표 자세·조립·안착 조건 미충족');
  if(r.minimum_actuator_margin_Nm< -1e-8)reasons.push('관절 토크 한계 위반');
  if(r.reference_geometry?.passed===false)reasons.push('기준 경로 형상 검사 실패');
  if(Object.keys(r.unexpected_loaded_contact_penetrations_m||{}).length)reasons.push('의도하지 않은 하중 접촉');
  return reasons;
}
const el=(tag,text)=>{const n=document.createElement(tag);if(text!==undefined)n.textContent=text;return n;};
function table(headers){const t=el('table'),h=el('tr');for(const s of headers)h.append(el('th',s));const head=el('thead');head.append(h);t.append(head,el('tbody'));return t;}
function row(t,values){const r=el('tr');for(const v of values){const c=el('td');typeof v==='string'?c.textContent=v:c.append(v);r.append(c);}t.tBodies[0].append(r);return r;}
function note(root,text){const p=el('p',text);p.className='fine';root.append(p);}
function reasonText(s){
 if(s.includes('Dominated cost bound'))return '실현된 keyframes의 비용 하한으로 가지치기 (물리적 불가능 판정 아님)';
 if(s.includes('IK failed')||s.includes('IK target')||s.includes('unreachable'))return '주어진 초기값의 IK 실패 · 전역 도달 불가능 증명 아님';
 if(s.includes('Structurally insufficient'))return '자유 물체의 6차원 wrench를 지지하는 rank 부족';
 return s;
}
export function renderFullResults(data,root){
 root.replaceChildren();const variants=data.report.ablation_results||{},task=data.scenario.replace('full_','');
 root.append(el('h4',`Ablation 비교 · ${Object.keys(variants).length}조건`));
 const t=table(['조건 · 기록 재생','판정','최대 위치 오차','하중 접촉 유지','실패 이유 / 비교 범위']);root.append(t);
 for(const [name,r] of Object.entries(variants)){
  const a=el('a',`${labels[name]||name} ↗`);a.href=`?model=full_${task}${name==='full'?'':'_'+name}#ablation-results`;a.title=name;
  const f=minContact(r),reasons=failureReasons(r),same=r.selection?.continuous_plan_reused;
  const detail=reasons.length?reasons.join(' · '):same?'동일한 연속 계획 재사용 · 새 물리 실행':name==='full'?'모든 실행 기준 통과':'이 조건에서도 실행 기준 통과';
  const tr=row(t,[a,r.execution_passed?'PASS':'FAIL',r.object_tracking?`${(1000*maxError(r)).toFixed(2)} mm`:'—',f.length?`${(100*Math.min(...f)).toFixed(1)}%`:'—',detail]);tr.dataset.variant=name;tr.dataset.passed=String(r.execution_passed);
 }
 note(root,'각 조건은 1회 실행입니다. FAIL은 위 기준 중 하나 이상 미달을 뜻합니다. 접촉 비활성화는 접촉 물리의 필요성을 확인하는 음성 대조군이며, 외란 조건은 스트레스 테스트입니다. P 유지율은 계획 법선 하중 > 0.1 N 구간 기준입니다.');
 const s=variants.full?.selection||data.report.selection;const heading=el('h4','원방법의 접촉 유형·팔 배정 탐색');heading.id='selection-results';root.append(heading);
 const tool=data.provenance.canonical_point_tool;
 if(tool){const length=Math.hypot(...tool.ee_to_tip.slice(0,3).map(row=>row[3]));note(root,`P 공구 EE → 접촉점: ${(1000*length).toFixed(0)} mm · 구 반경 ${(1000*tool.radius_m).toFixed(0)} mm. URDF의 실제 장착 길이이며 화면만 이동시키는 보정값이 아닙니다. 최소 길이 최적화는 수행하지 않았습니다.`);}
 const patterns=[...new Set([...(s.accepted_candidates||[]),...(s.pruned_candidates||[]),...(s.rejections||[])].map(c=>c.types.join('/')))];
 note(root,`G = gripper, P = 동일 spherical tip. 허용 유형: ${patterns.join(', ')} · 팔 배정 포함 ${s.candidate_count}후보. 두 번째 지지 접촉은 P로 제한합니다. 현재 목적함수에는 공구 수·유형 비용이 없으며, 선택한 공구는 시퀀스 전체에서 고정됩니다.`);
 const fixed=variants.fixed_contact_types?.selection;
 if(fixed?.selected_types?.join('')===s.selected_types.join('')&&fixed.selected_assignment?.join('')===s.selected_assignment.join(''))note(root,`선택 결과 ${s.selected_types.join('/')} · 팔 ${s.selected_assignment.join('/')}는 수동 고정 기준과 같습니다. 이 비교는 유형 선택의 성능 향상을 입증하지 못합니다. 실행 중 공구 교환이나 재선택은 구현하지 않았습니다.`);
 const candidates=table(['유형','물리 팔 배정','결과','비용 / 비용 하한','판정 근거']);root.append(candidates);
 for(const c of s.accepted_candidates||[]){const chosen=c.types.join('')===s.selected_types.join('')&&c.assignment.join('')===s.selected_assignment.join('');row(candidates,[c.types.join('/'),c.assignment.join('/'),chosen?'선택':'계획 통과',c.objective.toFixed(3),chosen?'선택된 원방법의 연속 계획과 실행 통과':'연속 계획 통과 · 별도 물리 실행 미확인']);}
 for(const c of s.pruned_candidates||[])row(candidates,[c.types.join('/'),[...new Set(c.assignments.map(a=>a.join('/')))].join(' → '),'가지치기',c.sequence_cost_lower_bound.toFixed(3),reasonText(c.reason)]);
 for(const c of s.rejections||[])row(candidates,[c.types.join('/'),[...new Set(c.assignments.map(a=>a.join('/')))].join(' → '),'탈락','—',reasonText(c.reason)]);
 note(root,'작은 비용을 선택합니다. 비용 하한은 해당 keyframe 해에 대한 조건부 비교이며 전역 IK 최적성 증명이 아닙니다. 현재 조합에서는 G의 wrench 여유가 유리해 수동 배치와 같은 결과가 나왔습니다.');
}
