// English presentation only. Saved measurements, identifiers, and provenance
// remain in their original files and the raw-data download is unchanged.
export const paperTitle='Scale-Aware Functional Contact Transfer from Human Hands to Multi-Arm Robots';
export const taskTitles={panel:'Panel reorientation and placement',box:'Free articulated box opening',assembly:'Tabletop–leg assembly'};
export const ablationTitles={full:'Selected method',no_source_refinement:'Without source refinement',fixed_contact_types:'Fixed contact types',fixed_path:'Fixed path',no_arm_force_feedforward:'Without arm reaction feedforward',contact_disabled:'Contact disabled · negative control',external_push:'External disturbance · peak 5 N'};
const titles={full_panel:taskTitles.panel,full_box:taskTitles.box,full_assembly:taskTitles.assembly,panel_fixture:'Panel transport, placement, and release',mobile_panel:'Workspace extension · RBY1 + Panda',free_box_lid:'Free box · lid opening',free_folding_object:'Free folding panel',free_portable_drawer:'Free cabinet · drawer opening',table_assembly:taskTitles.assembly,free_box_push:'Free box · 5 N disturbance',mobile_push_2:'Mobile cooperation · 2 N disturbance',mobile_push_8:'Mobile cooperation · 8 N disturbance',mobile_push_25:'Mobile cooperation · 25 N disturbance',dual_cito:'Dual arm · restricted CITO',panda_robotiq_lift:'Panda × 3 · Robotiq initial lift',panda_coupled_lift:'Panda × 3 · optimized initial lift',robotiq_2f85:'Robotiq 2F-85 · jaw-force component check',three_arm:'Cartesian × 3 · contact interfaces',three_franka:'Panda × 3 · contact interfaces',hinged_door:'Fixed hinge · door opening',folding_panel:'Fixed hinge · panel unfolding',drawer:'Fixed rail · drawer opening'};
const events={'과도응답 비교':'Matched transient response','외력 시작':'External force onset','외력 종료':'External force end','외력 최대 · 화살표':'Peak external force · arrow','최대 자세 오차':'Maximum orientation error'};
const captions={
 '3-Panda · 소형 접촉 refinement의 wrench polytope 제약 유무':'Three Panda arms · task-wrench-robust source contact refinement enabled or disabled',
 '3대 Panda가 grip + 두 point support로 패널을 들어 올립니다. 실제 기구·접촉 물리에서 측정한 동작이며 첫 lift 구간만 검증합니다.':'Three Panda arms lift a panel using one gripper-mediated and two unilateral point contacts. Native mechanism and contact dynamics are recorded for the initial lift only.',
 'Robotiq 2F-85의 실제 링크·mesh · 내부 기구 제약과 유한 구동기 · 자유 블록을 양 jaw 약 40 N으로 파지한 요소 시험입니다.':'A native Robotiq 2F-85 grasp-force component test with original meshes, internal linkage constraints, a finite actuator, and a free block; approximately 40 N per jaw.',
 '가구 조립 · 상판 지지·다리 삽입·릴리스':'Tabletop support, leg insertion, and release',
 '동일 과제·source·후보 팔 배정 · 다른 접촉 유형으로 전체 시퀀스 계획·실행':'Matched task, source refinement, and arm assignment; complete-sequence planning and execution with an alternative contact type',
 '자유 박스 · 본체 지지·뚜껑 열기':'Free articulated box · base support and lid opening',
 '패널 · 운반·재배향·안착·릴리스':'Panel transport, reorientation, placement, and release',
 'Panda·Robotiq 원본 mesh + 동일 spherical tip URDF · 측정된 native 관절과 자유 물체':'Original Panda/Robotiq meshes and common spherical-tool URDF; measured native joints and free-object states',
 '동일 spherical tip 3개 · 측정된 native 관절/자유 패널':'Three identical spherical tools; measured native joints and free-panel states',
 '실제 URDF/mesh와 native 접촉 · 실패 판정도 보존':'Physical URDF/mesh geometry and native contacts; failed checks are retained',
 '원본 Robotiq STL과 URDF · 측정된 8개 관절과 자유 블록의 자세를 재생합니다.':'Original Robotiq STL/URDF geometry; playback of eight measured joint coordinates and the free-block pose',
 '원본 URDF·mesh · 수동 그리퍼 링크를 포함한 native 관절 및 자유 패널 기록':'Original URDF/meshes; recorded native joints, passive gripper links, and free-panel motion',
 'native 접촉에서 측정한 jaw별 법선력 중 큰 값 · 목표 40 N · 설정 상한 60 N / jaw':'Larger measured jaw-normal force · target 40 N · component-model bound 60 N per jaw',
 '동일 위치·시간의 수직 외력 · source geometry만 전달 · target geometry 고정':'Matched vertical force at the same point and time · source geometry alone is transferred · target contact geometry is fixed',
 '선택된 원방법과 같은 비용·물성·제어 설정':'Same objective, mechanical parameters, and controller settings as the selected method',
 '유형·팔 배정 후보 탐색 및 전체 구간 최적화 결과 · midpoint 구성 업데이트 변형':'Contact-type/arm-assignment selection and complete-sequence optimization · midpoint configuration-update variants',
 '측정된 패널 높이 · source refinement + target realization + 첫 구간 CITO · midpoint 구성 업데이트 변형':'Measured panel height · source refinement, target realization, and initial-transition CITO · midpoint configuration-update variant',
};
const hangul=/[\uac00-\ud7a3]/;
export const eventLabel=id=>events[id]||id;
export function recordingTitle(id,fallback=''){
 if(titles[id])return titles[id];
 const full=id.match(/^full_(panel|box|assembly)_(.+)$/);
 if(full)return `${taskTitles[full[1]]} · ${ablationTitles[full[2]]||full[2]}`;
 const forced=id.match(/^counterfactual_(panel|box|assembly)_(.+)$/);
 if(forced)return `${taskTitles[forced[1]]} · forced ${forced[2].split('').join('/')}`;
 const source=id.match(/^robust_refinement_(polytope|nominal)_(-?\d+)N$/);
 if(source){const force=Number(source[2]);return `Source wrench robustness ${source[1]==='polytope'?'ON':'OFF'} · ${force===0?'no disturbance':`${Math.abs(force)} N ${force<0?'upward':'downward'}`}`;}
 return hangul.test(fallback)?id.replaceAll('_',' '):fallback||id;
}
export function englishRecording(original){
 const data={...original,title:recordingTitle(original.id,original.title)};
 if(original.display)data.display=Object.fromEntries(Object.entries(original.display).map(([k,v])=>[k,captions[v]||v]));
 if(original.events)data.events=original.events.map(e=>({...e,id:eventLabel(e.id)}));
 if(original.keyframes)data.keyframes=original.keyframes.map(k=>({...k,id:eventLabel(k.id)}));
 return data;
}
