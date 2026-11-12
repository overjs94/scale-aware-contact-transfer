// Organize by experimental purpose, independently of robot or physics engine.
// Membership in a paper task does not certify execution of the paper pipeline.
export const categories = [
  { id: "paper", title: "논문 목표", description: "Contact contraction을 다중 팔로 실현하는 목표 과제입니다. 전체 계획·실행 기록과 기존 기준 동작을 구분하며, 각 기록의 검증 범위를 표시합니다.", groups: ["핵심 과제 · 단일 강체", "확장 · 모바일 협력", "확장 · 자유 articulated", "보조 · 다물체 조립"] },
  { id: "embodiment", title: "단일 embodiment", description: "각 로봇의 URDF·기구학·관절 추종을 확인합니다. 논문 목표의 협력 물체 조작 실험과 구분합니다.", groups: ["로봇팔 · 관절 추종", "휴머노이드 · 기구학"] },
  { id: "diagnostic", title: "요소·강건성", description: "외란 응답, 접촉 도구와 수동 관절, 제한 CITO를 개별적으로 확인하는 진단 실험입니다.", groups: ["외력 · 강건성", "다중 팔 · 도구·계획 검증", "단일 팔 · 수동 관절", "그리퍼 · 구동·파지력"] },
];

const entries = [
  ["full_panel", "paper", 0, "패널 · 전체 구간 최적화", "유형·팔 배정 탐색 → CITO → 접촉 실행"],
  ["full_box", "paper", 2, "자유 박스 · 전체 구간 최적화", "본체·뚜껑 functional contact → 세 팔"],
  ["full_assembly", "paper", 3, "가구 조립 · 전체 구간 최적화", "두 자유 부품 → 삽입·하중 전달·릴리스"],
  ["panel_fixture", "paper", 0, "패널 운반 → 지그 → 릴리스", "Panda × 3 · G / P / P", true],
  ["mobile_panel", "paper", 1, "작업영역 확장 · RBY1 + Panda", "모바일 듀얼암 + 고정 팔", true],
  ["free_box_lid", "paper", 2, "자유 박스 · 뚜껑 열기", "Panda × 3 · 본체 grip / support + 뚜껑", true],
  ["free_folding_object", "paper", 2, "자유 패널 · 날개 펼치기", "Panda × 3 · 본체 지지 + 날개", true],
  ["free_portable_drawer", "paper", 2, "자유 수납함 · 서랍 당기기", "Panda × 3 · 본체 지지 + 서랍", true],
  ["table_assembly", "paper", 3, "책상 상판·다리 조립", "Panda × 3 · 두 자유 부품", true],
  ["franka_panda", "embodiment", 0, "Franka Panda", "7 DoF · 관절 추종"],
  ["rb5", "embodiment", 0, "Rainbow RB5-850E", "6 DoF · 관절 추종"],
  ["rb3", "embodiment", 0, "Rainbow RB3", "6 DoF · 관절 추종"],
  ["ur5e", "embodiment", 0, "UR5e", "6 DoF · 관절 추종"],
  ["kinova_gen3", "embodiment", 0, "Kinova Gen3", "7 DoF · 관절 추종"],
  ["kuka_iiwa7", "embodiment", 0, "KUKA iiwa 7", "7 DoF · 관절 추종"],
  ["xarm7", "embodiment", 0, "UFACTORY xArm7", "7 DoF · 관절 추종"],
  ["unitree_g1", "embodiment", 1, "Unitree G1", "29 DoF + floating base · 기구학"],
  ["free_box_push", "diagnostic", 0, "자유 박스 · 외력 5 N", "3-arm · 본체에 외력 · 화살표 표시", "free_box_lid"],
  ["mobile_push_2", "diagnostic", 0, "모바일 협력 · 외력 2 N", "RBY1 + Panda · 화살표 표시", "mobile_panel"],
  ["mobile_push_8", "diagnostic", 0, "모바일 협력 · 외력 8 N", "RBY1 + Panda · 화살표 표시", "mobile_panel"],
  ["mobile_push_25", "diagnostic", 0, "모바일 협력 · 외력 25 N", "RBY1 + Panda · 화살표 표시", "mobile_panel"],
  ["dual_cito", "diagnostic", 1, "Dual-arm · 제한 CITO", "Cartesian arm × 2 · 최적화 궤적"],
  ["panda_robotiq_lift", "diagnostic", 1, "Panda × 3 · Robotiq 첫 lift", "G / P / P · 최적화 + native 접촉 실행"],
  ["panda_coupled_lift", "diagnostic", 1, "Panda × 3 · 최적화된 첫 lift", "G / P / P · 첫 구간 계획·실행"],
  ["robotiq_2f85", "diagnostic", 3, "Robotiq 2F-85 · 양 jaw 40 N", "실제 mesh · 수동 링크·구동기 · 자유 블록"],
  ["three_arm", "diagnostic", 1, "Cartesian × 3 · 접촉 도구", "Point / gripper / suction"],
  ["three_franka", "diagnostic", 1, "Panda × 3 · 접촉 도구", "Point / gripper / suction", true],
  ["hinged_door", "diagnostic", 2, "고정 힌지 · 문 열기", "단일 팔 · point contact", true],
  ["folding_panel", "diagnostic", 2, "고정 힌지 · 패널 펼치기", "단일 팔 · point contact", true],
  ["drawer", "diagnostic", 2, "고정 레일 · 서랍 당기기", "단일 팔 · gripper", true],
];

const catalog = new Map(entries.map(([id, category, group, title, detail, preset], order) =>
  [id, { id, category, group, title, detail, preset: preset === true ? id : preset, order }]));

export function libraryEntry(model) {
  const ablation = model.id.match(/^full_(panel|box|assembly)_(.+)$/);
  if (ablation) return {id:model.id,category:'diagnostic',group:ablation[2]==='external_push'?0:1,title:model.title,detail:'전체 과제 · '+ablation[2],order:entries.length};
  return catalog.get(model.id) || catalog.get(model.scenario) || {
    id: model.id, category: "diagnostic", group: 1, title: model.title,
    detail: "개별 검증", order: entries.length,
  };
}

export function experimentPresets() {
  return entries.map(([id]) => catalog.get(id)).filter(entry => entry.preset === entry.id);
}

export function recordingScope(data) {
  const entry = libraryEntry(data), category = categories.find(c => c.id === entry.category);
  const report = data.report || {};
  const sections = report.paper_sections_executed || {};
  const full = report.full_paper_pipeline_executed === true &&
    ["III-B", "III-C", "III-D", "IV"].every(section => sections[section] === true);
  let status, detail;
  if (data.full_task_recording) {
    status = "전체 구간 · " + (report.execution_passed ? "검증 통과" : "검사 실패 포함");
    detail = "접촉 유형·팔 배정 후보, 이동 CITO·하중 전달 QP·릴리스 동역학과 native 접촉 실행을 기록합니다. Midpoint 구성 업데이트 변형이며 source 실패 시 기하 prior 사용 여부와 ablation 결과를 보고서에 표시합니다.";
    if(report.variant==="fixed_path") detail="고정된 보간 경로에서 힘·토크만 계산하는 비교 실행입니다. 전체 계획과 endpoint·구간 시간을 맞추며, CITO의 동역학 적분 잔차와 실제 접촉 실행 결과를 구분하여 표시합니다.";
  } else if (full) {
    status = "Sec. III-B–IV 실행 기록";
    detail = "전체 단계의 실행 provenance가 포함된 기록입니다. 개별 검사 결과는 실행 보고서에서 확인하세요.";
  } else if (entry.category === "paper") {
    status = "기준 동작 · 전체 최적화 미검증";
    detail = "다중 팔의 접촉 동역학 기준 실행입니다. Sec. III-B–IV 전체 계획의 성공 결과는 아직 아닙니다.";
  } else if (entry.category === "embodiment") {
    status = data.engine === "kinematic_preview" ? "기구학 미리보기" : "관절 추종 검증";
    detail = data.engine === "kinematic_preview" ? "URDF 관절·floating base의 좌표 재생입니다. 물체 접촉, 균형 제어와 보행을 검증하지 않았습니다." : "단일 로봇 모델의 관절 명령 추종 기록입니다. 협력 물체 조작과 접촉력 제어의 검증 결과는 아닙니다.";
  } else if (data.scenario === "coupled_panel_lift") {
    status = "다중 팔 · 첫 lift " + (report.execution_passed ? "검증" : "검사 실패");
    detail = "Source refinement와 target realization, 주어진 G/P/P 배치의 첫 구간 CITO 및 접촉 실행 기록입니다. Midpoint 구성 업데이트 변형이며, 전체 재배향·안착·릴리스와 III-D 탐색의 검증은 남아 있습니다.";
  } else if (data.scenario === "robotiq_clamp") {
    status = "그리퍼 파지력 · 요소 검증";
    detail = "수동 링크를 포함한 실제 그리퍼 기구와 유한 구동기, 자유 블록의 파지력 시험입니다. 논문 목표의 다중 팔 contact realization과 구분합니다.";
  } else if ((data.external_disturbances || []).length) {
    const passed = report.robustness?.all_task_checks_passed ?? report.execution_passed;
    status = "외란 시험" + (passed === false ? " · 검사 실패 포함" : "");
    detail = "기준 계획 실행 중 외력을 가한 기록입니다. 외력 크기·방향은 화살표로 표시하며, 접촉·하중 한계 위반도 보고서에 유지합니다.";
  } else if (data.engine === "cito_trajectory") {
    status = "제한 CITO · 요소 검증";
    detail = "단순화한 접촉 후보 모델의 최적화 궤적입니다. 전체 논문 실험의 완료를 뜻하지 않습니다.";
  } else {
    status = "접촉·도구 · 요소 검증";
    detail = entry.group === 2 ? "세계에 고정된 수동 관절을 단일 팔로 조작하는 요소 시험입니다. 자유 articulated body의 다중 팔 목표 과제는 논문 목표에서 확인하세요." : "주어진 접촉 도구와 경로의 동작을 확인하는 요소 시험입니다. 전체 Sec. III-B–IV 실행 여부와 별도로 구분합니다.";
  }
  return { entry, category, status, detail, full };
}
