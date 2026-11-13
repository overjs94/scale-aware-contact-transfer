// Organize by experimental purpose, independently of robot or physics engine.
// Membership in a paper task does not certify execution of the paper pipeline.
import {recordingTitle} from './presentation-en.js';
export const categories = [
  { id: "paper", title: "Primary tasks", description: "Multi-arm realization of supplied contracted contacts. Complete-sequence implementations, legacy baselines, and their validation scope are identified separately.", groups: ["Rigid-object manipulation", "Mobile cooperation", "Articulated-object extension", "Assembly extension"] },
  { id: "embodiment", title: "Embodiment checks", description: "URDF, kinematics, and joint-tracking checks for individual embodiments. These are separate from cooperative object-manipulation tasks.", groups: ["Robot arms · joint tracking", "Humanoid · kinematics"] },
  { id: "diagnostic", title: "Diagnostics & robustness", description: "Component checks for disturbance response, contact interfaces, passive joints, and restricted contact-implicit trajectory optimization.", groups: ["External disturbances", "Multi-arm planning & interfaces", "Single-arm passive-joint tasks", "Gripper actuation & force"] },
];

const entries = [
  ["full_panel", "paper", 0, "Panel reorientation and placement", "Contact-type/arm-assignment selection → CITO → contact execution"],
  ["full_box", "paper", 2, "Free articulated box opening", "Functional contacts on the base and lid → three-arm realization"],
  ["full_assembly", "paper", 3, "Tabletop–leg assembly", "Two free parts → insertion, load transfer, and release"],
  ["panel_fixture", "paper", 0, "Panel transport, placement, and release", "Panda × 3 · G / P / P", true],
  ["mobile_panel", "paper", 1, "Workspace extension · RBY1 + Panda", "Mobile dual arm + fixed arm", true],
  ["free_box_lid", "paper", 2, "Free box · lid opening", "Panda × 3 · base grasp/support + lid manipulation", true],
  ["free_folding_object", "paper", 2, "Free folding panel", "Panda × 3 · base support + hinged-panel manipulation", true],
  ["free_portable_drawer", "paper", 2, "Free cabinet · drawer opening", "Panda × 3 · base support + drawer manipulation", true],
  ["table_assembly", "paper", 3, "Tabletop–leg assembly", "Panda × 3 · two free parts", true],
  ["franka_panda", "embodiment", 0, "Franka Panda", "7 DoF · joint tracking"],
  ["rb5", "embodiment", 0, "Rainbow RB5-850E", "6 DoF · joint tracking"],
  ["rb3", "embodiment", 0, "Rainbow RB3", "6 DoF · joint tracking"],
  ["ur5e", "embodiment", 0, "UR5e", "6 DoF · joint tracking"],
  ["kinova_gen3", "embodiment", 0, "Kinova Gen3", "7 DoF · joint tracking"],
  ["kuka_iiwa7", "embodiment", 0, "KUKA iiwa 7", "7 DoF · joint tracking"],
  ["xarm7", "embodiment", 0, "UFACTORY xArm7", "7 DoF · joint tracking"],
  ["unitree_g1", "embodiment", 1, "Unitree G1", "29 DoF + floating base · kinematics"],
  ["free_box_push", "diagnostic", 0, "Free box · 5 N disturbance", "Three arms · external base load with force arrow", "free_box_lid"],
  ["mobile_push_2", "diagnostic", 0, "Mobile cooperation · 2 N disturbance", "RBY1 + Panda · force arrow", "mobile_panel"],
  ["mobile_push_8", "diagnostic", 0, "Mobile cooperation · 8 N disturbance", "RBY1 + Panda · force arrow", "mobile_panel"],
  ["mobile_push_25", "diagnostic", 0, "Mobile cooperation · 25 N disturbance", "RBY1 + Panda · force arrow", "mobile_panel"],
  ["dual_cito", "diagnostic", 1, "Dual arm · restricted CITO", "Cartesian arm × 2 · optimized trajectory"],
  ["panda_robotiq_lift", "diagnostic", 1, "Panda × 3 · Robotiq initial lift", "G / P / P · optimization and native contact execution"],
  ["panda_coupled_lift", "diagnostic", 1, "Panda × 3 · optimized initial lift", "G / P / P · initial-transition planning and execution"],
  ["robotiq_2f85", "diagnostic", 3, "Robotiq 2F-85 · 40 N per jaw", "Original meshes · passive linkage, actuator, and free block"],
  ["three_arm", "diagnostic", 1, "Cartesian × 3 · contact interfaces", "Point / gripper / suction"],
  ["three_franka", "diagnostic", 1, "Panda × 3 · contact interfaces", "Point / gripper / suction", true],
  ["hinged_door", "diagnostic", 2, "Fixed hinge · door opening", "Single arm · unilateral point contact", true],
  ["folding_panel", "diagnostic", 2, "Fixed hinge · panel unfolding", "Single arm · unilateral point contact", true],
  ["drawer", "diagnostic", 2, "Fixed rail · drawer opening", "Single arm · gripper-mediated contact", true],
];

const catalog = new Map(entries.map(([id, category, group, title, detail, preset], order) =>
  [id, { id, category, group, title, detail, preset: preset === true ? id : preset, order }]));

export function libraryEntry(model) {
  if(/^margin_(hold|platform|compact)_/.test(model.id))return {id:model.id,category:'diagnostic',group:0,title:model.title,detail:'Inside-margin load · matched ON/OFF · recovery observed',order:entries.length-1};
  if(model.id.startsWith('robust_refinement_'))return {id:model.id,category:'diagnostic',group:0,title:recordingTitle(model.id,model.title),detail:'Three arms · box-vertex source robustness · matched disturbances',order:entries.length};
  if(model.id.startsWith('counterfactual_'))return {id:model.id,category:'diagnostic',group:1,title:recordingTitle(model.id,model.title),detail:'Alternative contact type · complete-sequence comparison',order:entries.length};
  const ablation = model.id.match(/^full_(panel|box|assembly)_(.+)$/);
  if (ablation) return {id:model.id,category:'diagnostic',group:ablation[2]==='external_push'?0:1,title:recordingTitle(model.id,model.title),detail:'Complete task · '+ablation[2],order:entries.length};
  return catalog.get(model.id) || catalog.get(model.scenario) || {
    id: model.id, category: "diagnostic", group: 1, title: model.title,
    detail: "Component check", order: entries.length,
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
  if (data.known_modeling_issues?.length) {
    status = 'Historical geometry · revalidation required';
    detail = 'The historical base handle has a 50 mm gap to the wall without connecting geometry. Its original measurements are preserved, but do not validate the repaired geometry. See Free articulated box opening (full_box) for the repaired model and seven-condition comparison.';
  } else if(data.comparison_recording){
    status=(data.study_kind==='source_robustness'?'Source robustness · ':'Forced contact type · ')+(report.execution_passed?'PASS':'FAIL');
    detail=data.study_kind==='source_robustness'?'A controlled source-refinement diagnostic with box-vertex robustness enabled or disabled. Controller, mechanics, and disturbances are matched; transferred target contact sites are fixed during stationary support. This is not a moving-object CITO test or a closed-loop stability proof.':'Complete-sequence planning and native execution with a forced alternative contact type. Objective values, planning rejection, and execution-criterion violations are reported separately.';
    if(report.margin_hold){
      const check=report.margin_hold.certificate;
      status=`Margin validation · ${report.margin_hold.outcome.replaceAll('_',' ')}`;
      detail=`Same controller and applied load in ON/OFF. Initial static certificate: ${check.inside_initial_static_box?'inside':'outside'} this condition’s box · peak / margin = ${check.load_to_margin_ratio.toFixed(3)}. Recovery after load removal is measured. Finite-time loss of support is distinct from mathematical divergence.`;
    }
  } else if (data.full_task_recording) {
    status = "Complete sequence · " + (report.execution_passed ? "execution checks passed" : "failed checks retained");
    detail = "Recorded contact-type/arm-assignment selection, moving-object CITO, load-transfer QPs, release dynamics, and native contact execution. The implementation uses midpoint variants; source geometric fallback and ablation results are stated explicitly.";
    if(report.variant==="fixed_path") detail="Fixed-path baseline: optimize reactions and torques on a prescribed interpolated path. Endpoints and phase durations match the complete plan. Integration residuals and measured contact-execution results are reported separately.";
  } else if (full) {
    status = "Sec. III-B–IV execution record";
    detail = "Execution provenance covers all listed stages. See the report for individual checks.";
  } else if (entry.category === "paper") {
    status = "Nominal baseline · complete optimization not verified";
    detail = "A nominal multi-arm contact-dynamics baseline, not a successful complete Sec. III-B–IV planning result.";
  } else if (entry.category === "embodiment") {
    status = data.engine === "kinematic_preview" ? "Kinematic preview" : "Joint-tracking check";
    detail = data.engine === "kinematic_preview" ? "Playback of URDF joint and floating-base coordinates. Object contact, balance control, and locomotion are not validated." : "Recorded joint-command tracking for one robot model. This does not validate cooperative object manipulation or contact-force control.";
  } else if (data.scenario === "coupled_panel_lift") {
    status = "Multi-arm initial lift · " + (report.execution_passed ? "checked" : "check failed");
    detail = "Source refinement, target realization, initial-transition CITO, and contact execution for supplied G/P/P interfaces. This midpoint implementation does not cover complete reorientation, placement, release, or Sec. III-D search.";
  } else if (data.scenario === "robotiq_clamp") {
    status = "Gripper-force component check";
    detail = "A free-block grasp-force test using the native gripper mechanism, passive linkage, and finite actuator. This component check is separate from multi-arm contact realization.";
  } else if ((data.external_disturbances || []).length) {
    const passed = report.robustness?.all_task_checks_passed ?? report.execution_passed;
    status = "Disturbance test" + (passed === false ? " · failed checks retained" : "");
    detail = "External loads are applied during execution of a reference plan. Arrows show the applied force; contact and load-limit violations remain in the report.";
  } else if (data.engine === "cito_trajectory") {
    status = "Restricted CITO · component check";
    detail = "An optimized trajectory for a simplified contact-candidate model. This does not establish completion of the full manuscript evaluation.";
  } else {
    status = "Contact-interface component check";
    detail = entry.group === 2 ? "A single-arm component test with a world-fixed passive joint. See Primary tasks for cooperative manipulation of a free articulated object." : "A component test of supplied contact interfaces and paths. Its scope is separate from complete Sec. III-B–IV execution.";
  }
  return { entry, category, status, detail, full };
}
