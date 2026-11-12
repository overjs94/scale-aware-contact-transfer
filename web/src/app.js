import { createStudio } from "./studio.js";
import { initializeExperiment } from "./experiment.js";
import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { applyRecordedPose, disposeModel, loadURDFModel } from "./urdf-view.js";

const $ = (id) => document.getElementById(id);
let dataset, frameFloat = 0, playing = true, loop = true, speed = 1, loadToken = 0;
let backend = false, geometry = [], objectMesh, chartValues = [], downloadUrl, displayedPositions = [];
let urdfModels = [], geometryToken = 0;
const scene = new THREE.Scene();
scene.fog = new THREE.FogExp2(0x0c1423, .06);
const camera = new THREE.PerspectiveCamera(39, 1, .01, 100);
camera.up.set(0, 0, 1);
let renderer, canvasFallback = false;
try { renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true }); }
catch {
  canvasFallback = true;
  const canvas = document.createElement("canvas");
  canvas.setAttribute("aria-label", "CPU 투영 3D 로봇 뷰어");
  renderer = { domElement: canvas, setPixelRatio() {}, setClearColor() {},
    setSize(width, height) { canvas.width = width * Math.min(devicePixelRatio, 2); canvas.height = height * Math.min(devicePixelRatio, 2); },
    render: drawCanvasScene };
}
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
renderer.setClearColor(0x0c1423, 0);
renderer.outputColorSpace = THREE.SRGBColorSpace;
renderer.toneMapping = THREE.ACESFilmicToneMapping;
renderer.toneMappingExposure = 1.25;
$("viewport").prepend(renderer.domElement);
const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true; controls.dampingFactor = .07;
controls.minDistance = .25; controls.maxDistance = 12;
scene.add(new THREE.AmbientLight(0xb9cce5, 1.7));
const sun = new THREE.DirectionalLight(0xddeaff, 3.5);
sun.position.set(3, -2, 5); scene.add(sun);
const rim = new THREE.DirectionalLight(0x4ac7ba, 1.8);
rim.position.set(-2, 3, 2); scene.add(rim);
const forceArrow=new THREE.ArrowHelper(new THREE.Vector3(0,1,0),new THREE.Vector3(),.5,0xef683c,.08,.04);forceArrow.visible=false;scene.add(forceArrow);
let grid;
const studio=createStudio(scene,renderer,sun,rim,canvasFallback);
studio.set("studio");
const robotGroup = new THREE.Group(); scene.add(robotGroup);
const up = new THREE.Vector3(0, 1, 0);
let center = new THREE.Vector3(0, 0, .4), extent = 1.;

$("scene-style").onchange=()=>studio.set($("scene-style").value,grid);

function resize() {
  const box = $("viewport").getBoundingClientRect();
  renderer.setSize(box.width, box.height);
  camera.aspect = box.width / box.height; camera.updateProjectionMatrix();
}
new ResizeObserver(resize).observe($("viewport")); resize();
function toast(message) { $("toast").textContent = message; $("toast").classList.add("visible"); setTimeout(() => $("toast").classList.remove("visible"), 6000); }
function number(x) { return x === null || x === undefined ? "—" : Math.abs(x) < .001 && x !== 0 ? x.toExponential(1) : x.toFixed(4); }
function engineLabel(data) {
  if (data.engine === "mujoco_contact_recording") return "MuJoCo · 접촉 동역학";
  return data.engine === "kinematic_preview" ? "기구학 미리보기" : data.engine === "cito_trajectory" ? "CITO · 최적화 궤적" : data.live ? "MuJoCo · 새 계산 결과" : "MuJoCo · 계산 결과";
}
function resetCamera() {
  controls.target.copy(center);
  camera.position.copy(center).add(new THREE.Vector3(1.6, -2.4, 1.6).multiplyScalar(extent));
  controls.update();
}
function buildGeometry(data) {
  ++geometryToken; urdfModels = [];
  while (robotGroup.children.length) {
    disposeModel(robotGroup.children[0]);
  }
  geometry = []; objectMesh = undefined;
  for (const item of (data.instances?.some(i=>["static_jig","articulated_object"].includes(i.role)) ? [] : data.environment) || []) {
    if (item.type !== "box") continue;
    const mesh = new THREE.Mesh(new THREE.BoxGeometry(...item.size), new THREE.MeshStandardMaterial({color:item.color, transparent:true, opacity:.6}));
    mesh.position.fromArray(item.position); if(item.quaternion_xyzw) mesh.quaternion.fromArray(item.quaternion_xyzw); robotGroup.add(mesh);
  }
  const points = data.positions[0].map(p => new THREE.Vector3(...p));
  const box = new THREE.Box3().setFromPoints(points);
  center = box.getCenter(new THREE.Vector3());
  extent = Math.max(.4, box.getSize(new THREE.Vector3()).length() * .65);
  const radius = Math.max(.009, Math.min(.022, extent * .018));
  const bodyMaterial = new THREE.MeshStandardMaterial({ color: 0x97a9bd, metalness: .48, roughness: .32 });
  const jointMaterial = new THREE.MeshStandardMaterial({ color: 0x256c75, metalness: .5, roughness: .3 });
  const endpointMaterial = new THREE.MeshStandardMaterial({ color: 0xf0ba78, metalness: .3, roughness: .32, emissive: 0x432309, emissiveIntensity: .25 });
  data.edges.forEach(([a, b]) => {
    const mesh = new THREE.Mesh(new THREE.CylinderGeometry(radius * .75, radius * .75, 1, 12), bodyMaterial);
    robotGroup.add(mesh); geometry.push({ mesh, a, b, cylinder: true });
  });
  const endpoints = new Set(data.link_names.map((n, i) => n === data.frame || n.endsWith("link2") ? i : -1));
  data.link_names.forEach((name, i) => {
    if (data.object?.index === i) return;
    const mesh = new THREE.Mesh(new THREE.SphereGeometry(radius * (endpoints.has(i) ? 1.6 : 1.25), 18, 12), endpoints.has(i) ? endpointMaterial : jointMaterial);
    robotGroup.add(mesh); geometry.push({ mesh, a: i });
  });
  if (data.object) {
    objectMesh = new THREE.Mesh(new THREE.BoxGeometry(...data.object.size),
      new THREE.MeshStandardMaterial({ color: 0xf1ba79, metalness: .15, roughness: .45, transparent: true, opacity: .8 }));
    robotGroup.add(objectMesh);
    const outline = new THREE.LineSegments(new THREE.EdgesGeometry(objectMesh.geometry), new THREE.LineBasicMaterial({ color: 0xffd9aa }));
    objectMesh.add(outline);
  }
  if (grid) { scene.remove(grid); grid.geometry.dispose(); }
  grid = new THREE.GridHelper(extent * 5, 20, 0x36506c, 0x1d3048);
  grid.rotation.x = Math.PI / 2;
  grid.position.set(center.x, center.y, Math.min(0, box.min.z) - .005);
  grid.material.transparent = true; grid.material.opacity = .8; scene.add(grid);
  studio.set($("scene-style")?.value||"studio",grid);
  resetCamera();
}
async function loadGeometry() {
  const token = ++geometryToken, data = dataset, mode = $("geometry-mode").value;
  urdfModels.forEach(item => disposeModel(item.robot)); urdfModels = [];
  $("geometry-status").dataset.state = "loading";
  updateGeometry(frameFloat);
  if (mode === "skeleton") {
    $("geometry-status").textContent = data.id === "dual_cito"
      ? "두 Cartesian arm과 물체의 좌표 재생입니다. 이 제한 모델에는 외형 mesh가 없습니다."
      : "URDF 링크 좌표를 표시합니다. Visual 또는 collision을 선택하면 원본 형상을 불러옵니다.";
    $("geometry-status").dataset.state = "skeleton";
    $("loading").classList.add("hidden"); return;
  }
  $("geometry-status").textContent = "URDF와 mesh를 불러오는 중입니다…";
  $("loading").classList.remove("hidden");
  try {
    const instances = data.instances || [{ manifest: `./models/${data.id}/model.json` }];
    const results = await Promise.allSettled(instances.map(async spec => {
      const robot = await loadURDFModel(spec.manifest, mode);
      studio.materialize(robot,spec.role,mode);
      const recording = data.instances ? { ...spec, q: data.q.map(q => spec.q_indices.map(i => q[i])) } : data;
      try { applyRecordedPose(robot, recording, frameFloat); }
      catch (error) { disposeModel(robot); throw error; }
      return { robot, recording };
    }));
    const loaded = results.filter(r => r.status === "fulfilled").map(r => r.value);
    const failed = results.find(r => r.status === "rejected");
    if (token !== geometryToken || data !== dataset || failed) {
      loaded.forEach(item => disposeModel(item.robot));
      if (failed) throw failed.reason;
      return;
    }
    urdfModels = loaded; loaded.forEach(item => robotGroup.add(item.robot));
    $("geometry-status").textContent = canvasFallback
      ? "URDF/mesh 로드 완료. WebGL을 사용할 수 없어 링크 좌표를 CPU로 표시합니다."
      : mode === "collision"
        ? "URDF collision 형상을 표시합니다. 형상 표시는 접촉 계산·충돌 검증 결과와 별개입니다."
        : data.engine === "mujoco_contact_recording"
          ? data.scenario === "mobile_panel" ? "공식 RBY1 URDF·mesh, 차동구동 모바일 베이스와 고정 Panda의 실제 접촉 실행입니다." : data.articulated_object ? "수동 관절의 URDF와 측정된 움직임을 표시합니다. 물체 actuator 없이 로봇 접촉으로 구동했습니다." : data.scenario === "panel_fixture" ? "원본 Panda URDF·mesh, 자유 패널 URDF와 고정 지그를 표시합니다. 소형 접촉 refinement 후 실행한 기록입니다." : data.scenario === "table_assembly" ? "Panda 3대와 두 자유 부품의 실제 접촉 실행입니다. 상판·다리의 visual/collision URDF에 빈 socket과 peg 형상을 포함합니다." : "3개 URDF 팔과 도구의 실제 접촉 실행입니다. Point는 구형 tool, gripper는 두 손가락 접촉, suction은 압력 제한 seal 모델입니다."
          : "URDF visual·mesh의 origin과 scale을 적용했습니다. 기록된 시뮬레이션은 관절 추종 검증입니다.";
    $("geometry-status").dataset.state = "ready";
    $("geometry-status").dataset.model = data.id;
    updateGeometry(frameFloat);
  } catch (error) {
    if (token !== geometryToken) return;
    $("geometry-status").textContent = error.message + " 링크 좌표 표시를 선택하거나 네트워크를 확인해 주세요.";
    $("geometry-status").dataset.state = "error";
    toast(error.message);
  } finally {
    if (token === geometryToken) $("loading").classList.add("hidden");
  }
}
function updateGeometry(value) {
  if (!dataset) return;
  const lo = Math.floor(value), hi = Math.min(lo + 1, dataset.times.length - 1), f = value - lo;
  const positions = dataset.positions[lo].map((p, i) => new THREE.Vector3(...p).lerp(new THREE.Vector3(...dataset.positions[hi][i]), f));
  geometry.forEach(item => {
    const a = positions[item.a];
    if (!item.cylinder) { item.mesh.visible = $("geometry-mode").value === "skeleton"; item.mesh.position.copy(a); return; }
    const b = positions[item.b], delta = b.clone().sub(a), length = delta.length();
    item.mesh.visible = $("geometry-mode").value === "skeleton" && length > 1e-6;
    item.mesh.position.copy(a).add(b).multiplyScalar(.5);
    item.mesh.scale.set(1, length, 1);
    if (length > 1e-6) item.mesh.quaternion.setFromUnitVectors(up, delta.normalize());
  });
  if (objectMesh) {
    objectMesh.position.copy(positions[dataset.object.index]);
    if (dataset.object.poses) objectMesh.quaternion.fromArray(dataset.object.poses[lo], 3)
      .slerp(new THREE.Quaternion().fromArray(dataset.object.poses[hi], 3), f);
  }
  urdfModels.forEach(item => applyRecordedPose(item.robot, item.recording, value));
  const time=dataset.times[lo]*(1-f)+dataset.times[hi]*f;const pulses=dataset.external_disturbances||[];const pulse=pulses.find(p=>time>=p.start&&time<=p.start+p.duration);forceArrow.visible=false;
  if(pulse&&dataset.objects?.[0]?.poses){const object=dataset.objects[0],pose=object.poses[lo],next=object.poses[hi],quat=new THREE.Quaternion().fromArray(pose,3).slerp(new THREE.Quaternion().fromArray(next,3),f);const origin=new THREE.Vector3(...pose.slice(0,3)).lerp(new THREE.Vector3(...next.slice(0,3)),f).add(new THREE.Vector3(...pulse.point_body_m).applyQuaternion(quat));const magnitude=Math.sin(Math.PI*(time-pulse.start)/pulse.duration)**2;const force=new THREE.Vector3(...pulse.force_world_N).multiplyScalar(magnitude);if(force.length()>.01){forceArrow.position.copy(origin);forceArrow.setDirection(force.clone().normalize());forceArrow.setLength(.12+force.length()*.015,.08,.04);forceArrow.visible=true;}}
  $('disturbance-state').hidden=!pulses.length;
  $('disturbance-state').textContent=pulse?`외력 적용 중 · ${(Math.sin(Math.PI*(time-pulse.start)/pulse.duration)**2*Math.hypot(...pulse.force_world_N)).toFixed(2)} N · world frame` : pulses.length?`외력: ${pulses.map(p=>`${p.start}–${p.start+p.duration} s / 최대 ${Math.hypot(...p.force_world_N)} N`).join(', ')}`:'';
  displayedPositions = positions;
}
// Preserve orbit/zoom and real URDF coordinates when WebGL is unavailable.
// This projects the same 3D points through the same THREE camera on the CPU.
function drawCanvasScene() {
  const canvas = renderer.domElement, ctx = canvas.getContext("2d");
  const w = canvas.width, h = canvas.height, scale = Math.min(devicePixelRatio, 2);
  ctx.clearRect(0, 0, w, h);
  if(studio.mode==="studio"){ctx.fillStyle="#dce3e9";ctx.fillRect(0,0,w,h);}
  if (!dataset || !displayedPositions.length) return;
  camera.updateMatrixWorld();
  const project = p => { const v = p.clone().project(camera); return [(v.x + 1) * w / 2, (1 - v.y) * h / 2, v.z]; };
  const line = (a, b, color, width) => {
    const p = project(a), q = project(b);
    if (p[2] > 1 || q[2] > 1) return;
    ctx.strokeStyle = color; ctx.lineWidth = width * scale; ctx.lineCap = "round";
    ctx.beginPath(); ctx.moveTo(p[0], p[1]); ctx.lineTo(q[0], q[1]); ctx.stroke();
  };
  const z = grid.position.z, half = extent * 2.5;
  for (let i = -10; i <= 10; ++i) {
    const d = half * i / 10, color = i === 0 ? "#36506c" : "#1d3048";
    line(new THREE.Vector3(center.x - half, center.y + d, z), new THREE.Vector3(center.x + half, center.y + d, z), color, .8);
    line(new THREE.Vector3(center.x + d, center.y - half, z), new THREE.Vector3(center.x + d, center.y + half, z), color, .8);
  }
  for (const item of dataset.environment || []) {
    if (item.type !== "box") continue;
    const corners = Array.from({length:8}, (_, i) => new THREE.Vector3(...item.size.map((s,j) => ((i >> j) & 1 ? .5 : -.5)*s)).applyQuaternion(new THREE.Quaternion().fromArray(item.quaternion_xyzw || [0,0,0,1])).add(new THREE.Vector3(...item.position)));
    corners.forEach((p,i) => { for (let j=0;j<3;j++) if (!(i & (1<<j))) line(p,corners[i | (1<<j)],"#64768b",1); });
  }
  const edges = [...dataset.edges].sort((a, b) =>
    project(displayedPositions[b[0]])[2] - project(displayedPositions[a[0]])[2]);
  edges.forEach(([a, b]) => {
    line(displayedPositions[a], displayedPositions[b], "#546b82", 10);
    line(displayedPositions[a], displayedPositions[b], "#aabbd0", 5);
  });
  displayedPositions.forEach((p, i) => {
    if (dataset.object?.index === i) return;
    const v = project(p); if (v[2] > 1) return;
    const endpoint = dataset.link_names[i] === dataset.frame || dataset.link_names[i].endsWith("link2");
    ctx.fillStyle = endpoint ? "#f0ba78" : "#3da99e";
    ctx.strokeStyle = "#112d3b"; ctx.lineWidth = 2 * scale;
    ctx.beginPath(); ctx.arc(v[0], v[1], (endpoint ? 6 : 4) * scale, 0, Math.PI * 2); ctx.fill(); ctx.stroke();
  });
  if (dataset.object) {
    const origin = displayedPositions[dataset.object.index], size = dataset.object.size;
    const corners = Array.from({length:8}, (_, i) => new THREE.Vector3(...size.map((s,j) => ((i >> j) & 1 ? .5 : -.5) * s))
      .applyQuaternion(objectMesh.quaternion).add(origin));
    corners.forEach((p,i) => { for(let j=0;j<3;j++) if (!(i & (1<<j))) line(p,corners[i | (1<<j)],"#f1ba79",2); });
  }
  for(const part of dataset.objects || []) {
    const lo=Math.floor(frameFloat), hi=Math.min(lo+1,dataset.times.length-1), f=frameFloat-lo;
    const position=new THREE.Vector3().fromArray(part.poses[lo]).lerp(new THREE.Vector3().fromArray(part.poses[hi]),f);
    const rotation=new THREE.Quaternion().fromArray(part.poses[lo],3).slerp(new THREE.Quaternion().fromArray(part.poses[hi],3),f);
    for(const box of part.boxes){
      const localRotation=new THREE.Quaternion().fromArray(box.quaternion_xyzw);
      const corners=Array.from({length:8},(_,i)=>new THREE.Vector3(...box.size.map((s,j)=>((i>>j)&1?.5:-.5)*s)).applyQuaternion(localRotation).add(new THREE.Vector3(...box.position)).applyQuaternion(rotation).add(position));
      corners.forEach((p,i)=>{for(let j=0;j<3;j++)if(!(i&(1<<j)))line(p,corners[i|(1<<j)],part.id==='tabletop'?"#f1ba79":"#d49965",2);});
    }
  }
  ctx.fillStyle = "#8fa6c3"; ctx.font = `${9 * scale}px sans-serif`;
  ctx.fillText("CPU 3D PROJECTION · WebGL unavailable", 18 * scale, h - 40 * scale);
}
function drawChart() {
  chartValues = dataset.chart_values || (dataset.errors?.some(x => x !== null) ? dataset.errors :
    dataset.object ? dataset.positions.map(p => p[dataset.object.index][2] - dataset.positions[0][dataset.object.index][2]) : []);
  $("chart-label").textContent = dataset.chart_label || (dataset.object ? "OBJECT LIFT" : "JOINT TRACKING ERROR");
  $("error-unit").textContent = dataset.chart_unit || (dataset.object ? "m" : "rad");
  $("chart-note").textContent = dataset.engine === "kinematic_preview" ? "기구학 재생: 동역학 추종 오차 없음" : dataset.scenario === "table_assembly" ? "실제 peg tip / socket 위치에서 계산 · 28 s에 다리 gripper 해제" : dataset.instances ? "실제 접촉 동역학 실행 기록 · keyframe별 추종·접촉 상태 확인" : dataset.object ? "1 kg 물체 · 두 Cartesian arm · 3 cm 이동" : "관절 위치 오차의 2-norm";
  if (!chartValues.length) { $("chart-line").setAttribute("points", ""); $("chart-area").setAttribute("d", ""); return; }
  const min = Math.min(0, ...chartValues), max = Math.max(1e-8, ...chartValues);
  const points = chartValues.map((y, i) => [i / (chartValues.length - 1) * 260, 80 - (y-min) / (max-min) * 66]);
  $("chart-line").setAttribute("points", points.map(p => p.join(",")).join(" "));
  $("chart-area").setAttribute("d", "M0,85 L" + points.map(p => p.join(",")).join(" L") + " L260,85 Z");
}
function showData(data) {
  $("toast").classList.remove("visible");
  if (dataset?.id === "dual_cito" && data.id !== "dual_cito") $("geometry-mode").value = "visual";
  dataset = data; frameFloat = 0;
  document.querySelector(".run-panel").hidden = !!data.instances;
  window.dispatchEvent(new CustomEvent("recording-selected", {detail:data}));
  $("geometry-mode").disabled = data.id === "dual_cito";
  if (data.id === "dual_cito") $("geometry-mode").value = "skeleton";
  $("model-title").textContent = data.title;
  $("model-description").textContent = data.engine === "kinematic_preview" ? "29개 관절과 floating base를 가진 휴머노이드의 기구학을 확인합니다." : data.engine === "cito_trajectory" ? "접촉을 유지하며 물체를 들어 올리는 제한 모델의 최적화 결과입니다." : "URDF 관성으로 계산한 폐루프 토크 제어 결과를 재생합니다.";
  if (data.instances) $("model-description").textContent = data.articulated_object ? "수동 물체 관절 · 로봇 접촉으로 구동 · 주어진 접촉점과 관절 경로의 힘·토크 최적화" : data.scenario === "mobile_panel" ? "RBY1 모바일 듀얼암 + 고정 Panda · 초기 지지 전달 → 고정 작업영역 밖으로 이동 → 지그 안착·릴리스" : data.scenario === "panel_fixture" ? "본 실험: 소형 접촉 위치 refinement → 패널 운반·재배향 → 지그 안착·하중 전달 → 전체 릴리스 · 자유 강체 실행" : data.scenario === "table_assembly" ? "밑면이 위인 상판 grip & support → 12° reorientation → 다리 정렬·삽입 → gripper 해제 · 30초 실제 동역학 실행" : `주어진 contracted contact → ${data.keyframes.length}개 keyframe 최적화 → 3개 팔의 접촉 동역학 실행입니다.`;
  $("engine-badge").innerHTML = "<span></span>" + engineLabel(data);
  $("engine-badge").classList.toggle("kinematic", data.engine === "kinematic_preview");
  $("scene-label").textContent = data.engine === "cito_trajectory" ? "COUPLED ROBOT–OBJECT TRAJECTORY" : data.engine === "kinematic_preview" ? "FLOATING-BASE KINEMATIC PREVIEW" : "RECORDED CLOSED-LOOP SIMULATION";
  if (data.instances) $("scene-label").textContent = data.articulated_object ? "PASSIVE ARTICULATED OBJECT · MEASURED CONTACT DYNAMICS" : `COOPERATIVE CONTACT DYNAMICS · ${data.keyframes.length} KEYFRAMES`;
  $("dof").textContent = data.dof; $("base").textContent = data.scenario === "mobile_panel" ? "Mobile SE(2)" : data.floating_base ? "Floating +6" : "Fixed";
  $("physics-step").textContent = data.summary.physics_dt ? (data.summary.physics_dt * 1000).toFixed(0) + " ms" : data.engine === "cito_trajectory" ? "100 ms" : "—";
  const end = data.times.at(-1);
  $("duration").textContent = end.toFixed(1) + " s";
  $("time-total").textContent = end.toFixed(2) + " s"; $("chart-end").textContent = end.toFixed(1) + " s";
  $("tool-frame").textContent = data.frame;
  $("scope-description").textContent = data.description;
  $("keyframe-list").replaceChildren();
  for (const keyframe of [...(data.keyframes || []), ...(data.events || [])].sort((a,b)=>a.time-b.time)) {
    const button = document.createElement("button"); button.className = "keyframe-choice";
    button.textContent = keyframe.id + " · " + keyframe.time.toFixed(1) + " s";
    button.onclick = () => {
      playing = false; $("play").textContent = "▶"; $("play").setAttribute("aria-label", "재생");
      let hi = data.times.findIndex(t => t >= keyframe.time); if (hi < 0) hi = data.times.length-1;
      const lo = Math.max(0, hi-1);
      frameFloat = lo+(keyframe.time-data.times[lo])/Math.max(1e-12, data.times[hi]-data.times[lo]); updateFrame();
    };
    $("keyframe-list").append(button);
  }
  document.querySelectorAll(".model-choice").forEach(button => {
    button.classList.toggle("selected", button.dataset.id === data.id);
    button.setAttribute("aria-pressed", button.dataset.id === data.id);
  });
  if (downloadUrl) URL.revokeObjectURL(downloadUrl);
  downloadUrl = URL.createObjectURL(new Blob([JSON.stringify(data)], { type: "application/json" }));
  $("download-data").href = downloadUrl; $("download-data").download = data.id + "-simulation.json";
  $("run").disabled = !backend || data.engine === "kinematic_preview" || data.id === "dual_cito" || !!data.instances;
  $("amplitude").disabled = $("run-duration").disabled = !!data.instances;
  $("backend-state").textContent = data.instances ? "조건을 바꿔 다시 계산: contact-transfer demo-three-arm --viewer web"
    : backend ? "Python backend 연결됨 · 새 MuJoCo 계산을 실행합니다." : "이 링크는 계산 결과 재생용입니다. 새 계산은 로컬 또는 Codespaces의 Python 서버에서 실행합니다.";
  buildGeometry(data); drawChart(); updateFrame();
  loadGeometry();
}
function updateFrame() {
  if (!dataset) return;
  const index = Math.min(Math.round(frameFloat), dataset.times.length - 1);
  const time = dataset.times[0] + frameFloat / (dataset.times.length - 1) * dataset.times.at(-1);
  $("timeline").value = frameFloat / (dataset.times.length - 1);
  $("time-now").textContent = time.toFixed(2) + " s";
  $("error-now").textContent = chartValues.length ? number(chartValues[index]) : "—";
  $("tool-state").textContent = (["panel_fixture","mobile_panel"].includes(dataset.scenario)||dataset.articulated_object) ? Object.entries(dataset.tool_states).map(([name,values])=>`${name}: ${values[index]?"접촉":"해제"}`).join(" · ") : dataset.scenario === "table_assembly" ? `상판 gripper: ${dataset.tool_states.top_gripper_contact[index] ? "양쪽 pad 접촉" : "분리"} · 구형 support: ${dataset.tool_states.point_contact[index] ? "접촉" : "분리"} · 다리 gripper: ${dataset.tool_states.leg_gripper_contact[index] ? "양쪽 pad 접촉" : "해제"} · socket: ${dataset.tool_states.socket_contact[index] ? "접촉" : "비접촉"}` : dataset.tool_states
    ? `구형 tool: ${dataset.tool_states.point_contact[index] ? "접촉" : "분리"} · Gripper: ${dataset.tool_states.gripper_contact[index] ? "양쪽 pad 접촉" : "미파지"} · Suction: ${dataset.tool_states.suction_sealed[index] ? "seal 유지" : "해제"}` : "";
  const ratio = dataset.torque_ratio[index];
  $("torque-now").textContent = ratio === null || ratio === undefined ? "—" : (ratio * 100).toFixed(1) + "%";
  $("torque-meter").style.width = Math.min(100, (ratio || 0) * 100) + "%";
  $("chart-cursor").setAttribute("x1", frameFloat / (dataset.times.length - 1) * 260);
  $("chart-cursor").setAttribute("x2", frameFloat / (dataset.times.length - 1) * 260);
  updateGeometry(frameFloat);
}
async function selectModel(id) {
  const token = ++loadToken; $("loading").classList.remove("hidden");
  try {
    const response = await fetch("./data/" + id + ".json");
    if (!response.ok) throw Error("데이터를 불러오지 못했습니다.");
    const data = await response.json();
    if (token === loadToken) showData(data);
  } catch (error) { if (token === loadToken) { toast(error.message); $("loading").classList.add("hidden"); } }
}
$("play").onclick = () => { playing = !playing; $("play").textContent = playing ? "Ⅱ" : "▶"; $("play").setAttribute("aria-label", playing ? "일시 정지" : "재생"); };
$("timeline").oninput = event => { if (dataset) { frameFloat = Number(event.target.value) * (dataset.times.length - 1); updateFrame(); } };
$("speed").onchange = event => speed = Number(event.target.value);
$("loop").onclick = () => { loop = !loop; $("loop").classList.toggle("active", loop); $("loop").setAttribute("aria-pressed", loop); };
$("reset-camera").onclick = resetCamera;
$("geometry-mode").onchange = loadGeometry;
$("run").onclick = async () => {
  $("run").disabled = true; $("run").textContent = "계산 중…";
  try {
    const response = await fetch("./api/simulate", { method: "POST", headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ model: dataset.id, duration: Number($("run-duration").value), amplitude: Number($("amplitude").value) }) });
    const result = await response.json();
    if (!response.ok) throw Error(result.error || "시뮬레이션 실패");
    result.live = true; showData(result); toast("새 MuJoCo 계산이 완료되었습니다.");
  } catch (error) { toast(error.message); }
  finally { $("run").textContent = "Run simulation ↗"; $("run").disabled = !backend || dataset.id === "dual_cito" || dataset.floating_base || !!dataset.instances; }
};
async function init() {
  const response = await fetch("./data/index.json"), index = await response.json();
  $("model-count").textContent = String(index.models.length).padStart(2, "0");
  $("test-count").textContent = index.tests_passed;
  const models = [...index.models].sort((a, b) => a.id === "mobile_panel" ? -1 : b.id === "mobile_panel" ? 1 : a.id === "panel_fixture" ? -1 : b.id === "panel_fixture" ? 1 : a.id === "dual_cito" ? -1 : b.id === "dual_cito" ? 1 : a.floating_base - b.floating_base || a.title.localeCompare(b.title));
  models.forEach(model => {
    const button = document.createElement("button");
    button.className = "model-choice"; button.dataset.id = model.id;
    button.setAttribute("aria-pressed", "false");
    const icon = document.createElement("span"); icon.className = "model-icon"; icon.textContent = model.id === "dual_cito" ? "⋈" : model.floating_base ? "♧" : "⌁";
    const label = document.createElement("span"), title = document.createElement("span"), meta = document.createElement("span");
    title.className = "model-name"; title.textContent = model.title;
    meta.className = "model-meta"; meta.textContent = model.id === "three_arm" ? "P · G · SUCTION / 3 KEYFRAMES" : model.id === "dual_cito" ? "CONTACT-IMPLICIT · 6 DoF" : model.dof + " DoF · " + (model.floating_base ? "KINEMATIC PREVIEW" : "MUJOCO");
    label.append(title, meta);
    const dot = document.createElement("span"); dot.className = "choice-dot"; dot.textContent = "●";
    button.append(icon, label, dot); button.onclick = () => selectModel(model.id); $("model-list").append(button);
  });
  const requestedModel = new URLSearchParams(location.search).get("model");
  await selectModel(index.models.some(model => model.id === requestedModel) ? requestedModel : "mobile_panel");
  try {
    const health = await fetch("./api/health");
    if (health.ok) backend = (await health.json()).ready === true;
  } catch {}
  $("backend-state").textContent = dataset?.instances ? "조건을 바꿔 다시 계산: contact-transfer demo-three-arm --viewer web"
    : backend ? "Python backend 연결됨 · 새 MuJoCo 계산을 실행합니다." : "이 링크는 계산 결과 재생용입니다. 새 계산은 로컬 또는 Codespaces의 Python 서버에서 실행합니다.";
  $("run").disabled = !backend || dataset?.floating_base || dataset?.id === "dual_cito" || !!dataset?.instances;
}
let previous = performance.now();
function animate(now) {
  requestAnimationFrame(animate);
  const dt = Math.min(.05, (now - previous) / 1000); previous = now;
  if (dataset && playing) {
    frameFloat += dt * speed * (dataset.times.length - 1) / dataset.times.at(-1);
    if (frameFloat > dataset.times.length - 1) {
      frameFloat = loop ? 0 : dataset.times.length - 1;
      if (!loop) { playing = false; $("play").textContent = "▶"; }
    }
    updateFrame();
  }
  controls.update(); renderer.render(scene, camera);
}
requestAnimationFrame(animate);
init().catch(error => { toast(error.message); $("loading").classList.add("hidden"); });

window.addEventListener("experiment-result", event => showData(event.detail));
initializeExperiment().catch(error => { document.getElementById("experiment-state").textContent=error.message; });

window.addEventListener("experiment-preview", event => selectModel(event.detail));
