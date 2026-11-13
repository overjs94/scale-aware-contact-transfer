import { createStudio } from "./studio.js";
import { initializeExperiment } from "./experiment.js";
import { categories, libraryEntry, recordingScope } from "./library.js";
import { englishRecording } from './presentation-en.js';
import { disturbanceScale } from './disturbance.js';
import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { applyRecordedPose, disposeModel, loadURDFModel } from "./urdf-view.js";

const $ = (id) => document.getElementById(id);
let dataset, frameFloat = 0, playing = true, loop = true, speed = 1, loadToken = 0;
const embeddedComparison=new URLSearchParams(location.search).get('embed')==='comparison';
if(embeddedComparison){document.documentElement.classList.add('comparison-embed');playing=false;}
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
  canvas.setAttribute("aria-label", "CPU-projected 3D robot viewer");
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
const forceArrow=new THREE.ArrowHelper(new THREE.Vector3(0,1,0),new THREE.Vector3(),.5,0xef683c,.08,.04);forceArrow.visible=false;for(const mesh of [forceArrow.line,forceArrow.cone]){mesh.material.depthTest=false;mesh.renderOrder=1100;}scene.add(forceArrow);
let grid;let contactOverlay;let nativeContactOverlay;
const contactColors=[0x25bfa3,0xf4a340,0xa788f5];
const contactNames={base_grasp:'Base grasp',base_support:'Base support',moving_manipulation:'Lid grasp',edge_grip:'Panel-edge grasp',under_left:'Left panel support',under_right:'Right panel support',top_gripper:'Tabletop grasp',point_support:'Tabletop support',leg_gripper:'Leg grasp',support_0:'Left support',support_1:'Center support',support_2:'Right support'};

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
  if (data.engine === "mujoco_contact_recording") return "MuJoCo · contact dynamics";
  return data.engine === "kinematic_preview" ? "Kinematic preview" : data.engine === "cito_trajectory" ? "CITO · optimized trajectory" : data.live ? "MuJoCo · new execution" : "MuJoCo · recorded execution";
}
function resetCamera() {
  if(embeddedComparison){controls.target.set(0,0,.45);camera.position.set(2.3,-3.5,2.4);controls.update();return;}
  controls.target.copy(center);
  camera.position.copy(center).add(new THREE.Vector3(1.6, -2.4, 1.6).multiplyScalar(extent));
  controls.update();
}
function buildGeometry(data) {
  ++geometryToken; urdfModels = [];
  const supplied=data.contact_contraction?.input_type==='supplied_contracted_contacts';
  for(const option of $('contact-display').options)if(['both','dense'].includes(option.value))option.disabled=supplied;
  if(supplied&&['both','dense'].includes($('contact-display').value))$('contact-display').value='contracted';
  while (robotGroup.children.length) {
    disposeModel(robotGroup.children[0]);
  }
  geometry = []; objectMesh = undefined;
  nativeContactOverlay=new THREE.Group();robotGroup.add(nativeContactOverlay);
  for(let i=0;i<6;i++){const marker=new THREE.Mesh(new THREE.SphereGeometry(.004,12,8),new THREE.MeshBasicMaterial({color:0xffffff,depthTest:false}));marker.renderOrder=1003;marker.visible=false;nativeContactOverlay.add(marker);}
  $('native-contact-controls').hidden=!data.native_contact_samples;
  $('full-result-links').hidden=!data.full_task_recording&&!data.comparison_recording;
  document.querySelectorAll('[data-baseline-link]').forEach(link=>link.hidden=!data.full_task_recording);
  $('contact-focus').replaceChildren(new Option('Choose a contact',''));
  for(const g of data.contact_contraction?.groups||[])$('contact-focus').append(new Option(contactNames[g.group_id]||g.group_id,g.group_id));
  const tool=data.provenance?.canonical_point_tool;$('native-contact-note').hidden=!data.native_contact_samples;
  $('native-contact-note').textContent=data.native_contact_samples?'Colored markers: planned representative contacts. '+(tool?`P mount, end effector → contact point ${(1000*Math.hypot(...tool.ee_to_tip.slice(0,3).map(row=>row[3]))).toFixed(0)} mm / sphere radius ${1000*tool.radius_m} mm. `:'')+(data.scenario==='full_box'?'G grasps the protruding base and lid handles. A rigid metal attachment joins the base handle to the wall.':''):'';
  contactOverlay=new THREE.Group();robotGroup.add(contactOverlay);
  for(const [i,g] of (data.contact_contraction?.groups||[]).entries()){
    const group=new THREE.Group();group.userData.contactGroup=g;const color=contactColors[i%contactColors.length];
    const points=new THREE.Points(new THREE.BufferGeometry().setFromPoints(g.target_points_m.map(p=>new THREE.Vector3(...p))),new THREE.PointsMaterial({color,size:.007,depthTest:false}));points.userData.dense=true;points.renderOrder=1001;group.add(points);
    const anchor=new THREE.Mesh(new THREE.SphereGeometry(.006,14,10),new THREE.MeshBasicMaterial({color,depthTest:false}));anchor.position.fromArray(g.target_anchor_m);anchor.userData.anchor=true;anchor.renderOrder=1002;group.add(anchor);contactOverlay.add(group);
  }
  $('contraction-state').replaceChildren();if(data.contact_contraction){const c=data.contact_contraction;$('contraction-state').append(c.input_type==='supplied_contracted_contacts'?` · Supplied ${c.contracted_contact_count} contracted contacts → 3 arms · `:` · ${c.dense_sample_count} samples → ${c.contracted_contact_count} anchors → 3 arms · `);for(const [i,g] of c.groups.entries()){const label=document.createElement('span');label.style.color='#'+contactColors[i%contactColors.length].toString(16).padStart(6,'0');label.textContent=` ${g.group_id}@${g.link_id} → ${c.assignment.find(a=>a.group_id===g.group_id).arm_id} `;$('contraction-state').append(label);}}
  for (const item of (data.instances?.some(i=>["static_jig","articulated_object"].includes(i.role)) ? [] : data.environment) || []) {
    if (item.type !== "box") continue;
    const mesh = new THREE.Mesh(new THREE.BoxGeometry(...item.size), new THREE.MeshStandardMaterial({color:item.color, transparent:true, opacity:.6}));
    mesh.position.fromArray(item.position); if(item.quaternion_xyzw) mesh.quaternion.fromArray(item.quaternion_xyzw); robotGroup.add(mesh);
  }
  const points = data.positions[0].map(p => new THREE.Vector3(...p));
  const box = new THREE.Box3().setFromPoints(points);
  center = box.getCenter(new THREE.Vector3());
  extent = Math.max(data.viewer_min_extent || .4, box.getSize(new THREE.Vector3()).length() * .65);
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
      ? "Playback of two Cartesian arms and an object. This restricted model has no exterior meshes."
      : "URDF link coordinates. Select visual or collision geometry to load the original shapes.";
    $("geometry-status").dataset.state = "skeleton";
    $("loading").classList.add("hidden"); return;
  }
  $("geometry-status").textContent = "Loading URDF and meshes…";
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
      ? "URDF/meshes loaded. WebGL is unavailable; link coordinates are projected on the CPU."
      : mode === "collision"
        ? "URDF collision geometry. Geometry display is separate from contact computation and collision-validation results."
        : data.engine === "mujoco_contact_recording"
          ? data.scenario === "mobile_panel" ? "Official RBY1 URDF/meshes; measured contact execution with a differential-drive mobile base and a fixed Panda." : data.free_articulated_object ? "Native dynamics of a free base and internal joint; three-arm realization of link-aware contracted contacts." : data.articulated_object ? "URDF geometry and measured passive-joint motion, driven through robot contact without object actuators." : data.scenario === "panel_fixture" ? "Original Panda URDF/meshes, a free panel, and a fixed fixture; recorded execution after small-scale source contact refinement." : data.scenario === "table_assembly" ? "Three Panda arms and two free parts in native contact execution. Tabletop and leg URDFs include socket and peg visual/collision geometry." : "Native contact execution with three URDF arms and interfaces: a spherical point tool, a two-finger gripper, and a pressure-limited suction seal."
          : "URDF mesh origins and scales are applied. This recording checks joint tracking.";
    if (data.display?.geometry && !canvasFallback) $("geometry-status").textContent = data.display.geometry;
    $("geometry-status").dataset.state = "ready";
    $("geometry-status").dataset.model = data.id;
    updateGeometry(frameFloat);
    if(embeddedComparison)parent.postMessage({type:'contact-transfer-ready',id:data.id,duration:data.times.at(-1)},location.origin);
  } catch (error) {
    if (token !== geometryToken) return;
    $("geometry-status").textContent = error.message + " Select link coordinates or check network access.";
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
  for(const group of contactOverlay?.children||[]){const contact=group.userData.contactGroup,object=dataset.objects?.find(o=>o.id===contact.link_id);if(!object)continue;const pose=object.poses[lo],next=object.poses[hi];group.position.fromArray(pose).lerp(new THREE.Vector3().fromArray(next),f);group.quaternion.fromArray(pose,3).slerp(new THREE.Quaternion().fromArray(next,3),f);const mode=$('contact-display').value;for(const item of group.children)item.visible=mode==='both'||(item.userData.dense&&mode==='dense')||(item.userData.anchor&&mode==='contracted');}
  const sample=Math.round(value),patches=dataset.native_contact_samples?.frames[sample]||[];
  nativeContactOverlay?.children.forEach((marker,i)=>{marker.visible=!!patches[i]&&$('native-contact-visible').checked;if(patches[i])marker.position.fromArray(patches[i].position_world_m);});
  $('native-contact-state').dataset.count=String(patches.length);
  $('native-contact-state').textContent=dataset.native_contact_samples?`native ${dataset.times[sample].toFixed(3)} s · ${patches.length} contact patches · no contact interpolation${patches.length?' · minimum signed gap '+(1000*Math.min(...patches.map(p=>p.minimum_gap_m))).toFixed(3)+' mm':''}`:'';
  const time=dataset.times[lo]*(1-f)+dataset.times[hi]*f;const pulses=dataset.external_disturbances||[];const pulse=pulses.find(p=>time>=p.start&&time<=p.start+p.duration);forceArrow.visible=false;
  if(pulse&&dataset.objects?.[0]?.poses){const object=dataset.objects.find(o=>o.native_body===pulse.body)||dataset.objects[0],pose=object.poses[lo],next=object.poses[hi],quat=new THREE.Quaternion().fromArray(pose,3).slerp(new THREE.Quaternion().fromArray(next,3),f);const origin=new THREE.Vector3(...pose.slice(0,3)).lerp(new THREE.Vector3(...next.slice(0,3)),f).add(new THREE.Vector3(...pulse.point_body_m).applyQuaternion(quat));const magnitude=disturbanceScale(pulse,time);const force=new THREE.Vector3(...pulse.force_world_N).multiplyScalar(magnitude);if(force.length()>.01){forceArrow.position.copy(origin);forceArrow.setDirection(force.clone().normalize());forceArrow.setLength(.12+force.length()*.015,.08,.04);forceArrow.visible=true;}}
  $('disturbance-state').hidden=!pulses.length;$('disturbance-state').dataset.active=String(forceArrow.visible);
  $('disturbance-state').textContent=pulse?`Applied external force · ${(disturbanceScale(pulse,time)*Math.hypot(...pulse.force_world_N)).toFixed(2)} N · world frame` : pulses.length?`External disturbance: ${pulses.map(p=>`${p.start}–${p.start+p.duration} s / peak ${Math.hypot(...p.force_world_N)} N`).join(', ')}`:'';
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
  for(const group of contactOverlay?.children||[]){group.updateMatrixWorld(true);const g=group.userData.contactGroup,mode=$('contact-display').value;if(mode==='off')continue;ctx.fillStyle='#'+contactColors[contactOverlay.children.indexOf(group)%contactColors.length].toString(16).padStart(6,'0');for(const point of (mode==='dense'||mode==='both'?g.target_points_m:[])){const p=project(group.localToWorld(new THREE.Vector3(...point)));ctx.beginPath();ctx.arc(p[0],p[1],2*scale,0,Math.PI*2);ctx.fill();}if(mode==='contracted'||mode==='both'){const p=project(group.localToWorld(new THREE.Vector3(...g.target_anchor_m)));ctx.beginPath();ctx.arc(p[0],p[1],6*scale,0,Math.PI*2);ctx.fill();}}
  for(const marker of nativeContactOverlay?.children||[])if(marker.visible){const p=project(marker.position);ctx.fillStyle='#fff';ctx.beginPath();ctx.arc(p[0],p[1],3*scale,0,Math.PI*2);ctx.fill();}
  if(forceArrow.visible){const a=forceArrow.position,b=new THREE.Vector3(0,1,0).applyQuaternion(forceArrow.quaternion).multiplyScalar(forceArrow.line.scale.y+.08).add(a);line(a,b,"#ff482f",4);const tip=project(b),tail=project(a),angle=Math.atan2(tip[1]-tail[1],tip[0]-tail[0]);ctx.fillStyle="#ff482f";ctx.beginPath();ctx.moveTo(tip[0],tip[1]);for(const sign of [-1,1])ctx.lineTo(tip[0]-15*scale*Math.cos(angle+sign*.4),tip[1]-15*scale*Math.sin(angle+sign*.4));ctx.closePath();ctx.fill();}
  ctx.fillStyle = "#8fa6c3"; ctx.font = `${9 * scale}px sans-serif`;
  ctx.fillText("CPU 3D PROJECTION · WebGL unavailable", 18 * scale, h - 40 * scale);
}
function drawChart() {
  chartValues = dataset.chart_values || (dataset.errors?.some(x => x !== null) ? dataset.errors :
    dataset.object ? dataset.positions.map(p => p[dataset.object.index][2] - dataset.positions[0][dataset.object.index][2]) : []);
  $("chart-label").textContent = dataset.chart_label || (dataset.object ? "OBJECT LIFT" : "JOINT TRACKING ERROR");
  $("error-unit").textContent = dataset.chart_unit || (dataset.object ? "m" : "rad");
  $("chart-note").textContent = dataset.engine === "kinematic_preview" ? "Kinematic playback; no dynamic tracking-error measurement" : dataset.scenario === "table_assembly" ? "Computed from measured peg-tip/socket positions · leg gripper opens at 28 s" : dataset.instances ? "Measured contact-dynamics execution · inspect keyframe tracking and contact state" : dataset.object ? "1 kg object · two Cartesian arms · 3 cm displacement" : "Euclidean norm of joint-position error";
  if (dataset.display?.chart_note) $("chart-note").textContent = dataset.display.chart_note;
  if (!chartValues.length) { $("chart-line").setAttribute("points", ""); $("chart-area").setAttribute("d", ""); return; }
  const min = Math.min(0, ...chartValues), max = Math.max(1e-8, ...chartValues);
  const points = chartValues.map((y, i) => [i / (chartValues.length - 1) * 260, 80 - (y-min) / (max-min) * 66]);
  $("chart-line").setAttribute("points", points.map(p => p.join(",")).join(" "));
  $("chart-area").setAttribute("d", "M0,85 L" + points.map(p => p.join(",")).join(" L") + " L260,85 Z");
}
function selectLibraryCategory(id) {
  for (const button of document.querySelectorAll(".category-choice")) {
    button.setAttribute("aria-pressed", String(button.dataset.category === id));
  }
  for (const panel of document.querySelectorAll(".library-category")) panel.hidden = panel.dataset.category !== id;
  $("library-description").textContent = categories.find(category => category.id === id).description;
  revealLibraryChoice();
}
function revealLibraryChoice() {
  if (!matchMedia("(max-width:680px)").matches) return;
  const panel = document.querySelector('.library-category:not([hidden])');
  const choice = panel?.querySelector('.model-choice.selected');
  if (choice) panel.scrollLeft += choice.getBoundingClientRect().left-panel.getBoundingClientRect().left-(panel.clientWidth-choice.clientWidth)/2;
}
function buildLibrary(models) {
  for (const category of categories) {
    const members = models.map(model => ({ model, entry: libraryEntry(model) }))
      .filter(item => item.entry.category === category.id).sort((a,b) => a.entry.order-b.entry.order);
    if (!members.length) continue;
    const choice = document.createElement("button");
    choice.className = "category-choice"; choice.dataset.category = category.id;
    choice.setAttribute("aria-controls", "library-" + category.id);
    choice.setAttribute("aria-pressed", "false");
    const label = document.createElement("span"), count = document.createElement("span");
    label.textContent = category.title; count.textContent = members.length;
    choice.append(label, count); choice.onclick = () => selectLibraryCategory(category.id);
    $("library-categories").append(choice);
    const panel = document.createElement("div");
    panel.id = "library-" + category.id; panel.className = "library-category";
    panel.dataset.category = category.id;
    for (const [index, title] of category.groups.entries()) {
      const items = members.filter(item => item.entry.group === index);
      if (!items.length) continue;
      const group = document.createElement("section"); group.className = "library-group";
      const heading = document.createElement("h3"); heading.textContent = title;
      group.append(heading);
      for (const { model, entry } of items) {
        const button = document.createElement("button");
        button.className = "model-choice"; button.dataset.id = model.id;
        button.setAttribute("aria-pressed", "false");
        const label = document.createElement("span"), name = document.createElement("span"), meta = document.createElement("span");
        name.className = "model-name"; name.textContent = entry.title;
        meta.className = "model-meta"; meta.textContent = entry.detail;
        label.append(name, meta);
        const dot = document.createElement("span"); dot.className = "choice-dot"; dot.textContent = "●";
        button.append(label, dot); button.onclick = () => selectModel(model.id); group.append(button);
      }
      panel.append(group);
    }
    $("model-list").append(panel);
  }
}
function showData(original) {
  const data=englishRecording(original);
  $("toast").classList.remove("visible");
  if (dataset?.id === "dual_cito" && data.id !== "dual_cito") $("geometry-mode").value = "visual";
  dataset = data; frameFloat = 0;
  if(embeddedComparison)playing=false;
  document.querySelector(".run-panel").hidden = !!data.instances;
  window.dispatchEvent(new CustomEvent("recording-selected", {detail:data}));
  $("geometry-mode").disabled = data.id === "dual_cito";
  if (data.id === "dual_cito") $("geometry-mode").value = "skeleton";
  $("model-title").textContent = data.title;
  const scope = recordingScope(data);
  selectLibraryCategory(scope.entry.category);
  $("scenario-category").textContent = scope.category.title + " / " + scope.category.groups[scope.entry.group];
  $("scenario-status").textContent = scope.status;
  $("scenario-status").dataset.fullPipeline = String(scope.full);
  $("scenario-scope").textContent = scope.detail;
  if (document.querySelector(`.model-choice[data-id="${CSS.escape(data.id)}"]`)) {
    const url = new URL(location.href); url.searchParams.set("model", data.id); history.replaceState(null, "", url);
  }
  $("model-description").textContent = data.engine === "kinematic_preview" ? "Kinematic playback of a humanoid with 29 joints and a floating base." : data.engine === "cito_trajectory" ? "An optimized restricted-model trajectory that lifts an object while maintaining contact." : "Recorded closed-loop torque control using URDF inertial parameters.";
  if (data.instances) $("model-description").textContent = data.free_articulated_object ? "Native dynamics of a free base and internal joint; three-arm realization of link-aware contracted contacts." : data.articulated_object ? "Passive object joint · driven by robot contact · reactions/torques optimized for supplied contacts and joint path" : data.scenario === "mobile_panel" ? "RBY1 mobile dual arm + fixed Panda · support handoff → travel beyond fixed-base reach → fixture placement and release" : data.scenario === "panel_fixture" ? "Small-scale source contact refinement → panel transport/reorientation → fixture placement/load transfer → complete release · native free-body execution" : data.scenario === "table_assembly" ? "Inverted tabletop grasp/support → 12° reorientation → leg alignment/insertion → gripper release · 30 s native dynamic execution" : `Supplied contracted contacts → ${data.keyframes.length} optimized keyframes → three-arm contact-dynamics execution.`;
  $("engine-badge").innerHTML = "<span></span>" + engineLabel(data);
  $("engine-badge").classList.toggle("kinematic", data.engine === "kinematic_preview");
  $("scene-label").textContent = data.engine === "cito_trajectory" ? "COUPLED ROBOT–OBJECT TRAJECTORY" : data.engine === "kinematic_preview" ? "FLOATING-BASE KINEMATIC PREVIEW" : "RECORDED CLOSED-LOOP SIMULATION";
  if (data.instances) $("scene-label").textContent = data.free_articulated_object ? "LINK-AWARE CONTRACTION → THREE-ARM REALIZATION · FREE ARTICULATED BODY" : data.articulated_object ? "PASSIVE ARTICULATED OBJECT · MEASURED CONTACT DYNAMICS" : `COOPERATIVE CONTACT DYNAMICS · ${data.keyframes.length} KEYFRAMES`;
  if (data.display?.description) $("model-description").textContent = data.display.description;
  if (data.display?.scene_label) $("scene-label").textContent = data.display.scene_label;
  $("dof").textContent = data.dof; $("base").textContent = data.free_articulated_object ? "Free object + joint" : data.scenario === "mobile_panel" ? "Mobile SE(2)" : data.floating_base ? "Floating +6" : "Fixed";
  $("physics-step").textContent = data.summary.physics_dt ? Number((data.summary.physics_dt * 1000).toFixed(3)) + " ms" : data.engine === "cito_trajectory" ? "100 ms" : "—";
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
      playing = false; $("play").textContent = "▶"; $("play").setAttribute("aria-label", "Play");
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
  revealLibraryChoice();
  if (downloadUrl) URL.revokeObjectURL(downloadUrl);
  downloadUrl = URL.createObjectURL(new Blob([JSON.stringify(original)], { type: "application/json" }));
  $("download-data").href = downloadUrl; $("download-data").download = data.id + "-simulation.json";
  $("run").disabled = !backend || data.engine === "kinematic_preview" || data.id === "dual_cito" || !!data.instances;
  $("amplitude").disabled = $("run-duration").disabled = !!data.instances;
  $("backend-state").textContent = data.instances ? "To compute a new trial: contact-transfer demo-three-arm --viewer web"
    : backend ? "Python backend connected · new MuJoCo trials can be computed." : "This page replays recorded results. New computations require the Python backend locally or in Codespaces.";
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
  $("tool-state").textContent = (dataset.tool_states && !dataset.tool_states.suction_sealed && dataset.scenario !== "table_assembly") ? Object.entries(dataset.tool_states).map(([name,values])=>`${name}: ${values[index]?"Contact":"Released"}`).join(" · ") : dataset.scenario === "table_assembly" ? `Tabletop gripper: ${dataset.tool_states.top_gripper_contact[index] ? "Both pads in contact" : "Separated"} · Spherical support: ${dataset.tool_states.point_contact[index] ? "Contact" : "Separated"} · Leg gripper: ${dataset.tool_states.leg_gripper_contact[index] ? "Both pads in contact" : "Released"} · socket: ${dataset.tool_states.socket_contact[index] ? "Contact" : "No contact"}` : dataset.tool_states
    ? `Spherical tool: ${dataset.tool_states.point_contact[index] ? "Contact" : "Separated"} · Gripper: ${dataset.tool_states.gripper_contact[index] ? "Both pads in contact" : "No grasp"} · Suction: ${dataset.tool_states.suction_sealed[index] ? "Seal retained" : "Released"}` : "";
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
    if (!response.ok) throw Error("Could not load the recording.");
    const data = await response.json();
    if (token === loadToken) showData(data);
  } catch (error) { if (token === loadToken) { toast(error.message); $("loading").classList.add("hidden"); } }
}
$("play").onclick = () => { playing = !playing; $("play").textContent = playing ? "Ⅱ" : "▶"; $("play").setAttribute("aria-label", playing ? "Pause" : "Play"); };
$("timeline").oninput = event => { if (dataset) { frameFloat = Number(event.target.value) * (dataset.times.length - 1); updateFrame(); } };
$("speed").onchange = event => speed = Number(event.target.value);
$("loop").onclick = () => { loop = !loop; $("loop").classList.toggle("active", loop); $("loop").setAttribute("aria-pressed", loop); };
$("reset-camera").onclick = resetCamera;
$("geometry-mode").onchange = loadGeometry;
$("contact-display").onchange=()=>updateFrame();
$("native-contact-visible").onchange=()=>updateFrame();
$("contact-focus").onchange=()=>{
  const id=$('contact-focus').value,g=dataset?.contact_contraction?.groups.find(g=>g.group_id===id),object=dataset?.objects?.find(o=>o.id===g?.link_id);if(!g||!object)return;
  playing=false;$('play').textContent='▶';$('play').setAttribute('aria-label','Play');frameFloat=Math.round(frameFloat);updateFrame();
  const patches=dataset.native_contact_samples?.frames[frameFloat]?.filter(p=>p.group_id===id)||[],pose=object.poses[frameFloat];
  const point=patches.length?patches.reduce((a,p)=>a.add(new THREE.Vector3(...p.position_world_m)),new THREE.Vector3()).divideScalar(patches.length):new THREE.Vector3(...g.target_anchor_m).applyQuaternion(new THREE.Quaternion().fromArray(pose,3)).add(new THREE.Vector3(...pose.slice(0,3)));
  // Inspect the outside of the rotating lid, rather than looking through its
  // underside and hiding the raised handle behind the wood.
  const offset=new THREE.Vector3(id==='base_grasp'?-.35:.25,-.45,.35).applyQuaternion(new THREE.Quaternion().fromArray(pose,3));
  controls.target.copy(point);camera.position.copy(point).add(offset);controls.update();
};
$("run").onclick = async () => {
  $("run").disabled = true; $("run").textContent = "Computing…";
  try {
    const response = await fetch("./api/simulate", { method: "POST", headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ model: dataset.id, duration: Number($("run-duration").value), amplitude: Number($("amplitude").value) }) });
    const result = await response.json();
    if (!response.ok) throw Error(result.error || "Simulation request failed");
    result.live = true; showData(result); toast("New MuJoCo execution completed.");
  } catch (error) { toast(error.message); }
  finally { $("run").textContent = "Run simulation ↗"; $("run").disabled = !backend || dataset.id === "dual_cito" || dataset.floating_base || !!dataset.instances; }
};
async function init() {
  const response = await fetch("./data/index.json"), index = await response.json();
  $("model-count").textContent = String(index.models.length).padStart(2, "0");
  buildLibrary(index.models);
  const requestedModel = new URLSearchParams(location.search).get("model");
  await selectModel(index.models.some(model => model.id === requestedModel) ? requestedModel : ["full_assembly","full_panel","full_box","free_box_lid"].find(id=>index.models.some(m=>m.id===id)));
  try {
    const health = await fetch("./api/health");
    if (health.ok) backend = (await health.json()).ready === true;
  } catch {}
  $("backend-state").textContent = dataset?.full_task_recording ? "New plans and physical executions are computed with tools/run_full_scenarios.py. This public page replays saved results." : dataset?.instances ? "To compute a new trial: contact-transfer demo-three-arm --viewer web"
    : backend ? "Python backend connected · new MuJoCo trials can be computed." : "This page replays recorded results. New computations require the Python backend locally or in Codespaces.";
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

window.addEventListener('message',event=>{
  if(!embeddedComparison||event.origin!==location.origin||event.source!==parent||event.data?.type!=='contact-transfer-seek'||!dataset)return;
  const time=Number(event.data.time_s);if(!Number.isFinite(time))return;
  let low=0,high=dataset.times.length-1;
  while(high-low>1){const mid=Math.floor((low+high)/2);if(dataset.times[mid]<=time)low=mid;else high=mid;}
  frameFloat=Math.min(dataset.times.length-1,Math.max(0,low+(time-dataset.times[low])/(dataset.times[high]-dataset.times[low])));
  playing=false;$('play').textContent='▶';$('play').setAttribute('aria-label','Play');updateFrame();
});
