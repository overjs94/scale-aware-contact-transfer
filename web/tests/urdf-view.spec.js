import { test, expect } from "@playwright/test";
import { libraryEntry } from "../src/library.js";

// Keep CI deterministic and offline. Real upstream assets are also exercised
// separately during integration QA; these fixtures test URDF transforms.
const stl = `solid triangle
facet normal 0 0 1
outer loop
vertex 0 0 0
vertex 1 0 0
vertex 0 1 0
endloop
endfacet
endsolid triangle`;
const dae = `<?xml version="1.0"?><COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
<asset><unit name="meter" meter="1"/><up_axis>Z_UP</up_axis></asset>
<library_geometries><geometry id="g"><mesh><source id="p"><float_array id="a" count="9">0 0 0 1 0 0 0 1 0</float_array><technique_common><accessor source="#a" count="3" stride="3"><param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/></accessor></technique_common></source><vertices id="v"><input semantic="POSITION" source="#p"/></vertices><triangles count="1"><input semantic="VERTEX" source="#v" offset="0"/><p>0 1 2</p></triangles></mesh></geometry></library_geometries>
<library_visual_scenes><visual_scene id="s"><node><instance_geometry url="#g"/></node></visual_scene></library_visual_scenes><scene><instance_visual_scene url="#s"/></scene></COLLADA>`;

test.beforeEach(async ({ page }) => {
  await page.route("https://raw.githubusercontent.com/**", route => route.fulfill({
    contentType: route.request().url().toLowerCase().endsWith(".dae") ? "application/xml" : "application/octet-stream",
    body: route.request().url().toLowerCase().endsWith(".obj") ? "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n" : route.request().url().toLowerCase().endsWith(".dae") ? dae : stl,
    headers: { "Access-Control-Allow-Origin": "*" },
  }));
  await page.goto("/?model=three_franka");
  await page.evaluate(async () => { window.modelAPI = await import("/test-module.js"); });
});

for (const id of ["franka_panda", "rb3", "rb5", "kinova_gen3", "unitree_g1", "kuka_iiwa7", "ur5e", "xarm7"]) {
  test(`${id}: recorded FK and both geometry modes`, async ({ page }) => {
    const result = await page.evaluate(async id => {
      const { loadURDFModel, applyRecordedPose, disposeModel } = window.modelAPI;
      const data = await (await fetch(`/data/${id}.json`)).json();
      const output = [];
      for (const mode of ["visual", "collision"]) {
        const robot = await loadURDFModel(`/models/${id}/model.json`, mode);
        let maxError = 0, meshes = 0;
        robot.traverse(node => { if (node.isMesh) meshes++; });
        for (const index of [0, Math.floor(data.q.length/2), data.q.length-1]) {
          applyRecordedPose(robot, data, index);
          data.link_names.forEach((name, i) => {
            const p = robot.links[name].matrixWorld.elements.slice(12, 15);
            p.forEach((value, j) => { maxError = Math.max(maxError, Math.abs(value-data.positions[index][i][j])); });
          });
        }
        output.push({ mode, maxError, meshes }); disposeModel(robot);
      }
      return output;
    }, id);
    for (const row of result) { expect(row.meshes).toBeGreaterThan(0); expect(row.maxError).toBeLessThan(1e-6); }
  });
}

test("mesh origin, scale and joint rotation compose in URDF order", async ({ page }) => {
  await page.route("**/custom/model.json", route => route.fulfill({ json: { urdf: "robot.urdf", packages: { fixture: "http://127.0.0.1:8123/custom" } } }));
  await page.route("**/custom/robot.urdf", route => route.fulfill({ body: `<robot name="fixture"><link name="base"/><link name="tool"><visual><origin xyz="1 2 3" rpy="0 0 1.5707963267948966"/><geometry><mesh filename="package://fixture/triangle.stl" scale="2 3 4"/></geometry></visual></link><joint name="j" type="continuous"><parent link="base"/><child link="tool"/><axis xyz="0 0 1"/></joint></robot>` }));
  await page.route("**/custom/triangle.stl", route => route.fulfill({ body: stl }));
  const vertices = await page.evaluate(async () => {
    const { loadURDFModel, applyRecordedPose, disposeModel } = window.modelAPI;
    const robot = await loadURDFModel("/custom/model.json");
    applyRecordedPose(robot, { dof: 1, joint_names: ["j"], floating_base: false, q: [[Math.PI/2]] }, 0);
    let points;
    robot.traverse(node => {
      if (!node.isMesh) return;
      const a = node.geometry.attributes.position;
      points = Array.from({ length: a.count }, (_, i) => {
        // Matrix multiplication independent of applyRecordedPose.
        const e = node.matrixWorld.elements, x = a.getX(i), y = a.getY(i), z = a.getZ(i);
        return [e[0]*x+e[4]*y+e[8]*z+e[12], e[1]*x+e[5]*y+e[9]*z+e[13], e[2]*x+e[6]*y+e[10]*z+e[14]];
      });
    });
    disposeModel(robot); return points;
  });
  const expected = [[-2, 1, 3], [-4, 1, 3], [-2, -2, 3]];
  vertices.forEach((p, i) => p.forEach((value, j) => expect(value).toBeCloseTo(expected[i][j], 10)));
});

test("missing mesh rejects the model and mode switching preserves playback", async ({ page }) => {
  await page.waitForFunction(() => document.querySelector("#geometry-status").dataset.state === "ready");
  await page.locator("#play").click();
  await page.locator("#timeline").evaluate(el => { el.value = "0.5"; });
  await page.locator("#timeline").dispatchEvent("input");
  await page.locator("#geometry-mode").selectOption("collision");
  await expect(page.locator("#geometry-status")).toHaveAttribute("data-state", "ready");
  await expect(page.locator("#timeline")).toHaveValue("0.5");
  await page.route("https://raw.githubusercontent.com/**", route => route.fulfill({ status: 404, headers: { "Access-Control-Allow-Origin": "*" }, body: "missing" }));
  await page.locator("#geometry-mode").selectOption("visual");
  await expect(page.locator("#geometry-status")).toHaveAttribute("data-state", "error");
  await page.locator("#geometry-mode").selectOption("skeleton");
  await expect(page.locator("#geometry-status")).toHaveAttribute("data-state", "skeleton");
});

test("three physical tools replay as three URDF instances with keyframe events", async ({ page }) => {
  await page.locator('.category-choice[data-category="diagnostic"]').click();
  await page.locator('.model-choice[data-id="three_arm"]').click();
  await expect(page.locator("#geometry-status")).toHaveAttribute("data-state", "ready");
  await expect(page.locator("#geometry-status")).toHaveAttribute("data-model", "three_arm");
  await expect(page.locator(".keyframe-choice")).toHaveCount(4);
  await page.getByRole("button", { name: "lift · 1.5 s", exact: true }).click();
  await expect(page.locator("#time-now")).toHaveText("1.50 s");
  await expect(page.locator("#tool-state")).toContainText("seal 유지");
  await expect(page.locator("#torque-now")).toContainText("%");
  await page.getByRole("button", { name: "vent_and_release · 3.0 s", exact: true }).click();
  await expect(page.locator("#tool-state")).toContainText("Suction: 해제");
  const errors = await page.evaluate(async () => {
    const data = await (await fetch("/data/three_arm.json")).json();
    const {loadURDFModel, applyRecordedPose, disposeModel} = window.modelAPI;
    const errors = [];
    for (const spec of data.instances) {
      const model = await loadURDFModel(spec.manifest);
      const recording = {...spec, q: data.q.map(q => spec.q_indices.map(i => q[i]))};
      let error = 0;
      for (const frame of [15, 75, 135, data.times.length-1]) {
        applyRecordedPose(model, recording, frame);
        for (const [name, link] of Object.entries(model.links)) {
          const index = data.link_names.indexOf(spec.id+"_"+name);
          link.matrixWorld.elements.slice(12,15).forEach((v,j) => {error=Math.max(error,Math.abs(v-data.positions[frame][index][j]));});
        }
      }
      errors.push(error); disposeModel(model);
    }
    return errors;
  });
  expect(errors).toHaveLength(3);
  errors.forEach(error => expect(error).toBeLessThan(1e-7));
  await page.locator("#geometry-mode").selectOption("collision");
  await expect(page.locator("#geometry-status")).toHaveAttribute("data-state", "ready");
  await page.goto("/?model=three_arm");
  await expect(page.locator("#geometry-status")).toHaveAttribute("data-model", "three_arm");
  await expect(page.locator("#geometry-status")).toHaveAttribute("data-state", "ready");
  await expect(page.locator("#amplitude")).toBeDisabled();
});

test("library separates paper tasks, single embodiments and diagnostics with matching editors", async ({page})=>{
  await page.goto('/?model=free_box_lid');
  const catalog=await(await page.request.get('/data/index.json')).json();
  const counts=category=>catalog.models.filter(model=>libraryEntry(model).category===category).length;
  await expect(page.locator('.category-choice[data-category="paper"]')).toHaveAttribute('aria-pressed','true');
  await expect(page.locator('.model-choice:visible')).toHaveCount(counts('paper'));
  await expect(page.locator('#scenario-status')).toContainText('구형 형상 · 재검증 필요');
  await expect(page.locator('#scenario-scope')).toContainText('50 mm');
  await expect(page.locator('#scenario-status')).toHaveAttribute('data-full-pipeline','false');
  await page.locator('.category-choice[data-category="embodiment"]').click();
  await expect(page.locator('.model-choice:visible')).toHaveCount(8);
  await page.locator('.model-choice[data-id="rb5"]').click();
  await expect(page.locator('#scenario-category')).toContainText('단일 embodiment');
  await expect(page.locator('#scenario-status')).toHaveText('관절 추종 검증');
  await expect(page.locator('.experiment-builder')).toBeHidden();
  await expect(page).toHaveURL(/model=rb5/);
  await page.locator('.category-choice[data-category="paper"]').click();
  await page.locator('.model-choice[data-id="panel_fixture"]').click();
  await expect(page.locator('.experiment-builder')).toBeVisible();
  await expect(page.locator('#scenario-preset')).toHaveValue('panel_fixture');
  await expect(page.locator('#builder-title')).toContainText('패널');
  await page.locator('.category-choice[data-category="diagnostic"]').click();
  await expect(page.locator('.model-choice:visible')).toHaveCount(counts('diagnostic'));
  await page.locator('.model-choice[data-id="hinged_door"]').click();
  await expect(page.locator('#scenario-preset')).toHaveValue('hinged_door');
  await expect(page.locator('#scenario-scope')).toContainText('세계에 고정된 수동 관절');
  await expect(page.locator('#scenario-preset optgroup[label="논문 목표"] option')).toHaveCount(6);
  await expect(page.locator('#scenario-preset optgroup[label="요소·강건성"] option')).toHaveCount(4);
});

test("library deep links retain failed force tests and distinguish G1 preview on mobile", async ({page})=>{
  await page.setViewportSize({width:390,height:844});
  await page.goto('/?model=mobile_push_25');
  await expect(page.locator('.category-choice[data-category="diagnostic"]')).toHaveAttribute('aria-pressed','true');
  await expect(page.locator('#scenario-status')).toContainText('검사 실패 포함');
  await expect(page.locator('#scenario-preset')).toHaveValue('mobile_panel');
  await expect(page.locator('#robustness-result')).toContainText('전체 검사 FAIL');
  expect(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth)).toBe(true);
  await page.goto('/?model=unitree_g1');
  await expect(page.locator('.category-choice[data-category="embodiment"]')).toHaveAttribute('aria-pressed','true');
  await expect(page.locator('#scenario-status')).toHaveText('기구학 미리보기');
  await expect(page.locator('.experiment-builder')).toBeHidden();
  await expect(page.locator('.model-choice[data-id="unitree_g1"]')).toBeInViewport();
  expect(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth)).toBe(true);
});

test('three Panda arms use actual articulated URDF recordings', async ({page}) => {
  await expect(page.locator('#model-title')).toContainText('Franka Panda × 3');
  await expect(page.locator('#dof')).toHaveText('23');
  await expect(page.locator('#geometry-status')).toHaveAttribute('data-state','ready');
  await expect(page.locator('#result-tracking')).toContainText('PASS');
  const errors=await page.evaluate(async()=>{
    const data=await(await fetch('/data/three_franka.json')).json();
    const {loadURDFModel,applyRecordedPose,disposeModel}=window.modelAPI;
    const errors=[];
    for(const mode of ['visual','collision']) for(const spec of data.instances){
      const robot=await loadURDFModel(spec.manifest,mode);
      const recording={...spec,q:data.q.map(row=>spec.q_indices.map(i=>row[i]))};let error=0;
      for(const frame of [0,80,190]){
        applyRecordedPose(robot,recording,frame);
        for(const [name,link] of Object.entries(robot.links)){
          const i=data.link_names.indexOf(spec.id+'_'+name);
          link.matrixWorld.elements.slice(12,15).forEach((v,j)=>error=Math.max(error,Math.abs(v-data.positions[frame][i][j])));
        }
      }
      disposeModel(robot);errors.push(error);
    }
    return errors;
  });
  errors.forEach(error=>expect(error).toBeLessThan(1e-7));
});

test('native Robotiq linkage uses recorded passive joints and a free block',async({page})=>{
  const pageErrors=[];page.on('pageerror',error=>pageErrors.push(error.message));
  await page.goto('/?model=robotiq_2f85');
  await page.evaluate(async()=>{window.modelAPI=await import('/test-module.js');});
  await expect(page.locator('#geometry-status')).toHaveAttribute('data-state','ready');
  await expect(page.locator('#scenario-status')).toHaveText('그리퍼 파지력 · 요소 검증');
  await expect(page.locator('.category-choice[data-category="diagnostic"]')).toHaveAttribute('aria-pressed','true');
  const errors=await page.evaluate(async()=>{
    const data=await(await fetch('/data/robotiq_2f85.json')).json();
    const {loadURDFModel,applyRecordedPose,disposeModel}=window.modelAPI;const out=[];
    for(const mode of ['visual','collision']) for(const spec of data.instances){
      const model=await loadURDFModel(spec.manifest,mode);let error=0;
      const recording={...spec,q:data.q.map(row=>spec.q_indices.map(i=>row[i]))};
      for(const frame of [0,80,160]){
        applyRecordedPose(model,recording,frame);
        for(const [name,link] of Object.entries(model.links)){
          const index=spec.id==='gripper'?data.link_names.indexOf(name):data.link_names.indexOf('calibration_block');
          link.matrixWorld.elements.slice(12,15).forEach((v,j)=>error=Math.max(error,Math.abs(v-data.positions[frame][index][j])));
        }
      }
      disposeModel(model);out.push(error);
    }
    return out;
  });
  errors.forEach(error=>expect(error).toBeLessThan(1e-7));
  await page.locator('#geometry-mode').selectOption('collision');
  await expect(page.locator('#geometry-status')).toHaveAttribute('data-state','ready');
  expect(pageErrors).toEqual([]);
});

test('three-arm Robotiq lift replays measured linkage and free-panel poses',async({page})=>{
  const errors=[];page.on('pageerror',e=>errors.push(e.message));
  await page.goto('/?model=panda_robotiq_lift');
  await page.evaluate(async()=>{window.modelAPI=await import('/test-module.js');});
  await expect(page.locator('#geometry-status')).toHaveAttribute('data-state','ready');
  await expect(page.locator('#scenario-status')).toHaveText('다중 팔 · 첫 lift 검증');
  const result=await page.evaluate(async()=>{
    const data=await(await fetch('/data/panda_robotiq_lift.json')).json();
    const {loadURDFModel,applyRecordedPose,disposeModel}=window.modelAPI;let error=0;
    for(const spec of data.instances){
      const model=await loadURDFModel(spec.manifest);const recording={...spec,q:data.q.map(q=>spec.q_indices.map(i=>q[i]))};
      for(const frame of [0,200,400]){
        applyRecordedPose(model,recording,frame);
        for(const [name,link] of Object.entries(model.links)){
          const index=data.link_names.indexOf(spec.id+'_'+name);
          link.matrixWorld.elements.slice(12,15).forEach((v,j)=>error=Math.max(error,Math.abs(v-data.positions[frame][index][j])));
        }
      }
      disposeModel(model);
    }
    return {error,dof:data.dof,instances:data.instances.length,passed:data.report.execution_passed,full:data.report.full_paper_pipeline_executed};
  });
  expect(result).toMatchObject({dof:29,instances:4,passed:true,full:false});expect(result.error).toBeLessThan(1e-7);expect(errors).toEqual([]);
});

test('experiment form submits edits, polls and displays the completed recording',async({page})=>{
  let submitted;
  await page.route('**/api/health',route=>route.fulfill({json:{ready:true}}));
  await page.route('**/api/experiments',async route=>{submitted=route.request().postDataJSON();await route.fulfill({status:202,json:{id:'fixture'}});});
  await page.route('**/api/experiments/fixture',route=>route.fulfill({json:{state:'completed',stage:'complete',recording:'/data/three_franka.json'}}));
  await page.reload();
  await expect(page.locator('#experiment-run')).toBeEnabled();
  await page.locator('#panel-mass').fill('1.2');
  await page.getByLabel('lift z',{exact:true}).fill('0.65');
  await page.locator('#experiment-run').click();
  await expect(page.locator('#experiment-state')).toHaveText('계산 완료');
  expect(submitted.mass).toBe(1.2);expect(submitted.keyframes[1].object_pose[2]).toBe(.65);
  expect(submitted.contacts).toHaveLength(3);
  await expect(page.locator('#result-residual')).toContainText('e-');
});

test('experiment failures and cancellation stay visible',async({page})=>{
  await page.route('**/api/health',route=>route.fulfill({json:{ready:true}}));
  await page.route('**/api/experiments',route=>route.fulfill({status:202,json:{id:'fixture'}}));
  await page.route('**/api/experiments/fixture',route=>route.fulfill({json:{state:'running',stage:'optimization'}}));
  await page.route('**/api/experiments/fixture/cancel',route=>route.fulfill({json:{state:'cancelled'}}));
  await page.reload();await page.locator('#experiment-run').click();
  await expect(page.locator('#experiment-state')).toContainText('IPOPT');
  await page.locator('#experiment-cancel').click();
  await expect(page.locator('#experiment-state')).toHaveText('취소됨');
  await page.route('**/api/experiments',route=>route.fulfill({status:400,json:{error:'normal must point out of the panel surface'}}));
  await page.locator('#experiment-run').click();
  await expect(page.locator('#experiment-state')).toContainText('normal must point out');
});

test('table assembly renders three arms and two free URDF parts through all eight keyframes',async({page})=>{
  await page.goto('/?model=table_assembly');
  await expect(page.locator('#geometry-status')).toHaveAttribute('data-state','ready');
  await expect(page.locator('#model-title')).toContainText('책상 조립');
  await expect(page.locator('#dof')).toHaveText('25');
  await expect(page.locator('.keyframe-choice')).toHaveCount(9);
  await expect(page.locator('#result-seal')).toContainText('SEATED');
  await page.getByRole('button',{name:'reorient_tabletop · 12.0 s',exact:true}).click();
  await expect(page.locator('#time-now')).toHaveText('12.00 s');
  await expect(page.locator('#tool-state')).toContainText('상판 gripper: 양쪽 pad 접촉');
  await page.getByRole('button',{name:'assembled_and_released · 30.0 s',exact:true}).click();
  await expect(page.locator('#tool-state')).toContainText('다리 gripper: 해제');
  await expect(page.locator('#tool-state')).toContainText('socket: 접촉');
  await expect(page.locator('#chart-label')).toHaveText('MEASURED INSERTION DEPTH');
  await expect(page.locator('#error-unit')).toHaveText('mm');
  await page.evaluate(async()=>{window.modelAPI=await import('/test-module.js');});
  const errors=await page.evaluate(async()=>{
    const data=await(await fetch('/data/table_assembly.json')).json();
    const {loadURDFModel,applyRecordedPose,disposeModel}=window.modelAPI;const rows=[];
    for(const mode of ['visual','collision'])for(const spec of data.instances){
      const robot=await loadURDFModel(spec.manifest,mode);let error=0;
      const recording={...spec,q:data.q.map(q=>spec.q_indices.map(i=>q[i]))};
      for(const frame of [0,240,540,600]){
        applyRecordedPose(robot,recording,frame);
        for(const [name,link] of Object.entries(robot.links)){
          const i=data.link_names.indexOf(spec.id+'_'+name);
          link.matrixWorld.elements.slice(12,15).forEach((v,j)=>error=Math.max(error,Math.abs(v-data.positions[frame][i][j])));
        }
      }rows.push(error);disposeModel(robot);
    }return rows;
  });
  expect(errors).toHaveLength(10);errors.forEach(error=>expect(error).toBeLessThan(1e-7));
});

test('assembly duration submission uses the new Python scenario API',async({page})=>{
  let payload;
  await page.route('**/api/health',route=>route.fulfill({json:{ready:true}}));
  await page.route('**/api/experiments',route=>{payload=route.request().postDataJSON();return route.fulfill({status:202,json:{id:'assembly'}});});
  await page.route('**/api/experiments/assembly',route=>route.fulfill({json:{state:'completed',stage:'complete',recording:'/data/table_assembly.json'}}));
  await page.goto('/?model=table_assembly');
  await expect(page.locator('#experiment-run')).toBeEnabled();
  await page.locator('#assembly-duration').fill('36');
  await page.locator('#experiment-run').click();
  await expect(page.locator('#experiment-state')).toHaveText('계산 완료');
  expect(payload).toEqual({preset:'table_assembly',duration:36});
  await expect(page.locator('#result-seal')).toContainText('SEATED');
});

test('primary panel recording refines source anchors, seats on a URDF jig and releases all tools',async({page})=>{
  await page.goto('/?model=panel_fixture');
  await expect(page.locator('#model-title')).toContainText('패널 → 지그');
  await expect(page.locator('#geometry-status')).toHaveAttribute('data-state','ready');
  await expect(page.locator('#scenario-preset')).toHaveValue('panel_fixture');
  await expect(page.locator('#source-result')).toContainText('synthetic 입력');
  await expect(page.locator('#source-result')).toContainText('1.053 → 1.116');
  await expect(page.locator('#result-seal')).toContainText('SUPPORTED');
  await page.getByRole('button',{name:'released · 30.0 s',exact:true}).click();
  await expect(page.locator('#tool-state')).toContainText('edge_gripper_contact: 해제');
  await expect(page.locator('#tool-state')).toContainText('support_right_contact: 해제');
  await expect(page.locator('#chart-label')).toHaveText('MEASURED PANEL HEIGHT');
  const errors=await page.evaluate(async()=>{
    const {loadURDFModel,applyRecordedPose,disposeModel}=await import('/test-module.js');
    const data=await(await fetch('/data/panel_fixture.json')).json();const rows=[];
    for(const mode of ['visual','collision'])for(const spec of data.instances){
      const robot=await loadURDFModel(spec.manifest,mode);let error=0;
      for(const frame of [0,240,440,600]){
        applyRecordedPose(robot,{...spec,q:data.q.map(q=>spec.q_indices.map(i=>q[i]))},frame);
        for(const [name,link] of Object.entries(robot.links)){
          const i=data.link_names.indexOf(spec.id+'_'+name);
          if(i<0)throw Error('Missing recorded link '+spec.id+'_'+name);
          link.matrixWorld.elements.slice(12,15).forEach((v,j)=>error=Math.max(error,Math.abs(v-data.positions[frame][i][j])));
        }
      }rows.push(error);disposeModel(robot);
    }return rows;
  });
  expect(errors).toHaveLength(10);errors.forEach(e=>expect(e).toBeLessThan(1e-7));
});

test('panel API submits the source refinement ablation and duration',async({page})=>{
  let payload;
  await page.route('**/api/health',route=>route.fulfill({json:{ready:true}}));
  await page.route('**/api/experiments',route=>{payload=route.request().postDataJSON();return route.fulfill({status:202,json:{id:'panel'}});});
  await page.route('**/api/experiments/panel',route=>route.fulfill({json:{state:'completed',stage:'complete',recording:'/data/panel_fixture.json'}}));
  await page.goto('/?model=panel_fixture');await expect(page.locator('#experiment-run')).toBeEnabled();
  await page.locator('#source-refinement').uncheck();await page.locator('#assembly-duration').fill('36');
  await page.locator('#experiment-run').click();await expect(page.locator('#experiment-state')).toHaveText('계산 완료');
  expect(payload).toMatchObject({preset:'panel_fixture',duration:36,refine_source:false,contracted_source:{contact_frame:'object',origin:'synthetic_contracted_fixture'}});
});

test('mobile RBY1 and fixed Panda replay wheel-driven workspace extension',async({page})=>{
  await page.goto('/?model=mobile_panel');
  await expect(page.locator('#model-title')).toContainText('RBY1 + Panda');
  await expect(page.locator('#geometry-status')).toHaveAttribute('data-state','ready');
  await expect(page.locator('#base')).toHaveText('Mobile SE(2)');
  await expect(page.locator('#scenario-preset')).toHaveValue('mobile_panel');
  await expect(page.locator('#preset-tag')).toHaveText('RBY1 + PANDA');
  await expect(page.locator('#workspace-result')).toContainText('고정 Panda');
  await expect(page.locator('#workspace-result')).toContainText('1.399 m');
  await expect(page.locator('#result-seal')).toContainText('SUPPORTED');
  await page.getByRole('button',{name:'outside_fixed_workspace · 18.0 s',exact:true}).click();
  await expect(page.locator('#tool-state')).toContainText('fixed_arm_support: 해제');
  await expect(page.locator('#tool-state')).toContainText('rby_gripper: 접촉');
  await expect(page.locator('#chart-label')).toHaveText('MEASURED MOBILE BASE TRAVEL');
  await page.getByRole('button',{name:'released · 36.0 s',exact:true}).click();
  await expect(page.locator('#tool-state')).toContainText('rby_gripper: 해제');
  const errors=await page.evaluate(async()=>{
    const {loadURDFModel,applyRecordedPose,disposeModel}=await import('/test-module.js');
    const data=await(await fetch('/data/mobile_panel.json')).json();const errors=[];
    for(const mode of ['visual','collision'])for(const spec of data.instances){
      const robot=await loadURDFModel(spec.manifest,mode);let error=0;
      for(const frame of [0,120,360,480,720]){
        applyRecordedPose(robot,{...spec,q:data.q.map(q=>spec.q_indices.map(i=>q[i]))},frame);
        for(const [name,link] of Object.entries(robot.links)){
          const i=data.link_names.indexOf(spec.id+'_'+name);if(i<0)throw Error('Missing '+spec.id+'_'+name);
          link.matrixWorld.elements.slice(12,15).forEach((v,j)=>error=Math.max(error,Math.abs(v-data.positions[frame][i][j])));
        }
      }errors.push(error);disposeModel(robot);
    }return errors;
  });
  expect(errors).toHaveLength(8);errors.forEach(e=>expect(e).toBeLessThan(1e-7));
});

test('mobile form submits the mixed embodiment and source JSON to Python',async({page})=>{
  let payload;
  await page.route('**/api/health',route=>route.fulfill({json:{ready:true}}));
  await page.route('**/api/experiments',route=>{payload=route.request().postDataJSON();return route.fulfill({status:202,json:{id:'mobile'}});});
  await page.route('**/api/experiments/mobile',route=>route.fulfill({json:{state:'completed',stage:'complete',recording:'/data/mobile_panel.json'}}));
  await page.goto('/?model=mobile_panel');await expect(page.locator('#experiment-run')).toBeEnabled();await page.locator('#assembly-duration').fill('45');
  await page.locator('#experiment-run').click();await expect(page.locator('#experiment-state')).toHaveText('계산 완료');
  expect(payload).toMatchObject({preset:'mobile_panel',duration:45,contracted_source:{origin:'synthetic_mobile_handover_fixture'}});
});

for(const id of ['hinged_door','folding_panel','drawer']){
  test(`${id}: passive measured joint and studio rendering preserve URDF FK`,async({page})=>{
    await page.goto(`/?model=${id}`);
    await expect(page.locator('#geometry-status')).toHaveAttribute('data-state','ready');
    await expect(page.locator('#result-tracking')).toContainText('PASS');
    await expect(page.locator('#result-seal')).toContainText('0 actuators');
    await expect(page.locator('#scenario-preset')).toHaveValue(id);
    await expect(page.locator('#assembly-duration')).toHaveValue('14');
    await page.getByRole('button',{name:'mid_motion · 6.5 s',exact:true}).click();
    await page.locator('#scene-style').selectOption('analysis');
    await page.locator('#scene-style').selectOption('studio');
    await expect(page.locator('#time-now')).toHaveText('6.50 s');
    const error=await page.evaluate(async id=>{
      const {loadURDFModel,applyRecordedPose,disposeModel}=await import('/test-module.js');
      const d=await(await fetch(`/data/${id}.json`)).json();let error=0;
      for(const mode of ['visual','collision'])for(const spec of d.instances){
        const robot=await loadURDFModel(spec.manifest,mode),rec={...spec,q:d.q.map(row=>spec.q_indices.map(i=>row[i]))};
        for(const frame of [0,195,d.q.length-1]){applyRecordedPose(robot,rec,frame);for(const [name,link] of Object.entries(robot.links)){const i=d.link_names.indexOf(spec.id+'_'+name);link.matrixWorld.elements.slice(12,15).forEach((v,j)=>error=Math.max(error,Math.abs(v-d.positions[frame][i][j])));}}
        disposeModel(robot);
      }return error;
    },id);
    expect(error).toBeLessThan(1e-7);
    await page.locator('#geometry-mode').selectOption('collision');
    await expect(page.locator('#geometry-status')).toHaveAttribute('data-state','ready');
  });
}
test('external pulse shows measured recovery and preserves capability failure',async({page})=>{
  await page.goto('/?model=mobile_push_25');
  await expect(page.locator('#geometry-status')).toHaveAttribute('data-state','ready');
  await expect(page.locator('#robustness-result')).toContainText('자세 회복 PASS');
  await expect(page.locator('#robustness-result')).toContainText('전체 검사 FAIL');
  await page.locator('#play').click();
  const expectedForce=await page.locator('#timeline').evaluate(el=>{el.value=String(16.25/36);el.dispatchEvent(new Event('input'));return (25*Math.sin(Math.PI*(Number(el.value)*36-16)/.5)**2).toFixed(2);});
  await expect(page.locator('#disturbance-state')).toContainText(expectedForce+' N');
  await page.locator('#timeline').evaluate(el=>{el.value=String(18/36);el.dispatchEvent(new Event('input'));});
  await expect(page.locator('#disturbance-state')).not.toContainText('적용 중');
});

for(const id of ['free_box_lid','free_folding_object','free_portable_drawer']){
 test(`${id}: contracted link-aware contacts realize on three arms and a free articulated body`,async({page})=>{
  await page.goto(`/?model=${id}`);
  await expect(page.locator('#geometry-status')).toHaveAttribute('data-state','ready');
  await expect(page.locator('#dof')).toHaveText('25');
  await expect(page.locator('#base')).toHaveText('Free object + joint');
  await expect(page.locator('#source-result')).toContainText('125 dense samples → 3 link-aware anchors → 3 arms');
  await expect(page.locator('#result-tracking')).toContainText('PASS');
  await expect(page.locator('#contraction-state')).toContainText('base_grasp@base');
  await expect(page.locator('#contraction-state')).toContainText('moving_manipulation@moving');
  await page.getByRole('button',{name:'mid_internal_motion · 9.0 s',exact:true}).click();
  await page.locator('#contact-display').selectOption('contracted');
  await page.locator('#contact-display').selectOption('dense');
  await expect(page.locator('#time-now')).toHaveText('9.00 s');
  const errors=await page.evaluate(async id=>{
   const data=await(await fetch(`/data/${id}.json`)).json(),{loadURDFModel,applyRecordedPose,disposeModel}=await import('/test-module.js');let error=0;
   for(const mode of ['visual','collision'])for(const spec of data.instances){const model=await loadURDFModel(spec.manifest,mode);for(const frame of [0,270,data.times.length-1]){applyRecordedPose(model,{...spec,q:data.q.map(row=>spec.q_indices.map(i=>row[i]))},frame);for(const [name,link] of Object.entries(model.links)){const i=data.link_names.indexOf(spec.id+'_'+name);link.matrixWorld.elements.slice(12,15).forEach((v,j)=>error=Math.max(error,Math.abs(v-data.positions[frame][i][j])));}}disposeModel(model);}return error;
  },id);expect(errors).toBeLessThan(1e-7);
 });
}
test('force peak event pauses with an explicitly visible force arrow',async({page})=>{
 await page.goto('/?model=free_box_push');
 await expect(page.locator('#geometry-status')).toHaveAttribute('data-state','ready');
 await page.getByRole('button',{name:'외력 최대 · 화살표 · 8.5 s',exact:true}).click();
 await expect(page.locator('#time-now')).toHaveText('8.50 s');
 await expect(page.locator('#disturbance-state')).toHaveAttribute('data-active','true');
 await expect(page.locator('#disturbance-state')).toContainText('5.00 N');
 await page.getByRole('button',{name:'target · 13.0 s',exact:true}).click();
 await expect(page.locator('#disturbance-state')).toHaveAttribute('data-active','false');
});

test('full assembly external push replays applied force and all controlled ablations',async({page})=>{
 await page.goto('/?model=full_assembly_external_push');
 await expect(page.locator('#geometry-status')).toHaveAttribute('data-state','ready');
 const d=await page.evaluate(async()=>await(await fetch('/data/full_assembly_external_push.json')).json());
 expect(d.report.execution_passed).toBe(true);expect(Object.keys(d.report.ablation_results)).toHaveLength(7);
 expect(d.report.ablation_results.no_arm_force_feedforward.execution_passed).toBe(false);
 expect(d.report.ablation_results.contact_disabled.execution_passed).toBe(false);
 expect(d.report.ablation_results.fixed_path.execution_passed).toBe(true);
 expect(d.report.ablation_results.fixed_path.selection.all_moving_object_intervals_cito_optimized).toBe(false);
 expect(d.external_disturbances).toHaveLength(1);
 const peak=d.events.find(e=>e.id==='외력 최대 · 화살표');
 await page.getByRole('button',{name:`외력 최대 · 화살표 · ${peak.time.toFixed(1)} s`,exact:true}).click();
 await expect(page.locator('#disturbance-state')).toHaveAttribute('data-active','true');
 await expect(page.locator('#disturbance-state')).toContainText('5.00 N');
 await expect(page.locator('#ablation-results tr[data-variant="contact_disabled"]')).toContainText('음성 대조군');
});

test('free-object form preserves edited dense groups while retiming',async({page})=>{
 await page.route('**/api/health',r=>r.fulfill({json:{ready:true,experiments_ready:true}}));let submitted;
 await page.route('**/api/experiments',async r=>{submitted=r.request().postDataJSON();await r.fulfill({status:202,json:{id:'dense'}});});
 await page.route('**/api/experiments/dense',r=>r.fulfill({json:{state:'completed',stage:'complete',recording:'/data/free_box_lid.json'}}));
 await page.goto('/?model=free_box_lid');await expect(page.locator('#scenario-preset')).toHaveValue('free_box_lid');await expect(page.locator('#experiment-run')).toBeEnabled();
 await page.locator('#contacts-json').evaluate(el=>{const d=JSON.parse(el.value);d[1].weights[0]=2;el.value=JSON.stringify(d);});
 await page.locator('#assembly-duration').fill('22');await page.locator('#experiment-run').click();
 await expect.poll(()=>submitted?.duration).toBe(22);expect(submitted.preset).toBe('free_box_lid');expect(submitted.dense_source_groups[1].weights[0]).toBe(2);
});

for (const task of ['assembly','box','panel']) {
  test(`full ${task}: optimized multi-arm task retains measured URDF and acceptance provenance`,async({page})=>{
    await page.goto(`/?model=full_${task}`);
    await page.evaluate(async()=>{window.modelAPI=await import("/test-module.js");});
    await expect(page.locator('#geometry-status')).toHaveAttribute('data-state','ready');
    await expect(page.locator('#scenario-status')).toContainText('전체 구간 · 검증 통과');
    await expect(page.locator('#result-tracking')).toContainText('PASS');
    await expect(page.locator('#result-forces')).toContainText('PASS');
    await expect(page.locator('#ablation-results tr[data-variant="full"]')).toContainText('PASS');
    await expect(page.locator('#full-result-links')).toContainText('실패 이유');
    await expect(page.locator('#ablation-results')).toContainText('성능 향상을 입증하지 못합니다');
    await expect(page.locator('#experiment-form')).toBeHidden();
    const result=await page.evaluate(async task=>{
      const data=await(await fetch(`/data/full_${task}.json`)).json(),{loadURDFModel,applyRecordedPose,disposeModel}=window.modelAPI;let maximum=0;
      for(const mode of ['visual','collision'])for(const spec of data.instances){
        const robot=await loadURDFModel(spec.manifest,mode),recording={...spec,q:data.q.map(row=>spec.q_indices.map(i=>row[i]))};
        for(const k of [0,Math.floor(data.q.length/2),data.q.length-1]){
          applyRecordedPose(robot,recording,k);
          for(const [name,link] of Object.entries(robot.links)){
            const i=data.link_names.indexOf(spec.id+'_'+name);if(i<0)continue;
            link.matrixWorld.elements.slice(12,15).forEach((v,j)=>{maximum=Math.max(maximum,Math.abs(v-data.positions[k][i][j]));});
          }
        }disposeModel(robot);
      }
      return {maximum,data};
    },task);
    expect(result.maximum).toBeLessThan(1e-6);
    const d=result.data,r=d.report;
    expect(d.native_contact_samples.source).toBe('raw_native_contacts');expect(d.native_contact_samples.interpolated).toBe(false);
    expect(d.native_contact_samples.sample_times_s).toEqual(d.times);expect(d.native_contact_samples.frames).toHaveLength(d.times.length);
    await page.locator('#timeline').evaluate(el=>{el.value='.8';el.dispatchEvent(new Event('input'));});
    await page.locator('#contact-focus').selectOption(d.contact_contraction.groups[0].group_id);
    await expect(page.locator('#play')).toHaveAttribute('aria-label','재생');
    await expect(page.locator('#native-contact-state')).toContainText('보간 없음');
    expect(Number(await page.locator('#native-contact-state').getAttribute('data-count'))).toBeGreaterThan(0);
    expect(d.full_task_recording).toBe(true);expect(d.multi_arm).toBe(true);expect(r.execution_passed).toBe(true);
    expect(r.full_variant_pipeline_executed).toBe(true);expect(r.full_paper_pipeline_executed).toBe(false);
    expect(r.selection.selection_executed).toBe(true);expect(r.selection.candidate_count).toBeGreaterThanOrEqual(12);
    expect(r.selection.intervals).toBe(r.selection.keyframes-1);expect(r.selection.keyframes).toBeGreaterThanOrEqual(5);
    expect(d.contact_contraction.network_inference_executed).toBe(false);expect(d.contact_contraction.dense_sample_count).toBe(0);
    expect(d.provenance.canonical_point_tool.radius_m).toBe(.025);
    expect(d.provenance.canonical_point_tool.ee_to_tip.slice(0,3).map(row=>row[3])).toEqual([0,0,.165]);
    expect(r.object_equality_constraints_used).toBe(0);expect(r.sampled_interface_wrench_capability.passed).toBe(true);
    const negative=page.locator('#ablation-results tr[data-variant="contact_disabled"]');await expect(negative).toContainText('FAIL');await expect(negative).toContainText('목표');
    if(task==='box'){
      const fixed=r.ablation_results.fixed_path,peak=Math.max(...Object.values(fixed.jaw_force).flatMap(j=>j.all_time_maximum_per_jaw_N||j.maximum_per_jaw_N));
      await expect(page.locator('#ablation-results tr[data-variant="fixed_path"]')).toContainText(peak.toFixed(2)+' N / 40 N');
      const object=d.instances.find(i=>i.id==='object');const xml=await page.evaluate(async manifest=>{const m=await(await fetch(manifest)).json();return await(await fetch(new URL(m.urdf,new URL(manifest,location.href)))).text();},object.manifest);
      expect(xml).toContain('base_handle_connector');
      const final=d.native_contact_samples.frames.at(-1);expect(final.filter(p=>p.group_id==='moving_manipulation')).toHaveLength(2);
    }
    if(task==='assembly'){expect(r.task_metrics.leg_released).toBe(true);expect(r.task_metrics.lateral_error_m).toBeLessThan(.002);}
  });
}

test('matched source-polytope ablation preserves both outcomes and the applied force arrow',async({page})=>{
 await page.goto('/?model=robust_refinement_polytope_-4N');
 await expect(page.locator('#geometry-status')).toHaveAttribute('data-state','ready');
 await expect(page.locator('#scenario-status')).toContainText('Source robustness · PASS');
 await expect(page.locator('#source-result')).toContainText('ON · 8 vertices');
 const results=page.locator('#algorithm-comparison-results');await expect(results).toContainText('Source polytope ON/OFF');
 await expect(results.locator('tr[data-robustness="true"][data-force="-4"]')).toContainText('PASS');
 await expect(results.locator('tr[data-robustness="false"][data-force="-4"]')).toContainText('FAIL');
 const study=await page.evaluate(async()=>await(await fetch('/data/algorithm-comparisons.json')).json());
 const trials=study.source_robustness.trials;expect(trials).toHaveLength(14);expect(study.source_robustness.controller_settings_equal).toBe(true);
 for(const enabled of [true,false]){const same=trials.filter(t=>t.robustness_enabled===enabled);expect(new Set(same.map(t=>t.native_plan_id)).size).toBe(1);expect(same.find(t=>t.force_N===0).execution_passed).toBe(true);}
 const on=trials.find(t=>t.force_N===-4&&t.robustness_enabled),off=trials.find(t=>t.force_N===-4&&!t.robustness_enabled);
 expect(on.inside_initial_static_box).toBe(true);expect(off.inside_initial_static_box).toBe(false);expect(on.source_rho).toBeGreaterThan(off.source_rho);
 await page.getByRole('button',{name:'외력 최대 · 화살표 · 4.5 s',exact:true}).click();
 await expect(page.locator('#disturbance-state')).toHaveAttribute('data-active','true');await expect(page.locator('#disturbance-state')).toContainText('4.00 N');
 await page.goto('/?model=robust_refinement_nominal_-4N');await expect(page.locator('#geometry-status')).toHaveAttribute('data-state','ready');
 await expect(page.locator('#scenario-status')).toContainText('Source robustness · FAIL');await expect(page.locator('#result-tracking')).toContainText('FAIL');await expect(page.locator('#source-result')).toContainText('OFF · nominal only');
});

for(const [task,types] of [['panel','PPP'],['box','PPG']])test(`forced ${task} ${types}: worse sequence and measured execution failure remain visible`,async({page})=>{
 await page.goto(`/?model=counterfactual_${task}_${types}`);await expect(page.locator('#geometry-status')).toHaveAttribute('data-state','ready');
 await expect(page.locator('#scenario-status')).toContainText('강제 접촉 유형 · FAIL');await expect(page.locator('#source-result')).toContainText(types.split('').join(' / '));
 const study=await page.evaluate(async()=>await(await fetch('/data/algorithm-comparisons.json')).json());const group=study.contact_types[task],other=group.counterfactuals.find(r=>r.types.join('')===types);
 expect(other.stage).toBe('native_execution');expect(other.new_physical_trial_executed).toBe(true);expect(other.sequence_objective).toBeGreaterThan(group.selected_sequence_objective);expect(other.execution_passed).toBe(false);
 await expect(page.locator('#algorithm-comparison-results')).toContainText('시퀀스 비용');
 const d=await page.evaluate(async id=>await(await fetch('/data/'+id+'.json')).json(),`counterfactual_${task}_${types}`);
 expect(d.report.selection.source_refinement_enabled).toBe(true);expect(d.report.object_pose_prescribed).toBe(false);expect(d.report.object_equality_constraints_used).toBe(0);
});
