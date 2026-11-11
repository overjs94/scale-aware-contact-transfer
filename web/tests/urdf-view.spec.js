import { test, expect } from "@playwright/test";

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
    body: route.request().url().toLowerCase().endsWith(".dae") ? dae : stl,
    headers: { "Access-Control-Allow-Origin": "*" },
  }));
  await page.goto("/");
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
