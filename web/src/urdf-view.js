import * as THREE from "three";
import URDFLoader from "urdf-loader";
import { OBJLoader } from "three/addons/loaders/OBJLoader.js";

export function disposeModel(root) {
  const geometries = new Set(), materials = new Set(), textures = new Set();
  root?.traverse(node => {
    if (node.geometry) geometries.add(node.geometry);
    for (const material of (Array.isArray(node.material) ? node.material : [node.material])) {
      if (!material) continue;
      materials.add(material);
      for (const value of Object.values(material)) if (value?.isTexture) textures.add(value);
    }
  });
  geometries.forEach(g => g.dispose()); materials.forEach(m => m.dispose()); textures.forEach(t => t.dispose());
  root?.removeFromParent();
}

export async function loadURDFModel(manifestURL, mode = "visual") {
  if (!["visual", "collision"].includes(mode)) throw Error("Unknown URDF geometry mode");
  const response = await fetch(manifestURL);
  if (!response.ok) throw Error("Could not load the URDF model manifest.");
  const manifest = await response.json();
  const urdfURL = new URL(manifest.urdf, new URL(manifestURL, location.href));
  const source = await fetch(urdfURL);
  if (!source.ok) throw Error("Could not load the URDF file.");
  const xml = await source.text();
  const document = new DOMParser().parseFromString(xml, "text/xml");
  if (document.querySelector("parsererror") || document.documentElement.tagName !== "robot") throw Error("Invalid URDF document.");
  if (!document.querySelector(`link > ${mode}`)) throw Error(`URDF has no ${mode} geometry.`);
  // A sentinel keeps primitive-only URDFs and nested DAE texture loads in the
  // same completion path. A failed mesh never becomes a silent partial model.
  const manager = new THREE.LoadingManager(), failures = [];
  const loader = new URDFLoader(manager);
  loader.packages = Object.fromEntries(Object.entries(manifest.packages || {}).map(([name, path]) =>
    [name, new URL(path, new URL(manifestURL, location.href)).href.replace(/\/$/, "")]));
  loader.parseVisual = mode === "visual";
  loader.parseCollision = mode === "collision";
  const meshLoader = loader.loadMeshCb;
  loader.loadMeshCb = (url, mgr, material, done) => {
    const extension = new URL(url, location.href).pathname.split(".").at(-1).toLowerCase();
    if (extension === "obj") {
      new OBJLoader(mgr).load(url, obj => {
        obj.traverse(node => { if (node.isMesh) node.material = material; });
        done(obj);
      }, undefined, error => done(null, error));
    } else if (["stl", "dae"].includes(extension)) {
      meshLoader(url, mgr, material, done);
    } else {
      failures.push(url); done(null, Error("Unsupported mesh: " + extension));
    }
  };
  let robot;
  await new Promise((resolve, reject) => {
    manager.onError = url => failures.push(url);
    manager.onLoad = () => failures.length ? reject(Error("Could not load mesh: " + failures[0])) : resolve();
    manager.itemStart("urdf-parse");
    try { robot = loader.parse(document, urdfURL.href.slice(0, urdfURL.href.lastIndexOf("/")+1)); }
    catch (error) { reject(error); }
    finally { manager.itemEnd("urdf-parse"); }
  }).catch(error => { disposeModel(robot); throw error; });
  if (mode === "collision") {
    robot.traverse(node => {
      if (node.isMesh) {
        node.material = new THREE.MeshStandardMaterial({ color: 0x5bdac8, roughness: .65,
          transparent: true, opacity: .65, wireframe: true });
      }
    });
  }
  robot.userData.manifest = manifest;
  robot.userData.geometryMode = mode;
  return robot;
}

export function applyRecordedPose(robot, data, value) {
  if (!Array.isArray(data.joint_names) || data.joint_names.length !== data.dof) throw Error("Joint names are missing from the recording.");
  const lo = Math.floor(value), hi = Math.min(lo+1, data.q.length-1), f = value-lo;
  const offset = data.floating_base ? 7 : 0;
  const joints = { ...(data.locked_joints || {}) };
  for (let i = 0; i < data.joint_names.length; i++) {
    const name = data.joint_names[i];
    if (!robot.joints[name]) throw Error("Recorded joint is missing from the URDF: " + name);
    // Replay measured soft-limit overshoot faithfully; validation owns limits.
    robot.joints[name].ignoreLimits = true;
    joints[name] = THREE.MathUtils.lerp(data.q[lo][offset+i], data.q[hi][offset+i], f);
  }
  robot.setJointValues(joints);
  if (data.floating_base) {
    robot.position.fromArray(data.q[lo]).lerp(new THREE.Vector3().fromArray(data.q[hi]), f);
    robot.quaternion.fromArray(data.q[lo], 3).slerp(new THREE.Quaternion().fromArray(data.q[hi], 3), f);
  } else {
    robot.position.fromArray(data.base_pose?.position || [0, 0, 0]);
    robot.quaternion.fromArray(data.base_pose?.quaternion_xyzw || [0, 0, 0, 1]);
  }
  robot.updateMatrixWorld(true);
}
