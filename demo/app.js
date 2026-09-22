import * as THREE from "three";
import { OrbitControls } from "/vendor/OrbitControls.js";
import { STLLoader } from "/vendor/STLLoader.js";
import { LineSegments2 } from "/vendor/lines/LineSegments2.js";
import { LineSegmentsGeometry } from "/vendor/lines/LineSegmentsGeometry.js";
import { LineMaterial } from "/vendor/lines/LineMaterial.js";

const $ = (id) => document.getElementById(id);
const scene = new THREE.Scene();
scene.background = new THREE.Color("#eeede8");
scene.add(new THREE.HemisphereLight(0xffffff, 0xabb3bb, 2.0));
const sun = new THREE.DirectionalLight(0xfff7ed, 2.2);
sun.position.set(0.2, -0.4, 1.2);
sun.castShadow = true;
sun.shadow.mapSize.set(2048, 2048);
Object.assign(sun.shadow.camera, {
  left: -0.65,
  right: 0.65,
  top: 0.65,
  bottom: -0.65,
  near: 0.01,
  far: 3,
});
sun.shadow.bias = -0.0002;
sun.shadow.normalBias = 0.001;
scene.add(sun);
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;
renderer.setClearColor("#eeede8");
$("viewport").append(renderer.domElement);
const camera = new THREE.PerspectiveCamera(38, 1, 0.005, 10);
camera.up.set(0, 0, 1);
const controls = new OrbitControls(camera, renderer.domElement);
controls.target.set(0.16, 0, 0.1);
controls.enableDamping = true;
controls.minDistance = 0.3;
controls.maxDistance = 20;
let viewCenter = new THREE.Vector3(0.16, 0, 0.105), viewRadius = 0.7;
function perspective() {
  $("perspective").setAttribute("aria-pressed", "true");
  $("top").setAttribute("aria-pressed", "false");
  camera.up.set(0, 0, 1);
  camera.position.copy(viewCenter).add(new THREE.Vector3(0.85, 1.05, 0.85).multiplyScalar(viewRadius));
  controls.target.copy(viewCenter);
  controls.update();
}
perspective();
$("perspective").onclick = perspective;
$("top").onclick = () => {
  $("perspective").setAttribute("aria-pressed", "false");
  $("top").setAttribute("aria-pressed", "true");
  camera.up.set(0, 1, 0);
  camera.position.copy(viewCenter).add(new THREE.Vector3(0, 0, viewRadius * 1.8));
  controls.target.copy(viewCenter);
  controls.update();
};
const material = (color, more = {}) =>
  new THREE.MeshStandardMaterial({ color, roughness: 0.82, flatShading: true, ...more });
function mesh(geometry, mat, parent = scene) {
  const obj = new THREE.Mesh(geometry, mat);
  obj.castShadow = true;
  obj.receiveShadow = false;
  parent.add(obj);
  return obj;
}
const grid = new THREE.GridHelper(0.8, 16, 0xa8a9a4, 0xcacac3);
grid.rotation.x = Math.PI / 2;
grid.position.set(0.18, 0, 0.0003);
grid.material.transparent = true;
grid.material.opacity = 0.18;
scene.add(grid);
const robotVisuals = [];
let robotMode = null, genericRobotKey = null;
const objectsRoot = new THREE.Group();
scene.add(objectsRoot);
const table = mesh(
  new THREE.PlaneGeometry(1.1, 0.85),
  material("#dfddd5", { side: THREE.DoubleSide }),
);
table.visible = false;
const matrix = (rows) => new THREE.Matrix4().fromArray(rows.flat()).transpose();
let revision = null;
const objectMeshes = new Map();
const objectCards = new Map();
const objectData = new Map();
let selectedObject = null;
const selectionOutline = new THREE.Box3Helper(new THREE.Box3(), 0xb65339);
selectionOutline.visible = false;
scene.add(selectionOutline);
const coordinates = (values) => values ? values.map(v => v.toFixed(3)).join(" / ") : "Unknown";
function updateSelection() {
  const obj = objectData.get(selectedObject), group = objectMeshes.get(selectedObject);
  $("selection-panel").hidden = !obj;
  selectionOutline.visible = !!group;
  if (!obj) return;
  $("selected-name").textContent = englishName(obj.name);
  $("selected-kind").textContent = obj.asset ? `Shared shape · ${obj.asset.replaceAll("_", " ")}` : relation(obj.role || obj.shape);
  $("selected-position").textContent = coordinates(group?.position.toArray() || obj.position);
  $("selected-dimensions").textContent = coordinates(obj.dimensions || obj.size);
  const source = obj.source;
  $("selected-source").textContent = typeof source === "string" ? source : source?.method?.replaceAll("_", " ") || "Not recorded";
  if (group) selectionOutline.box.setFromObject(group);
}
function selectObject(name) {
  selectedObject = selectedObject === name ? null : name;
  for (const card of $("objects").children) {
    card.classList.toggle("selected", card.dataset.name === selectedObject);
    if (card.tagName === "BUTTON") card.setAttribute("aria-pressed", card.dataset.name === selectedObject);
  }
  updateSelection();
}
const raycaster = new THREE.Raycaster();
let pointerStart;
renderer.domElement.addEventListener("pointerdown", e => { pointerStart = [e.clientX, e.clientY]; });
renderer.domElement.addEventListener("pointerup", e => {
  if (!pointerStart || Math.hypot(e.clientX-pointerStart[0], e.clientY-pointerStart[1]) > 5) return;
  const rect = renderer.domElement.getBoundingClientRect();
  raycaster.setFromCamera(new THREE.Vector2((e.clientX-rect.left)/rect.width*2-1, -(e.clientY-rect.top)/rect.height*2+1), camera);
  const hit = raycaster.intersectObjects(objectsRoot.children, true)[0];
  if (hit) selectObject(hit.object.parent.userData.name);
});
function disposeObjects() {
  objectMeshes.clear();
  objectData.clear();
  objectCards.clear();
  for (const object of [...objectsRoot.children]) {
    objectsRoot.remove(object);
    object.traverse((c) => {
      c.geometry?.dispose();
      c.material?.dispose();
    });
  }
}
function objectColor(obj) {
  if (obj.rgba) return new THREE.Color(...obj.rgba.slice(0, 3)).getStyle();
  return obj.shape === "cup"
    ? "#304943"
    : obj.shape === "ellipsoid"
      ? "#eef0e3"
      : /yellow/i.test(obj.name)
        ? "#f2bb2f"
        : "#a4beb2";
}
function buildObject(obj) {
  const group = new THREE.Group();
  group.userData.name = obj.name;
  objectsRoot.add(group);
  objectMeshes.set(obj.name, group);
  group.position.fromArray(obj.position);
  const size = obj.size,
    color = objectColor(obj);
  if (obj.vertices) {
    mesh(bufferGeometry(obj), material(color, {side: THREE.DoubleSide,
      transparent: (obj.rgba?.[3] ?? 1) < 1, opacity: obj.rgba?.[3] ?? 1}), group);
    setObjectPose(group, obj);
  } else if (obj.shape === "box")
    mesh(new THREE.BoxGeometry(...size), material(color), group);
  else if (obj.shape === "ellipsoid") {
    const o = mesh(new THREE.SphereGeometry(1, 36, 24), material(color), group);
    o.scale.fromArray(size);
  } else if (obj.shape === "cup") {
    const [outer, inner, h] = size;
    for (const [radius, color, side] of [
      [outer, "#304943", THREE.FrontSide],
      [inner, "#98443d", THREE.BackSide],
    ]) {
      const o = mesh(
        new THREE.CylinderGeometry(radius, radius, h, 64, 1, true),
        material(color, { side }),
        group,
      );
      o.rotation.x = Math.PI / 2;
    }
    const bottom = mesh(
      new THREE.CylinderGeometry(inner, inner, Math.min(0.006, h / 8), 64),
      material("#98443d"),
      group,
    );
    bottom.rotation.x = Math.PI / 2;
    bottom.position.z = -h / 2 + Math.min(0.003, h / 16);
    const rim = mesh(
      new THREE.RingGeometry(inner, outer, 64),
      material("#456053", { side: THREE.DoubleSide }),
      group,
    );
    rim.position.z = h / 2;
  }
}
const relations = {
  table: "On table",
  confirmed: "Held by gripper",
  pending: "Grasp in progress",
  release_pending: "Released · awaiting confirmation",
  in_cup_confirmed: "Inside cup",
  unknown_pending_fresh_observation: "Awaiting observation",
};
const englishName = (value) => value ? value.replaceAll("_", " ").replace(/\b\w/g, c => c.toUpperCase()) : "Object";
function relation(value) {
  return relations[value] || (value || "Unknown").replaceAll("_", " ");
}
function bufferGeometry(data) {
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute("position", new THREE.Float32BufferAttribute(data.vertices.flat(), 3));
  geometry.setIndex(data.faces.flat());
  geometry.computeVertexNormals();
  return geometry;
}
function setObjectPose(group, obj) {
  if (obj.position) group.position.fromArray(obj.position);
  if (obj.quaternion) {
    const [w, x, y, z] = obj.quaternion;
    group.quaternion.set(x, y, z, w);
  }
}
function showGenericRobot(state) {
  const signature = JSON.stringify(state.robot_visuals);
  if (signature !== genericRobotKey) {
    for (let i = robotVisuals.length - 1; i >= 0; i--) {
      const visual = robotVisuals[i];
      if (!visual.generic) continue;
      scene.remove(visual.mesh);
      visual.mesh.geometry.dispose();
      visual.mesh.material.dispose();
      robotVisuals.splice(i, 1);
    }
    for (const visual of state.robot_visuals || []) {
      const obj = mesh(bufferGeometry(visual), material(objectColor(visual), {flatShading: false}));
      obj.matrixAutoUpdate = false;
      robotVisuals.push({mesh: obj, link: visual.link, origin: new THREE.Matrix4(),
        scale: new THREE.Vector3(1, 1, 1), generic: true});
    }
    genericRobotKey = signature;
  }
  robotMode = "generic";
  for (const visual of robotVisuals) visual.mesh.visible = !!visual.generic;
  $("robot-name").textContent = state.robot_name || "Environment";
}
function fitView(focusObjects = false) {
  const bounds = new THREE.Box3();
  for (const [name, obj] of objectMeshes) {
    if (!surfaceObjects.has(name)) bounds.expandByObject(obj);
  }
  if (bounds.isEmpty()) bounds.setFromObject(objectsRoot);
  for (const visual of robotVisuals) if (visual.mesh.visible && !focusObjects) bounds.expandByObject(visual.mesh);
  if (bounds.isEmpty()) return;
  bounds.getCenter(viewCenter);
  viewRadius = Math.max(bounds.getSize(new THREE.Vector3()).length() * 0.9, 0.24);
  camera.far = Math.max(10, viewRadius * 10);
  controls.maxDistance = viewRadius * 8;
  controls.minDistance = viewRadius * 0.08;
  camera.updateProjectionMatrix();
  perspective();
}
const surfaceObjects = new Set();
$("fit-view").onclick = () => fitView();
$("focus-object").onclick = () => {
  const group = objectMeshes.get(selectedObject);
  if (!group) return;
  const bounds = new THREE.Box3().setFromObject(group);
  bounds.getCenter(viewCenter);
  viewRadius = Math.max(bounds.getSize(new THREE.Vector3()).length() * 0.9, 0.24);
  perspective();
};
let pathsVisible = true;
$("paths-toggle").onclick = () => {
  pathsVisible = !pathsVisible;
  $("paths-toggle").setAttribute("aria-pressed", pathsVisible);
  for (const obj of [trail, fullTrail, tipMarker]) if (obj) obj.visible = pathsVisible;
};
function showObjects(objects) {
  disposeObjects();
  $("objects").replaceChildren();
  $("object-count").textContent = objects.length;
  for (const obj of objects) {
    objectData.set(obj.name, obj);
    if (obj.drawable) buildObject(obj);
    const card = document.createElement("button");
    card.type = "button";
    card.dataset.name = obj.name;
    card.onclick = () => selectObject(obj.name);
    card.setAttribute("aria-pressed", selectedObject === obj.name);
    card.className = "object" + (selectedObject === obj.name ? " selected" : "");
    const title = document.createElement("div");
    title.className = "object-name";
    const dot = document.createElement("i");
    dot.className = "dot";
    dot.style.background = objectColor(obj);
    title.append(dot, document.createTextNode(englishName(obj.name)));
    card.append(title);
    const rel = document.createElement("p");
    rel.textContent = obj.geometry_error ? "Geometry unavailable" : obj.asset ? `Shape · ${obj.asset.replaceAll("_", " ")}` : relation(obj.relation || obj.role);
    card.append(rel);
    const location = document.createElement("small");
    location.textContent = obj.position
      ? obj.position.map((x) => (x * 1000).toFixed(0)).join(" / ") + " mm"
      : "Position unknown";
    card.append(location);
    objectCards.set(obj.name, location);
    $("objects").append(card);
  }
  if (!objects.length) {
    const p = document.createElement("p");
    p.className = "empty";
    p.textContent = "No objects recorded";
    $("objects").append(p);
  }
  updateSelection();
}
function setPlane(plane, planeObject = null) {
  surfaceObjects.clear();
  if (planeObject) surfaceObjects.add(planeObject);
  // Neutral support surfaces keep the remembered objects visually distinct.
  objectMeshes.get(planeObject)?.traverse(obj => {
    if (!obj.isMesh) return;
    obj.material.color.set("#e3e0d7");
    obj.receiveShadow = true;
    obj.castShadow = false;
  });
  table.visible = !!plane && !objectMeshes.has(planeObject);
  grid.visible = !objectMeshes.has(planeObject);
  grid.position.z =
    plane && Math.abs(plane[2]) > 1e-8 ? -plane[3] / plane[2] + 0.0003 : 0.0003;
  if (plane) {
    const [nx, ny, nz, d] = plane,
      n = new THREE.Vector3(nx, ny, nz);
    table.position.copy(n).multiplyScalar(-d / n.lengthSq());
    table.quaternion.setFromUnitVectors(
      new THREE.Vector3(0, 0, 1),
      n.normalize(),
    );
  }
}
function pose(frames, knownLinks = null) {
  for (const v of robotVisuals) {
    const frame = frames[v.link];
    if (!frame) continue;
    const known = knownLinks?.[v.link] ?? true;
    v.mesh.matrix
      .copy(frame instanceof THREE.Matrix4 ? frame : matrix(frame))
      .multiply(v.origin)
      .scale(v.scale);
    v.mesh.matrixWorldNeedsUpdate = true;
    v.mesh.material.transparent = !known;
    v.mesh.material.opacity = known ? 1 : 0.45;
  }
}
let latestState = null,
  robot = null,
  run = null,
  selected = null,
  requestVersion = 0,
  playing = false,
  playhead = 0,
  sceneKey = "",
  lastFrame = 0;
let trail = null,
  fullTrail = null,
  tipMarker = null,
  trajectoryCounts = [];
function liveDisplay(state) {
  if (state.revision === revision) return;
  const first = revision === null;
  revision = state.revision;
  if (state.generic) showGenericRobot(state);
  else {
    robotMode = "legacy";
    for (const visual of robotVisuals) visual.mesh.visible = !visual.generic;
    $("robot-name").textContent = robot?.name || "Robot";
  }
  $("pose-status").textContent = state.pose_known
    ? "Latest recorded pose"
    : state.body_pose_known
      ? "Body pose recorded"
      : "Pose unavailable";
  pose(state.frames, state.known_links);
  showObjects(state.objects);
  setPlane(state.plane, state.plane_object);
  $("view-title").textContent = state.title || "Current scene";
  $("run-status").textContent = "Saved state";
  $("sample-badge").hidden = !state.example;
  if (!state.pose_known && state.generic) $("pose-status").textContent = "Latest saved object estimates";
  $("paths-toggle").disabled = true;
  if (first && state.generic) fitView(!state.pose_known);
}
function framesFor(values) {
  const q = Object.fromEntries(run.joints.map((name, i) => [name, values[i]]));
  const frames = { [robot.root]: new THREE.Matrix4() };
  for (const joint of robot.ordered) {
    const motion = new THREE.Matrix4();
    if (joint.type === "revolute" || joint.type === "continuous")
      motion.makeRotationAxis(joint.axisVector, q[joint.name] || 0);
    else if (joint.type === "prismatic")
      motion.makeTranslation(
        ...joint.axis.map((v) => v * (q[joint.name] || 0)),
      );
    frames[joint.child] = frames[joint.parent]
      .clone()
      .multiply(joint.originMatrix)
      .multiply(motion);
  }
  return frames;
}
function tip(frames) {
  return new THREE.Vector3().setFromMatrixPosition(
    frames[run.tip_link] || frames.gripper_link,
  );
}
function removeTrajectory() {
  for (const obj of [trail, fullTrail, tipMarker]) {
    if (obj) {
      scene.remove(obj);
      obj.traverse(child => { child.geometry?.dispose(); child.material?.dispose(); });
    }
  }
  trail = fullTrail = tipMarker = null;
  trajectoryCounts = [];
}
function makeTrajectory() {
  removeTrajectory();
  const samples = run.generic ? run.frames : run.samples.map(s => ({
    time: s[0], tips: {tool: tip(framesFor(s.slice(1))).toArray()},
  }));
  const names = [...new Set(samples.flatMap(s => Object.keys(s.tips)))];
  const palette = ["#1670ad", "#c34c29", "#7956a5", "#247661"];
  const colors = new Map(names.map((name, i) => [name, new THREE.Color(palette[i % palette.length])]));
  const positions = [], vertexColors = [];
  trajectoryCounts = [0];
  for (let i = 1; i < samples.length; i++) {
    for (const [name, point] of Object.entries(samples[i].tips)) {
      const previous = samples[i - 1].tips[name];
      if (previous && samples[i].time - samples[i - 1].time < 0.25) {
        positions.push(...previous, ...point);
        vertexColors.push(...colors.get(name).toArray(), ...colors.get(name).toArray());
      }
    }
    trajectoryCounts.push(positions.length / 6);
  }
  const line = (width, opacity) => {
    const geometry = new LineSegmentsGeometry();
    if (positions.length) {
      geometry.setPositions(positions);
      geometry.setColors(vertexColors);
    } else geometry.instanceCount = 0;
    const path = new LineSegments2(geometry, new LineMaterial({
      vertexColors: true, linewidth: width, transparent: true, opacity,
      depthTest: false, depthWrite: false,
    }));
    path.frustumCulled = false;
    path.renderOrder = 4;
    scene.add(path);
    return path;
  };
  fullTrail = line(2.5, 0.27);
  trail = line(4, 0.95);
  trail.renderOrder = 5;
  tipMarker = new THREE.Group();
  scene.add(tipMarker);
  $("trajectory-keys").replaceChildren();
  for (const name of names) {
    const marker = mesh(new THREE.SphereGeometry(0.007, 16, 12),
      new THREE.MeshBasicMaterial({color: colors.get(name), depthTest: false}), tipMarker);
    marker.name = name;
    marker.renderOrder = 6;
    const label = document.createElement("span"), swatch = document.createElement("i");
    swatch.style.background = colors.get(name).getStyle();
    label.append(swatch, document.createTextNode(englishName(name)));
    $("trajectory-keys").append(label);
  }
}
function drawTrajectory(index, tips) {
  trail.geometry.instanceCount = trajectoryCounts[index];
  for (const marker of tipMarker.children) {
    const point = tips[marker.name];
    marker.visible = !!point;
    if (point) marker.position.fromArray(point);
  }
}
function sampleAt(t) {
  const samples = run.samples;
  let low = 0,
    high = samples.length - 1;
  while (low < high) {
    const mid = Math.ceil((low + high) / 2);
    if (samples[mid][0] <= t) low = mid;
    else high = mid - 1;
  }
  const a = samples[low],
    b = samples[Math.min(low + 1, samples.length - 1)],
    dt = b[0] - a[0];
  // Interpolate nearby telemetry only; preserve stationary gaps between commands.
  const f =
    dt > 0 && dt < 0.25 ? THREE.MathUtils.clamp((t - a[0]) / dt, 0, 1) : 0;
  return { index: low, q: a.slice(1).map((v, i) => v + (b[i + 1] - v) * f) };
}
function sceneAt(t, frames) {
  let objects = [],
    attachment = null,
    key = "";
  for (let i = 0; i < run.scene_states.length; i++) {
    const state = run.scene_states[i];
    if (state.time > t) break;
    if (state.objects) {
      objects = state.objects;
      key += "o" + i;
    }
    if (state.attachment) {
      attachment = state.attachment;
      key += "a" + i;
    }
  }
  if (key !== sceneKey) {
    showObjects(
      objects.map((o) => ({
        ...o,
        relation:
          o.shape === "box"
            ? attachment === "attached"
              ? "confirmed"
              : attachment === "released"
                ? "in_cup_confirmed"
                : o.relation
            : o.relation,
      })),
    );
    sceneKey = key;
  }
  const block = objectMeshes.get("Yellow block");
  if (!block) return;
  if (attachment === "attached") {
    const tool = frames[run.tip_link] || frames.gripper_link;
    block.position.fromArray(run.attachment_local).applyMatrix4(tool);
    block.quaternion.setFromRotationMatrix(tool);
  } else if (attachment === "released") {
    const cup = objects.find((o) => o.shape === "cup");
    if (cup) {
      block.position.set(
        cup.position[0],
        cup.position[1],
        cup.position[2] - cup.size[2] / 2 + 0.019,
      );
      block.quaternion.identity();
    }
  }
  const location = objectCards.get("Yellow block");
  if (location)
    location.textContent =
      block.position
        .toArray()
        .map((x) => (x * 1000).toFixed(0))
        .join(" / ") + " mm";
}
const clockText = (s) =>
  `${Math.floor(s / 60)}:${Math.floor(s % 60)
    .toString()
    .padStart(2, "0")}`;
function drawReplay() {
  if (!run) return;
  if (run.generic) {
    let low = 0, high = run.frames.length - 1;
    while (low < high) {
      const mid = Math.ceil((low + high) / 2);
      if (run.frames[mid].time <= playhead) low = mid;
      else high = mid - 1;
    }
    const sample = run.frames[low];
    if (run.scenes) {
      let snapshot = run.initial, snapshotKey = "initial";
      for (let i = 0; i < run.scenes.length; i++) {
        if (run.scenes[i].time > playhead) break;
        snapshot = run.scenes[i].state;
        snapshotKey = String(i);
      }
      if (snapshotKey !== sceneKey) {
        showObjects(snapshot.objects);
        setPlane(snapshot.plane, snapshot.plane_object);
        sceneKey = snapshotKey;
      }
    }
    if (sample.source) $("pose-status").textContent = sample.source === "observed" ? "Measured checkpoint" : "Commanded joint motion";
    pose(sample.frames);
    for (const [name, state] of Object.entries(sample.objects)) {
      if (objectData.has(name)) Object.assign(objectData.get(name), state);
      const obj = objectMeshes.get(name);
      if (obj) setObjectPose(obj, state);
      const label = objectCards.get(name);
      if (label && state.position) label.textContent = state.position.map(v => (v * 1000).toFixed(0)).join(" / ") + " mm";
    }
    updateSelection();
    drawTrajectory(low, sample.tips);
  } else {
    const { index, q } = sampleAt(playhead), frames = framesFor(q);
    pose(frames);
    sceneAt(playhead, frames);
    drawTrajectory(index, {tool: tip(frames).toArray()});
  }
  $("scrubber").value = playhead;
  $("timecode").textContent =
    `${clockText(playhead)} / ${clockText(run.summary.duration)}`;
  const current = run.actions.find(
      (a) => a.start <= playhead && playhead <= a.end,
    ),
    previous = run.actions.filter((a) => a.end < playhead).at(-1);
  $("action-name").textContent =
    current?.name ||
    (playhead >= run.summary.duration
      ? "End of recording"
      : previous
        ? `Review · ${previous.name}`
        : "Initial observation");
}
function setPlaying(value) {
  playing = value;
  $("play").innerHTML = value ? "Ⅱ <span>Pause</span>" : "▶ <span>Play</span>";
  $("play").setAttribute("aria-label", value ? "Pause replay" : "Play replay");
}
function selectButtons() {
  document.querySelectorAll(".run-item").forEach((b) => {
    b.classList.toggle("active", b.dataset.id === selected);
    b.setAttribute(
      "aria-current",
      b.dataset.id === selected ? "true" : "false",
    );
  });
  $("live").classList.toggle("active", selected === null);
}
async function selectRun(id) {
  const version = ++requestVersion;
  selected = id;
  run = null;
  setPlaying(false);
  selectButtons();
  $("message").hidden = true;
  $("pose-status").textContent = "Loading recording…";
  $("playback").hidden = true;
  removeTrajectory();
  try {
    const data = await json("/api/runs/" + encodeURIComponent(id));
    if (version !== requestVersion) return;
    if (!data.generic) await loadLegacyRobot();
    if (version !== requestVersion) return;
    run = data;
    if (run.generic) {
      showGenericRobot(run.initial);
      showObjects(run.initial.objects);
      pose(run.initial.frames);
      setPlane(run.initial.plane, run.initial.plane_object);
      fitView(!run.initial.pose_known);
    } else {
      robotMode = "legacy";
      for (const visual of robotVisuals) visual.mesh.visible = !visual.generic;
      $("robot-name").textContent = robot.name;
    }
    playhead = 0;
    sceneKey = "";
    $("view-mode").textContent = "RUN REPLAY";
    $("view-title").textContent = run.summary.title;
    $("run-status").textContent = run.summary.status;
    $("pose-status").textContent = run.summary.status === "Preview" ? "Simulated motion" : "Recorded joint motion";
    $("trajectory-source").textContent = run.trajectory_source || (run.summary.status === "Preview" ? "Simulation" : "Recorded joints");
    $("trajectory-legend").hidden = false;
    $("paths-toggle").disabled = false;
    $("playback").hidden = false;
    $("run-detail").textContent =
      `${run.summary.actions} recorded actions · ${run.checkpoints.length} checkpoints`;
    $("scrubber").max = run.summary.duration;
    $("checkpoints").replaceChildren();
    for (const checkpoint of run.checkpoints) {
      const b = document.createElement("button");
      b.className = "checkpoint";
      b.style.left = `${(100 * checkpoint.time) / run.summary.duration}%`;
      b.title = `${checkpoint.name} · ${clockText(checkpoint.time)}`;
      b.setAttribute("aria-label", b.title);
      b.onclick = () => {
        playhead = checkpoint.time;
        drawReplay();
      };
      $("checkpoints").append(b);
    }
    setPlane(run.generic ? run.initial.plane : run.plane, run.initial?.plane_object);
    makeTrajectory();
    for (const obj of [trail, fullTrail, tipMarker]) if (obj) obj.visible = pathsVisible;
    drawReplay();
  } catch (error) {
    if (version !== requestVersion) return;
    $("pose-status").textContent = "Recording unavailable";
    $("message").textContent =
      "Could not load this recording. Select another run or return to the live environment.";
    $("message").hidden = false;
    console.error(error);
  }
}
$("live").onclick = () => {
  requestVersion++;
  selected = null;
  run = null;
  revision = null;
  setPlaying(false);
  removeTrajectory();
  selectButtons();
  $("view-mode").textContent = "WORKSPACE";
  $("view-title").textContent = "Current scene";
  $("run-status").textContent = "Saved state";
  $("trajectory-legend").hidden = true;
  $("playback").hidden = true;
  $("message").hidden = true;
  if (latestState) liveDisplay(latestState);
};
$("play").onclick = () => {
  if (!run) return;
  if (playhead >= run.summary.duration) playhead = 0;
  setPlaying(!playing);
  drawReplay();
};
$("restart").onclick = () => {
  playhead = 0;
  setPlaying(false);
  drawReplay();
};
$("scrubber").oninput = (e) => {
  playhead = Number(e.target.value);
  if (run && Math.abs(playhead - run.summary.duration) < 0.001)
    playhead = run.summary.duration;
  drawReplay();
};
async function json(url) {
  const response = await fetch(url, {
    cache: "no-store",
    signal: AbortSignal.timeout(20000),
  });
  if (!response.ok) throw Error(`Request failed: ${response.status}`);
  return response.json();
}
async function poll() {
  try {
    const update = await json("/api/state?since=" + encodeURIComponent(latestState?.revision || ""));
    if (!update.unchanged) latestState = update;
    $("connection").textContent = "Scene file connected";
    $("connection").style.color = "#526b59";
    $("updated").textContent =
      "Updated " +
      new Date(latestState.updated_at).toLocaleTimeString("en-US", {
        hour: "2-digit",
        minute: "2-digit",
      });
    if (selected === null) liveDisplay(latestState);
  } catch (e) {
    $("connection").textContent = "Scene file unavailable";
    $("connection").style.color = "#b65339";
  } finally {
    setTimeout(poll, 1500);
  }
}
let historySignature = "";
async function loadHistory() {
  try {
    const runs = await json("/api/runs"),
      signature = JSON.stringify(runs);
    if (signature !== historySignature) {
      historySignature = signature;
      $("runs").replaceChildren();
      $("run-count").textContent = runs.length;
      let previousDate = "";
      for (const item of runs) {
        const date = new Date(item.started_at * 1000),
          label = date.toLocaleDateString("en-US", {
            month: "short",
            day: "numeric",
            year: "numeric",
          });
        if (label !== previousDate) {
          const heading = document.createElement("div");
          heading.className = "date-heading";
          heading.textContent = label;
          $("runs").append(heading);
          previousDate = label;
        }
        const b = document.createElement("button");
        b.className = "run-item";
        b.dataset.id = item.id;
        b.title = item.id;
        b.onclick = () => selectRun(item.id);
        const name = document.createElement("span");
        name.className = "run-title";
        name.textContent = item.title;
        const meta = document.createElement("span");
        meta.className = "run-meta";
        meta.textContent = `${date.toLocaleTimeString("en-US", { hour: "2-digit", minute: "2-digit", hour12: false })} · ${clockText(item.duration)}`;
        const status = document.createElement("span");
        status.className = "status";
        status.textContent = item.status;
        meta.append(status);
        b.append(name, meta);
        $("runs").append(b);
      }
      if (!runs.length) {
        const p = document.createElement("p");
        p.className = "empty";
        p.textContent = "No recorded runs yet";
        $("runs").append(p);
      }
      selectButtons();
    }
  } catch (e) {
    if (!historySignature) $("runs").textContent = "Run history unavailable";
  } finally {
    setTimeout(loadHistory, 15000);
  }
}
function resize() {
  const w = $("viewport").clientWidth,
    h = $("viewport").clientHeight;
  renderer.setSize(w, h);
  camera.aspect = w / h;
  camera.zoom = Math.min(1, camera.aspect / 1.1);
  camera.updateProjectionMatrix();
}
new ResizeObserver(resize).observe($("viewport"));
function render(now) {
  requestAnimationFrame(render);
  if (playing && run) {
    playhead = Math.min(
      run.summary.duration,
      playhead +
        Math.min((now - lastFrame) / 1000, 0.2) * Number($("speed").value),
    );
    drawReplay();
    if (playhead >= run.summary.duration) setPlaying(false);
  }
  lastFrame = now;
  controls.update();
  renderer.render(scene, camera);
}
requestAnimationFrame(render);
async function loadLegacyRobot() {
    if (robot) return;
    robot = await json("/api/legacy-robot");
    robot.ordered = [];
    let pending = [...robot.joints],
      known = new Set([robot.root]);
    while (pending.length) {
      const ready = pending.filter((j) => known.has(j.parent));
      if (!ready.length) throw Error("Invalid robot hierarchy");
      for (const j of ready) {
        j.originMatrix = matrix(j.origin);
        j.axisVector = new THREE.Vector3(...j.axis);
        robot.ordered.push(j);
        known.add(j.child);
      }
      pending = pending.filter((j) => !ready.includes(j));
    }
    const loader = new STLLoader(),
      cache = new Map();
    await Promise.all(
      robot.visuals.map(async (v) => {
        if (!cache.has(v.mesh))
          cache.set(
            v.mesh,
            loader.loadAsync(
              "/robot/" + v.mesh.replace(/^package:\/\/[^/]+\//, ""),
            ),
          );
        const geometry = await cache.get(v.mesh);
        const motor = /sts3215|waveshare/.test(v.mesh);
        const obj = mesh(geometry, material(motor ? "#303e3d" : "#d48251"));
        obj.matrixAutoUpdate = false;
        robotVisuals.push({
          mesh: obj,
          link: v.link,
          origin: matrix(v.origin),
          scale: new THREE.Vector3(...v.scale),
        });
      }),
    );
}
async function init() {
  try {
    const description = await json("/api/robot");
    if (!description.generic) await loadLegacyRobot();
    resize();
    $("loading").remove();
    poll();
    loadHistory();
  } catch (e) {
    $("loading").textContent = "Scene could not load. Refresh to retry.";
    console.error(e);
  }
}
init();
