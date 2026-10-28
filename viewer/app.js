// Sirb viewer: static, no build step. Positions are int16 centimetres (frames, n, 3), x east, y north, z up.
// Three.js frame: (x, z, -y) so that north points away from the default camera.
import * as THREE from "three";
import { OrbitControls } from "three/addons/OrbitControls.js";
import { STRINGS, CERT_AR, getLang, storeLang, applyDocLang, makeT, makeNum, makeInt, translateDom } from "./i18n.js";

const $ = (s, r = document) => r.querySelector(s);
const el = (tag, attrs = {}, ...kids) => {
  const e = document.createElement(tag);
  for (const [k, v] of Object.entries(attrs)) {
    if (k === "class") e.className = v; else if (k === "text") e.textContent = v; else e.setAttribute(k, v);
  }
  for (const k of kids) if (k != null) e.append(k);
  return e;
};

// ------------------------------------------------------------------ state
const S = {
  lang: getLang(), t: null, num: null, int: null,
  theme: "dark", index: null, show: null, cert: null,
  scenId: "wind_gnss", mode: "height", playing: false, speed: 4, time: 0, T: 0, dt: 0.2,
  cam: "front", tour: 0, fc: { on: false, step: 1 }, classic: false, frame: null,
  A: null, B: null, dataA: null, dataB: null, loadToken: 0, dirty: true,
};
const SPEEDS = [0.5, 1, 2, 4];
const VIEW_KEY = "sirb.viewer.view";
// ?bench=1 (used by scripts/viewer_bench.py): reads the dataset from ./bench/ instead of ./data/ and records
// per-frame timings in window.__bench. It never changes what the normal page does.
const BENCH = new URLSearchParams(location.search).get("bench") === "1";
const DATA = BENCH ? "bench" : "data";
const bench = BENCH ? { raf: [], update: [], render: [], total: [] } : null;
if (BENCH) window.__bench = bench;
const reducedMotion = matchMedia("(prefers-reduced-motion: reduce)").matches;

// ------------------------------------------------------------------ theme
function initialTheme() {
  try { const v = localStorage.getItem("sirb.viewer.theme"); if (v === "dark" || v === "light") return v; } catch (e) { /* ignore */ }
  return matchMedia("(prefers-color-scheme: light)").matches ? "light" : "dark";
}
function applyTheme(theme, store) {
  S.theme = theme;
  document.documentElement.dataset.theme = theme;
  if (store) { try { localStorage.setItem("sirb.viewer.theme", theme); } catch (e) { /* ignore */ } }
  $("#themeBtn").textContent = theme === "dark" ? S.t("theme_light") : S.t("theme_dark");
  for (const v of [S.A, S.B]) v && v.rebuildStatic();
  S.dirty = true;
}
matchMedia("(prefers-color-scheme: light)").addEventListener?.("change", (e) => {
  let stored = null;
  try { stored = localStorage.getItem("sirb.viewer.theme"); } catch (err) { /* ignore */ }
  if (!stored) applyTheme(e.matches ? "light" : "dark", false);
});

// ------------------------------------------------------------------ colours
const HEIGHT_STOPS = {
  dark: [[0x7b, 0x4b, 0xff], [0x2f, 0x7b, 0xff], [0x19, 0xc4, 0xe8], [0xc8, 0xf6, 0xff]],
  light: [[0x5b, 0x2f, 0xd0], [0x1f, 0x5f, 0xe0], [0x0a, 0x9f, 0xc4], [0x0b, 0x6b, 0x80]],
};
const SEP = { red: [1.0, 0.27, 0.27], amber: [1.0, 0.69, 0.13], green: [0.2, 0.78, 0.35] };
function heightColor(h, out, o) {
  const st = HEIGHT_STOPS[S.theme]; const x = Math.min(Math.max(h, 0), 1) * (st.length - 1);
  const i = Math.min(Math.floor(x), st.length - 2), f = x - i;
  out[o] = (st[i][0] + f * (st[i + 1][0] - st[i][0])) / 255;
  out[o + 1] = (st[i][1] + f * (st[i + 1][1] - st[i][1])) / 255;
  out[o + 2] = (st[i][2] + f * (st[i + 1][2] - st[i][2])) / 255;
}
const KIND_COLOR = { motor_failure: 0xff2bd6, comms_loss: 0xffffff, other: 0x9d7bff };

// ------------------------------------------------------------------ data
const cache = new Map();
async function fetchBuf(path) {
  if (!cache.has(path)) {
    cache.set(path, fetch(path).then(async (r) => {
      if (!r.ok) throw new Error(`${path}: HTTP ${r.status}`);
      return r.arrayBuffer();
    }).catch((e) => { cache.delete(path); throw e; }));
  }
  return cache.get(path);
}
async function fetchJSON(path) {
  const r = await fetch(path);
  if (!r.ok) throw new Error(`${path}: HTTP ${r.status}`);
  return r.json();
}
async function loadScenarioData(show, sc) {
  const [pb, nb] = await Promise.all([fetchBuf(`${DATA}/${show.id}/${sc.pos}`), fetchBuf(`${DATA}/${show.id}/${sc.nn}`)]);
  const pos = new Int16Array(pb), nn = new Uint16Array(nb);
  const n = show.n, K = show.frames;
  if (pos.length !== K * n * 3 || nn.length !== K * n) throw new Error(`${sc.pos}: unexpected size`);
  const scl = S.index.pos_scale, dmin = show.d_min_m;
  const prefMin = new Float32Array(K), frameMin = new Float32Array(K), ever = new Uint8Array(n);
  let run = Infinity, maxZ = 0, kMin = 0, best = Infinity;
  for (let k = 0; k < K; k++) {
    let m = Infinity;
    for (let i = 0; i < n; i++) {
      const v = nn[k * n + i]; if (v < m) m = v;
      if (v * scl < dmin) ever[i] = 1;
      const z = pos[(k * n + i) * 3 + 2]; if (z > maxZ) maxZ = z;
    }
    if (m < best) { best = m; kMin = k; }
    run = Math.min(run, m); prefMin[k] = run * scl; frameMin[k] = m * scl;
  }
  let everBelow = 0; for (let i = 0; i < n; i++) everBelow += ever[i];
  const tight = { k: kMin, t: kMin * S.index.dt, d: best * scl };
  return { pos, nn, K, n, prefMin, frameMin, tight, everBelow, maxZ: maxZ * scl, sc };
}

// ------------------------------------------------------------------ 3D view
const V1 = new THREE.Vector3(), V2 = new THREE.Vector3(), V3 = new THREE.Vector3(), V4 = new THREE.Vector3(), UP = new THREE.Vector3(0, 1, 0);
class View {
  constructor(root, which) {
    this.which = which; this.root = root;
    this.canvas = $("canvas", root); this.overlay = $(".overlay", root);
    this.renderer = new THREE.WebGLRenderer({ canvas: this.canvas, antialias: true, powerPreference: "high-performance" });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    this.scene = new THREE.Scene();
    this.camera = new THREE.PerspectiveCamera(45, 1.6, 1, 4000);
    this.controls = new OrbitControls(this.camera, this.canvas);
    this.controls.enableDamping = false;
    this.controls.maxPolarAngle = Math.PI * 0.499;
    this.controls.listenToKeyEvents(this.canvas);
    this.controls.addEventListener("change", () => { S.dirty = true; if (!View.syncing) View.sync(this); });
    this.statics = new THREE.Group(); this.scene.add(this.statics);
    this.mesh = null; this.markers = null; this.events = [];
    this.data = null; this.show = null;
    // closest pair: a thin bar between the two drones, a wire halo round each and a distance label
    this.label = $(".pair-label", root); this.pair = null; this.pairMid = new THREE.Vector3(); this.tx = this.ty = this.tz = null;
    this.pairMat = new THREE.MeshBasicMaterial({ color: 0xffffff });
    this.haloMat = new THREE.MeshBasicMaterial({ color: 0xffffff, wireframe: true, transparent: true, opacity: 0.85 });
    this.pairMesh = new THREE.Mesh(new THREE.CylinderGeometry(0.28, 0.28, 1, 10), this.pairMat);
    this.halos = [0, 1].map(() => new THREE.Mesh(new THREE.SphereGeometry(2.1, 14, 8), this.haloMat));
    for (const o of [this.pairMesh, ...this.halos]) { o.frustumCulled = false; o.visible = false; this.scene.add(o); }
    this.controls.addEventListener("start", () => { if (!View.syncing) { S.cam = null; refreshCam(); } });
    this.ro = new ResizeObserver(() => this.resize()); this.ro.observe($(".canvas-wrap", root));
  }
  static sync(src) {
    const dst = src === S.A ? S.B : S.A; if (!dst) return;
    View.syncing = true;
    dst.camera.position.copy(src.camera.position);
    dst.controls.target.copy(src.controls.target);
    dst.camera.zoom = src.camera.zoom; dst.camera.updateProjectionMatrix();
    dst.controls.update();
    View.syncing = false; S.dirty = true;
  }
  resize() {
    const w = this.canvas.clientWidth, h = this.canvas.clientHeight; if (!w || !h) return;
    this.renderer.setSize(w, h, false); this.camera.aspect = w / h; this.camera.updateProjectionMatrix(); S.dirty = true;
  }
  setShow(show, cert) {
    this.show = show; this.cert = cert; this.zTop = cert?.limits?.z_max ?? 120;
    this.tx = new Float32Array(show.n); this.ty = new Float32Array(show.n); this.tz = new Float32Array(show.n); this.pair = null;
    // instanced drones
    if (this.mesh) { this.scene.remove(this.mesh); this.mesh.geometry.dispose(); this.mesh.material.dispose(); }
    const geo = new THREE.IcosahedronGeometry(0.9, 1);
    this.mesh = new THREE.InstancedMesh(geo, new THREE.MeshBasicMaterial(), show.n);
    this.mesh.frustumCulled = false;
    this.mesh.instanceMatrix.setUsage(THREE.DynamicDrawUsage);
    this.mesh.setColorAt(0, new THREE.Color(1, 1, 1));
    this.mesh.instanceColor.setUsage(THREE.DynamicDrawUsage);
    this.scene.add(this.mesh);
    this.rebuildStatic();
    const g = show.geofence, W = Math.max(g.x[1] - g.x[0], g.y[1] - g.y[0]);
    const cx = (g.x[0] + g.x[1]) / 2, cy = (g.y[0] + g.y[1]) / 2;
    this.home = { target: new THREE.Vector3(cx, 35, -cy), pos: new THREE.Vector3(cx + 0.25 * W, 0.5 * W, -cy + 0.95 * W) };
    this.resetCamera();
  }
  // Camera presets, framed on the extent of the formations (S.frame, metres: x east, y north, z up).
  setPreset(kind, f) {
    const tv = Math.tan(THREE.MathUtils.degToRad(this.camera.fov) / 2), th = tv * this.camera.aspect;
    const W = Math.max(f.x1 - f.x0, 1), D = Math.max(f.y1 - f.y0, 1), H = Math.max(f.z1 - f.z0, 1), pad = 1.2;
    const target = new THREE.Vector3((f.x0 + f.x1) / 2, (f.z0 + f.z1) / 2, -(f.y0 + f.y1) / 2);
    let dir, dist;
    if (kind === "side") { dir = new THREE.Vector3(1, 0.06, 0); dist = Math.max(H / 2 / tv, D / 2 / th) * pad + W / 2; }
    else if (kind === "top") { dir = new THREE.Vector3(0, 1, 0.001); dist = Math.max(D / 2 / tv, W / 2 / th) * pad + H / 2; }
    else if (kind === "oblique") { dir = new THREE.Vector3(0.45, 0.5, 0.95); dist = (Math.max(W, D, H) / 2 / Math.min(tv, th)) * pad; }
    else { dir = new THREE.Vector3(0, 0.06, 1); dist = Math.max(H / 2 / tv, W / 2 / th) * pad + D / 2; }
    this.camera.position.copy(target).addScaledVector(dir.normalize(), dist);
    this.controls.target.copy(target); this.controls.update(); S.dirty = true;
  }
  resetCamera() {
    this.camera.position.copy(this.home.pos); this.controls.target.copy(this.home.target); this.controls.update();
  }
  rebuildStatic() {
    if (!this.show) return;
    const dark = S.theme === "dark", cs = getComputedStyle(document.documentElement);
    this.scene.background = new THREE.Color(cs.getPropertyValue("--canvas").trim() || "#0a0e14");
    for (const c of [...this.statics.children]) { this.statics.remove(c); c.geometry?.dispose(); c.material?.dispose(); }
    const g = this.show.geofence, crowd = this.show.crowd, zt = this.zTop;
    const P = (x, y, z) => new THREE.Vector3(x, z, -y);
    const add = (o) => { this.statics.add(o); return o; };
    const cx = (g.x[0] + g.x[1]) / 2;
    // ground grid (10 m cells) spanning geofence and crowd region
    const yMin = Math.min(g.y[0], crowd.y - 60), yMax = g.y[1] + 40, xHalf = 220;
    const size = Math.max(2 * xHalf, yMax - yMin);
    const grid = new THREE.GridHelper(size, Math.round(size / 10), dark ? 0x3a4a5e : 0x8fa0b5, dark ? 0x222d3b : 0xc3ceda);
    grid.position.set(cx, -0.05, -(yMin + yMax) / 2); add(grid);
    // crowd region (y <= crowd.y)
    const cw = 2 * xHalf, ch = crowd.y - yMin;
    if (ch > 0) {
      const m = new THREE.Mesh(new THREE.PlaneGeometry(cw, ch), new THREE.MeshBasicMaterial({ color: dark ? 0x7a1f2b : 0xe08a95, transparent: true, opacity: 0.35, side: THREE.DoubleSide }));
      m.rotation.x = -Math.PI / 2; m.position.set(cx, 0, -(crowd.y - ch / 2)); add(m);
    }
    // geofence outline on ground and at ceiling
    for (const z of [0, zt]) {
      const pts = [P(g.x[0], g.y[0], z), P(g.x[1], g.y[0], z), P(g.x[1], g.y[1], z), P(g.x[0], g.y[1], z)];
      add(new THREE.LineLoop(new THREE.BufferGeometry().setFromPoints(pts), new THREE.LineBasicMaterial({ color: dark ? 0x7cc0ff : 0x0b5cc5 })));
    }
    const corners = [];
    for (const [x, y] of [[g.x[0], g.y[0]], [g.x[1], g.y[0]], [g.x[1], g.y[1]], [g.x[0], g.y[1]]]) corners.push(P(x, y, 0), P(x, y, zt));
    add(new THREE.LineSegments(new THREE.BufferGeometry().setFromPoints(corners), new THREE.LineBasicMaterial({ color: dark ? 0x3d6a99 : 0x6d97c8 })));
    // crowd line (solid) and buffer line (dashed)
    const xa = cx - xHalf, xb = cx + xHalf;
    add(new THREE.Line(new THREE.BufferGeometry().setFromPoints([P(xa, crowd.y, 0.1), P(xb, crowd.y, 0.1)]), new THREE.LineBasicMaterial({ color: dark ? 0xff5c6e : 0xc4001a })));
    const bl = new THREE.Line(new THREE.BufferGeometry().setFromPoints([P(xa, crowd.y + crowd.min_distance_m, 0.1), P(xb, crowd.y + crowd.min_distance_m, 0.1)]),
      new THREE.LineDashedMaterial({ color: dark ? 0xffb454 : 0xa35a00, dashSize: 6, gapSize: 4 }));
    bl.computeLineDistances(); add(bl);
    S.dirty = true;
  }
  setData(data, sc) {
    this.data = data; this.sc = sc;
    if (this.markers) { this.scene.remove(this.markers); this.markers.geometry.dispose(); this.markers.material.dispose(); this.markers = null; }
    // one marker per event drone (first event per drone wins)
    const seen = new Set(); this.events = [];
    for (const e of (sc?.events || []).slice().sort((a, b) => a.t - b.t)) if (!seen.has(e.drone)) { seen.add(e.drone); this.events.push(e); }
    if (this.events.length) {
      this.markers = new THREE.InstancedMesh(new THREE.OctahedronGeometry(1, 0), new THREE.MeshBasicMaterial({ wireframe: true }), this.events.length);
      this.markers.frustumCulled = false;
      this.markers.instanceMatrix.setUsage(THREE.DynamicDrawUsage);
      this.events.forEach((e, i) => this.markers.setColorAt(i, new THREE.Color(KIND_COLOR[e.kind] ?? KIND_COLOR.other)));
      this.scene.add(this.markers);
    }
    S.dirty = true;
  }
  // Write instance matrices / colours for time t and return live readouts.
  update(t) {
    const d = this.data; if (!d || !this.mesh) return null;
    const { pos, nn, K, n } = d, dt = S.dt, sc = S.index.pos_scale, dmin = this.show.d_min_m;
    const f = Math.min(Math.max(t / dt, 0), K - 1);
    const i0 = Math.min(Math.floor(f), K - 1), i1 = Math.min(i0 + 1, K - 1), a = f - i0;
    const mA = this.mesh.instanceMatrix.array, cA = this.mesh.instanceColor.array;
    const sepMode = S.mode === "sep", zmax = Math.max(d.maxZ, 1);
    const tx = this.tx, ty = this.ty, tz = this.tz;
    let below = 0, iMin = 0, nvMin = Infinity;
    const ev = this.events.length ? new Map(this.events.map((e, i) => [e.drone, i])) : null;
    const mk = this.markers ? this.markers.instanceMatrix.array : null;
    const o0 = i0 * n, o1 = i1 * n;
    for (let i = 0; i < n; i++) {
      const p0 = (o0 + i) * 3, p1 = (o1 + i) * 3;
      const x = (pos[p0] + a * (pos[p1] - pos[p0])) * sc;
      const y = (pos[p0 + 1] + a * (pos[p1 + 1] - pos[p0 + 1])) * sc;
      const z = (pos[p0 + 2] + a * (pos[p1 + 2] - pos[p0 + 2])) * sc;
      const nv = (nn[o0 + i] + a * (nn[o1 + i] - nn[o0 + i])) * sc;
      tx[i] = x; ty[i] = z; tz[i] = -y;
      if (nv < nvMin) { nvMin = nv; iMin = i; }
      const isBelow = nv < dmin; if (isBelow) below++;
      const s = sepMode && isBelow ? 1.6 : 1, m = i * 16;
      mA[m] = s; mA[m + 1] = 0; mA[m + 2] = 0; mA[m + 3] = 0; mA[m + 4] = 0; mA[m + 5] = s; mA[m + 6] = 0; mA[m + 7] = 0;
      mA[m + 8] = 0; mA[m + 9] = 0; mA[m + 10] = s; mA[m + 11] = 0; mA[m + 12] = x; mA[m + 13] = z; mA[m + 14] = -y; mA[m + 15] = 1;
      const c = i * 3;
      if (sepMode) {
        const col = nv < dmin ? SEP.red : nv < 1.25 * dmin ? SEP.amber : SEP.green;
        cA[c] = col[0]; cA[c + 1] = col[1]; cA[c + 2] = col[2];
      } else heightColor(z / zmax, cA, c);
      if (ev) {
        const mi = ev.get(i);
        if (mi !== undefined) {
          const e = this.events[mi], on = t >= e.t, ms = on ? (e.kind === "motor_failure" ? 3.4 : 2.6) : 0, q = mi * 16;
          mk.fill(0, q, q + 16); mk[q] = ms; mk[q + 5] = ms; mk[q + 10] = ms; mk[q + 12] = x; mk[q + 13] = z; mk[q + 14] = -y; mk[q + 15] = 1;
        }
      }
    }
    // closest pair at this instant: the drone with the smallest nearest neighbour distance and its neighbour
    let jMin = -1, best = Infinity;
    for (let j = 0; j < n; j++) {
      if (j === iMin) continue;
      const dx = tx[j] - tx[iMin], dy = ty[j] - ty[iMin], dz = tz[j] - tz[iMin], dd = dx * dx + dy * dy + dz * dz;
      if (dd < best) { best = dd; jMin = j; }
    }
    const pd = Math.sqrt(best); this.pair = { i: iMin, j: jMin, d: pd };
    for (const q of [iMin, jMin]) { const m = q * 16, s = Math.max(mA[m], 1) * 1.5; mA[m] = s; mA[m + 5] = s; mA[m + 10] = s; }
    this.placePair(pd < dmin);
    this.mesh.instanceMatrix.needsUpdate = true; this.mesh.instanceColor.needsUpdate = true;
    if (this.markers) this.markers.instanceMatrix.needsUpdate = true;
    const triggered = this.events.length ? (this.sc.events || []).filter((e) => t >= e.t).length : 0;
    return { minNow: pd, pair: this.pair, minSoFar: Math.min(d.prefMin[i0], pd), below, n, triggered, totalEvents: (this.sc?.events || []).length };
  }
  placePair(bad) {
    const { i, j } = this.pair, tx = this.tx, ty = this.ty, tz = this.tz;
    const a = V1.set(tx[i], ty[i], tz[i]), b = V2.set(tx[j], ty[j], tz[j]);
    this.pairMid.copy(a).add(b).multiplyScalar(0.5);
    V3.copy(b).sub(a); const len = V3.length();
    this.pairMesh.position.copy(this.pairMid); this.pairMesh.scale.set(1, Math.max(len, 1e-3), 1);
    if (len > 1e-6) this.pairMesh.quaternion.setFromUnitVectors(UP, V3.normalize());
    this.halos[0].position.copy(a); this.halos[1].position.copy(b);
    const col = bad ? 0xff4545 : S.theme === "dark" ? 0xffffff : 0x111d2b;
    this.pairMat.color.setHex(col); this.haloMat.color.setHex(col);
    for (const o of [this.pairMesh, ...this.halos]) o.visible = true;
  }
  placeLabel() {
    const p = this.pair, lab = this.label; if (!lab) return;
    if (!p || !this.data) { lab.hidden = true; return; }
    V4.copy(this.pairMid).project(this.camera);
    const w = this.canvas.clientWidth, h = this.canvas.clientHeight;
    const x = (V4.x * 0.5 + 0.5) * w, y = (-V4.y * 0.5 + 0.5) * h;
    lab.hidden = V4.z > 1 || x < 0 || x > w || y < 0 || y > h;
    const text = `${S.num(p.d, 2)} ${S.t("unit_m")}`;
    if (lab.textContent !== text) lab.textContent = text;
    lab.classList.toggle("bad", p.d < this.show.d_min_m);
    lab.style.left = `${x.toFixed(1)}px`; lab.style.top = `${y.toFixed(1)}px`;
  }
  render() { this.renderer.render(this.scene, this.camera); this.placeLabel(); }
}

// ------------------------------------------------------------------ DOM builders
function methodsHref(anchor) { return `methods.html?lang=${S.lang}#${anchor}`; }
function numLink(text, anchor) {
  const a = el("a", { class: "num", href: methodsHref(anchor), text });
  a.title = S.t("methods_title");
  return a;
}
function withUnit(text, anchor, unit = true) {
  const f = document.createDocumentFragment(); f.append(numLink(text, anchor));
  if (unit) f.append(" ", el("span", { class: "unit", text: S.t("unit_m") }));
  return f;
}

const RO = { A: {}, B: {} };
function buildReadouts() {
  const t = S.t;
  for (const [which, dlId, anchor] of [["A", "#roA", "sampled-nn"], ["B", "#roB", "stress-model"]]) {
    const dl = $(dlId); dl.replaceChildren(); const refs = RO[which];
    const row = (key, label, anch) => {
      const dd = el("dd"); const link = numLink("—", anch); dd.append(link);
      const unit = el("span", { class: "unit" }); dd.append(" ", unit);
      dl.append(el("div", {}, el("dt", { text: label }), dd)); refs[key] = { dd, link, unit };
    };
    row("now", t("readout_min_now"), anchor); row("sofar", t("readout_min_sofar"), anchor); row("below", t("readout_below"), anchor);
    if (which === "B") row("events", t("readout_events"), "events");
    if (which === "A") dl.title = t("sampled_note", { dt: S.num(S.dt, 1) });
  }
}
function setRO(which, r) {
  const refs = RO[which], dmin = S.show.d_min_m, num = S.num;
  if (!r) return;
  const upd = (k, text, unit, bad) => {
    const o = refs[k]; if (!o) return;
    if (o.link.textContent !== text) o.link.textContent = text;
    if (o.unit.textContent !== unit) o.unit.textContent = unit;
    o.dd.classList.toggle("is-bad", !!bad);
  };
  upd("now", num(r.minNow, 2), r.pair ? `${S.t("unit_m")} · ${S.t("pair_label", { a: S.int(r.pair.i), b: S.int(r.pair.j) })}` : S.t("unit_m"), r.minNow < dmin);
  upd("sofar", num(r.minSoFar, 2), S.t("unit_m"), r.minSoFar < dmin);
  upd("below", S.int(r.below), S.t("of_n", { k: "", n: S.int(r.n) }).replace(/^\s*/, ""), r.below > 0);
  if (which === "B") upd("events", S.int(r.triggered), S.t("of_n", { k: "", n: S.int(r.totalEvents) }).replace(/^\s*/, ""), false);
}

function buildLegend() {
  const ul = $("#legend"); ul.replaceChildren(); const t = S.t;
  const item = (cls, style, text) => ul.append(el("li", {}, el("span", { class: `sw ${cls}`, style }), el("span", { text })));
  if (S.mode === "height") {
    const st = HEIGHT_STOPS[S.theme].map((c) => `rgb(${c.join(",")})`).join(",");
    item("grad", `background:linear-gradient(90deg,${st})`, t("legend_height"));
  } else {
    item("", "background:rgb(255,69,69)", t("legend_red")); item("", "background:rgb(255,176,32)", t("legend_amber"));
    item("", "background:rgb(51,199,89)", t("legend_green")); ul.append(el("li", { text: t("legend_below_big") }));
  }
  const dark = S.theme === "dark";
  item("line", `background:${dark ? "#7cc0ff" : "#0b5cc5"}`, t("legend_geo"));
  item("line", `background:${dark ? "#ff5c6e" : "#c4001a"}`, t("legend_crowd"));
  item("dash", `border-color:${dark ? "#ffb454" : "#a35a00"}`, t("legend_buffer"));
  item("dia", "border-color:#ff2bd6", t("legend_marker_motor"));
  item("dia", `border-color:${dark ? "#fff" : "#222"}`, t("legend_marker_comms"));
  ul.append(el("li", { text: t("legend_size") }));
}

function buildSpeedButtons() {
  const g = $("#speedGroup"); g.replaceChildren();
  for (const sp of SPEEDS) {
    const b = el("button", { type: "button", class: "btn seg-btn", "aria-pressed": String(sp === S.speed), "data-speed": String(sp), text: `${S.num(sp, sp % 1 ? 1 : 0)}×` });
    b.addEventListener("click", () => { S.speed = sp; buildSpeedButtons(); });
    g.append(b);
  }
}

function fmtTime(t) { return S.num(t, 1); }
function updateTransport() {
  $("#playBtn").textContent = S.playing ? S.t("pause") : S.t("play");
  $("#playBtn").setAttribute("aria-pressed", String(S.playing));
  const out = $("#timeOut"); out.textContent = S.t("time_readout", { t: fmtTime(S.time), total: fmtTime(S.T) });
  const sc = $("#scrub"); const v = String(Math.round(S.time * 50)); if (sc.value !== v) sc.value = v;
  sc.setAttribute("aria-valuetext", out.textContent);
  updateStage();
}

function renderBanner() {
  const st = S.index?.safety_statement ?? STRINGS.en.safety_statement;
  $("#bannerText").textContent = S.lang === "ar" ? S.t("safety_statement") : st;
  const o = $("#bannerOrig"); o.hidden = S.lang !== "ar";
  o.replaceChildren(el("span", { text: S.t("banner_en_original") }), el("span", { lang: "en", dir: "ltr", class: "en-block", text: st }));
}

function renderShowMeta() {
  const s = S.show; if (!s) return; const m = $("#showMeta"); m.replaceChildren();
  $("#showBlurb").textContent = S.lang === "ar" ? s.blurb_ar : s.blurb_en;
  const tpl = S.t("show_meta", { n: "\u0001", dur: "\u0002", dmin: "\u0003" }).split(/(\u0001|\u0002|\u0003)/);
  for (const part of tpl) {
    if (part === "\u0001") m.append(S.int(s.n)); else if (part === "\u0002") m.append(S.num(s.duration_s, 1));
    else if (part === "\u0003") m.append(numLink(S.num(s.d_min_m, 2), "declared-threshold")); else m.append(part);
  }
}

function fillSelects() {
  const ss = $("#showSelect"); ss.replaceChildren();
  for (const sh of S.index.shows) ss.append(el("option", { value: sh.id, text: sh.name }));
  if (S.show) ss.value = S.show.id;
  const sc = $("#scenSelect"); sc.replaceChildren();
  if (S.show) {
    for (const s of S.show.scenarios) sc.append(el("option", { value: s.id, text: S.lang === "ar" ? s.label_ar : s.label_en }));
    sc.value = S.scenId;
  }
}

function setBlock(parent, node) { parent.replaceChildren(node); }
const kv = (dl, label, valueNode) => { dl.append(el("dt", { text: label }), el("dd", {}, valueNode)); };
const flip = (arr, key) => (S.lang === "ar" && CERT_AR[key] && CERT_AR[key].length === arr.length ? CERT_AR[key] : arr);

function renderCert() {
  const body = $("#certBody"), t = S.t, num = S.num; const c = S.cert, sh = S.show;
  if (!c) { body.textContent = t("cert_loading"); return; }
  const r = c.results, lim = c.limits, wrap = el("div");
  const pass = c.verdict === "PASS";
  const vlink = el("a", { class: `verdict ${pass ? "pass" : "fail"}`, href: methodsHref("exact-verifier"), text: `${pass ? "" : "✖ "}${t(pass ? "pass" : "fail")}` });
  wrap.append(el("div", {}, el("span", { class: "note", text: t("verdict") + " " }), vlink));
  wrap.append(el("p", { class: "assumed", text: t("verdict_note") }));
  const dl = el("dl", { class: "kv" });
  kv(dl, t("cert_min_sep"), (() => { const f = withUnit(num(r.min_separation, 3), "exact-verifier");
    if (r.min_separation_pair && r.min_separation_time != null) f.append(el("div", { class: "note", text: t("cert_min_sep_at", { t: num(r.min_separation_time, 1), a: S.int(r.min_separation_pair[0]), b: S.int(r.min_separation_pair[1]) }) }));
    return f; })());
  kv(dl, t("cert_dmin"), (() => { const f = withUnit(num(lim.min_separation_m, 2), "declared-threshold");
    if (lim.sources?.min_separation_m === "assumption") f.append(el("span", { class: "chip", text: t("cert_dmin_assumed") })); return f; })());
  kv(dl, t("cert_margin"), withUnit(num(r.min_separation - lim.min_separation_m, 3), "exact-verifier"));
  kv(dl, t("cert_recheck"), el("a", { class: "num", href: methodsHref("recomputation-check"), text: (sh.recomputation_ok ? "" : "✖ ") + t(sh.recomputation_ok ? "recheck_yes" : "recheck_no") }));
  const full = c.fleet_sha256, short = `${full.slice(0, 8)}…${full.slice(-8)}`;
  const shaWrap = el("div", {}, el("a", { class: "num", href: methodsHref("content-hash") }, el("code", { class: "hash", text: short })), " ");
  const copy = el("button", { type: "button", class: "btn ev-btn", text: t("copy") }); const stat = el("span", { class: "note", role: "status" });
  copy.addEventListener("click", async () => {
    let ok = false;
    try { await navigator.clipboard.writeText(full); ok = true; } catch (e) {
      try { const ta = el("textarea"); ta.value = full; document.body.append(ta); ta.select(); ok = document.execCommand("copy"); ta.remove(); } catch (e2) { ok = false; }
    }
    stat.textContent = t(ok ? "copied" : "copy_failed");
  });
  shaWrap.append(copy, " ", stat);
  const det = el("details", {}, el("summary", { text: t("show_full_hash") }), el("code", { class: "hash hash-full", text: full }));
  shaWrap.append(det); kv(dl, t("cert_sha"), shaWrap);
  kv(dl, t("cert_pairs"), numLink(t("cert_pairs_val", { seg: S.int(r.n_segments), pairs: S.int(r.n_pairs_checked) }), "prefilter"));
  wrap.append(dl);
  // dynamics table
  wrap.append(el("h3", { text: t("cert_dyn_title") }));
  const tb = el("table", { class: "tbl" }); const tbody = el("tbody");
  const rows = [["dyn_speed_h", r.max_speed_h, lim.vmax_h], ["dyn_speed_up", r.max_speed_up, lim.vmax_up], ["dyn_speed_down", r.max_speed_down, lim.vmax_down],
    ["dyn_accel", r.max_accel, lim.amax], ["dyn_jerk", r.max_jerk, lim.jmax], ["dyn_geofence", r.geofence_margin, null], ["dyn_crowd", r.crowd_margin, null]];
  for (const [k, v, l] of rows) {
    const td = el("td", {}); td.append(numLink(num(v, 2), "dynamics")); if (l != null) td.append(" / ", numLink(num(l, 2), "dynamics"));
    tbody.append(el("tr", {}, el("th", { scope: "row", text: t(k) }), td));
  }
  tb.append(tbody); wrap.append(tb);
  const lst = (arr, key) => { const ul = el("ul", { class: "list" }); const useAr = S.lang === "ar" && flip(arr, key) !== arr;
    flip(arr, key).forEach((x) => ul.append(el("li", { text: x, lang: useAr ? "ar" : "en", dir: useAr ? "rtl" : "ltr" }))); return ul; };
  wrap.append(el("h3", {}, el("a", { href: methodsHref("guarantees"), text: t("cert_guarantees") })), lst(c.guarantees, "guarantees"));
  wrap.append(el("h3", {}, el("a", { href: methodsHref("guarantees"), text: t("cert_not") })), lst(c.not_guaranteed, "not_guaranteed"));
  wrap.append(el("h3", { text: t("cert_statement") }), el("p", { class: "note", lang: S.lang, text: S.lang === "ar" ? CERT_AR.statement : c.statement }));
  wrap.append(el("p", { class: "note", text: t("cert_text_note") }));
  setBlock(body, wrap);
}

function renderScenario() {
  const body = $("#scenBody"), t = S.t, num = S.num, sc = S.show?.scenarios.find((s) => s.id === S.scenId);
  if (!sc) return; const wrap = el("div");
  wrap.append(el("p", { class: "assumed", text: t("scen_assumed") }));
  wrap.append(el("p", { class: "note", text: t("scen_scope") }));
  wrap.append(el("h3", { text: S.lang === "ar" ? sc.label_ar : sc.label_en }));
  if (sc.id === "nominal") { wrap.append(el("p", { class: "note", text: t("scen_nominal_note") })); }
  const dl = el("dl", { class: "kv" });
  kv(dl, t("scen_min_nn"), withUnit(num(sc.min_nn_m, 2), sc.id === "nominal" ? "sampled-nn" : "stress-model"));
  if (sc.id !== "nominal") {
    kv(dl, t("scen_max_err"), withUnit(num(sc.max_tracking_error_m, 2), "tracking-error"));
    kv(dl, t("scen_seed"), numLink(S.int(S.index.seed), "fixed-seed"));
    const cfg = el("ul", { class: "list" });
    for (const [k, v] of Object.entries(sc.config || {})) {
      let val;
      if (typeof v === "number") val = numLink(new Intl.NumberFormat(S.lang === "ar" ? "ar-u-nu-arab" : "en", { maximumFractionDigits: 3, useGrouping: false }).format(v), "stress-config");
      else { val = numLink(Object.entries(v).map(([a, b]) => `${a}: ${b}`).join(", "), "events"); val.dir = "ltr"; val.style.unicodeBidi = "isolate"; }
      cfg.append(el("li", {}, `${STRINGS[S.lang]["cfg_" + k] ?? STRINGS.en["cfg_" + k] ?? k}: `, val));
    }
    kv(dl, t("scen_config"), cfg);
  }
  wrap.append(dl);
  if (sc.id !== "nominal") {
    wrap.append(el("h3", {}, el("a", { href: methodsHref("events"), text: t("scen_events") })));
    if (!sc.events?.length) wrap.append(el("p", { class: "note", text: t("scen_no_events") }));
    else {
      const ul = el("ul", { class: "list" });
      for (const e of sc.events.slice().sort((a, b) => a.t - b.t)) {
        const li = el("li", { text: t("event_row", { d: S.int(e.drone), kind: t("kind_" + e.kind), t: num(e.t, 1) }) });
        const b = el("button", { type: "button", class: "btn ev-btn", text: t("jump") + " " + num(e.t, 1) });
        b.addEventListener("click", () => { S.playing = false; S.time = Math.min(e.t + 1, S.T); S.dirty = true; updateTransport(); });
        li.append(b); ul.append(li);
      }
      wrap.append(ul);
    }
    const bal = el("p", { class: "note" }); bal.append(el("a", { href: methodsHref("ballistic"), text: t("scen_ballistic") }), ": ",
      sc.ballistic?.length ? sc.ballistic.map((d) => S.int(d)).join(", ") : t("scen_none"));
    wrap.append(bal);
  }
  setBlock(body, wrap);
}

// ------------------------------------------------------------------ guided layer
const sep = () => (S.lang === "ar" ? "، " : ", ");
const stageName = (st) => (S.lang === "ar" ? st.name_ar : st.name_en);
const scenLabel = (sc) => (S.lang === "ar" ? sc.label_ar : sc.label_en);
const nominalExact = () => {
  const r = S.cert?.results; if (!r) return null;
  const pr = r.min_separation_pair || [null, null];
  return { d: r.min_separation, t: r.min_separation_time ?? S.dataA?.tight.t ?? 0, a: pr[0], b: pr[1] };
};
function jumpTo(t) { S.playing = false; S.time = Math.min(Math.max(t, 0), S.T); S.dirty = true; updateTransport(); }
function setMode(m) {
  S.mode = m; $("#modeGroup").querySelectorAll("button").forEach((x) => x.setAttribute("aria-pressed", String(x.dataset.mode === m)));
  buildLegend(); S.dirty = true;
}

// Camera
function refreshCam() { $("#camGroup").querySelectorAll("button").forEach((b) => b.setAttribute("aria-pressed", String(b.dataset.cam === S.cam))); }
function applyCamera(kind) {
  if (!S.A || !S.frame) return;
  S.cam = kind; S.A.setPreset(kind, S.frame); View.sync(S.A); refreshCam(); S.dirty = true;
}
// Extent of the formations (hold windows of the stages between launch and landing), for camera framing.
function computeFrame(d, show) {
  const sc = S.index.pos_scale, n = d.n, st = show.stages || [];
  let x0 = Infinity, x1 = -Infinity, y0 = Infinity, y1 = -Infinity, z0 = 0, z1 = -Infinity;
  for (let s = 1; s < st.length - 1; s++) {
    const k0 = Math.max(0, Math.ceil(st[s].t_arrive / S.dt - 1e-9)), k1 = Math.min(d.K - 1, Math.floor(st[s].t_end / S.dt + 1e-9));
    for (let k = k0; k <= k1; k++) for (let i = 0; i < n; i++) {
      const p = (k * n + i) * 3, x = d.pos[p] * sc, y = d.pos[p + 1] * sc, z = d.pos[p + 2] * sc;
      if (x < x0) x0 = x; if (x > x1) x1 = x; if (y < y0) y0 = y; if (y > y1) y1 = y; if (z > z1) z1 = z;
    }
  }
  if (!Number.isFinite(x0)) { const g = show.geofence; x0 = g.x[0]; x1 = g.x[1]; y0 = g.y[0]; y1 = g.y[1]; z1 = 100; }
  return { x0, x1, y0, y1, z0, z1 };
}

// Verdict cards in plain language
function vcard(root, ok, title, detail, note, onTight) {
  root.className = `vcard guide-only ${ok ? "ok" : "bad"}`;
  const kids = [el("span", { class: "vc-t", text: title }), el("p", { text: detail })];
  if (note) kids.push(el("p", { class: "vc-n", text: note }));
  if (onTight) { const b = el("button", { type: "button", class: "btn", text: S.t("go_tight") }); b.addEventListener("click", onTight); kids.push(b); }
  root.replaceChildren(...kids);
}
function renderVerdicts() {
  const t = S.t, num = S.num, sh = S.show; if (!sh) return;
  const a = $("#vcA"), b = $("#vcB"), dmin = num(sh.d_min_m, 1);
  const ex = nominalExact();
  if (S.cert && ex) {
    const pass = S.cert.verdict === "PASS";
    vcard(a, pass, t(pass ? "vc_nom_pass_t" : "vc_nom_fail_t"), t("vc_nom_d", { d: num(ex.d, 2), t: num(ex.t, 1), dmin }), "", () => jumpTo(ex.t));
  } else { a.className = "vcard guide-only"; a.replaceChildren(el("p", { text: t("loading") })); }
  const d = S.dataB;
  if (d) {
    const bad = d.everBelow > 0, args = { d: num(d.tight.d, 2), t: num(d.tight.t, 1), dmin, k: S.int(d.everBelow), n: S.int(d.n) };
    vcard(b, !bad, t(bad ? "vc_str_bad_t" : "vc_str_ok_t"), t(bad ? "vc_str_bad_d" : "vc_str_ok_d", args), t("vc_str_note", { dt: num(S.dt, 1) }), () => jumpTo(d.tight.t));
  } else { b.className = "vcard guide-only"; b.replaceChildren(el("p", { text: t("loading") })); }
}

// Stages and markers on the timeline
function stageIndexAt(t) {
  const st = S.show?.stages || []; let k = 0;
  for (let i = 0; i < st.length; i++) if (t >= st[i].t_start - 1e-9) k = i;
  return k;
}
function renderTimeline() {
  const band = $("#tlBand"), mk = $("#tlMarkers"), sh = S.show; band.replaceChildren(); mk.replaceChildren();
  if (!sh?.stages || !S.T) return;
  const st = sh.stages;
  st.forEach((s, k) => {
    const end = k + 1 < st.length ? st[k + 1].t_start : S.T, pct = ((end - s.t_start) / S.T) * 100;
    const b = el("button", { type: "button", class: "tl-seg", style: `flex-basis:${pct.toFixed(3)}%`, title: `${k + 1}. ${stageName(s)}`, text: `${S.int(k + 1)} ${stageName(s)}`, "data-k": String(k) });
    b.addEventListener("click", () => jumpTo(k === 0 ? 0 : s.t_arrive));
    band.append(b);
  });
  const add = (cls, t, label) => {
    const b = el("button", { type: "button", class: `tl-mk ${cls}`, style: `left:${((t / S.T) * 100).toFixed(3)}%`, title: label, "aria-label": label });
    b.addEventListener("click", () => jumpTo(t)); mk.append(b);
  };
  const ex = nominalExact();
  if (ex) add("tight-nom", ex.t, S.t("tl_tight_nom", { t: S.num(ex.t, 1) }));
  if (S.dataB) add("tight-str", S.dataB.tight.t, S.t("tl_tight_str", { t: S.num(S.dataB.tight.t, 1) }));
  const sc = S.show.scenarios.find((s) => s.id === S.scenId), seen = new Set();
  for (const e of (sc?.events || []).slice().sort((a, b) => a.t - b.t)) {
    if (seen.has(e.drone)) continue; seen.add(e.drone);
    add("ev", e.t, S.t("tl_event", { kind: S.t("kind_" + e.kind), d: S.int(e.drone), t: S.num(e.t, 1) }));
  }
  stageKey = "";
  updateStage();
}
let stageKey = "";
function updateStage() {
  const st = S.show?.stages, out = $("#stageNow"); if (!st || !out) return;
  const k = stageIndexAt(S.time), s = st[k], done = S.time >= S.T - 1e-6;
  const ph = done ? "phase_end" : S.time < s.t_arrive - 1e-9 ? "phase_move" : "phase_hold";
  const key = `${S.lang}|${k}|${ph}`; if (key === stageKey) return; stageKey = key;
  out.replaceChildren(S.t("stage_cur", { k: S.int(k + 1), n: S.int(st.length), name: stageName(s) }), " ", el("span", { class: "ph", text: `· ${S.t(ph)}` }));
  $("#tlBand").querySelectorAll(".tl-seg").forEach((b) => {
    const cur = Number(b.dataset.k) === k; b.classList.toggle("cur", cur);
    if (cur) b.setAttribute("aria-current", "true"); else b.removeAttribute("aria-current");
  });
}

// Guided tour
function tourContent(k) {
  const t = S.t, num = S.num, sh = S.show, st = sh.stages, dmin = num(sh.d_min_m, 1);
  if (k === 1) return t("tour1_c", { n: S.int(st.length), list: st.map(stageName).join(sep()) });
  if (k === 2) {
    const ex = nominalExact(); if (!ex) return t("loading");
    return t("tour2_c", { e: num(ex.d, 2), t: num(ex.t, 1), a: S.int(ex.a), b: S.int(ex.b), dmin });
  }
  if (k === 3) {
    const d = S.dataB, sc = sh.scenarios.find((s) => s.id === S.scenId); if (!d || !sc) return t("loading");
    const res = t(d.everBelow > 0 ? "tour3_bad" : "tour3_ok", { k: S.int(d.everBelow), dmin, t: num(d.tight.t, 1) });
    return t("tour3_c", { scen: scenLabel(sc), res });
  }
  return t("tour4_c");
}
function renderSteps() {
  const ol = $("#steps"); ol.replaceChildren();
  for (let k = 1; k <= 4; k++) {
    const b = el("button", { type: "button", class: "step-btn" }, el("span", { class: "no", text: S.int(k) }), el("span", { text: S.t(`tour${k}_t`) }));
    if (S.tour === k) b.setAttribute("aria-current", "step");
    b.addEventListener("click", () => goStep(k));
    ol.append(el("li", {}, b));
  }
}
function renderTour() {
  const body = $("#tourBody"), k = S.tour;
  $("#tourStart").hidden = k !== 0; $("#tourExit").hidden = k === 0;
  $("#tourBack").hidden = k <= 1; $("#tourNext").hidden = k === 0 || k >= 4;
  if (!k || !S.show) { body.replaceChildren(el("p", { class: "note", text: S.t("tour_idle") })); return; }
  body.replaceChildren(el("span", { class: "step-no", text: S.t("tour_step", { k: S.int(k), n: S.int(4) }) }), el("h3", { text: S.t(`tour${k}_t`) }), el("p", { text: tourContent(k) }));
}
function goStep(k) {
  S.tour = k; const ex = nominalExact();
  if (S.show && S.dataA) {
    if (k === 1) { setMode("height"); jumpTo(S.show.stages[1]?.t_start ?? 0); S.playing = !reducedMotion; updateTransport(); }
    else if (k === 2 && ex) { setMode("sep"); jumpTo(ex.t); }
    else if (k === 3 && S.dataB) { setMode("sep"); jumpTo(S.dataB.tight.t); }
    else if (k === 4) { S.fc.on = true; renderFC(); $("#why").scrollIntoView({ block: "nearest" }); }
  }
  renderSteps(); renderTour();
}
function exitTour() { S.tour = 0; renderSteps(); renderTour(); }

// Frame check comparison
function renderFC() {
  const t = S.t, num = S.num, on = S.fc.on, tg = $("#fcToggle"), steps = $("#fcSteps"), out = $("#fcOut"), evd = $("#fcEvidence");
  tg.setAttribute("aria-pressed", String(on)); steps.hidden = !on; out.hidden = !on; evd.hidden = !on; $("#tlFrames").hidden = !on;
  if (!on) return;
  steps.replaceChildren();
  for (const sp of [1, 2, 5]) {
    const b = el("button", { type: "button", class: "btn seg-btn", "aria-pressed": String(sp === S.fc.step), text: `${num(sp, 0)} ${t("unit_s")}` });
    b.addEventListener("click", () => { S.fc.step = sp; renderFC(); }); steps.append(b);
  }
  $("#tlFrames").style.setProperty("--fc-step", String(((S.fc.step / Math.max(S.T, 1)) * 100).toFixed(4)));
  const d = S.dataA, ex = nominalExact(), kids = [];
  if (d && ex) {
    const every = Math.max(1, Math.round(S.fc.step / S.dt)); let m = Infinity;
    for (let k = 0; k < d.K; k += every) m = Math.min(m, d.frameMin[k]);
    const diff = m - ex.d;
    kids.push(el("p", {}, ...t("fc_sees", { step: num(S.fc.step, 0), d: "\u0001", e: "\u0002" }).split(/(\u0001|\u0002)/).map((x) =>
      x === "\u0001" ? el("span", { class: "fc-num", text: num(m, 2) }) : x === "\u0002" ? el("span", { class: "fc-num", text: num(ex.d, 2) }) : x)));
    kids.push(el("p", { class: "note", text: diff > 0.005 ? t("fc_over", { x: num(diff, 2) }) : t("fc_same") }));
  }
  out.replaceChildren(...kids);
  const ev = S.index?.evidence; if (!ev) { evd.replaceChildren(); return; }
  const pc = (v) => num(v * 100, v * 100 < 1 && v > 0 ? 2 : v === 0 ? 0 : 1), r = ev.crossings.false_pass_rate, rz = ev.razor;
  const ul = el("ul", {}, el("li", { text: t("fc_ev_cross", { a: pc(r["0.5"]), b: pc(r["0.2"]), c: pc(r["0.05"]) }) }),
    el("li", { text: t("fc_ev_razor", { n: S.int(rz.n), x: S.int(rz.exact_wrong), y: pc(rz.frame_wrong_rate["0.01"]) }) }));
  evd.replaceChildren(el("h3", { text: t("fc_ev_title") }), ul, el("p", { class: "note" }, el("a", { href: "results.html", text: t("fc_ev_src") })));
}

// Guided or classic layout
function applyViewMode(classic, store) {
  S.classic = classic; document.body.classList.toggle("classic", classic);
  $("#viewModeBtn").textContent = S.t(classic ? "guided" : "classic");
  if (store) { try { localStorage.setItem(VIEW_KEY, classic ? "classic" : "guided"); } catch (e) { /* ignore */ } }
  if (classic && S.tour) exitTour();
  S.dirty = true;
}
function renderGuide() { renderSteps(); renderTour(); renderVerdicts(); renderTimeline(); renderFC(); }

// ------------------------------------------------------------------ orchestration
function showOverlay(which, text) { (which === "A" ? S.A : S.B).overlay.textContent = text || ""; }
function showError(err) {
  const g = $("#globalError"); g.hidden = false; g.replaceChildren();
  g.append(el("p", { text: S.t("load_error", { msg: String(err?.message || err) }) }), el("p", {}, S.t("load_hint")));
  const b = el("button", { type: "button", class: "btn", text: S.t("retry") }); b.addEventListener("click", () => { g.hidden = true; loadShow(S.show?.id ?? S.index.shows[0].id); });
  g.append(b);
}

async function loadShow(id) {
  const token = ++S.loadToken; const show = S.index.shows.find((s) => s.id === id) ?? S.index.shows[0];
  S.show = show; S.playing = false; S.time = 0; S.dt = S.index.dt; S.T = (show.frames - 1) * S.dt; S.cert = null; S.dataA = S.dataB = null; S.frame = null;
  if (S.tour) S.tour = 0;
  if (!show.scenarios.some((s) => s.id === S.scenId)) S.scenId = show.scenarios.find((s) => s.id !== "nominal")?.id ?? "nominal";
  fillSelects(); renderShowMeta(); renderCert(); renderScenario(); renderGuide();
  $("#scrub").max = String(Math.round(S.T * 50)); $("#globalError").hidden = true;
  for (const w of ["A", "B"]) showOverlay(w, S.t("loading_show", { name: show.name }));
  updateTransport();
  try {
    const cert = await fetchJSON(`${DATA}/${show.certificate}`); if (token !== S.loadToken) return;
    S.cert = cert; renderCert(); renderVerdicts(); renderTimeline();
    S.A.setShow(show, cert); S.B.setShow(show, cert);
    const nomSc = show.scenarios.find((s) => s.id === "nominal");
    const [dA] = await Promise.all([loadScenarioData(show, nomSc)]); if (token !== S.loadToken) return;
    S.dataA = dA; S.A.setData(dA, nomSc); showOverlay("A", "");
    S.frame = computeFrame(dA, show); applyCamera(S.cam ?? "front");
    renderTimeline(); renderFC();
    await loadScenario(S.scenId, token);
  } catch (e) { for (const w of ["A", "B"]) showOverlay(w, String(e.message || e)); showError(e); }
  S.dirty = true;
}
async function loadScenario(id, token = S.loadToken) {
  const sc = S.show.scenarios.find((s) => s.id === id); S.scenId = id; renderScenario(); S.dataB = null; renderVerdicts(); renderTimeline();
  showOverlay("B", S.t("loading"));
  try {
    const d = await loadScenarioData(S.show, sc); if (token !== S.loadToken || id !== S.scenId) return;
    S.dataB = d; S.B.setData(d, sc); showOverlay("B", "");
    renderVerdicts(); renderTimeline(); if (S.tour === 3) renderTour();
  } catch (e) { showOverlay("B", String(e.message || e)); showError(e); }
  S.dirty = true;
}

function setLang(lang, store = true) {
  S.lang = lang; S.t = makeT(lang); S.num = makeNum(lang); S.int = makeInt(lang);
  if (store) storeLang(lang);
  applyDocLang(lang); translateDom(document, S.t);
  document.title = S.t("app_title");
  $("#langBtn").textContent = S.t("lang_switch"); $("#langBtn").title = S.t("lang_switch_label");
  $("#langBtn").lang = lang === "ar" ? "en" : "ar";
  $("#themeBtn").textContent = S.theme === "dark" ? S.t("theme_light") : S.t("theme_dark");
  $("#keysHint").textContent = S.t("keys_hint");
  $("#footNote").textContent = S.t("foot_note");
  document.querySelectorAll('a[href^="methods.html"]').forEach((a) => {
    if (a.classList.contains("num")) return; const h = a.getAttribute("href").replace(/^methods\.html(\?lang=\w+)?/, `methods.html?lang=${lang}`); a.setAttribute("href", h);
  });
  renderBanner(); buildReadouts(); buildLegend(); buildSpeedButtons();
  $("#modeGroup").querySelectorAll("button").forEach((b) => b.setAttribute("aria-pressed", String(b.dataset.mode === S.mode)));
  $("#viewModeBtn").textContent = S.t(S.classic ? "guided" : "classic");
  if (S.index) { fillSelects(); renderShowMeta(); renderCert(); renderScenario(); renderGuide(); updateTransport(); }
  else renderSteps();
  S.dirty = true;
}

let last = performance.now();
function loop(now) {
  requestAnimationFrame(loop);
  const dtr = Math.min((now - last) / 1000, 0.1);
  if (bench && S.show && S.dataA && S.dataB) { bench.raf.push(now - last); S.playing = true; S.dirty = true; }
  last = now;
  if (S.playing) {
    S.time += dtr * S.speed;
    if (S.time >= S.T) { if (bench) S.time = 0; else { S.time = S.T; S.playing = false; } }
    updateTransport(); S.dirty = true;
  }
  if (!S.dirty || !S.show) return;
  S.dirty = false;
  const t0 = bench ? performance.now() : 0;
  const rA = S.A.update(S.time), rB = S.B.update(S.time);
  setRO("A", rA); setRO("B", rB);
  const t1 = bench ? performance.now() : 0;
  S.A.render(); S.B.render();
  if (bench && S.dataA && S.dataB) {
    const t2 = performance.now();
    bench.update.push(t1 - t0); bench.render.push(t2 - t1); bench.total.push(t2 - t0);
  }
  window.__sirb = { time: S.time, rA, rB, frames: (window.__sirb?.frames ?? 0) + 1 };
}

function wire() {
  $("#showSelect").addEventListener("change", (e) => loadShow(e.target.value));
  $("#scenSelect").addEventListener("change", (e) => loadScenario(e.target.value));
  $("#langBtn").addEventListener("click", () => setLang(S.lang === "ar" ? "en" : "ar"));
  $("#themeBtn").addEventListener("click", () => applyTheme(S.theme === "dark" ? "light" : "dark", true));
  $("#playBtn").addEventListener("click", () => { if (!S.show) return; if (!S.playing && S.time >= S.T) S.time = 0; S.playing = !S.playing; updateTransport(); S.dirty = true; });
  $("#restartBtn").addEventListener("click", () => { S.time = 0; S.dirty = true; updateTransport(); });
  const step = (d) => { S.playing = false; S.time = Math.min(Math.max(S.time + d * S.dt, 0), S.T); S.dirty = true; updateTransport(); };
  $("#backBtn").addEventListener("click", () => step(-1)); $("#fwdBtn").addEventListener("click", () => step(1));
  const sc = $("#scrub");
  sc.addEventListener("input", () => { S.time = Math.min(Number(sc.value) / 50, S.T); S.dirty = true; updateTransport(); });
  sc.addEventListener("keydown", (e) => {
    const big = e.shiftKey ? S.dt : 1; let handled = true;
    if (e.key === "ArrowRight" || e.key === "ArrowUp") S.time = Math.min(S.time + big, S.T);
    else if (e.key === "ArrowLeft" || e.key === "ArrowDown") S.time = Math.max(S.time - big, 0);
    else if (e.key === "Home") S.time = 0; else if (e.key === "End") S.time = S.T; else handled = false;
    if (handled) { e.preventDefault(); S.dirty = true; updateTransport(); }
  });
  $("#modeGroup").querySelectorAll("button").forEach((b) => b.addEventListener("click", () => {
    S.mode = b.dataset.mode; $("#modeGroup").querySelectorAll("button").forEach((x) => x.setAttribute("aria-pressed", String(x === b)));
    buildLegend(); S.dirty = true;
  }));
  $("#camGroup").querySelectorAll("button").forEach((b) => b.addEventListener("click", () => applyCamera(b.dataset.cam)));
  $("#viewModeBtn").addEventListener("click", () => applyViewMode(!S.classic, true));
  $("#tourStart").addEventListener("click", () => goStep(1));
  $("#tourNext").addEventListener("click", () => goStep(Math.min(S.tour + 1, 4)));
  $("#tourBack").addEventListener("click", () => goStep(Math.max(S.tour - 1, 1)));
  $("#tourExit").addEventListener("click", exitTour);
  $("#fcToggle").addEventListener("click", () => { S.fc.on = !S.fc.on; renderFC(); });
  document.addEventListener("keydown", (e) => {
    if (e.target.closest?.("select,input,textarea,canvas")) return;
    if (e.code === "Space" && e.target === document.body) { e.preventDefault(); $("#playBtn").click(); }
  });
}

async function main() {
  S.theme = initialTheme(); document.documentElement.dataset.theme = S.theme;
  try { S.classic = localStorage.getItem(VIEW_KEY) === "classic"; } catch (e) { /* ignore */ }
  document.body.classList.toggle("classic", S.classic);
  setLang(S.lang, false); wire();
  try { S.A = new View($("#viewA"), "A"); S.B = new View($("#viewB"), "B"); }
  catch (e) { const g = $("#globalError"); g.hidden = false; g.textContent = S.t("webgl_error"); return; }
  S.A.resize(); S.B.resize();
  for (const w of ["A", "B"]) showOverlay(w, S.t("loading"));
  try { S.index = await fetchJSON(`${DATA}/index.json`); } catch (e) { for (const w of ["A", "B"]) showOverlay(w, ""); showError(e); return; }
  renderBanner(); fillSelects();
  await loadShow(S.index.shows[0].id);
  setLang(S.lang, false); // re-render dynamic parts now that the index is loaded
}
window.__sirbForce = () => { S.dirty = true; }; // test hook
requestAnimationFrame(loop);
main();
