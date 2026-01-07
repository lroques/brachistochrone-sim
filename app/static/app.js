const canvas = document.getElementById("canvas");
const ctx = canvas.getContext("2d");

const curveSelect = document.getElementById("curveSelect");
const gInput = document.getElementById("gInput");
const runBtn = document.getElementById("runBtn");
const resetBtn = document.getElementById("resetBtn");
const centerCBtn = document.getElementById("centerCBtn");
const timeOut = document.getElementById("timeOut");
const statusOut = document.getElementById("statusOut");
const curveOut = document.getElementById("curveOut");

const W = canvas.width;
const H = canvas.height;

// -------------------------
// FIXED RESOLUTION
// -------------------------
const N = 1000; // fixed resolution, user cannot change

// -------------------------
// WORLD <-> CANVAS SETTINGS
// -------------------------
const PX_PER_M = 100;

// Padding for axes and labels (in pixels)
const PAD_L = 70;
const PAD_B = 60;
const PAD_R = 20;
const PAD_T = 20;

// Tick in meters
const TICK_EVERY_M = 1;
const TICK_LEN = 6;

// Visible world extents (in meters) for axes/grid
const VIEW_W_M = (W - PAD_L - PAD_R) / PX_PER_M;
const VIEW_H_M = (H - PAD_T - PAD_B) / PX_PER_M;

// -------------------------
// WORLD POINTS (meters)
// Convention: world y increases upward.
// Points can be outside the visible region (including negative).
// -------------------------
let A_w = { x: 0.7, y: 3.2 };
let B_w = { x: 7.7, y: 0.2 };
let C_w = { x: (A_w.x + B_w.x) / 2, y: (A_w.y + B_w.y) / 2 - 1.5 };

// -------------------------
// SIMULATION STATE
// -------------------------
let path_w = [];
let times = [];
let totalTime = 0;
let feasible = true;

let running = false;
let animT = 0;          // simulated time (seconds)
let lastFrameMs = null; // for real-time animation

// Drag state
let dragging = null; // "A" | "B" | "C" | null
const PICK_RADIUS_PX = 14;

// -------------------------
// Helpers
// -------------------------
function lerp(a, b, t) { return a + (b - a) * t; }
function dist_w(p, q) { return Math.hypot(p.x - q.x, p.y - q.y); }
function getMousePosCanvas(evt) {
  const rect = canvas.getBoundingClientRect();
  return { x: evt.clientX - rect.left, y: evt.clientY - rect.top };
}

// -------------------------
// WORLD <-> CANVAS
// -------------------------
function worldToCanvas(pw) {
  const x = PAD_L + pw.x * PX_PER_M;
  const y = H - PAD_B - pw.y * PX_PER_M;
  return { x, y };
}
function canvasToWorld(pc) {
  const x = (pc.x - PAD_L) / PX_PER_M;
  const y = (H - PAD_B - pc.y) / PX_PER_M;
  return { x, y };
}

function pointHit(which, mouse_c) {
  const p = which === "A" ? A_w : which === "B" ? B_w : C_w;
  const pc = worldToCanvas(p);
  return Math.hypot(mouse_c.x - pc.x, mouse_c.y - pc.y) <= PICK_RADIUS_PX;
}

function activeCurveAllowsDrag() {
  // In bezier mode: allow A/B/C drag
  // In cycloid mode: allow A/B drag
  const curve = curveSelect.value;
  if (curve === "bezier") return "ABC";
  return "AB";
}

// -------------------------
// CURVES (world meters)
// -------------------------
function buildBezier(n) {
  const pts = [];
  for (let i = 0; i < n; i++) {
    const t = i / (n - 1);
    const x =
      (1 - t) * (1 - t) * A_w.x +
      2 * (1 - t) * t * C_w.x +
      t * t * B_w.x;
    const y =
      (1 - t) * (1 - t) * A_w.y +
      2 * (1 - t) * t * C_w.y +
      t * t * B_w.y;
    pts.push({ x, y });
  }
  return pts;
}

// Cycloid fitted from A to B (requires dx>0, B lower than A)
function buildCycloid(n) {
  const dx = B_w.x - A_w.x;
  const dy_down = A_w.y - B_w.y; // positive if B lower

  // If not valid (e.g. B above A or dx <= 0), still build a bezier-like fallback
  // so user sees something; time will be flagged as impossible.
  if (dy_down <= 0 || dx <= 0) return buildBezier(n);

  function f(theta) {
    const denom = theta - Math.sin(theta);
    if (denom <= 1e-10) return 1e9;
    const ratio = (1 - Math.cos(theta)) / denom;
    return ratio - (dy_down / dx);
  }

  let lo = 0.01, hi = 10.0;
  for (let k = 0; k < 120; k++) {
    const mid = 0.5 * (lo + hi);
    const flo = f(lo);
    const fmid = f(mid);
    if (Math.abs(fmid) < 1e-12) { lo = hi = mid; break; }
    if (flo * fmid <= 0) hi = mid;
    else lo = mid;
  }

  const thetaEnd = 0.5 * (lo + hi);
  const R = dx / (thetaEnd - Math.sin(thetaEnd));

  const pts = [];
  for (let i = 0; i < n; i++) {
    const t = i / (n - 1);
    const th = thetaEnd * t;
    const x = A_w.x + R * (th - Math.sin(th));
    const y = A_w.y - R * (1 - Math.cos(th));
    pts.push({ x, y });
  }
  return pts;
}

// -------------------------
// PHYSICS (seconds)
// dt = ds / v_avg, v = sqrt(2 g drop), drop = A_y - y
// If curve goes above A, infeasible from rest.
// -------------------------
function computeTimes(pts_w, g) {
  const tArr = new Array(pts_w.length).fill(0);
  let t = 0;
  let ok = true;

  for (let i = 1; i < pts_w.length; i++) {
    const p0 = pts_w[i - 1];
    const p1 = pts_w[i];

    if (p0.y > A_w.y + 1e-12 || p1.y > A_w.y + 1e-12) ok = false;

    const ds = dist_w(p0, p1);

    const drop0 = A_w.y - p0.y;
    const drop1 = A_w.y - p1.y;

    const v0 = Math.sqrt(Math.max(0, 2 * g * drop0));
    const v1 = Math.sqrt(Math.max(0, 2 * g * drop1));

    const vavg = 0.5 * (v0 + v1);
    const dt = vavg > 1e-10 ? ds / vavg : 0;

    t += dt;
    tArr[i] = t;
  }

  return { tArr, total: t, feasible: ok };
}

function pointAtTime(pts_w, tArr, t) {
  if (t <= 0) return pts_w[0];
  if (t >= tArr[tArr.length - 1]) return pts_w[pts_w.length - 1];

  let lo = 0, hi = tArr.length - 1;
  while (hi - lo > 1) {
    const mid = (lo + hi) >> 1;
    if (tArr[mid] < t) lo = mid;
    else hi = mid;
  }

  const t0 = tArr[lo], t1 = tArr[hi];
  const alpha = (t - t0) / (t1 - t0);

  return {
    x: lerp(pts_w[lo].x, pts_w[hi].x, alpha),
    y: lerp(pts_w[lo].y, pts_w[hi].y, alpha),
  };
}

// -------------------------
// AXES (meters)
// -------------------------
function drawAxes() {
  ctx.save();
  ctx.strokeStyle = "rgba(255,255,255,0.28)";
  ctx.fillStyle = "rgba(255,255,255,0.60)";
  ctx.lineWidth = 1;
  ctx.font = "12px ui-sans-serif, system-ui, -apple-system, Segoe UI, Roboto, Arial";

  const origin_c = worldToCanvas({ x: 0, y: 0 });
  const xEnd_c = worldToCanvas({ x: VIEW_W_M, y: 0 });
  const yEnd_c = worldToCanvas({ x: 0, y: VIEW_H_M });

  // X axis
  ctx.beginPath();
  ctx.moveTo(origin_c.x, origin_c.y);
  ctx.lineTo(xEnd_c.x, xEnd_c.y);
  ctx.stroke();

  // Y axis
  ctx.beginPath();
  ctx.moveTo(origin_c.x, origin_c.y);
  ctx.lineTo(yEnd_c.x, yEnd_c.y);
  ctx.stroke();

  for (let xm = 0; xm <= VIEW_W_M + 1e-9; xm += TICK_EVERY_M) {
    const p = worldToCanvas({ x: xm, y: 0 });
    ctx.beginPath();
    ctx.moveTo(p.x, p.y);
    ctx.lineTo(p.x, p.y + TICK_LEN);
    ctx.stroke();
    ctx.fillText(`${xm.toFixed(0)}m`, p.x - 10, p.y + 22);
  }

  for (let ym = 0; ym <= VIEW_H_M + 1e-9; ym += TICK_EVERY_M) {
    const p = worldToCanvas({ x: 0, y: ym });
    ctx.beginPath();
    ctx.moveTo(p.x, p.y);
    ctx.lineTo(p.x - TICK_LEN, p.y);
    ctx.stroke();
    ctx.fillText(`${ym.toFixed(0)}m`, p.x - 40, p.y + 4);
  }

  ctx.fillStyle = "rgba(255,255,255,0.75)";
  ctx.fillText("x (m)", xEnd_c.x - 35, xEnd_c.y - 10);
  ctx.fillText("y (m)", yEnd_c.x + 10, yEnd_c.y + 12);

  ctx.fillStyle = "rgba(255,255,255,0.45)";
  ctx.fillText(`Échelle: 1m = ${PX_PER_M}px`, origin_c.x + 10, origin_c.y - 14);

  ctx.restore();
}

// -------------------------
// DRAW
// -------------------------
function drawPoint(pw, label, color) {
  const pc = worldToCanvas(pw);
  ctx.fillStyle = color;
  ctx.beginPath();
  ctx.arc(pc.x, pc.y, 7, 0, Math.PI * 2);
  ctx.fill();

  ctx.fillStyle = "rgba(255,255,255,0.85)";
  ctx.font = "13px ui-sans-serif, system-ui, -apple-system, Segoe UI, Roboto, Arial";
  ctx.fillText(label, pc.x + 10, pc.y + 4);
}

function drawScene(ball_w) {
  ctx.clearRect(0, 0, W, H);

  drawAxes();

  const curve = curveSelect.value;

  // Control polygon for Bezier
  if (curve === "bezier") {
    const A_c = worldToCanvas(A_w);
    const B_c = worldToCanvas(B_w);
    const C_c = worldToCanvas(C_w);

    ctx.save();
    ctx.strokeStyle = "rgba(255,255,255,0.20)";
    ctx.lineWidth = 1;
    ctx.setLineDash([6, 6]);
    ctx.beginPath();
    ctx.moveTo(A_c.x, A_c.y);
    ctx.lineTo(C_c.x, C_c.y);
    ctx.lineTo(B_c.x, B_c.y);
    ctx.stroke();
    ctx.restore();
  }

  // Curve
  ctx.beginPath();
  for (let i = 0; i < path_w.length; i++) {
    const pc = worldToCanvas(path_w[i]);
    if (i === 0) ctx.moveTo(pc.x, pc.y);
    else ctx.lineTo(pc.x, pc.y);
  }
  ctx.strokeStyle = feasible ? "#7aa2ff" : "rgba(255,80,80,0.9)";
  ctx.lineWidth = 3;
  ctx.stroke();

  // Points
  drawPoint(A_w, "A", "#56f2c3");
  drawPoint(B_w, "B", "#56f2c3");
  if (curve === "bezier") drawPoint(C_w, "C", "#ff66cc");

  // Ball
  const ball_c = worldToCanvas(ball_w);
  ctx.fillStyle = "#ffcc66";
  ctx.beginPath();
  ctx.arc(ball_c.x, ball_c.y, 10, 0, Math.PI * 2);
  ctx.fill();
}

// -------------------------
// REBUILD
// -------------------------
function rebuild() {
  const curve = curveSelect.value;
  curveOut.textContent = curve;

  if (curve === "bezier") path_w = buildBezier(N);
  else path_w = buildCycloid(N);

  const g = parseFloat(gInput.value || "9.81");
  const res = computeTimes(path_w, g);

  times = res.tArr;
  totalTime = res.total;
  feasible = res.feasible;

  statusOut.textContent = feasible ? "OK" : "Impossible (remonte au-dessus de A)";
  statusOut.style.color = feasible ? "#56f2c3" : "#ff8080";

  running = false;
  animT = 0;
  lastFrameMs = null;

  timeOut.textContent = "—";
  drawScene(path_w[0]);
}

// -------------------------
// REAL-TIME ANIMATION
// -------------------------
function step(nowMs) {
  if (!running) return;

  if (lastFrameMs === null) lastFrameMs = nowMs;

  const dt = (nowMs - lastFrameMs) / 1000; // seconds
  lastFrameMs = nowMs;

  animT += dt;

  const ball_w = pointAtTime(path_w, times, animT);
  drawScene(ball_w);

  if (animT >= totalTime) {
    running = false;
    drawScene(path_w[path_w.length - 1]);
    timeOut.textContent = feasible ? totalTime.toFixed(3) : "∞";
    lastFrameMs = null;
    return;
  }

  requestAnimationFrame(step);
}

// -------------------------
// CURSOR / DRAG
// -------------------------
function updateCursor(mouse_c) {
  if (running) { canvas.style.cursor = "default"; return; }
  const allowed = activeCurveAllowsDrag();
  const hits = [];
  if (allowed.includes("A") && pointHit("A", mouse_c)) hits.push("A");
  if (allowed.includes("B") && pointHit("B", mouse_c)) hits.push("B");
  if (allowed.includes("C") && pointHit("C", mouse_c)) hits.push("C");
  canvas.style.cursor = hits.length ? "grab" : "default";
}

canvas.addEventListener("mousedown", (evt) => {
  if (running) return;
  const mouse_c = getMousePosCanvas(evt);
  const allowed = activeCurveAllowsDrag();

  if (allowed.includes("C") && pointHit("C", mouse_c)) dragging = "C";
  else if (allowed.includes("A") && pointHit("A", mouse_c)) dragging = "A";
  else if (allowed.includes("B") && pointHit("B", mouse_c)) dragging = "B";
  else dragging = null;

  if (dragging) canvas.style.cursor = "grabbing";
});

canvas.addEventListener("mousemove", (evt) => {
  const mouse_c = getMousePosCanvas(evt);

  if (!dragging) {
    updateCursor(mouse_c);
    return;
  }

  const w = canvasToWorld(mouse_c);

  // No clamping: fully free points
  if (dragging === "A") A_w = { x: w.x, y: w.y };
  else if (dragging === "B") B_w = { x: w.x, y: w.y };
  else if (dragging === "C") C_w = { x: w.x, y: w.y };

  timeOut.textContent = "—";
  rebuild();
});

window.addEventListener("mouseup", () => {
  dragging = null;
  canvas.style.cursor = "default";
});

// -------------------------
// UI EVENTS
// -------------------------
runBtn.addEventListener("click", () => {
  if (!path_w.length) rebuild();

  running = true;
  animT = 0;
  lastFrameMs = null;

  timeOut.textContent = "—";
  requestAnimationFrame(step);
});

resetBtn.addEventListener("click", () => {
  A_w = { x: 0.7, y: 3.2 };
  B_w = { x: 7.7, y: 0.2 };
  C_w = { x: (A_w.x + B_w.x) / 2, y: (A_w.y + B_w.y) / 2 - 1.5 };
  rebuild();
});

centerCBtn.addEventListener("click", () => {
  C_w = { x: (A_w.x + B_w.x) / 2, y: (A_w.y + B_w.y) / 2 - 1.0 };
  rebuild();
});

curveSelect.addEventListener("change", rebuild);
gInput.addEventListener("change", rebuild);

// init
rebuild();
