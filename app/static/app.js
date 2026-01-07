const canvas = document.getElementById("canvas");
const ctx = canvas.getContext("2d");

const curveSelect = document.getElementById("curveSelect");
const gInput = document.getElementById("gInput");
const nInput = document.getElementById("nInput");
const runBtn = document.getElementById("runBtn");
const resetBtn = document.getElementById("resetBtn");
const timeOut = document.getElementById("timeOut");
const curveOut = document.getElementById("curveOut");

const W = canvas.width;
const H = canvas.height;

// -------------------------
// WORLD <-> CANVAS SETTINGS
// -------------------------
// Scale: pixels per meter (controls zoom)
const PX_PER_M = 100;

// Padding for axes and labels (in pixels)
const PAD_L = 70;
const PAD_B = 60;
const PAD_R = 20;
const PAD_T = 20;

// Tick in meters
const TICK_EVERY_M = 1;
const TICK_LEN = 6;

// World extents (in meters)
// You can tweak these if you want more/less room.
const WORLD_W_M = (W - PAD_L - PAD_R) / PX_PER_M;
const WORLD_H_M = (H - PAD_T - PAD_B) / PX_PER_M;

// -------------------------
// WORLD COORDINATES (meters)
// -------------------------
// Define A and B in meters
// Convention: world y increases upward (standard math).
// On canvas, y increases downward, so we convert accordingly.
const A_w = { x: 0.7, y: 3.2 };  // start
const B_w = { x: 7.7, y: 0.2 };  // end (lower)

// -------------------------
// SIMULATION STATE
// -------------------------
let path_w = [];         // curve sampled points in world meters
let times = [];          // cumulative time at each sample (seconds)
let totalTime = 0;       // seconds
let animT = 0;           // current time (seconds)
let running = false;

function lerp(a, b, t) {
  return a + (b - a) * t;
}
function dist_w(p, q) {
  return Math.hypot(p.x - q.x, p.y - q.y); // meters
}

// -------------------------
// WORLD <-> CANVAS CONVERSION
// -------------------------
function worldToCanvas(pw) {
  // world origin (0,0) is bottom-left of drawing area
  const x = PAD_L + pw.x * PX_PER_M;
  const y = H - PAD_B - pw.y * PX_PER_M;
  return { x, y };
}

function canvasToWorld(pc) {
  const x = (pc.x - PAD_L) / PX_PER_M;
  const y = (H - PAD_B - pc.y) / PX_PER_M;
  return { x, y };
}

// -------------------------
// CURVE BUILDERS (world meters)
// -------------------------
function buildLine(n) {
  const pts = [];
  for (let i = 0; i < n; i++) {
    const t = i / (n - 1);
    pts.push({
      x: lerp(A_w.x, B_w.x, t),
      y: lerp(A_w.y, B_w.y, t),
    });
  }
  return pts;
}

// Quadratic Bezier in world meters
function buildBezier(n, bulge_m = 2.4) {
  const mid = { x: (A_w.x + B_w.x) / 2, y: (A_w.y + B_w.y) / 2 };
  // Put control point below the chord to get a steeper start (lower y)
  const C = { x: mid.x, y: mid.y - bulge_m };

  const pts = [];
  for (let i = 0; i < n; i++) {
    const t = i / (n - 1);
    const x =
      (1 - t) * (1 - t) * A_w.x +
      2 * (1 - t) * t * C.x +
      t * t * B_w.x;
    const y =
      (1 - t) * (1 - t) * A_w.y +
      2 * (1 - t) * t * C.y +
      t * t * B_w.y;
    pts.push({ x, y });
  }
  return pts;
}

// Arc of circle approximation via Bezier
function buildCircleArc(n) {
  // Use a milder bulge than Bezier
  return buildBezier(n, 1.8);
}

// Cycloid in world meters fitted so it ends at B_w.
// Parametric cycloid: x = R(θ - sinθ), y = R(1 - cosθ)
// We align start at A_w and end at B_w by solving θ_end and R.
function buildCycloid(n) {
  // Δx, Δy in world meters
  const dx = B_w.x - A_w.x;
  const dy_down = A_w.y - B_w.y; 
  // dy_down is positive if B is lower than A (as expected)

  // If dy_down <= 0, cycloid formula isn't appropriate
  if (dy_down <= 0 || dx <= 0) return buildLine(n);

  // We solve for θ so that:
  // R(θ - sinθ) = dx
  // R(1 - cosθ) = dy_down
  // => dy_down / dx = (1 - cosθ) / (θ - sinθ)
  function f(theta) {
    const denom = theta - Math.sin(theta);
    if (denom <= 1e-10) return 1e9;
    const ratio = (1 - Math.cos(theta)) / denom;
    return ratio - (dy_down / dx);
  }

  // Bisection search for theta in (0, 10)
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
    // minus because cycloid y increases downward in its standard form;
    // in our world, y increases upward => we subtract the downward drop
    pts.push({ x, y });
  }
  return pts;
}

// -------------------------
// PHYSICS: compute cumulative times (seconds)
// -------------------------
// Use energy: v = sqrt(2 g Δh)
// Δh is vertical drop from start: Δh = A_w.y - y
function computeTimes(pts_w, g) {
  const tArr = new Array(pts_w.length).fill(0);
  let t = 0;

  for (let i = 1; i < pts_w.length; i++) {
    const p0 = pts_w[i - 1];
    const p1 = pts_w[i];

    const ds = dist_w(p0, p1); // meters

    const drop = A_w.y - p1.y; // meters (positive when p1 is below A)
    const v = Math.sqrt(Math.max(0, 2 * g * drop)); // m/s

    const dt = v > 1e-10 ? ds / v : 0;
    t += dt;
    tArr[i] = t;
  }

  return { tArr, total: t };
}

function pointAtTime(pts_w, tArr, t) {
  if (t <= 0) return pts_w[0];
  if (t >= tArr[tArr.length - 1]) return pts_w[pts_w.length - 1];

  // binary search
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
// DRAW AXES (meters)
// -------------------------
function drawAxes() {
  ctx.save();
  ctx.strokeStyle = "rgba(255,255,255,0.28)";
  ctx.fillStyle = "rgba(255,255,255,0.60)";
  ctx.lineWidth = 1;
  ctx.font = "12px ui-sans-serif, system-ui, -apple-system, Segoe UI, Roboto, Arial";

  // Axis lines in canvas coordinates
  const origin_c = worldToCanvas({ x: 0, y: 0 });
  const xEnd_c = worldToCanvas({ x: WORLD_W_M, y: 0 });
  const yEnd_c = worldToCanvas({ x: 0, y: WORLD_H_M });

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

  // ticks
  const tick = TICK_EVERY_M;

  // X ticks
  for (let xm = 0; xm <= WORLD_W_M + 1e-9; xm += tick) {
    const p = worldToCanvas({ x: xm, y: 0 });
    ctx.beginPath();
    ctx.moveTo(p.x, p.y);
    ctx.lineTo(p.x, p.y + TICK_LEN);
    ctx.stroke();
    ctx.fillText(`${xm.toFixed(0)}m`, p.x - 10, p.y + 22);
  }

  // Y ticks
  for (let ym = 0; ym <= WORLD_H_M + 1e-9; ym += tick) {
    const p = worldToCanvas({ x: 0, y: ym });
    ctx.beginPath();
    ctx.moveTo(p.x, p.y);
    ctx.lineTo(p.x - TICK_LEN, p.y);
    ctx.stroke();
    ctx.fillText(`${ym.toFixed(0)}m`, p.x - 40, p.y + 4);
  }

  // labels
  ctx.fillStyle = "rgba(255,255,255,0.75)";
  ctx.fillText("x (m)", xEnd_c.x - 35, xEnd_c.y - 10);
  ctx.fillText("y (m)", yEnd_c.x + 10, yEnd_c.y + 12);

  // scale indicator
  ctx.fillStyle = "rgba(255,255,255,0.45)";
  ctx.fillText(`Échelle: 1m = ${PX_PER_M}px`, origin_c.x + 10, origin_c.y - 14);

  ctx.restore();
}

// -------------------------
// RENDER SCENE
// -------------------------
function drawScene(ball_w) {
  ctx.clearRect(0, 0, W, H);

  drawAxes();

  // Draw curve
  ctx.beginPath();
  for (let i = 0; i < path_w.length; i++) {
    const pc = worldToCanvas(path_w[i]);
    if (i === 0) ctx.moveTo(pc.x, pc.y);
    else ctx.lineTo(pc.x, pc.y);
  }
  ctx.strokeStyle = "#7aa2ff";
  ctx.lineWidth = 3;
  ctx.stroke();

  // Draw A and B
  const A_c = worldToCanvas(A_w);
  const B_c = worldToCanvas(B_w);

  ctx.fillStyle = "#56f2c3";
  ctx.beginPath();
  ctx.arc(A_c.x, A_c.y, 7, 0, Math.PI * 2);
  ctx.fill();
  ctx.beginPath();
  ctx.arc(B_c.x, B_c.y, 7, 0, Math.PI * 2);
  ctx.fill();

  // Labels A and B
  ctx.fillStyle = "rgba(255,255,255,0.75)";
  ctx.font = "13px ui-sans-serif, system-ui, -apple-system, Segoe UI, Roboto, Arial";
  ctx.fillText("A", A_c.x - 18, A_c.y - 10);
  ctx.fillText("B", B_c.x + 10, B_c.y + 4);

  // Draw ball
  const ball_c = worldToCanvas(ball_w);
  ctx.fillStyle = "#ffcc66";
  ctx.beginPath();
  ctx.arc(ball_c.x, ball_c.y, 10, 0, Math.PI * 2);
  ctx.fill();
}

// -------------------------
// REBUILD (curve + times)
// -------------------------
function rebuild() {
  const n = Math.max(100, parseInt(nInput.value || "400", 10));
  const curve = curveSelect.value;
  curveOut.textContent = curve;

  if (curve === "line") path_w = buildLine(n);
  else if (curve === "circle") path_w = buildCircleArc(n);
  else if (curve === "bezier") path_w = buildBezier(n);
  else path_w = buildCycloid(n);

  const g = parseFloat(gInput.value || "9.81");
  const res = computeTimes(path_w, g);

  times = res.tArr;
  totalTime = res.total;

  // Reset animation and time display
  animT = 0;
  running = false;
  timeOut.textContent = "—";

  drawScene(path_w[0]);
}

// -------------------------
// ANIMATION LOOP
// -------------------------
function step() {
  if (!running) return;

  // animation speed factor for visual comfort
  animT += 0.016 * (totalTime / 3);

  const ball_w = pointAtTime(path_w, times, animT);
  drawScene(ball_w);

  if (animT >= totalTime) {
    running = false;
    drawScene(path_w[path_w.length - 1]);

    // Show final time only at the end
    timeOut.textContent = totalTime.toFixed(3);
    return;
  }

  requestAnimationFrame(step);
}

// -------------------------
// EVENTS
// -------------------------
runBtn.addEventListener("click", () => {
  if (!path_w.length) rebuild();
  animT = 0;
  running = true;

  // hide time during run
  timeOut.textContent = "—";

  requestAnimationFrame(step);
});

resetBtn.addEventListener("click", rebuild);
curveSelect.addEventListener("change", rebuild);
gInput.addEventListener("change", rebuild);
nInput.addEventListener("change", rebuild);

// init
rebuild();
