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

// Points A (start) and B (end) in canvas coords
const A = {x: 140, y: 120};
const B = {x: 840, y: 420};

// Physics / simulation state
let path = [];
let times = [];
let totalTime = 0;
let animT = 0;
let running = false;

function lerp(a,b,t){ return a + (b-a)*t; }
function dist(p,q){ return Math.hypot(p.x-q.x, p.y-q.y); }

function buildLine(n){
  const pts = [];
  for(let i=0;i<n;i++){
    const t = i/(n-1);
    pts.push({x: lerp(A.x,B.x,t), y: lerp(A.y,B.y,t)});
  }
  return pts;
}

// Arc of circle through A,B with a chosen "bulge" (approx)
function buildCircleArc(n){
  // Choose a control point below line to define a circle-ish arc
  const mid = {x:(A.x+B.x)/2, y:(A.y+B.y)/2};
  const bulge = 180;
  const C = {x: mid.x, y: mid.y + bulge};

  // quadratic bezier as approximation of arc
  const pts=[];
  for(let i=0;i<n;i++){
    const t=i/(n-1);
    const x = (1-t)*(1-t)*A.x + 2*(1-t)*t*C.x + t*t*B.x;
    const y = (1-t)*(1-t)*A.y + 2*(1-t)*t*C.y + t*t*B.y;
    pts.push({x,y});
  }
  return pts;
}

// Simple quadratic Bezier with adjustable bulge
function buildBezier(n){
  const mid = {x:(A.x+B.x)/2, y:(A.y+B.y)/2};
  const C = {x: mid.x, y: mid.y + 240};
  const pts=[];
  for(let i=0;i<n;i++){
    const t=i/(n-1);
    const x = (1-t)*(1-t)*A.x + 2*(1-t)*t*C.x + t*t*B.x;
    const y = (1-t)*(1-t)*A.y + 2*(1-t)*t*C.y + t*t*B.y;
    pts.push({x,y});
  }
  return pts;
}

// Cycloid between A and B (scaled). For brachistochrone, the cycloid is parametric.
// We fit it numerically by solving for theta_end so that x,y match B relative to A.
function buildCycloid(n){
  // Parametric cycloid: x = R(θ - sinθ), y = R(1 - cosθ)
  // We want Δx = B.x - A.x, Δy = B.y - A.y (note: canvas y down)
  const dx = B.x - A.x;
  const dy = B.y - A.y;

  // Find θ such that dy/dx matches (1 - cosθ) / (θ - sinθ) = dy/dx
  // Solve f(θ) = R*(1 - cosθ) - dy with R = dx / (θ - sinθ)
  function f(theta){
    const denom = theta - Math.sin(theta);
    if(denom <= 1e-6) return 1e9;
    const R = dx / denom;
    return R*(1 - Math.cos(theta)) - dy;
  }

  // simple bisection in (0, 10)
  let lo = 0.01, hi = 10.0;
  for(let k=0;k<80;k++){
    const mid = 0.5*(lo+hi);
    if(f(mid) === 0) { lo=hi=mid; break; }
    if(f(lo)*f(mid) <= 0) hi = mid;
    else lo = mid;
  }
  const thetaEnd = 0.5*(lo+hi);
  const R = dx / (thetaEnd - Math.sin(thetaEnd));

  const pts=[];
  for(let i=0;i<n;i++){
    const t=i/(n-1);
    const th = thetaEnd*t;
    const x = A.x + R*(th - Math.sin(th));
    const y = A.y + R*(1 - Math.cos(th));
    pts.push({x,y});
  }
  return pts;
}

function computeTimes(pts, g){
  // Discretize time: dt = ds / v, with v = sqrt(2 g Δh)
  // Here Δh is vertical drop from start: h = y - A.y (since canvas y increases downward)
  const tArr = new Array(pts.length).fill(0);
  let t=0;

  for(let i=1;i<pts.length;i++){
    const p0=pts[i-1];
    const p1=pts[i];
    const ds=dist(p0,p1);

    const dh = (p1.y - A.y);
    const v = Math.sqrt(Math.max(0, 2*g*dh/100)); 
    // /100 -> scale from pixels to "meters-ish" to keep times readable

    const dt = (v > 1e-8) ? ds / v : 0;
    t += dt;
    tArr[i]=t;
  }
  return {tArr, total:t};
}

function pointAtTime(pts, tArr, t){
  if(t<=0) return pts[0];
  if(t>=tArr[tArr.length-1]) return pts[pts.length-1];

  // binary search
  let lo=0, hi=tArr.length-1;
  while(hi-lo>1){
    const mid = (lo+hi)>>1;
    if(tArr[mid] < t) lo=mid;
    else hi=mid;
  }
  const t0=tArr[lo], t1=tArr[hi];
  const alpha = (t - t0)/(t1 - t0);
  return {
    x: lerp(pts[lo].x, pts[hi].x, alpha),
    y: lerp(pts[lo].y, pts[hi].y, alpha)
  };
}

function drawScene(ballPos){
  ctx.clearRect(0,0,W,H);

  // axes / background hint
  ctx.globalAlpha = 0.25;
  ctx.beginPath();
  ctx.moveTo(0, H-40);
  ctx.lineTo(W, H-40);
  ctx.strokeStyle = "#ffffff";
  ctx.lineWidth = 1;
  ctx.stroke();
  ctx.globalAlpha = 1;

  // Path
  ctx.beginPath();
  for(let i=0;i<path.length;i++){
    const p=path[i];
    if(i===0) ctx.moveTo(p.x,p.y);
    else ctx.lineTo(p.x,p.y);
  }
  ctx.strokeStyle = "#7aa2ff";
  ctx.lineWidth = 3;
  ctx.stroke();

  // Start & End
  ctx.fillStyle = "#56f2c3";
  ctx.beginPath(); ctx.arc(A.x,A.y,7,0,Math.PI*2); ctx.fill();
  ctx.beginPath(); ctx.arc(B.x,B.y,7,0,Math.PI*2); ctx.fill();

  // Ball
  ctx.fillStyle = "#ffcc66";
  ctx.beginPath();
  ctx.arc(ballPos.x, ballPos.y, 10, 0, Math.PI*2);
  ctx.fill();
}

function rebuild(){
  const n = Math.max(100, parseInt(nInput.value || "400",10));
  const curve = curveSelect.value;
  curveOut.textContent = curve;

  if(curve === "line") path = buildLine(n);
  else if(curve === "circle") path = buildCircleArc(n);
  else if(curve === "bezier") path = buildBezier(n);
  else path = buildCycloid(n);

  const g = parseFloat(gInput.value || "9.81");
  const res = computeTimes(path, g);
  times = res.tArr;
  totalTime = res.total;
  timeOut.textContent = totalTime.toFixed(2);

  animT = 0;
  running = false;
  drawScene(path[0]);
}

function step(ts){
  if(!running) return;

  // advance time
  animT += 0.016 * totalTime/3; // speed factor for visible animation
  const p = pointAtTime(path, times, animT);
  drawScene(p);

  if(animT >= totalTime){
    running = false;
    drawScene(path[path.length-1]);
    return;
  }
  requestAnimationFrame(step);
}

runBtn.addEventListener("click", ()=>{
  if(!path.length) rebuild();
  animT = 0;
  running = true;
  requestAnimationFrame(step);
});
resetBtn.addEventListener("click", rebuild);
curveSelect.addEventListener("change", rebuild);
gInput.addEventListener("change", rebuild);
nInput.addEventListener("change", rebuild);

rebuild();
