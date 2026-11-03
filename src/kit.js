/* ============================================================
   Robotics Lab kit: rigid transforms, oriented primitives, depth sorting, assembly compose.
   Built on core.js (Colorfigures). Every part is drawn in its link's frame and registered with an
   oriented box (OBB) so the painter's sort works for links at any angle.
   ============================================================ */
const DEG = Math.PI / 180;
const T_I = [1,0,0,0, 0,1,0,0, 0,0,1,0];                       // 3x4 row-major [R | t]
function tMul(A, B){ const o = new Array(12);
  for (let i = 0; i < 3; i++){ const a0 = A[i*4], a1 = A[i*4+1], a2 = A[i*4+2];
    for (let j = 0; j < 4; j++) o[i*4+j] = a0*B[j] + a1*B[4+j] + a2*B[8+j] + (j === 3 ? A[i*4+3] : 0); }
  return o; }
const tCat = (...Ts) => Ts.reduce(tMul);
const tTr = (x, y, z) => [1,0,0,x, 0,1,0,y, 0,0,1,z];
const tRz = a => { const c = Math.cos(a), s = Math.sin(a); return [c,-s,0,0, s,c,0,0, 0,0,1,0]; };
const tRy = a => { const c = Math.cos(a), s = Math.sin(a); return [c,0,s,0, 0,1,0,0, -s,0,c,0]; };
const tRx = a => { const c = Math.cos(a), s = Math.sin(a); return [1,0,0,0, 0,c,-s,0, 0,s,c,0]; };
const tAp = (T, p) => [T[0]*p[0]+T[1]*p[1]+T[2]*p[2]+T[3], T[4]*p[0]+T[5]*p[1]+T[6]*p[2]+T[7], T[8]*p[0]+T[9]*p[1]+T[10]*p[2]+T[11]];
const tVec = (T, n) => [T[0]*n[0]+T[1]*n[1]+T[2]*n[2], T[4]*n[0]+T[5]*n[1]+T[6]*n[2], T[8]*n[0]+T[9]*n[1]+T[10]*n[2]];
const tAx = (T, k) => [T[k], T[4+k], T[8+k]];
const vAdd = (a, b) => [a[0]+b[0], a[1]+b[1], a[2]+b[2]], vSc = (a, k) => [a[0]*k, a[1]*k, a[2]*k];
const vLen = a => Math.hypot(a[0], a[1], a[2]), vLerp = (a, b, t) => [a[0]+(b[0]-a[0])*t, a[1]+(b[1]-a[1])*t, a[2]+(b[2]-a[2])*t];
const clamp = (v, a, b) => Math.max(a, Math.min(b, v));
const wrap180 = a => ((a + 540) % 360) - 180;

/* ---------- oriented boxes ---------- */
function obbT(T, lo, hi){ const m = [0,1,2].map(i => (lo[i] + hi[i]) / 2);
  return { c: tAp(T, m), ax: [tAx(T,0), tAx(T,1), tAx(T,2)], h: [0,1,2].map(i => Math.abs(hi[i] - lo[i]) / 2) }; }
const obbW = (lo, hi) => obbT(T_I, lo, hi);
function obbPts(pts, e = .05){ const lo = [0,1,2].map(i => Math.min(...pts.map(p => p[i])) - e), hi = [0,1,2].map(i => Math.max(...pts.map(p => p[i])) + e); return obbW(lo, hi); }
function obbRod(a, b, r){ const d = sub(b, a), L = vLen(d) || 1e-6, u = vSc(d, 1/L), w = norm(cross(u, Math.abs(u[2]) < .9 ? [0,0,1] : [1,0,0])), v = cross(u, w);
  return { c: vLerp(a, b, .5), ax: [u, w, v], h: [L/2, r, r] }; }
function obbCorners(o){ const out = [];
  for (const sx of [-1,1]) for (const sy of [-1,1]) for (const sz of [-1,1])
    out.push([0,1,2].map(i => o.c[i] + sx*o.h[0]*o.ax[0][i] + sy*o.h[1]*o.ax[1][i] + sz*o.h[2]*o.ax[2][i]));
  return out; }

/* ---------- painter's order: separating axes of both boxes, then depth; topological so long parts sort right ---------- */
const obbRad = (o, n) => o.h[0]*Math.abs(dot(o.ax[0], n)) + o.h[1]*Math.abs(dot(o.ax[1], n)) + o.h[2]*Math.abs(dot(o.ax[2], n));
function obbOrder(A, B){
  for (const n of [...A.ax, ...B.ax]){ const v = dot(CAM_V, n); if (Math.abs(v) < .03) continue;
    const ca = dot(A.c, n), cb = dot(B.c, n), ra = obbRad(A, n), rb = obbRad(B, n);
    if (ca + ra <= cb - rb + 1e-3) return v > 0 ? -1 : 1;     // A sits on the low side of B along n
    if (cb + rb <= ca - ra + 1e-3) return v > 0 ? 1 : -1; }
  return 0; }
function dsort(nodes){
  const n = nodes.length; if (n < 2) return nodes.map(x => x.svg).join('');
  for (const d of nodes){ const cs = obbCorners(d.obb).map(p => P(...p)), xs = cs.map(p => p[0]), ys = cs.map(p => p[1]);
    d.sb = [Math.min(...xs), Math.min(...ys), Math.max(...xs), Math.max(...ys)]; d.dp = dot(d.obb.c, CAM_V); }
  const outE = nodes.map(() => []), indeg = new Array(n).fill(0);
  for (let i = 0; i < n; i++) for (let j = i + 1; j < n; j++){ const a = nodes[i], b = nodes[j];
    if (a.sb[2] < b.sb[0] || b.sb[2] < a.sb[0] || a.sb[3] < b.sb[1] || b.sb[3] < a.sb[1]) continue;
    let r = obbOrder(a.obb, b.obb); if (!r) r = a.dp <= b.dp ? -1 : 1;
    if (r < 0){ outE[i].push(j); indeg[j]++; } else { outE[j].push(i); indeg[i]++; } }
  const done = new Array(n).fill(false), out = [];
  for (let c = 0; c < n; c++){ let best = -1;
    for (let i = 0; i < n; i++) if (!done[i] && indeg[i] <= 0 && (best < 0 || nodes[i].dp < nodes[best].dp)) best = i;
    if (best < 0) for (let i = 0; i < n; i++) if (!done[i] && (best < 0 || indeg[i] < indeg[best] || (indeg[i] === indeg[best] && nodes[i].dp < nodes[best].dp))) best = i;
    done[best] = true; out.push(nodes[best].svg); for (const j of outE[best]) indeg[j]--; }
  return out.join(''); }

/* ---------- scene builder: robots register parts; compose() filters by assembly step and sorts ---------- */
let SB = null;
const sbStart = () => (SB = { items: [], groups: {}, g: null, p: null, anchor: {} });
const sbPart = p => { SB.p = p; };
const sbGroup = (name, T, lo, hi) => { SB.groups[name] = obbT(T, lo, hi); SB.g = name; };
const sbFree = () => { SB.g = null; };
const sbAnchor = (p, at) => { SB.anchor[p] = at; };
function sbPut(svg, obb, o = {}){ if (!svg) return;
  const p = 'p' in o ? o.p : SB.p; SB.items.push({ svg, obb, g: o.g !== undefined ? o.g : SB.g, p, cover: !!o.cover, flat: !!o.flat });
  if (p && !SB.anchor[p]) SB.anchor[p] = obb.c; }

/* ---------- oriented primitives (frame T, local coordinates) ---------- */
const axMap = { x: (s,u,v) => [s,u,v], y: (s,u,v) => [u,s,v], z: (s,u,v) => [u,v,s] };
function qBox(T, lo, hi, M, o = {}){
  const [x0,y0,z0] = lo, [x1,y1,z1] = hi, sh = o.sh || toneI, w = o.w ?? .9;
  const F = [[[1,0,0], [[x1,y0,z0],[x1,y1,z0],[x1,y1,z1],[x1,y0,z1]]], [[-1,0,0], [[x0,y0,z0],[x0,y1,z0],[x0,y1,z1],[x0,y0,z1]]],
             [[0,1,0], [[x0,y1,z0],[x1,y1,z0],[x1,y1,z1],[x0,y1,z1]]], [[0,-1,0], [[x0,y0,z0],[x1,y0,z0],[x1,y0,z1],[x0,y0,z1]]],
             [[0,0,1], [[x0,y0,z1],[x1,y0,z1],[x1,y1,z1],[x0,y1,z1]]], [[0,0,-1], [[x0,y0,z0],[x1,y0,z0],[x1,y1,z0],[x0,y1,z0]]]];
  let s = '';
  for (const [n, q] of F){ const nw = tVec(T, n); if (!vis(nw)) continue; s += shape(q.map(p => tAp(T, p)), sh(nw, M), w); if (o.deco) s += o.deco(n) || ''; }
  sbPut(s, obbT(T, lo, hi), o); return s; }
// convex profile uv swept along local axis ax from a0 to a1: rounded links, housings, plates
function qExt(T, uv, a0, a1, M, o = {}){
  const mp = axMap[o.ax || 'x'], sh = o.sh || tone, w = o.w ?? 1.6;
  const A = uv.map(([u,v]) => tAp(T, mp(a0,u,v))), B = uv.map(([u,v]) => tAp(T, mp(a1,u,v))), dn = tVec(T, mp(1,0,0));
  let s = loft(A, B, n => sh(n, M));
  const capB = vis(dn), cap = capB ? B : A, cn = capB ? dn : vSc(dn, -1);
  if (vis(cn)){ s += fill(pd(cap), sh(cn, M)) + ink(pd(cap), w*.55); if (o.capDeco) s += o.capDeco(capB) || ''; }
  s += hullInk([...A, ...B], w);
  const us = uv.map(p => p[0]), vs = uv.map(p => p[1]);
  sbPut(s, obbT(T, mp(Math.min(a0,a1), Math.min(...us), Math.min(...vs)), mp(Math.max(a0,a1), Math.max(...us), Math.max(...vs))), o); return s; }
function qRod(a, b, r, M, o = {}){
  const s = rod(a, b, r, M, o.sh || toneI, o.w ?? 1, o.r1 ?? r, o.n || 18) + (o.deco ? o.deco() : '');
  sbPut(s, obbRod(a, b, Math.max(r, o.r1 ?? r)), o); return s; }
function qCyl(T, ax, c, a0, a1, r, M, o = {}){ const mp = axMap[ax]; return qRod(tAp(T, mp(a0, c[0], c[1])), tAp(T, mp(a1, c[0], c[1])), r, M, o); }
// spur gear: a body with a toothed rim drawn on whichever face looks at the viewer
function qGear(T, ax, c, a0, a1, r, teeth, M, o = {}){
  const mp = axMap[ax], a = tAp(T, mp(a0, c[0], c[1])), b = tAp(T, mp(a1, c[0], c[1])), d = norm(sub(b, a)), end = vis(d) ? b : a;
  const U = tVec(T, mp(0,1,0)), V = tVec(T, mp(0,0,1)), pts = [], rot = o.rot || 0;
  for (let k = 0; k < teeth; k++) for (const [t, rr] of [[0, r], [.2, r*1.15], [.5, r*1.15], [.7, r]]){ const th = (k + t)/teeth*2*Math.PI + rot;
    pts.push(vAdd(end, vAdd(vSc(U, rr*Math.cos(th)), vSc(V, rr*Math.sin(th))))); }
  const hub = Array.from({length:12}, (_, i) => { const th = i/12*2*Math.PI; return vAdd(end, vAdd(vSc(U, r*.3*Math.cos(th)), vSc(V, r*.3*Math.sin(th)))); });
  const s = rod(a, b, r*1.04, M, toneI, .8, r*1.04, 20) + fill(pd(pts), M.l) + ink(pd(pts), .55) + fill(pd(hub), M.d) + ink(pd(hub), .4);
  sbPut(s, obbRod(a, b, r*1.15), o); return s; }
// a belt hugging two pulleys in one plane (axis = the pulleys' shaft direction)
function qBelt(c1, c2, r1, r2, axis, col = '#2b2724', o = {}){
  const n = norm(axis), U = norm(cross(n, Math.abs(n[2]) < .9 ? [0,0,1] : [1,0,0])), V = cross(n, U);
  const ring = (c, r) => Array.from({length:20}, (_, i) => { const t = i/20*2*Math.PI; return vAdd(c, vAdd(vSc(U, r*Math.cos(t)), vSc(V, r*Math.sin(t)))); });
  const pts = [...ring(c1, r1), ...ring(c2, r2)], h = hull(pts.map(p => [dot(sub(p, c1), U), dot(sub(p, c1), V)]));
  const loop = h.map(([u, v]) => vAdd(c1, vAdd(vSc(U, u), vSc(V, v)))), d = pd(loop), w = o.w || 1.5;
  const s = `<path d="${d}" fill="none" stroke="${INK}" stroke-width="${w}" stroke-linejoin="round"/><path d="${d}" fill="none" stroke="${col}" stroke-width="${f(w*.5)}" stroke-linejoin="round"/>`;
  sbPut(s, obbPts(loop, .3), o); return s; }
// coil spring / lead-screw thread between a and b
function qHelix(a, b, r, turns, col = METAL.l, o = {}){
  const d = norm(sub(b, a)), U = norm(cross(d, Math.abs(d[2]) < .9 ? [0,0,1] : [1,0,0])), V = cross(d, U), L = vLen(sub(b, a)), pts = [], n = Math.ceil(turns*14);
  for (let i = 0; i <= n; i++){ const t = i/n, th = t*turns*2*Math.PI; pts.push(vAdd(vAdd(a, vSc(d, L*t)), vAdd(vSc(U, r*Math.cos(th)), vSc(V, r*Math.sin(th))))); }
  const pth = pd(pts, false), w = o.w || 1.1;
  const s = `<path d="${pth}" fill="none" stroke="${INK}" stroke-width="${w}" stroke-linecap="round" stroke-linejoin="round"/><path d="${pth}" fill="none" stroke="${col}" stroke-width="${f(w*.45)}" stroke-linecap="round" stroke-linejoin="round"/>`;
  sbPut(s, obbRod(a, b, r), o); return s; }
// round each interior corner of a route (a cable bends, it doesn't kink)
const bendPts = (pts, r = 1.2) => { if (pts.length < 3) return pts; const out = [pts[0]];
  for (let i = 1; i < pts.length - 1; i++){ const [a, b, c] = [pts[i-1], pts[i], pts[i+1]];
    const la = vLen(sub(a, b)), lc = vLen(sub(c, b)), d = Math.min(r, la/2.2, lc/2.2);
    if (d < 1e-3){ out.push(b); continue; }
    const p = vLerp(b, a, d/la), q = vLerp(b, c, d/lc);
    for (let j = 0; j <= 4; j++){ const t = j/4; out.push([0,1,2].map(k => (1-t)*(1-t)*p[k] + 2*(1-t)*t*b[k] + t*t*q[k])); } }
  out.push(pts[pts.length-1]); return out; };
// a cable cut into short pieces, each with its own box, so it sorts behind or in front of the solids it passes
function qWire(pts, col, w = .3, o = {}){ const q = bendPts(pts, o.bend ?? 1.2), Lseg = o.seg || 4;
  for (let i = 0; i < q.length - 1; ){
    let j = i + 1, len = 0; while (j < q.length - 1 && len < Lseg){ len += vLen(sub(q[j], q[j-1])); j++; }
    if (len < Lseg) while (j < q.length - 1) j++;
    const seg = q.slice(i, j + 1), d = pd(seg, false);
    const s = `<path d="${d}" fill="none" stroke="${INK}" stroke-width="${f(2*w + .8)}" stroke-linecap="round" stroke-linejoin="round"/>`
      + `<path d="${d}" fill="none" stroke="${col}" stroke-width="${f(2*w)}" stroke-linecap="round" stroke-linejoin="round"/>`;
    sbPut(s, obbPts(seg, w), Object.assign({ flat: true }, o)); i = j; } }
// text lying on a face of frame T at local point p; u = reading direction, v = up (both local). Flips u so it never reads mirrored.
function txT(T, p, u, v, str, size, col = INK, weight = 800){
  const o = P(...tAp(T, p)); let U = Dv(...tVec(T, u)), V = Dv(...vSc(tVec(T, v), -1));
  if (U[0]*V[1] - U[1]*V[0] < 0) U = U.map(x => -x);
  if (U[0] < 0){ U = U.map(x => -x); V = V.map(x => -x); }     // turn it the right way up (a rotation, so still not mirrored)
  return `<text transform="${txM(o, U, V)}" font-size="${size}" fill="${col}" text-anchor="middle" font-family="ui-monospace,Menlo,monospace" font-weight="${weight}" letter-spacing=".04em">${str}</text>`; }
const dotAt = (p, r, col, w = .4) => { const q = P(...p), rr = r*1.2247; return `<circle cx="${f(q[0])}" cy="${f(q[1])}" r="${f(rr)}" fill="${col}" stroke="${INK}" stroke-width="${w}"/>`; };
// a ball (caster, foot pad, joint marker): shaded circle with a highlight
function qBall(c, r, M, o = {}){ const q = P(...c), rr = r*1.2247;
  const s = `<circle cx="${f(q[0])}" cy="${f(q[1])}" r="${f(rr)}" fill="${M.m}" stroke="${INK}" stroke-width="${o.w ?? .9}"/><circle cx="${f(q[0] - rr*.3)}" cy="${f(q[1] - rr*.3)}" r="${f(rr*.42)}" fill="${M.l}"/>`;
  sbPut(s, obbW(vSc([1,1,1], 0).map((_, i) => c[i] - r), [0,1,2].map(i => c[i] + r)), o); return s; }

/* ---------- materials shared by the robots ---------- */
const MOTOR = matOf('#4b5662'), ALU = matOf('#d4d8da'), STEEL = METAL, BRASS = matOf('#d8b25a'), REDUCER = matOf('#a9b1b5');
const PCB = { l:'#4fa36a', m:'#2f8a4a', d:'#1d5f31' }, RUBBER = matOf('#3a3633'), CABLE = { pwr:'#e2483d', sig:'#f4b829', gnd:'#3a3633', enc:'#4f86c6', air:'#5aa6d6' };
const BATT = matOf('#5a82c6'), SERVO = matOf('#2f3a48'), HORN = matOf('#f2efe7'), TYRE = matOf('#3d3935');

/* ---------- compose: assembly filter, x-ray covers, highlights, sorting ---------- */
function partShown(st, R, p){ const i = R._step[p]; return i == null || i < st.built; }
function compose(st, R){
  const k = ease(st.xr || 0), sel = st.sel, act = R.active ? R.active(st) : {}, groups = {}, nodes = [];
  const lift = Dv(0, 0, 26*(1 - ease(Math.min(1, st.pt ?? 1))));
  for (const it of SB.items){ if (it.p && !partShown(st, R, it.p)) continue;
    const anim = it.p && R._step[it.p] === st.built - 1 && st.pt < 1;
    let s = it.svg; const cls = (sel && it.p === sel) || anim ? 'hl' : act[it.p] ? 'on' : '';
    if (anim) s = `<g transform="translate(${f(lift[0])} ${f(lift[1])})" opacity="${f(Math.min(1, .15 + st.pt*1.5))}">${s}</g>`;
    s = `<g data-p="${it.p || ''}"${cls ? ` class="${cls}"` : ''}>${s}</g>`;
    if (it.g && SB.groups[it.g]){ const G = groups[it.g] || (groups[it.g] = { inner: [], cov: [] }); G[it.cover ? 'cov' : 'inner'].push({ obb: it.obb, svg: s }); }
    else nodes.push({ obb: it.obb, svg: s }); }
  for (const [g, v] of Object.entries(groups)){
    let s = dsort(v.inner);
    if (v.cov.length) s += `<g opacity="${f(1 - .84*k)}"${k > .5 ? ' pointer-events="none"' : ''}>${dsort(v.cov)}</g>`;
    nodes.push({ obb: SB.groups[g], svg: s }); }
  return dsort(nodes); }
// one cast shadow per object: every built solid of the robot goes into one hull; props (part.shadow) and part-less world
// items (obstacles) get their own
function shadowsOf(st, R, z0 = 0, clip = null){ const sets = {}; let s = '';
  SB.items.forEach((it, i) => { if (it.flat || (it.p && !partShown(st, R, it.p))) return; if (Math.max(...it.obb.h) < .7) return;
    const key = !it.p ? 'w' + i : (R.parts[it.p] && R.parts[it.p].shadow) || 'main'; (sets[key] = sets[key] || []).push(...obbCorners(it.obb)); });
  for (const pts of Object.values(sets)) s += castShadow(pts, z0);
  return clip ? clipTo(clip, s) : s; }
function labelsSVG(st, R){
  if (!document.getElementById('fig') || document.documentElement.classList.contains('nolabels')) return '';
  const k = ease(st.xr || 0), act = R.active ? R.active(st) : {}, items = [];
  for (const [p, d] of Object.entries(R.parts)){
    if (!d.lbl || !SB.anchor[p] || !partShown(st, R, p)) continue;
    if (d.inside ? k <= .6 : (k > .6 && !d.key)) continue;
    items.push({ at: SB.anchor[p], text: d.lbl, on: !!act[p] || st.sel === p }); }
  return items.length ? xrCallouts(items, R.bounds) : ''; }
function labScene(st, m, R){
  sbStart(); const d = R.draw(st, m) || {};
  return (d.under || '') + shadowsOf(st, R, d.z0 || 0, d.clip) + (d.floor || '') + compose(st, R) + (d.over || '') + labelsSVG(st, R); }

/* ---------- overlays ---------- */
function crashBurst(p, t){ const q = P(...p), r = (4 + 5*(1 - t))*INK_K*3, pts = [];
  for (let i = 0; i < 16; i++){ const a = i/16*2*Math.PI, rr = i % 2 ? r*.45 : r; pts.push([q[0] + rr*Math.cos(a), q[1] + rr*Math.sin(a)]); }
  return `<g opacity="${f(Math.min(1, t*2))}" pointer-events="none"><path d="${sp(pts)}" fill="#ffd23f" stroke="#e2483d" stroke-width="1.6"/><circle cx="${f(q[0])}" cy="${f(q[1])}" r="${f(r*.25)}" fill="#e2483d"/></g>`; }
const trailSVG = (pts, col = '#e2483d', w = 1.1, dash = '') => pts.length > 1 ? `<path d="${pd(pts, false)}" fill="none" stroke="${col}" stroke-width="${w}" stroke-linecap="round" stroke-linejoin="round" ${dash ? `stroke-dasharray="${dash}"` : ''} opacity=".85" pointer-events="none"/>` : '';
// the drag handle on a tool tip: a ring with a cross, plus a plumb line to the floor so height reads
function tipHandle(p, z0, on){ const q = P(...p), g = P(p[0], p[1], z0), r = 5*INK_K*1.6;
  return `<path d="M${f(q[0])} ${f(q[1])}L${f(g[0])} ${f(g[1])}" stroke="${INK}" stroke-width=".7" stroke-dasharray="1.5 1.5" fill="none" pointer-events="none"/>`
    + `<ellipse cx="${f(g[0])}" cy="${f(g[1])}" rx="${f(r*.6)}" ry="${f(r*.3)}" fill="none" stroke="${INK}" stroke-width=".7" pointer-events="none"/>`
    + act('tip', `<circle cx="${f(q[0])}" cy="${f(q[1])}" r="${f(r*1.9)}" fill="#fff" fill-opacity=".01"/><circle cx="${f(q[0])}" cy="${f(q[1])}" r="${f(r)}" fill="${on ? '#e2483d' : 'var(--accent,#f4b829)'}" fill-opacity=".35" stroke="${on ? '#e2483d' : INK}" stroke-width="1.3"/>`
      + `<path d="M${f(q[0]-r*1.5)} ${f(q[1])}H${f(q[0]+r*1.5)}M${f(q[0])} ${f(q[1]-r*1.5)}V${f(q[1]+r*1.5)}" stroke="${INK}" stroke-width=".9"/>`); }
// screen point (viewBox units) -> point on the plane z = z0 in scene coordinates
function screenToPlane(X, Y, z0){ const o = P(0, 0, z0), a = Dv(1,0,0), b = Dv(0,1,0), det = a[0]*b[1] - a[1]*b[0];
  if (Math.abs(det) < 1e-6) return null; const dx = X - o[0], dy = Y - o[1]; return [(dx*b[1] - dy*b[0]) / det, (a[0]*dy - a[1]*dx) / det]; }
// screen drag (viewBox units) -> world move in the camera plane
const dragWorld = (dx, dy) => vAdd(vSc(CAM_R, dx/CAM_K), vSc(CAM_D, dy/CAM_K));
// a screen drag -> move on a horizontal plane
function dragPlane(dx, dy){ const a = Dv(1,0,0), b = Dv(0,1,0), det = a[0]*b[1] - a[1]*b[0]; if (Math.abs(det) < .05) return [0, 0];
  return [(dx*b[1] - dy*b[0]) / det, (a[0]*dy - a[1]*dx) / det]; }
// floor disc with a grid that scrolls with the robot's world position (follow camera)
function floorDisc(cx, cy, R0, ox, oy, sp = 10){
  const rim = on(PZ(0), ellUV(cx, cy, R0, R0, 64)), rimB = on(PZ(-1.2), ellUV(cx, cy, R0, R0, 64));
  let g = '';
  const x0 = Math.floor((ox - R0) / sp) * sp, y0 = Math.floor((oy - R0) / sp) * sp;
  for (let x = x0; x <= ox + R0; x += sp) g += pd([[cx + x - ox, cy - R0 - 2, 0], [cx + x - ox, cy + R0 + 2, 0]], false);
  for (let y = y0; y <= oy + R0; y += sp) g += pd([[cx - R0 - 2, cy + y - oy, 0], [cx + R0 + 2, cy + y - oy, 0]], false);
  return loft(rimB, rim, n => toneI(n, BOARD)) + `<g data-floor="1">` + fill(pd(rim), '#ebe5d6') + clipTo(rim, `<path d="${g}" fill="none" stroke="#c9bfa9" stroke-width=".6"/>`) + `</g>` + ink(pd(rim), 1.1) + hullInk([...rim, ...rimB], 1.6); }
