/* ============================================================
   Robot 01 — six-axis industrial arm (a small 5 kg-class arm, ~70 cm reach). 1 unit = 1 cm.
   Truth (how a real one is built):
   - Base flange bolted to the plate. Inside the base housing a J1 AC servo drives a ring gear through a pinion; the ring
     gear is the input of an RV (cycloidal) reducer whose output is the turret, carried on a cross-roller bearing.
   - The turret carries the J2 housing (RV reducer + servo + brake + encoder) on one side and the upper arm on the other.
   - The upper arm carries the J3 housing (harmonic drive + servo) on its far end; the forearm turns on it.
   - The forearm's rear houses three small servos for J4/J5/J6. J4 turns a hollow torque tube by a gear pair; J5 and J6
     drive concentric shafts inside it by timing belts; at the wrist a bevel gear turns the J5 drive 90 degrees.
   - The harness enters at the base connector, runs up the hollow J1 axis, through the turret and inside the arm castings.
   - A controller cabinet holds the servo amplifiers; a teach pendant on a cable lets a person jog and record poses.
   Parts: flange, anchors | j1motor, j1gear, j1rv, bearing | drum | turret | j2rv, j2motor, j2housing | ua |
          j3hd, j3motor, j3housing | fa | wmotors, wgears, belts, tube | wrist, bevel, flange6 | harness, conn | ctrl, pendant |
          gripper, fingers, hoses | block
   ============================================================ */
const A6 = { BX: 44, BY: 34, ZS: 30, L2: 30, L3: 26, WX: 3.2, FL: 4, TL: 8.3 };
A6.LF = A6.L3 + A6.WX; A6.DT = A6.FL + A6.TL;
const A6_LIM = [[-170,170],[-40,100],[-75,150],[-180,180],[-120,120],[-270,270]];
const A6_VMAX = [90, 80, 100, 220, 220, 320];
const A6_RATIO = [121, 121, 101, 81, 81, 50];
const A6_NAMES = ['J1 base', 'J2 shoulder', 'J3 elbow', 'J4 wrist roll', 'J5 wrist bend', 'J6 flange'];
const A6_HOME = [0, 0, 0, 0, 90, 0];
const A6_BLOCK0 = [66, 20];

function a6FK(q){
  const T1 = tMul(tTr(A6.BX, A6.BY, 14), tRz(q[0]*DEG));
  const a2 = (90 - q[1])*DEG, b = (q[2] - q[1])*DEG;
  const T2 = tCat(T1, tTr(0, 0, A6.ZS - 14), tRy(-a2));
  const T3 = tCat(T2, tTr(A6.L2, 0, 0), tRy(-(b - a2)));
  const T4 = tCat(T3, tTr(A6.L3, 0, 0), tRx(q[3]*DEG));
  const T5 = tCat(T4, tTr(A6.WX, 0, 0), tRy(q[4]*DEG));
  const T6 = tCat(T5, tTr(A6.FL, 0, 0), tRx(q[5]*DEG));
  return { T1, T2, T3, T4, T5, T6, S: tAp(T2, [0,0,0]), E: tAp(T3, [0,0,0]), W: tAp(T5, [0,0,0]), tip: tAp(T6, [A6.TL, 0, 0]) };
}
// inverse kinematics, tool pointing straight down, J4 = 0, elbow up
function a6IK(p, q6){
  const W = [p[0], p[1], p[2] + A6.DT], dx = W[0] - A6.BX, dy = W[1] - A6.BY, rr = Math.hypot(dx, dy), h = W[2] - A6.ZS, D = Math.hypot(rr, h);
  if (D > A6.L2 + A6.LF - .05) return { err: 'Out of reach: the arm is fully stretched' };
  if (D < Math.abs(A6.L2 - A6.LF) + 1) return { err: 'Too close to the shoulder' };
  const a2 = Math.atan2(h, rr) + Math.acos(clamp((A6.L2**2 + D*D - A6.LF**2) / (2*A6.L2*D), -1, 1));
  const b = Math.atan2(h - A6.L2*Math.sin(a2), rr - A6.L2*Math.cos(a2));
  const q2 = 90 - a2/DEG, q = [Math.atan2(dy, dx)/DEG, q2, b/DEG + q2, 0, b/DEG + 90, q6];
  for (let i = 0; i < 6; i++) if (q[i] < A6_LIM[i][0] - 1e-6 || q[i] > A6_LIM[i][1] + 1e-6) return { err: `${A6_NAMES[i]} would pass its limit (${A6_LIM[i][q[i] < 0 ? 0 : 1]} deg)`, q };
  return { q };
}
function a6Check(q){
  const F = a6FK(q), g = [];
  for (const sx of [.8, 4.6, 8.6]) for (const sy of [-3.6, 3.6]) for (const sz of [-2, 2]) g.push(tAp(F.T6, [sx, sy, sz]));
  const pts = [['elbow', F.E, 5.5], ['wrist', F.W, 3.4], ['tool tip', F.tip, .3], ...g.map(p => ['gripper', p, 0])];
  for (const [nm, p, r] of pts){
    if (p[2] - r < 3.2) return { why: `Crash: the ${nm} hit the table`, at: [p[0], p[1], 3.2] };
    const rb = Math.hypot(p[0] - A6.BX, p[1] - A6.BY);
    if (nm !== 'elbow' && rb < 12.5 + r && p[2] < 15) return { why: 'Self-collision: the tool hit the base housing', at: p };
    if (nm !== 'elbow' && rb < 9 + r && p[2] < 36) return { why: 'Self-collision: the tool hit the turret', at: p };
    if (p[0] > 4 && p[0] < 20 && p[1] > 51 && p[1] < 68 && p[2] < 25) return { why: 'Crash: the arm ran into the controller cabinet', at: p };
  }
  return null;
}
// gravity torque about a joint axis (N m): sum of (r x F) . axis, positions in cm
function a6Torques(q, payload){
  const F = a6FK(q), g = 9.81, at = (T, x) => tAp(T, [x, 0, 0]);
  const masses = [[at(F.T2, A6.L2/2), 5.5, 2], [F.E, 2.2, 3], [at(F.T3, A6.L3*.4), 3.2, 3], [F.W, 1.1, 5], [at(F.T6, 3), .9, 5], [F.tip, payload, 5]];
  const tq = (o, ax, from) => masses.filter(m => m[2] >= from).reduce((s, [p, m]) => s + dot(cross(vSc(sub(p, o), .01), [0, 0, -m*g]), ax), 0);
  return [Math.abs(tq(F.S, tAx(F.T2, 1), 2)), Math.abs(tq(F.E, tAx(F.T3, 1), 3)), Math.abs(tq(F.W, tAx(F.T5, 1), 5))];
}
const A6_RATED = [140, 60, 12];

function a6Draw(st, m){
  const F = a6FK(st.q), { BX, BY, L2, L3 } = A6, CASE = matOf('#e2e0d8'), GRIP = matOf('#6f8fb3'), PAD = RUBBER;
  // ---- base flange and anchor bolts (world)
  sbFree(); sbPart('flange'); qRod([BX, BY, 3], [BX, BY, 4.4], 14.5, ALU, { w: 1.4 });
  sbPart('anchors'); for (const [ox, oy] of [[13.4,0],[0,13.4],[-13.4,0],[0,-13.4]]) qRod([BX+ox, BY+oy, 4.4], [BX+ox, BY+oy, 5.4], .9, STEEL, { n: 6, w: .7 });
  // ---- base housing: J1 servo, pinion + ring gear, RV reducer, cross-roller bearing, under the drum cover
  sbGroup('base', T_I, [BX-12, BY-12, 4.4], [BX+12, BY+12, 14]);
  const MX = BX - 4.45, MY = BY + 4.45, spin = st.q[0]*DEG;
  sbPart('j1motor'); qRod([MX, MY, 4.6], [MX, MY, 9.6], 3, MOTOR, { w: .9 }); qRod([MX, MY, 4.4], [MX, MY, 4.6], 2.4, DARK, { w: .6 });
  sbPart('j1gear'); qGear(T_I, 'z', [BX, BY], 9.6, 10.6, 4.4, 26, BRASS, { rot: spin }); qGear(T_I, 'z', [MX, MY], 9.6, 10.6, 1.7, 10, STEEL, { rot: -spin*121/10 });
  sbPart('j1rv'); qRod([BX, BY, 10.6], [BX, BY, 13.2], 7.2, REDUCER, { w: .9 });
  sbPart('bearing'); qRod([BX, BY, 13.2], [BX, BY, 14], 10.6, STEEL, { w: .8 });
  sbPart('harness'); qWire([[BX-12, BY-1.5, 6], [BX-3, BY-1.5, 6], [BX-1, BY, 8], [BX-1, BY, 13.8]], CABLE.sig, .45);
  sbPart('drum'); qRod([BX, BY, 4.4], [BX, BY, 14], 12, m, { sh: tone, cover: true, w: 2 });
  sbFree(); sbPart('conn'); qBox(T_I, [BX-15, BY-2, 4.4], [BX-12.6, BY+2, 7], DARK, { w: .8 });
  // ---- turret (turns with J1)
  sbPart('turret'); qRod([BX, BY, 14], [BX, BY, 15.6], 10.5, m, { sh: tone, w: 1.4 });
  sbGroup('tur', F.T1, [-8, -5, 1.6], [8, 5, 21]);
  sbPart('turret'); qExt(F.T1, rrUV(-8, 1.6, 8, 21, 5), -5, 5, m, { ax: 'y', cover: true, w: 2,
    capDeco: b => b ? '' : txT(F.T1, [0, -5.05, 8], [1,0,0], [0,0,1], 'IR-6', 2.6) });
  sbPart('harness'); qWire([[0, 0, 1.8], [0, 0, 13], [0, -4.4, 16]].map(p => tAp(F.T1, p)), CABLE.sig, .45);
  // J2 housing on the +y side: RV reducer, servo, encoder
  sbGroup('j2', F.T1, [-5.2, 5, 10.8], [5.2, 13.6, 21.2]);
  sbPart('j2rv'); qCyl(F.T1, 'y', [0, 16], 5.2, 8, 4.4, REDUCER, { w: .9 });
  sbPart('j2motor'); qCyl(F.T1, 'y', [0, 16], 8, 12.8, 3.5, MOTOR, { w: .9 }); qCyl(F.T1, 'y', [0, 16], 12.8, 13.4, 2.6, DARK, { w: .6 });
  sbPart('j2housing'); qCyl(F.T1, 'y', [0, 16], 5, 13.6, 5.2, m, { sh: tone, cover: true, w: 1.8 });
  sbFree(); sbPart('ua'); qCyl(F.T1, 'y', [0, 16], -6, -5, 3, STEEL, { w: .8 });
  // ---- upper arm casting (on the -y side of the turret)
  sbGroup('ua', F.T2, [-5.5, -14, -5.5], [L2 + 5.5, -6, 5.5]);
  sbPart('harness'); qWire([[1, -10, 2], [L2 - 1, -10, 2]].map(p => tAp(F.T2, p)), CABLE.sig, .45);
  sbPart('ua'); qExt(F.T2, rrUV(-5.5, -5.5, L2 + 5.5, 5.5, 5.5), -14, -6, m, { ax: 'y', cover: true, w: 2.2,
    capDeco: b => b ? '' : txT(F.T2, [L2/2, -14.05, -1.2], [1,0,0], [0,0,1], 'IR-6', 3.4) });
  sbFree(); sbPart('fa'); qCyl(F.T2, 'y', [L2, 0], -6, -5, 2.6, STEEL, { w: .8 });
  // ---- J3 housing on the forearm's +y side: harmonic drive + servo
  sbGroup('j3', F.T3, [-4.6, 5, -4.6], [4.6, 12, 4.6]);
  sbPart('j3hd'); qCyl(F.T3, 'y', [0, 0], 5.2, 7.6, 4, REDUCER, { w: .9 });
  sbPart('j3motor'); qCyl(F.T3, 'y', [0, 0], 7.6, 11.4, 3.2, MOTOR, { w: .9 }); qCyl(F.T3, 'y', [0, 0], 11.4, 11.9, 2.4, DARK, { w: .6 });
  sbPart('j3housing'); qCyl(F.T3, 'y', [0, 0], 5, 12, 4.6, m, { sh: tone, cover: true, w: 1.8 });
  // ---- forearm: three wrist servos at the rear, gear pair + two belts onto concentric shafts, torque tube
  sbGroup('fa', F.T3, [-9, -5, -4.8], [L3 - 1.2, 5, 4.8]);
  sbPart('wmotors'); for (const c of [[-2.4, 1.8], [2.4, 1.8], [0, -2.6]]) qCyl(F.T3, 'x', c, -8, -3.4, 2, MOTOR, { w: .8 });
  sbPart('wgears'); qGear(F.T3, 'x', [-2.4, 1.8], -3.4, -2.6, 1.2, 9, STEEL, { rot: st.q[3]*DEG*1.5 }); qGear(F.T3, 'x', [0, 0], -3.4, -2.6, 1.8, 14, BRASS, { rot: -st.q[3]*DEG });
  sbPart('belts'); qCyl(F.T3, 'x', [2.4, 1.8], -3.4, -1.8, .4, STEEL, { w: .5 }); qCyl(F.T3, 'x', [2.4, 1.8], -2.4, -1.8, .9, STEEL, { w: .6 }); qCyl(F.T3, 'x', [0, 0], -2.4, -1.8, 1.9, ALU, { w: .7 });
  qBelt(tAp(F.T3, [-2.1, 2.4, 1.8]), tAp(F.T3, [-2.1, 0, 0]), .95, 1.95, tAx(F.T3, 0), '#2b2724', { w: 1.1 });
  qCyl(F.T3, 'x', [0, -2.6], -3.4, -1.0, .4, STEEL, { w: .5 }); qCyl(F.T3, 'x', [0, -2.6], -1.6, -1.0, .8, STEEL, { w: .6 }); qCyl(F.T3, 'x', [0, 0], -1.6, -1.0, 1.4, ALU, { w: .7 });
  qBelt(tAp(F.T3, [-1.3, 0, -2.6]), tAp(F.T3, [-1.3, 0, 0]), .85, 1.45, tAx(F.T3, 0), '#2b2724', { w: 1.1 });
  sbPart('tube'); qCyl(F.T3, 'x', [0, 0], -1.0, L3 - 1.5, 1.5, STEEL, { w: .8 });
  sbPart('harness'); qWire([[0, -3.6, -3.2], [-6.5, -3.6, -3.2], [-8, -2.4, 0]].map(p => tAp(F.T3, p)), CABLE.sig, .4);
  sbPart('fa'); qExt(F.T3, rrUV(-9, -4.8, L3 - 1.2, 4.8, 4.8), -5, 5, m, { ax: 'y', cover: true, w: 2.1 });
  // ---- wrist: housing with the bevel gear (rolls with J4), two cheeks, J5 nose, flange
  sbGroup('wr', F.T4, [-1, -3.4, -3.4], [.8, 3.4, 3.4]);
  sbPart('bevel'); qRod(tAp(F.T4, [-.7, 0, 0]), tAp(F.T4, [.6, 0, 0]), 1.7, BRASS, { r1: .9, w: .7 });
  sbPart('wrist'); qCyl(F.T4, 'x', [0, 0], -1, .8, 3.4, m, { sh: tone, cover: true, w: 1.6 });
  sbFree();
  for (const [y0, y1] of [[2.6, 4], [-4, -2.6]]) qExt(F.T4, rrUV(.8, -2.8, 5.6, 2.8, 2.6), y0, y1, m, { ax: 'y', w: 1.3 });
  qExt(F.T5, rrUV(-2.4, -2.4, A6.FL, 2.4, 2.4), -2.4, 2.4, m, { ax: 'y', w: 1.5 });
  sbPart('flange6'); qCyl(F.T6, 'x', [0, 0], 0, .8, 3, STEEL, { w: 1 });
  // ---- gripper: pneumatic parallel jaws
  sbPart('gripper'); qExt(F.T6, rrUV(-3.6, -2, 3.6, 2, 1), .8, 4.6, GRIP, { ax: 'x', w: 1.4 });
  sbPart('fingers'); const gq = st.grip;
  for (const s of [1, -1]){ const lo = s > 0 ? [4.6, gq, -1.1] : [4.6, -gq - 1.1, -1.1], hi = s > 0 ? [8.6, gq + 1.1, 1.1] : [8.6, -gq, 1.1];
    qBox(F.T6, lo, hi, ALU, { w: .8 });
    qBox(F.T6, s > 0 ? [6.6, gq - .3, -1] : [6.6, -gq, -1], s > 0 ? [8.6, gq, 1] : [8.6, -gq + .3, 1], PAD, { w: .5 }); }
  sbPart('hoses'); for (const s of [1, -1]){ const a = tAp(F.T6, [2.6, s*1.2, 2.05]), b = tAp(F.T3, [-4, s*1.4, 4.9]), mid = vAdd(vLerp(a, b, .5), [0, 0, 6]);
    qWire([a, vLerp(a, mid, .5), mid, vLerp(mid, b, .5), b], CABLE.air, .3, { seg: 6 }); }
  // ---- controller cabinet + teach pendant + harness on the plate
  sbPart('ctrl'); qBox(T_I, [5, 52, 3], [19, 67, 24], CASE, { sh: tone, w: 1.6, deco: n => n[0] === 1 ?
      shape([[19.05, 54, 18], [19.05, 65, 18], [19.05, 65, 22], [19.05, 54, 22]], '#1d2a26', .7) + txT(T_I, [19.1, 59.5, 19.3], [0,-1,0], [0,0,1], st.mode === 'op' ? 'SERVO ON' : 'READY', 1.5, '#8af0ae')
      + [8, 10, 12, 14].map(z => line([[19.05, 55, z], [19.05, 64, z]], .6)).join('') + dotAt([19.1, 64, 16], .5, st.mode === 'op' ? '#43c25c' : '#8d877a') : '' });
  qRod([12, 64, 24], [12, 64, 24.8], 1.6, matOf('#f4b829'), { w: .7 }); qRod([12, 64, 24.8], [12, 64, 25.8], 1.2, matOf('#e2483d'), { w: .8 });
  sbPart('pendant'); qBox(T_I, [24, 58, 3], [32, 66, 4.4], DARK, { sh: tone, w: 1, deco: n => n[2] === 1 ? shape([[25, 59.5, 4.42], [29, 59.5, 4.42], [29, 64.5, 4.42], [25, 64.5, 4.42]], '#1d2a26', .5) + dotAt([30.6, 62, 4.45], .55, '#e2483d') : '' });
  qWire([[24, 62, 3.6], [21, 62, 3.4], [19.2, 61, 5]], DARK.l, .3);
  sbPart('harness'); qWire([[19.2, 55, 5], [24, 52, 3.5], [29, 40, 3.5], [BX-15, BY+1, 5.5]], CABLE.sig, .5, { seg: 5 });
  // ---- the work block (it can be picked up)
  sbPart('block'); const bp = st.carry ? vAdd(F.tip, [0, 0, -1.6]) : st.blockAt;
  qBox(T_I, [bp[0]-1.6, bp[1]-1.6, bp[2]-1.6], [bp[0]+1.6, bp[1]+1.6, bp[2]+1.6], matOf('#e2483d'), { sh: tone, w: 1 });
  // ---- overlays
  let over = '';
  if (st.mode === 'op'){
    if (st.trace) over += trailSVG(st.trail, '#e2483d', 1);
    over += tipHandle(F.tip, 3, !!st.crash);
    if (st.crash) over += crashBurst(st.crash.at, st.crash.t);
  }
  const reach = on(PZ(3.05), ellUV(BX, BY, A6.L2 + A6.LF, A6.L2 + A6.LF, 64));
  const floor = st.mode === 'op' ? clipTo(boardTop(0, 0, 84, 72, 3, 6), `<path d="${pd(reach)}" fill="none" stroke="#8f8774" stroke-width=".8" stroke-dasharray="2 2"/>`) : '';
  return { under: board(0, 0, 84, 72, 3, 6), z0: 3, clip: boardTop(0, 0, 84, 72, 3, 6), floor, over };
}

function a6Tick(st, dt, ui, lab){
  let moving = false; const nq = st.q.slice();
  for (let i = 0; i < 6; i++){ const d = st.qt[i] - st.q[i]; if (Math.abs(d) > 1e-4){ nq[i] += Math.sign(d)*Math.min(Math.abs(d), A6_VMAX[i]*st.speed*dt); moving = true; } }
  if (moving){ const c = a6Check(nq);
    if (c){ st.qt = st.q.slice(); st.qd = [0,0,0,0,0,0]; st.crash = { at: c.at, t: 1 }; st.crashes++; st.play = false; lab.flash(c.why + ' — motion stopped'); }
    else { st.qd = nq.map((v, i) => (v - st.q[i]) / Math.max(dt, 1e-3)); st.q = nq;
      const tp = a6FK(st.q).tip; const last = st.trail[st.trail.length - 1]; if (!last || vLen(sub(tp, last)) > .6){ st.trail.push(tp); if (st.trail.length > 400) st.trail.shift(); } } }
  else st.qd = [0,0,0,0,0,0];
  // gripper jaws
  const gd = st.gripT - st.grip; if (Math.abs(gd) > 1e-3){ st.grip += Math.sign(gd)*Math.min(Math.abs(gd), 6*dt); moving = true;
    if (st.gripT < 1 && st.grip <= 1.65 && !st.carry){ const tp = a6FK(st.q).tip; if (vLen(sub(vAdd(tp, [0, 0, -1.6]), st.blockAt)) < 2.2){ st.carry = true; st.gripT = st.grip = 1.6; lab.flash('Gripped the block'); } } }
  // playback of taught poses
  if (st.play && !moving){ if (st.hold > 0) st.hold -= dt; else { st.pi = (st.pi + 1) % st.poses.length; const pz = st.poses[st.pi]; st.qt = pz.q.slice(); st.gripT = pz.g; st.hold = .35; } moving = true; }
  if (st.crash){ st.crash.t -= dt/1.2; if (st.crash.t <= 0) st.crash = null; else moving = true; }
  return moving || st.dragging;
}

ROBOTS.arm6 = {
  id: 'arm6', fig: '01', name: 'Six-axis industrial arm', pal: 'mustard', kicker: '6 DOF · AC servos · RV + harmonic reducers',
  blurb: 'A small factory arm: six rotating joints, each a servo motor plus a precision gearbox. Build it from the base flange up to the gripper, then jog each joint or drag the tool tip and let inverse kinematics solve the angles.',
  size: [84, 72, 74], center: [42, 36], bounds: [24, 66, 14, 56, 0, 66],
  init: () => ({ q: A6_HOME.slice(), qt: A6_HOME.slice(), qd: [0,0,0,0,0,0], grip: 2.4, gripT: 2.4, speed: 1, payload: 1, trail: [], trace: true, crash: null, crashes: 0,
    poses: [], play: false, pi: -1, hold: 0, carry: false, blockAt: [A6_BLOCK0[0], A6_BLOCK0[1], 4.6], ik: true }),
  steps: [
    { title: 'Base flange & anchor bolts', parts: ['flange', 'anchors'], text: 'Everything starts with a stiff foundation. The cast flange is bolted to the plate with four anchor bolts; when the arm swings a payload at full reach, these bolts take the tipping moment.' },
    { title: 'J1 drive: servo, gears, reducer', parts: ['j1motor', 'j1gear', 'j1rv', 'bearing'], text: 'The J1 servo sits off-centre and turns a small pinion that meshes with a big ring gear (a first reduction). The ring gear drives an RV cycloidal reducer, and a cross-roller bearing carries the whole arm above it.' },
    { title: 'Base housing', parts: ['drum'], text: 'The housing closes the base. Turn on See-through to keep watching the J1 gear train inside it.' },
    { title: 'Turret', parts: ['turret'], text: 'The turret bolts to the reducer output and spins on the bearing: this is joint 1, the waist. Its column carries the shoulder axis 30 cm above the plate.' },
    { title: 'J2 shoulder drive', parts: ['j2rv', 'j2motor', 'j2housing'], text: 'The shoulder has the hardest job: it lifts everything beyond it. A large RV reducer (121:1) multiplies the servo torque; a spring-applied brake holds the arm when power is off.' },
    { title: 'Upper arm', parts: ['ua'], text: 'A hollow cast link, 30 cm between the shoulder and elbow axes. Hollow castings are stiff for their weight and give the cables a protected path.' },
    { title: 'J3 elbow drive', parts: ['j3hd', 'j3motor', 'j3housing'], text: 'At the elbow a strain-wave (harmonic) drive gives 101:1 in a thin package with near-zero backlash, so the forearm does not wobble.' },
    { title: 'Forearm casting', parts: ['fa'], text: 'The forearm turns on the elbow. Its rear bulge is a counterweight and the home for the three wrist motors.' },
    { title: 'Wrist drives (inside the forearm)', parts: ['wmotors', 'wgears', 'belts', 'tube'], peek: true, text: 'Putting the wrist motors at the back keeps weight off the wrist. J4 turns the hollow torque tube through a gear pair; J5 and J6 turn shafts nested inside it through timing belts.' },
    { title: 'Wrist unit & tool flange', parts: ['wrist', 'bevel', 'flange6'], text: 'The wrist rolls (J4), bends (J5, through a bevel gear that turns the drive 90 degrees) and spins the flange (J6). Three wrist axes meeting at one point is what makes the IK solvable in closed form.' },
    { title: 'Cable harness', parts: ['harness', 'conn'], peek: true, text: 'Power and encoder cables enter at the base connector, climb the hollow J1 axis, and run inside each casting with slack loops at the joints so they bend instead of twisting.' },
    { title: 'Controller & teach pendant', parts: ['ctrl', 'pendant'], text: 'The cabinet holds six servo amplifiers and the motion computer that runs the kinematics 1000+ times a second. The pendant lets a person jog joints and record poses; its red button cuts power to the motors.' },
    { title: 'Gripper & air hoses', parts: ['gripper', 'fingers', 'hoses'], text: 'A pneumatic parallel gripper bolts to the ISO flange. Two air lines (open / close) run back along the forearm.' },
    { title: 'Work block', parts: ['block'], text: 'Something to pick up. In Operate mode, drive the jaws around it and close the gripper.' },
  ],
  parts: {
    flange: { name: 'Base flange', kind: 'structure', lbl: 'BASE FLANGE', short: 'cast iron foundation', info: 'A thick cast disc that spreads the arm’s loads into the mounting plate. It is the reference everything else is measured from.', held: 'The mounting plate, by the anchor bolts', spec: '290 mm dia, 14 mm thick' },
    anchors: { name: 'Anchor bolts', kind: 'structure', short: 'M12 high-tensile bolts', info: 'Four bolts resist the tipping moment when the arm swings a load. Under-torqued bolts are a classic cause of arm vibration.', held: 'Threaded into the plate', spec: '4 x M12 class 12.9' },
    j1motor: { name: 'J1 servo motor', kind: 'motor', lbl: 'J1 SERVO', inside: true, short: '400 W AC servo + encoder', info: 'A brushless AC servo with a 17-bit absolute encoder on its back (131 072 counts per turn). The amplifier closes current, speed and position loops around it.', held: 'The base flange, inside the housing', spec: '400 W, 3000 rpm, 1.3 N m' },
    j1gear: { name: 'Pinion & ring gear', kind: 'gear', lbl: 'PINION + RING GEAR', inside: true, short: 'first reduction', info: 'The small motor pinion turns the big ring gear: a first speed reduction (and torque gain) before the RV reducer, and it lets the motor sit off-centre so the middle stays hollow for cables.', held: 'Pinion on the motor shaft; ring gear on the RV input', spec: '10 : 26 teeth' },
    j1rv: { name: 'J1 RV reducer', kind: 'gear', lbl: 'RV REDUCER', inside: true, short: 'cycloidal, 121:1 total', info: 'A cycloidal disc rolls inside a ring of pins, so many teeth share the load at once. Very stiff and shock-tolerant, which is why big joints use RV reducers.', held: 'The base flange; its output carries the turret', spec: '121:1 overall, backlash under 1 arc-min' },
    bearing: { name: 'Cross-roller bearing', kind: 'structure', lbl: 'CROSS-ROLLER BEARING', inside: true, short: 'takes all loads at once', info: 'Rollers alternate at 90 degrees in one V-groove, so one thin bearing takes up/down, sideways and tipping loads together. The whole arm stands on it.', held: 'Between base housing and turret', spec: 'Thin section, preloaded' },
    drum: { name: 'Base housing', kind: 'cover', lbl: 'BASE HOUSING', short: 'cast cover, IP54 sealed', info: 'A cast aluminium shell that keeps dust and coolant out of the J1 drive. Turn on See-through to look inside.', held: 'Bolted to the base flange' },
    turret: { name: 'Turret (J1 waist)', kind: 'structure', lbl: 'TURRET', key: true, short: 'rotates on joint 1', info: 'The rotating column. Joint 1 swings everything about the vertical axis: ±170 degrees, stopped short of a full turn so the cables are never wound up.', held: 'The RV reducer output, on the cross-roller bearing', spec: 'J1 range ±170°, 90°/s' },
    j2rv: { name: 'J2 RV reducer', kind: 'gear', lbl: 'J2 RV REDUCER', inside: true, short: '121:1, highest torque', info: 'The shoulder lifts the whole arm plus payload, so it gets the biggest reducer. Watch the J2 torque bar in Operate mode as you stretch the arm out.', held: 'The turret column', spec: '121:1, rated 140 N m' },
    j2motor: { name: 'J2 servo + brake', kind: 'motor', lbl: 'J2 SERVO + BRAKE', inside: true, short: '750 W servo, holding brake', info: 'A spring-applied brake clamps the shaft whenever power is off, so the arm does not fall. The encoder on the end tells the controller the exact shaft angle.', held: 'The J2 housing', spec: '750 W, 24 V brake' },
    j2housing: { name: 'J2 housing', kind: 'cover', lbl: 'J2 HOUSING', short: 'shoulder drive cover', info: 'Encloses the shoulder servo and reducer on the turret side opposite the upper arm.', held: 'Bolted to the turret' },
    ua: { name: 'Upper arm', kind: 'structure', lbl: 'UPPER ARM', key: true, short: 'hollow casting, 300 mm', info: 'The link between shoulder (J2) and elbow (J3). Its length L2 = 30 cm is one of the two numbers in the arm’s inverse kinematics triangle.', held: 'The J2 reducer output', spec: 'L2 = 300 mm, 5.5 kg' },
    j3hd: { name: 'J3 harmonic drive', kind: 'gear', lbl: 'J3 HARMONIC DRIVE', inside: true, short: 'strain wave, 101:1', info: 'An elliptical wave generator flexes a thin steel cup (the flexspline) against a rigid ring with 2 more teeth. One motor turn moves the output by just 2 teeth: 101:1 with zero backlash.', held: 'The upper arm’s far end', spec: '101:1, rated 60 N m' },
    j3motor: { name: 'J3 servo', kind: 'motor', lbl: 'J3 SERVO', inside: true, short: '400 W servo', info: 'Drives the elbow through the harmonic drive.', held: 'The J3 housing' },
    j3housing: { name: 'J3 housing', kind: 'cover', lbl: 'J3 HOUSING', short: 'elbow drive cover', info: 'Covers the elbow drive on the forearm side.', held: 'The forearm' },
    fa: { name: 'Forearm', kind: 'structure', lbl: 'FOREARM', key: true, short: 'casting, 260 mm', info: 'Elbow to wrist. The rear bulge holds the wrist motors so their weight sits near the elbow, reducing the torque J2 and J3 must provide.', held: 'The J3 harmonic drive output', spec: 'L3 = 260 mm' },
    wmotors: { name: 'Wrist servos J4 J5 J6', kind: 'motor', lbl: 'WRIST SERVOS J4-J6', inside: true, short: 'three 100 W servos', info: 'Mounted at the back of the forearm, far from the wrist, to keep the moving mass low.', held: 'The forearm rear housing', spec: '3 x 100 W' },
    wgears: { name: 'J4 gear pair', kind: 'gear', lbl: 'J4 GEAR PAIR', inside: true, short: 'motor to torque tube', info: 'A spur pinion on the J4 motor turns a larger gear on the torque tube, rolling the whole wrist.', held: 'Pinion on J4 motor; gear on the torque tube' },
    belts: { name: 'J5 / J6 timing belts', kind: 'gear', lbl: 'TIMING BELTS', inside: true, short: 'toothed belts to inner shafts', info: 'Toothed belts never slip, so the motor encoder still knows the exact joint angle. They let the motors sit off-axis while driving shafts on the centre line.', held: 'Pulleys on the J5/J6 motors and the nested shafts' },
    tube: { name: 'Torque tube (concentric shafts)', kind: 'gear', lbl: 'TORQUE TUBE', inside: true, short: 'three nested shafts', info: 'A hollow tube (J4) with two more shafts inside it (J5, J6), all on the forearm axis. Turning J4 also turns the inner shafts a little; the controller compensates for this coupling.', held: 'Bearings in the forearm' },
    wrist: { name: 'Wrist unit', kind: 'structure', lbl: 'WRIST', key: true, short: 'J4 roll + J5 bend', info: 'The wrist housing rolls with J4; between its cheeks the J5 nose bends ±120 degrees. J4, J5 and J6 axes all pass through one point: the wrist centre.', held: 'The torque tube end', spec: 'J5 range ±120°' },
    bevel: { name: 'Wrist bevel gear', kind: 'gear', lbl: 'BEVEL GEAR', inside: true, short: 'turns drive 90°', info: 'Conical gears turn the J5 shaft’s rotation through a right angle so it can bend the wrist.', held: 'Inside the wrist housing' },
    flange6: { name: 'J6 tool flange', kind: 'structure', lbl: 'TOOL FLANGE', short: 'ISO 9409 bolt circle', info: 'The standard bolt pattern every tool mounts to. J6 spins it ±270 degrees.', held: 'The J5 nose', spec: 'ISO 9409-1-31.5-4-M5' },
    harness: { name: 'Cable harness', kind: 'wiring', lbl: 'HARNESS', inside: true, short: 'power + encoder cables', info: 'Each servo needs 3 power wires, a brake pair and an encoder bus. Routing them through the hollow joints keeps them safe and stops them snagging.', held: 'Clamps inside each casting' },
    conn: { name: 'Base connector', kind: 'wiring', short: 'harness plug', info: 'One heavy connector joins the arm to the controller cable, so the arm can be swapped quickly.', held: 'The base flange' },
    ctrl: { shadow: 'ctrl', name: 'Controller cabinet', kind: 'electronics', lbl: 'CONTROLLER', key: true, short: 'servo amps + motion CPU', info: 'Runs inverse kinematics and trajectory planning, then sends position targets to six servo amplifiers every millisecond. The red mushroom button is the emergency stop.', held: 'The plate' },
    pendant: { shadow: 'pendant', name: 'Teach pendant', kind: 'electronics', lbl: 'TEACH PENDANT', short: 'jog + record poses', info: 'The handheld that operators use to jog joints and record waypoints (try Save pose / Play in Operate mode). A three-position enable switch must be held for the arm to move.', held: 'On a cable to the controller' },
    gripper: { name: 'Pneumatic gripper', kind: 'tool', lbl: 'GRIPPER', key: true, short: 'parallel jaws', info: 'A piston pushes two jaws together on linear guides. Air pressure sets the grip force; there is no position control, it is either open or closed.', held: 'The tool flange', spec: '6 bar, 120 N grip' },
    fingers: { name: 'Gripper fingers', kind: 'tool', short: 'with rubber pads', info: 'Custom aluminium fingers with rubber pads; the tool centre point (TCP) is between their tips.', held: 'The gripper jaws' },
    hoses: { name: 'Air hoses', kind: 'wiring', short: 'open / close lines', info: 'Two 4 mm air lines. They run outside the wrist with slack so J4–J6 can turn.', held: 'Clips on the forearm' },
    block: { shadow: 'block', name: 'Work block', kind: 'tool', short: '32 mm cube', info: 'A test part. Put the jaws round it (tip about 1.5 cm above the table) and press Close gripper.', held: 'The plate, or the gripper' },
  },
  concepts: [
    ['Degrees of freedom', 'Six joints give six DOF: three to put the tool anywhere in space (J1–J3) and three to point it in any direction (J4–J6). Fewer than six and some poses become impossible; more than six and there are infinitely many ways to reach each pose.'],
    ['Forward kinematics', 'Multiply one rotation + offset per joint, base to tip, and you get where the tool is. That is what draws this arm: each link is drawn in the frame left by the joint before it. Move a slider and watch the tip coordinates.'],
    ['Inverse kinematics', 'Drag the yellow ring on the tool tip. The lab finds J1 from the direction to the target, then solves the shoulder–elbow triangle (law of cosines with L2 and L3) and sets J5 so the tool keeps pointing down. Targets beyond L2 + L3 have no solution.'],
    ['Torque & gear reduction', 'Gravity torque = weight x horizontal distance. Stretch the arm out with a heavy payload and the J2 bar climbs. A 121:1 reducer lets a 1.3 N m motor hold over 140 N m at the joint, but the motor must spin 121x faster: see the motor rpm column.'],
    ['Joint limits & singularities', 'Every joint has hard stops (cables, castings). When J5 is near 0 the J4 and J6 axes line up: a wrist singularity, where two motors do the same thing and the IK can demand huge speeds.'],
    ['Teach & playback', 'Most industrial arms are programmed by jogging to poses and recording them. Save a few poses and press Play: the controller moves joint by joint, each limited to its maximum speed.'],
  ],
  draw: a6Draw,
  active: st => { const a = {}; const names = ['j1motor', 'j2motor', 'j3motor', 'wmotors', 'wmotors', 'wmotors']; (st.qd || []).forEach((v, i) => { if (Math.abs(v) > .5) a[names[i]] = true; });
    if (a.j1motor) a.j1gear = a.j1rv = true; if (a.j2motor) a.j2rv = true; if (a.j3motor) a.j3hd = true; if (a.wmotors) a.wgears = a.belts = a.tube = true; return a; },
  enterOp: st => { st.qt = st.q.slice(); },
  opHTML: () => `<div class="seg" id="a6mode"><button data-m="1">Drag tip (IK)</button><button data-m="0">Joint sliders (FK)</button></div>
    <p class="note" id="a6hint"></p>
    ${A6_NAMES.map((n, i) => slHTML('a6j' + i, n, A6_LIM[i][0], A6_LIM[i][1], 1, '°', 'Joint ' + (i+1))).join('')}
    ${slHTML('a6pay', 'Payload in gripper', 0, 8, .5, ' kg', 'Mass held at the tool tip')}
    ${slHTML('a6spd', 'Speed override', 10, 100, 5, '%', 'Scales every joint speed')}
    <div class="row2"><button id="a6grip">Close gripper</button><button id="a6home">Home</button><button id="a6save">Save pose</button><button id="a6play">Play poses</button><button id="a6clr">Clear</button><button id="a6tr" aria-pressed="true">Trace</button></div>`,
  opBind(el, st, ui, lab){
    const $ = id => document.getElementById(id);
    el.querySelectorAll('#a6mode button').forEach(b => b.onclick = () => { st.ik = b.dataset.m === '1'; ui.render(); });
    for (let i = 0; i < 6; i++) $('s_a6j' + i).oninput = e => { st.play = false; st.qt[i] = +e.target.value; lab.kick(); };
    $('s_a6pay').oninput = e => { st.payload = +e.target.value; ui.render(); };
    $('s_a6spd').oninput = e => { st.speed = +e.target.value / 100; ui.render(); };
    $('a6grip').onclick = () => { if (st.gripT > 2){ st.gripT = .4; } else { st.gripT = 2.4; if (st.carry){ st.carry = false; const tp = a6FK(st.q).tip; st.blockAt = [tp[0], tp[1], 4.6]; lab.flash('Released the block'); } } lab.kick(); ui.render(); };
    $('a6home').onclick = () => { st.play = false; st.qt = A6_HOME.slice(); lab.kick(); };
    $('a6save').onclick = () => { st.poses.push({ q: st.qt.slice(), g: st.gripT }); lab.flash(`Pose ${st.poses.length} saved`); ui.render(); };
    $('a6play').onclick = () => { if (st.poses.length < 2) return lab.flash('Save at least two poses first'); st.play = !st.play; st.pi = -1; st.hold = 0; lab.kick(); ui.render(); };
    $('a6clr').onclick = () => { st.poses = []; st.play = false; st.trail = []; ui.render(); };
    $('a6tr').onclick = () => { st.trace = !st.trace; ui.render(); };
  },
  opSync(el, st){
    const $ = id => document.getElementById(id);
    el.querySelectorAll('#a6mode button').forEach(b => b.setAttribute('aria-pressed', (b.dataset.m === '1') === st.ik));
    $('a6hint').textContent = st.ik ? 'Drag the yellow ring at the tool tip. The tool stays pointing down; the sliders follow the solved angles.' : 'Move one joint at a time and watch how far the tip travels: small shoulder moves swing the tip a long way.';
    for (let i = 0; i < 6; i++) slSync('a6j' + i, st.qt[i], `${st.q[i].toFixed(1)}°`, Math.abs(st.q[i] - A6_LIM[i][0]) < .6 || Math.abs(st.q[i] - A6_LIM[i][1]) < .6);
    slSync('a6pay', st.payload, st.payload.toFixed(1) + ' kg', st.payload > 5);
    slSync('a6spd', Math.round(st.speed*100), Math.round(st.speed*100) + '%');
    $('a6grip').textContent = st.gripT > 2 ? 'Close gripper' : 'Open gripper';
    $('a6play').textContent = st.play ? 'Stop' : `Play poses (${st.poses.length})`; $('a6play').setAttribute('aria-pressed', st.play);
    $('a6tr').setAttribute('aria-pressed', st.trace);
  },
  readout(st){
    const F = a6FK(st.q), t = a6Torques(st.q, st.payload), rows = [];
    for (let i = 0; i < 6; i++){ const rpm = Math.abs(st.qd[i]) / 6 * A6_RATIO[i];
      rows.push({ k: `J${i+1} ${st.q[i].toFixed(0)}°`, bar: Math.abs(st.qd[i]) / A6_VMAX[i], v: `${Math.abs(st.qd[i]).toFixed(0)}°/s · ${rpm.toFixed(0)} rpm`, tip: `Joint speed and motor speed through ${A6_RATIO[i]}:1` }); }
    const rel = sub(F.tip, [A6.BX, A6.BY, 3]);
    rows.push({ k: 'Tip X Y Z', v: rel.map(v => (v*10).toFixed(0)).join(' ') + ' mm', tip: 'Tool centre point relative to the base, millimetres' });
    rows.push({ k: 'Reach used', bar: Math.hypot(rel[0], rel[1]) / (A6.L2 + A6.LF + A6.DT), v: Math.round(Math.hypot(rel[0], rel[1]) * 10) + ' mm' });
    ['J2', 'J3', 'J5'].forEach((n, i) => rows.push({ k: `${n} gravity torque`, bar: t[i] / A6_RATED[i], v: `${t[i].toFixed(1)} N m`, warn: t[i] > A6_RATED[i], tip: `Rated ${A6_RATED[i]} N m` }));
    rows.push({ k: 'Crashes', v: String(st.crashes), warn: !!st.crash });
    return rows;
  },
  stats: st => { const rel = sub(a6FK(st.q).tip, [A6.BX, A6.BY, 3]); return [{ label: 'Tool tip mm', value: rel.map(v => Math.round(v*10)).join(' '), small: true }]; },
  tick: a6Tick,
  onDrag(st, dx, dy, phase, lab){
    if (phase === 'start'){ st.play = false; st.dragP = a6FK(st.qt).tip; if (!st.ik){ st.ik = true; } return; }
    if (phase !== 'move') return;
    const want = vAdd(st.dragP, dragWorld(dx, dy)), r = a6IK(want, st.qt[5]);
    if (r.err){ lab.flash(r.err); st.limitHit = true; return; }
    st.dragP = want; st.qt = r.q; },
  params(st, q){ if (q.has('q')){ const v = q.get('q').split(',').map(Number); if (v.length === 6) st.q = st.qt = v; } if (q.has('pay')) st.payload = +q.get('pay'); },
};
ROBOT_ORDER.push({ id: 'arm6', name: 'Six-axis industrial arm', blurb: 'Waist, shoulder, elbow and a three-axis wrist, each a servo and a precision reducer. Jog joints, drag the tip with inverse kinematics, watch gravity torque climb.', chips: ['6 DOF', 'AC servos', 'RV + harmonic', 'IK drag'] });
