/* ============================================================
   Robot 10 — humanoid upper body on a stand (two 4-DOF arms, 2-DOF neck, stereo camera head). 1 unit = 1 cm.
   Truth: a column holds the pelvis; an aluminium spine frame inside the chest carries the battery, the computer and the
   shoulder actuators. Each shoulder is three actuators in a row (pitch, roll, then yaw along the upper arm) whose axes
   meet at one point, so it works like a ball joint; the elbow is a fourth. The neck pans and tilts a head with two
   cameras a few centimetres apart (depth from disparity). Every joint is a compact actuator: motor, gear, encoder,
   current sensing, so the arm can limit its force near people.
   Four joints to place a hand at a point (3 numbers) leaves one spare: the elbow can swing round a circle while the hand
   stays still. That is redundancy.
   Parts: stand | spine, battery, pc | torso | shP, shR | shY, upper | elbow, fore, hand | neckP, neckT | head, cams | harness | ball
   ============================================================ */
const HU = { C: 30, SZ: 70, SY: 15.5, L1: 20, L2: 18, H: 7, NZ: 77, VMAX: 120 };
const HU_LIM = [[-60, 175], [-60, 130], [-95, 95], [0, 145]];
const HU_JN = ['Shoulder pitch', 'Shoulder roll', 'Shoulder yaw', 'Elbow'];
const HU_REST = [10, 8, 0, 25];
const HU_BALL0 = [56, 26, 62];

const huS = s => [HU.C, HU.C + s*HU.SY, HU.SZ];
function huFK(q, s){ const T1 = tMul(tTr(...huS(s)), tRy(-q[0]*DEG)), T2 = tMul(T1, tRx(s*q[1]*DEG)), T3 = tMul(T2, tRy(90*DEG)), T4 = tMul(T3, tRx(s*q[2]*DEG));
  const T5 = tCat(T4, tTr(HU.L1, 0, 0), tRy(-q[3]*DEG));
  return { T1, T2, T3, T4, T5, E: tAp(T5, [0,0,0]), W: tAp(T5, [HU.L2, 0, 0]), tip: tAp(T5, [HU.L2 + HU.H, 0, 0]) }; }
// hand (tip) at w, elbow swung phi degrees round the shoulder-hand line (0 = lowest); returns joint angles
function huIK(w, s, phi, prevYaw = 0){
  const S = huS(s), d0 = sub(w, S), d = vLen(d0), L2 = HU.L2 + HU.H;
  if (d > HU.L1 + L2 - .2) return { err: 'Out of reach: the arm is straight' }; if (d < 8) return { err: 'Too close to the shoulder' };
  const u = vSc(d0, 1/d), down = [0, 0, -1], n1r = sub(down, vSc(u, dot(down, u))), n1 = vLen(n1r) > 1e-3 ? norm(n1r) : norm(cross(u, [1, 0, 0])), n2 = vSc(cross(u, n1), s);
  const ca = (HU.L1**2 + d*d - L2**2) / (2*HU.L1*d), al = Math.acos(clamp(ca, -1, 1)), ph = phi*DEG;
  const E = vAdd(vAdd(S, vSc(u, HU.L1*Math.cos(al))), vSc(vAdd(vSc(n1, Math.cos(ph)), vSc(n2, Math.sin(ph))), HU.L1*Math.sin(al)));
  const a = vSc(sub(E, S), 1/HU.L1), b = norm(sub(w, E));
  const r = Math.asin(clamp(a[1], -1, 1))*s/DEG, p = Math.atan2(a[0], -a[2])/DEG;
  const e = Math.acos(clamp(dot(a, b), -1, 1))/DEG;
  const T3 = huFK([p, r, 0, 0], s).T3, x3 = tAx(T3, 0), y3 = tAx(T3, 1), z3 = tAx(T3, 2), v = sub(b, vSc(x3, dot(b, x3)));
  const yaw = vLen(v) < 1e-4 ? prevYaw : s*Math.atan2(-dot(v, y3), dot(v, z3))/DEG;
  const q = [p, r, yaw, e];
  for (let i = 0; i < 4; i++) if (q[i] < HU_LIM[i][0] || q[i] > HU_LIM[i][1]) return { err: `${s > 0 ? 'Left' : 'Right'} ${HU_JN[i].toLowerCase()} would pass its limit`, q };
  return { q }; }
// hands and elbows must stay out of the torso, the head and each other's way
function huSelf(qL, qR){ const pts = [];
  for (const [q, s] of [[qL, 1], [qR, -1]]){ const F = huFK(q, s); for (const k of [.5, 1]) pts.push(vLerp(F.E, F.tip, k)); pts.push(F.E); }
  for (const p of pts){ const x = p[0] - HU.C, y = p[1] - HU.C;
    if (Math.abs(x) < 9.5 && Math.abs(y) < 11.5 && p[2] > 40 && p[2] < 75) return { why: 'Self-collision: a hand hit the torso', at: p };
    if (Math.hypot(x, y) < 7.5 && p[2] > 76 && p[2] < 93) return { why: 'Self-collision: a hand hit the head', at: p }; }
  const a = huFK(qL, 1).tip, b = huFK(qR, -1).tip; if (vLen(sub(a, b)) < 4) return { why: 'The hands collided', at: vLerp(a, b, .5) };
  return null; }

function huArm(st, q, s, m){ const F = huFK(q, s), side = v => s*v;
  sbPart('shP'); qCyl(T_I, 'y', [HU.C, HU.SZ], HU.C + side(11.4), HU.C + side(15.6), 3.6, MOTOR, { w: 1.1, sh: tone });
  sbPart('shR'); qCyl(F.T1, 'x', [s*HU.SY*0, 0], -3.2, 3.2, 3.1, MOTOR, { w: 1, sh: tone });
  for (const x of [-3.2, 2.6]) qBox(F.T1, [x, Math.min(side(-1), side(2.6)), -2], [x + .6, Math.max(side(-1), side(2.6)), 2], m, { w: .7, sh: tone });
  sbPart('shY'); qCyl(F.T4, 'x', [0, 0], 3, 8, 2.6, MOTOR, { w: 1, sh: tone });
  sbPart('upper'); qExt(F.T4, ellUV(0, 0, 2.6, 2.6, 14), 8, HU.L1 - 2.6, m, { w: 1.5 });
  sbPart('elbow'); qCyl(F.T4, 'y', [HU.L1, 0], -2.4, 2.4, 2.6, MOTOR, { w: 1, sh: tone });
  sbPart('fore'); qExt(F.T5, ellUV(0, 0, 2.2, 2.2, 14), 2.4, HU.L2, m, { w: 1.4 });
  sbPart('hand'); const Tw = tMul(F.T5, tTr(HU.L2, 0, 0)); qBox(Tw, [0, -2.2, -1], [4, 2.2, 1], CERAMIC, { w: .9, sh: tone });
  for (let k = 0; k < 4; k++){ const y = -1.65 + k*1.1; qBox(Tw, [4, y - .4, -.7], [HU.H - (k === 0 || k === 3 ? .8 : 0), y + .4, .7], CERAMIC, { w: .6, sh: tone }); }
  qBox(Tw, [1, s > 0 ? 2.2 : -3.2, -.6], [3.8, s > 0 ? 3.2 : -2.2, .6], CERAMIC, { w: .6, sh: tone });
  sbPart('harness'); qWire([[HU.C - 3, HU.C + side(10), 68], tAp(F.T1, [0, side(2), 3.4]), tAp(F.T4, [5, 0, s*2.8]), tAp(F.T4, [HU.L1 - 3, 0, s*2.8])], '#3a3633', .3, { seg: 4 });
  return F; }

function huDraw(st, m){
  const C = HU.C;
  // ---- stand: base, column, pelvis
  sbPart('stand'); qRod([C, C, 3], [C, C, 4.6], 14, STEEL, { w: 1.4 }); qRod([C, C, 4.6], [C, C, 40], 2.6, ALU, { w: 1.2 }); qBox(T_I, [C - 6, C - 8, 40], [C + 6, C + 8, 46], DARK, { w: 1.1 });
  // ---- chest: spine frame, battery, computer, shoulder actuators inside the torso cover
  sbGroup('torso', T_I, [C - 9, C - 12, 46], [C + 9, C + 12, 74]);
  sbPart('spine'); for (const y of [-4, 4]) qBox(T_I, [C - 6, C + y - .6, 46], [C - 4.8, C + y + .6, 73], ALU, { w: .8 }); qBox(T_I, [C - 6, C - 11, 68], [C - 2, C + 11, 70.4], ALU, { w: .8 });
  sbPart('battery'); qBox(T_I, [C - 4.6, C - 6, 47], [C + 4, C + 6, 54], BATT, { w: .9, deco: n => n[0] === 1 ? txT(T_I, [C + 4.02, C, 50.5], [0,1,0], [0,0,1], '48V', 1.6, '#fff') : '' });
  sbPart('pc'); qBox(T_I, [C - 4.6, C - 7, 56], [C + 2, C + 7, 62], DARK, { w: .9 }); qBox(T_I, [C + 2, C - 5, 57], [C + 3.2, C + 5, 61], ALU, { w: .6, deco: n => n[0] === 1 ? [-3.6, -1.2, 1.2, 3.6].map(y => line([[C + 3.22, C + y, 57.5], [C + 3.22, C + y, 60.5]], .45)).join('') : '' });
  sbPart('harness'); qWire([[C + 1, C - 2, 54], [C + 2, C - 3, 56.5], [C + 1, C - 3, 61]], CABLE.pwr, .35); qWire([[C - 1, C + 3, 62], [C - 3, C + 6, 66], [C - 3, C + 10, 69]], CABLE.sig, .3);
  sbPart('torso'); qExt(T_I, rrUV(C - 9, C - 12, C + 9, C + 12, 5), 46, 74, m, { ax: 'z', cover: true, w: 2.2, capDeco: b => b ? '' : '' });
  sbFree(); if (facing([1, 0, 0])) sbPut(txT(T_I, [C + 9.05, C, 66], [0,1,0], [0,0,1], 'HU-10', 2.6) + dotAt([C + 9.1, C, 62], .7, st.mode === 'op' ? '#43c25c' : '#8d877a'), obbW([C + 9, C - 5, 61], [C + 9.1, C + 5, 68]), { p: 'torso' });
  // ---- arms
  const FL = huArm(st, st.qL, 1, m), FR = huArm(st, st.qR, -1, m);
  // ---- neck and head
  sbFree(); sbPart('neckP'); qRod([C - 1, C, 74], [C - 1, C, 77], 3, MOTOR, { w: 1, sh: tone });
  const Tn = tMul(tTr(C - 1, C, HU.NZ), tRz(st.neck[0]*DEG)), Th = tMul(Tn, tRy(st.neck[1]*DEG));
  sbPart('neckT'); qCyl(Tn, 'y', [0, 1.4], -2.4, 2.4, 2, MOTOR, { w: 1, sh: tone });
  sbPart('head'); qExt(Th, rrUV(-6, -6.5, 6.5, 6.5, 3), 3, 15, m, { ax: 'z', w: 1.8, sh: tone });
  qBox(Th, [6.5, -5.4, 7], [7.1, 5.4, 12.6], DARK, { w: 1, sh: tone });
  sbPart('cams'); for (const y of [-2.8, 2.8]) qCyl(Th, 'x', [y, 10], 7.1, 7.9, 1.2, STEEL, { w: .8, deco: () => vis(tAx(Th, 0)) ? dotAt(tAp(Th, [7.92, y, 10]), .7, '#3a3633', .3) : '' });
  // ---- target ball
  sbPart('ball'); qBall(st.ball, 2.4, matOf('#f4b829'), { w: 1.2 });
  const pt = boardTop(0, 0, 60, 60, 3, 6); let over = '', floor = '';
  if (st.mode === 'op'){ const eye = tAp(Th, [7.9, 0, 10]), q0 = P(...eye), q1 = P(...st.ball);
    over += `<path d="M${f(q0[0])} ${f(q0[1])}L${f(q1[0])} ${f(q1[1])}" stroke="#5a82c6" stroke-width=".8" stroke-dasharray="1.4 1.2" opacity=".75" pointer-events="none"/>`;
    if (st.trace) over += trailSVG(st.trailR, '#e2483d', .9) + trailSVG(st.trailL, '#3d5878', .9);
    // the swivel circle: every elbow position that keeps the hand where it is
    for (const [s, on] of [[1, st.mode2 !== 'right'], [-1, st.mode2 !== 'left']]){ if (!on || st.mode2 === 'wave') continue; const F = s > 0 ? FL : FR, Sh = huS(s), d0 = sub(F.tip, Sh), d = vLen(d0), u = vSc(d0, 1/d), L2 = HU.L2 + HU.H;
      const ca = (HU.L1**2 + d*d - L2**2) / (2*HU.L1*d); if (Math.abs(ca) > 1) continue; const al = Math.acos(ca), c = vAdd(Sh, vSc(u, HU.L1*Math.cos(al))), rr = HU.L1*Math.sin(al);
      const n1 = norm(sub([0, 0, -1], vSc(u, -u[2]))), n2 = cross(u, n1), ring = [];
      for (let k = 0; k <= 36; k++){ const a = k/36*2*Math.PI; ring.push(vAdd(c, vAdd(vSc(n1, rr*Math.cos(a)), vSc(n2, rr*Math.sin(a))))); }
      over += trailSVG(ring, '#c76a12', .7, '1.2 1.2'); }
    over += tipHandle(st.ball, 3, !!st.crash); if (st.crash) over += crashBurst(st.crash.at, st.crash.t); }
  return { under: board(0, 0, 60, 60, 3, 6), z0: 3, clip: pt, floor, over };
}

function huTargets(st, lab){
  const b = st.ball, md = st.mode2;
  if (md === 'wave'){ const t = st.t; st.qtR = [35, 85, 20*Math.sin(t*5), 95 + 15*Math.sin(t*5)]; st.qtL = HU_REST.slice(); return; }
  const want = { left: md === 'left' || md === 'both', right: md === 'right' || md === 'both' };
  for (const [s, key, on] of [[1, 'L', want.left], [-1, 'R', want.right]]){ const tk = 'qt' + key;
    if (!on){ st[tk] = HU_REST.slice(); continue; }
    const w = md === 'both' ? vAdd(b, [0, s*3, 0]) : b, r = huIK(w, s, st.swivel[key], st[tk][2]);
    if (r.err){ st.reachErr[key] = r.err; continue; } st.reachErr[key] = null; st[tk] = r.q; } }
function huTick(st, dt, ui, lab){
  st.t += dt; huTargets(st, lab);
  let moving = false; const vmax = HU.VMAX*st.speed, step = (q, qt) => q.map((v, i) => { const d = qt[i] - v; if (Math.abs(d) < 1e-4) return v; moving = true; return v + Math.sign(d)*Math.min(Math.abs(d), vmax*dt); });
  const nL = step(st.qL, st.qtL), nR = step(st.qR, st.qtR), c = huSelf(nL, nR);
  if (c){ if (!st.crash){ st.crashes++; lab.flash(c.why + ' — stopped short'); } st.crash = { at: c.at, t: 1 }; st.qtL = st.qL.slice(); st.qtR = st.qR.slice(); }
  else { const pL = huFK(st.qL, 1).tip, pR = huFK(st.qR, -1).tip; st.qL = nL; st.qR = nR; const tL = huFK(nL, 1).tip, tR = huFK(nR, -1).tip;
    st.hv = Math.max(vLen(sub(tL, pL)), vLen(sub(tR, pR)))/Math.max(dt, 1e-3);
    for (const [arr, p] of [[st.trailL, tL], [st.trailR, tR]]){ const l = arr[arr.length - 1]; if (!l || vLen(sub(l, p)) > .5){ arr.push(p); if (arr.length > 200) arr.shift(); } } }
  if (!moving) st.hv = 0;
  // neck: pan and tilt toward the ball
  const N = [HU.C - 1, HU.C, HU.NZ + 10], d = sub(st.ball, N), pan = clamp(Math.atan2(d[1], d[0])/DEG, -80, 80), tilt = clamp(Math.atan2(-d[2], Math.hypot(d[0], d[1]))/DEG, -40, 50);
  const nn = [pan, tilt]; st.neck = st.neck.map((v, i) => { const e = nn[i] - v; if (Math.abs(e) > 1e-3) moving = true; return v + clamp(e, -150*dt, 150*dt); });
  st.gaze = Math.hypot(pan - st.neck[0], tilt - st.neck[1]);
  if (st.crash){ st.crash.t -= dt/1.2; if (st.crash.t <= 0) st.crash = null; else moving = true; }
  return moving || st.dragging || st.mode2 === 'wave' || !!st.crash;
}

ROBOTS.humanoid = {
  id: 'humanoid', fig: '10', name: 'Humanoid upper body', pal: 'cobalt', kicker: '2 × 4-DOF arms · 2-DOF neck · stereo cameras',
  blurb: 'A torso on a stand with two human-proportioned arms and a camera head. Drag the ball: the head follows it and the hands reach for it, and the spare degree of freedom lets you swing each elbow without moving the hand.',
  size: [60, 60, 96], center: [30, 30], bounds: [4, 56, 4, 56, 0, 94],
  init: () => ({ qL: HU_REST.slice(), qR: HU_REST.slice(), qtL: HU_REST.slice(), qtR: HU_REST.slice(), neck: [0, 0], ball: HU_BALL0.slice(), swivel: { L: 20, R: 20 }, mode2: 'right', speed: .6,
    reachErr: { L: null, R: null }, t: 0, hv: 0, gaze: 0, trailL: [], trailR: [], trace: false, crash: null, crashes: 0 }),
  thumbState: { qR: [70, 20, -20, 60], qL: [20, 15, 0, 40], neck: [-10, 10] }, thumbYaw: 0,
  steps: [
    { title: 'Stand & pelvis', parts: ['stand'], text: 'A weighted base and a column hold the pelvis at table height. No legs: this robot is about arms, hands and eyes.' },
    { title: 'Spine, battery & computer', parts: ['spine', 'battery', 'pc'], peek: true, text: 'An aluminium spine frame runs up the chest with a cross-bar for the shoulders. The battery sits low, the computer above it runs vision and control.' },
    { title: 'Chest cover', parts: ['torso'], text: 'A shell over the spine. See-through keeps the inside in view.' },
    { title: 'Shoulder pitch & roll actuators', parts: ['shP', 'shR'], text: 'Pitch swings the arm forward and back; roll, mounted on the pitch output, lifts it sideways. Their axes cross at the shoulder centre.' },
    { title: 'Shoulder yaw & upper arms', parts: ['shY', 'upper'], text: 'The third shoulder actuator twists the upper arm about its own length. Three axes through one point: a motorised ball joint.' },
    { title: 'Elbows, forearms & hands', parts: ['elbow', 'fore', 'hand'], text: 'The elbow is the fourth joint. The hands here are passive: fixed fingers, no motors.' },
    { title: 'Neck pan & tilt', parts: ['neckP', 'neckT'], text: 'Two small actuators let the head look left and right, up and down.' },
    { title: 'Head & stereo cameras', parts: ['head', 'cams'], text: 'Two cameras 56 mm apart, like eyes. The same point lands in a slightly different place in each image; that shift (disparity) gives its distance.' },
    { title: 'Harness', parts: ['harness'], text: 'Power and a data bus down each arm, actuator to actuator.' },
    { title: 'Target ball', parts: ['ball'], text: 'Something to look at and reach for. In Operate, drag it around.' },
  ],
  parts: {
    stand: { name: 'Stand & pelvis', kind: 'structure', lbl: 'STAND', key: true, short: 'base + column', info: 'Heavy base so a fast arm swing cannot tip it.', held: 'The table' },
    spine: { name: 'Spine frame', kind: 'structure', lbl: 'SPINE', inside: true, short: 'aluminium frame', info: 'Carries all the arm loads down to the pelvis.', held: 'The pelvis' },
    battery: { name: 'Battery', kind: 'power', lbl: 'BATTERY', inside: true, short: '48 V', info: 'Low in the chest for a low centre of mass.', held: 'The spine' },
    pc: { name: 'Computer', kind: 'electronics', lbl: 'COMPUTER', inside: true, short: 'vision + control', info: 'Finds the ball in both camera images, computes its 3D position, solves both arms’ IK and runs the joint loops.', held: 'The spine' },
    torso: { name: 'Chest cover', kind: 'cover', lbl: 'TORSO', key: true, short: 'shell', info: 'Rounded, padded edges: a humanoid is meant to work next to people.', held: 'Clips on the spine' },
    shP: { name: 'Shoulder pitch actuators', kind: 'motor', lbl: 'SHOULDER PITCH', short: 'axis across the chest', info: 'The strongest joints: they carry the whole arm’s weight.', held: 'The shoulder cross-bar' },
    shR: { name: 'Shoulder roll actuators', kind: 'motor', lbl: 'SHOULDER ROLL', short: 'axis forward', info: 'Lifts the arm sideways (abduction).', held: 'The pitch outputs' },
    shY: { name: 'Shoulder yaw actuators', kind: 'motor', lbl: 'SHOULDER YAW', short: 'axis along the upper arm', info: 'Twists the upper arm: this is what swings the elbow round when you change the swivel.', held: 'The roll outputs' },
    upper: { name: 'Upper arms', kind: 'structure', lbl: 'UPPER ARM', key: true, short: '200 mm', info: 'Human proportions: upper arm a bit longer than forearm.', held: 'The yaw actuators', spec: 'L1 = 200 mm' },
    elbow: { name: 'Elbow actuators', kind: 'motor', lbl: 'ELBOW', short: '0–145°', info: 'The elbow angle alone sets how far the hand is from the shoulder.', held: 'The ends of the upper arms' },
    fore: { name: 'Forearms', kind: 'structure', lbl: 'FOREARM', short: '180 mm', info: 'Light: weight far out costs shoulder torque.', held: 'The elbow actuators', spec: 'L2 = 180 mm' },
    hand: { name: 'Hands', kind: 'tool', lbl: 'HAND', key: true, short: 'passive', info: 'Fixed fingers. A real humanoid hand has 6 to 20 more actuators.', held: 'The forearm ends' },
    neckP: { name: 'Neck pan actuator', kind: 'motor', short: 'look left / right', info: '±80 degrees.', held: 'The top of the chest' },
    neckT: { name: 'Neck tilt actuator', kind: 'motor', short: 'look up / down', info: '−40 to +50 degrees.', held: 'The pan output' },
    head: { name: 'Head', kind: 'cover', lbl: 'HEAD', key: true, short: 'camera housing', info: 'The dashed blue line shows where it is looking.', held: 'The tilt actuator' },
    cams: { name: 'Stereo cameras', kind: 'sensor', lbl: 'STEREO CAMERAS', short: '56 mm baseline', info: 'Depth = focal length × baseline / disparity. Close objects shift a lot between the two images, far ones hardly at all.', held: 'The face plate' },
    harness: { name: 'Harness', kind: 'wiring', short: 'power + bus', info: 'Runs down each arm past every actuator.', held: 'Clips' },
    ball: { shadow: 'ball', name: 'Target ball', kind: 'tool', short: 'drag it', info: 'Drag the yellow ring to move it.', held: 'Your mouse' },
  },
  concepts: [
    ['Redundancy', 'A point in space is 3 numbers; each arm has 4 joints. The spare one shows as the orange dashed circle: every elbow position on it keeps the hand on the ball. Move the swivel slider and the elbow slides round the circle while the hand stays put.'],
    ['Why use the spare joint?', 'To keep elbows tucked in near people, to stay away from joint limits, or to reach round an obstacle. Humans do it all the time: try holding your hand still and moving your elbow.'],
    ['Reachability', 'Reach = 20 + 25 cm from each shoulder. The ball near the chest is hard to reach (elbow folds fully); far out the arm goes straight and the IK has a single answer.'],
    ['Gaze control', 'Pan and tilt follow the ball so it stays in the middle of both camera images, where stereo depth is most accurate.'],
    ['Safe motion near people', 'Collaborative robots limit hand speed and force near humans: 250 mm/s is a common slow-speed limit. The speed slider scales every joint; the readout turns red above the limit.'],
  ],
  draw: huDraw,
  active: st => { const a = {}; const d = (q, qt) => q.some((v, i) => Math.abs(v - qt[i]) > .3); if (d(st.qL, st.qtL) || d(st.qR, st.qtR)) a.shP = a.shR = a.shY = a.elbow = true; if (st.gaze > .5) a.neckP = a.neckT = true; return a; },
  enterOp: st => { st.t = 0; },
  opHTML: () => `<div class="seg" id="humode">${[['right', 'Right hand'], ['left', 'Left hand'], ['both', 'Both'], ['wave', 'Wave']].map(([k, l]) => `<button data-m="${k}">${l}</button>`).join('')}</div>
    <p class="note">Drag the yellow ring to move the ball. The head tracks it; the chosen hand reaches for it.</p>
    ${slHTML('husl', 'Left elbow swivel', -90, 90, 1, '°', 'Redundancy: elbow round the circle')}${slHTML('husr', 'Right elbow swivel', -90, 90, 1, '°')}
    ${slHTML('husp', 'Speed limit', 10, 100, 5, '%')}
    <div class="row2"><button id="hureset">Reset ball</button><button id="hutr">Trace</button><button id="huclr">Clear trace</button></div>`,
  opBind(el, st, ui, lab){ const $ = id => document.getElementById(id);
    el.querySelectorAll('#humode button').forEach(b => b.onclick = () => { st.mode2 = b.dataset.m; st.t = 0; lab.kick(); ui.render(); });
    $('s_husl').oninput = e => { st.swivel.L = +e.target.value; lab.kick(); }; $('s_husr').oninput = e => { st.swivel.R = +e.target.value; lab.kick(); };
    $('s_husp').oninput = e => { st.speed = +e.target.value/100; ui.render(); };
    $('hureset').onclick = () => { st.ball = HU_BALL0.slice(); lab.kick(); };
    $('hutr').onclick = () => { st.trace = !st.trace; ui.render(); }; $('huclr').onclick = () => { st.trailL = []; st.trailR = []; ui.render(); }; },
  opSync(el, st){ const $ = id => document.getElementById(id);
    el.querySelectorAll('#humode button').forEach(b => b.setAttribute('aria-pressed', b.dataset.m === st.mode2));
    slSync('husl', st.swivel.L, `${st.swivel.L}°`); slSync('husr', st.swivel.R, `${st.swivel.R}°`); slSync('husp', Math.round(st.speed*100), Math.round(st.speed*100) + '%');
    $('hutr').setAttribute('aria-pressed', st.trace); },
  readout(st){ const rows = [], fmt = q => q.map(v => v.toFixed(0)).join(' ') + '°';
    rows.push({ k: 'Right arm', v: fmt(st.qR), tip: 'Pitch / roll / yaw / elbow, degrees' }); rows.push({ k: 'Left arm', v: fmt(st.qL), tip: 'Pitch / roll / yaw / elbow, degrees' });
    for (const [key, s] of [['R', -1], ['L', 1]]){ const F = huFK(st['q' + key], s), dist = vLen(sub(F.tip, st.ball)), on = st.mode2 === 'both' || st.mode2 === (s > 0 ? 'left' : 'right');
      if (on) rows.push({ k: `${s > 0 ? 'Left' : 'Right'} hand to ball`, bar: Math.min(1, dist/40), v: st.reachErr[key] ? 'can’t reach' : `${(Math.max(0, dist - (st.mode2 === 'both' ? 3 : 0))*10).toFixed(0)} mm`, warn: !!st.reachErr[key], tip: st.reachErr[key] || '' }); }
    const dS = Math.min(vLen(sub(st.ball, huS(1))), vLen(sub(st.ball, huS(-1))));
    rows.push({ k: 'Ball to shoulder', bar: dS/(HU.L1 + HU.L2 + HU.H), v: `${(dS*10).toFixed(0)} mm`, warn: dS > HU.L1 + HU.L2 + HU.H - .2, tip: 'Reach: 450 mm' });
    rows.push({ k: 'Neck pan, tilt', v: `${st.neck[0].toFixed(0)}° ${st.neck[1].toFixed(0)}°` });
    rows.push({ k: 'Gaze error', bar: st.gaze/30, v: `${st.gaze.toFixed(1)}°` });
    rows.push({ k: 'Hand speed', bar: st.hv*10/500, v: `${(st.hv*10).toFixed(0)} mm/s`, warn: st.hv*10 > 250, tip: 'Collaborative limit ~250 mm/s' });
    rows.push({ k: 'Self-collisions', v: `${st.crashes}`, warn: !!st.crash });
    return rows; },
  stats: st => [{ label: 'Ball X Y Z mm', value: `${Math.round((st.ball[0]-HU.C)*10)} ${Math.round((st.ball[1]-HU.C)*10)} ${Math.round((st.ball[2]-3)*10)}`, small: true }],
  tick: huTick,
  onDrag(st, dx, dy, phase, lab){ if (phase === 'start'){ st.dragP = st.ball.slice(); return; } if (phase !== 'move') return;
    const p = vAdd(st.dragP, dragWorld(dx, dy)); p[0] = clamp(p[0], 4, 58); p[1] = clamp(p[1], 2, 58); p[2] = clamp(p[2], 6, 100);
    if (Math.abs(p[0] - HU.C) < 11 && Math.abs(p[1] - HU.C) < 14 && p[2] < 96) return; st.dragP = p; st.ball = p.slice(); },
  params(st, q){ if (q.has('ball')) st.ball = q.get('ball').split(',').map(Number); if (q.has('hm')) st.mode2 = q.get('hm');
    if (q.has('sw')){ const v = +q.get('sw'); st.swivel = { L: v, R: v }; }
    if (q.get('settle') === '1'){ for (let k = 0; k < 400; k++) huTick(st, 1/60, null, { flash(){} }); } },
};
ROBOT_ORDER.push({ id: 'humanoid', name: 'Humanoid upper body', blurb: 'Two 4-DOF arms, a 2-DOF neck and a stereo camera head on a torso. The head tracks the ball, the hands reach for it, and the spare joint swings the elbow.', chips: ['10 DOF', 'Redundant', 'Vision'] });
