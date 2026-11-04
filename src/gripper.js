/* ============================================================
   Robot 07 — three-finger tendon gripper (underactuated, ~20 cm hand). 1 unit = 1 cm.
   Truth: each finger has three phalanges on pin joints but only one motor. The motor winds a tendon onto a spool; the
   tendon runs down the inside of the finger over every joint, so pulling it curls all three. Return springs on the back
   of each joint open the finger; the springs get stiffer toward the tip, so the finger closes base-first. When a phalanx
   touches the object it stops and the rest keep curling: the finger wraps itself around the shape with no extra motors
   or sensing. Force sensors in the pads tell the controller when to stop pulling.
   The hand hangs from an arm's Z slide; test objects sit on a plinth below.
   Parts: mount | motors, spools | ctrl | palm | prox, pins | mid, dist | springs | tendons | pads, fsr | plinth, objects
   ============================================================ */
const GP = { C: [21, 17], RP: 4, L: [4.4, 3.6, 2.8], TH: .75, RT: .6, KT: 60, VS: 1.6, MU: .7, PLT: 7 };
const GP_REST = [-26, 8, 8], GP_LIM = [80, 95, 85], GP_K = [.1, .17, .26];
const GP_PHI = [90, 210, 330].map(a => a*DEG);
const GP_OBJ = {
  ball: { label: 'Ball', cage: true, top: 8, kg: .3, c: '#ef6a4c', sdf: (r, h, hc) => Math.hypot(r, h - hc) - 4, hc: 4, note: 'Envelops: fingers curl under its equator, so it is caged.' },
  can:  { label: 'Can', top: 9, kg: .35, c: '#5a82c6', sdf: (r, h, hc) => gpBox(r, h - hc, 3, 4.5), hc: 4.5, note: 'Straight sides: nothing to curl under, so friction alone must carry its weight.' },
  box:  { label: 'Box', top: 5.2, kg: .2, c: '#d49c1f', sdf: (r, h, hc, phi) => gpBox(r, h - hc, 2.6/Math.max(Math.abs(Math.cos(phi)), Math.abs(Math.sin(phi))), 2.6), hc: 2.6, note: 'Flat faces and corners: the fingers meet it at different points.' },
  egg:  { label: 'Egg', cage: true, top: 6, kg: .06, c: '#f2efe7', sdf: (r, h, hc) => (Math.hypot(r/2.2, (h - hc)/3) - 1)*2.2, hc: 3, crush: 6, note: 'Fragile: breaks above 6 N on any finger. Keep the force limit low.' } };
function gpBox(x, y, a, b){ const dx = Math.abs(x) - a, dy = Math.abs(y) - b; return Math.hypot(Math.max(dx, 0), Math.max(dy, 0)) + Math.min(Math.max(dx, dy), 0); }

// finger chain in its own plane: r = distance from the hand axis, h = depth below the palm
function gpChain(th){ let r = GP.RP, h = 0, Th = 0; const pts = [[r, h]], ang = [];
  for (let j = 0; j < 3; j++){ Th += th[j]*DEG; r += -GP.L[j]*Math.sin(Th); h += GP.L[j]*Math.cos(Th); pts.push([r, h]); ang.push(Th); }
  return { pts, ang }; }
// signed distance from a finger-plane point to whatever is there (object, plinth, table)
function gpSdf(st, i, r, h){ const O = GP_OBJ[st.obj], zb = GP.PLT + st.objZ, hc = st.zp - (zb + O.hc);
  let d = st.broken ? 99 : O.sdf(r, h, hc, GP_PHI[i]);
  d = Math.min(d, gpBox(r, h - (st.zp - 5), 3.5, 2), st.zp - 3 - h); return d; }
function gpHits(st, i, th, from){ const { pts } = gpChain(th); let hit = -1;
  for (let j = from; j < 3; j++) for (let k = 1; k <= 5; k++){ const t = k/5, p = [pts[j][0] + (pts[j+1][0] - pts[j][0])*t, pts[j][1] + (pts[j+1][1] - pts[j][1])*t];
    if (gpSdf(st, i, p[0], p[1]) < GP.TH){ hit = j; break; } }
  return hit; }
// quasi-static finger: tendon tension T balances the springs (theta' = T r / k) unless a joint is stopped by contact
// (cap). Solve T from the tendon length wound in (s = sum r theta' + T / k_tendon), then move joints, probing for contact.
function gpSolveT(s, cap){ const used = T => [0, 1, 2].reduce((a, j) => a + GP.RT*Math.min(T*GP.RT/GP_K[j], cap[j])*DEG, 0) + T/GP.KT;
  if (s <= 0) return 0; let lo = 0, hi = 400; for (let k = 0; k < 40; k++){ const m = (lo + hi)/2; if (used(m) > s) hi = m; else lo = m; } return lo; }
function gpStep(st, dt){
  for (let i = 0; i < 3; i++){ const th = st.th[i], cap = [0, 1, 2].map(j => GP_LIM[j] - GP_REST[j]);
    for (let it = 0; it < 3; it++){ const T = gpSolveT(st.s[i], cap); st.T[i] = T;
      for (let j = 0; j < 3; j++){ const tgt = GP_REST[j] + Math.min(T*GP.RT/GP_K[j], cap[j]);
        if (tgt <= th[j]){ th[j] = tgt; continue; }
        while (th[j] < tgt - 1e-6){ const old = th[j]; th[j] = Math.min(tgt, th[j] + 1); if (gpHits(st, i, th, j) >= 0){ th[j] = old; cap[j] = old - GP_REST[j]; break; } } } }
    // contacts and pad forces: the torque the tendon makes that the spring does not take is pushing on the object
    st.con[i] = [0, 1, 2].map(j => { const { pts } = gpChain(th); let m = 99;
      for (let k = 0; k <= 5; k++){ const t = k/5; m = Math.min(m, gpSdf(st, i, pts[j][0] + (pts[j+1][0] - pts[j][0])*t, pts[j][1] + (pts[j+1][1] - pts[j][1])*t)); }
      return m < GP.TH + .12; });
    st.F[i] = st.con[i].reduce((a, c, j) => a + (c ? Math.max(0, st.T[i]*GP.RT - GP_K[j]*(th[j] - GP_REST[j]))/(GP.L[j]*.6) : 0), 0); } }
function gpGrasp(st){ const O = GP_OBJ[st.obj], hc = st.zp - (GP.PLT + st.objZ + O.hc);
  const touching = st.con.filter(c => c.some(Boolean)).length, caged = !!O.cage && st.con.filter((c, i) => c[2] && gpChain(st.th[i]).pts[3][1] > hc + .3).length >= 2;
  const hold = GP.MU*st.F.reduce((a, b) => a + b, 0), W = O.kg*9.81, n = st.con.flat().filter(Boolean).length;
  return { touching, caged, hold, W, n, ok: !st.broken && touching >= 2 && (caged || hold >= W*1.2), type: touching < 2 ? 'none' : n >= 6 ? 'envelope (power) grasp' : 'fingertip (pinch) grasp' }; }

function gpDraw(st, m){
  const [cx, cy] = GP.C, zp = st.zp, zb = GP.PLT + st.objZ;
  const W3 = (i, r, h) => [cx + r*Math.cos(GP_PHI[i]), cy + r*Math.sin(GP_PHI[i]), zp - h];
  // ---- mount: post, boom, Z slide sleeve and rod down to the palm
  sbPart('mount'); qBox(T_I, [4, cy - 2, 3], [8, cy + 2, 42], ALU, { w: 1.1 }); qBox(T_I, [3, cy - 3, 3], [9, cy + 3, 3.6], STEEL, { w: .7 });
  qBox(T_I, [4, cy - 1.6, 39], [cx + 2, cy + 1.6, 42], ALU, { w: 1.1 }); qBox(T_I, [cx - 2, cy - 2, 34], [cx + 2, cy + 2, 39], MOTOR, { w: .9 });
  qRod([cx, cy, 34], [cx, cy, zp + 4.6], .9, STEEL, { w: .8 }); qRod([cx, cy, zp + 4.6], [cx, cy, zp + 4], 2.4, STEEL, { w: .8 });
  // ---- palm: three motors with spools, controller board, cover
  sbGroup('palm', T_I, [cx - 6.5, cy - 6.5, zp], [cx + 6.5, cy + 6.5, zp + 4]);
  for (let i = 0; i < 3; i++){ const a = GP_PHI[i] + Math.PI, x = cx + 2.4*Math.cos(a), y = cy + 2.4*Math.sin(a);
    sbPart('motors'); qRod([x, y, zp + 3.8], [x, y, zp + 1.6], 1.1, MOTOR, { w: .8 });
    sbPart('spools'); qRod([x, y, zp + 1.6], [x, y, zp + .6], 1.25, BRASS, { w: .7 }); }
  sbPart('ctrl'); qBox(T_I, [cx - 1.4, cy - 4.8, zp + .4], [cx + 1.4, cy - 3.6, zp + 3.6], PCB, { w: .6 });
  sbPart('palm'); qExt(T_I, ellUV(cx, cy, 6.5, 6.5, 24), zp, zp + 4, m, { ax: 'z', cover: true, w: 1.8 });
  sbFree(); if (facing([0, -1, 0])) sbPut(txT(T_I, [cx, cy - 6.55, zp + 2], [1,0,0], [0,0,1], 'TRI-3', 1.6), obbW([cx - 3, cy - 6.6, zp + 1], [cx + 3, cy - 6.5, zp + 3]), { p: 'palm' });
  // ---- fingers
  const linkPart = ['prox', 'mid', 'dist'], wd = [1.1, 1, .9];
  for (let i = 0; i < 3; i++){ const { pts, ang } = gpChain(st.th[i]), u = [Math.cos(GP_PHI[i]), Math.sin(GP_PHI[i]), 0], t = [-u[1], u[0], 0];
    const wv = (dr, dh) => [u[0]*dr, u[1]*dr, -dh];
    sbPart('pins'); qRod(vAdd(W3(i, GP.RP, -.4), vSc(t, -1.6)), vAdd(W3(i, GP.RP, -.4), vSc(t, 1.6)), .5, ALU, { w: .6 });
    for (let j = 0; j < 3; j++){ const Th = ang[j], X = wv(-Math.sin(Th), Math.cos(Th)), N = wv(-Math.cos(Th), -Math.sin(Th)), T = tFrame(W3(i, ...pts[j]), X, t), s = dot(N, tAx(T, 2)) > 0 ? 1 : -1;
      const hit = st.con[i][j], col = m;
      sbPart(linkPart[j]); qBox(T, [.3, -wd[j], -.65], [GP.L[j] - .3, wd[j], .65], col, { w: .8, sh: tone });
      sbPart('pins'); qRod(vAdd(W3(i, ...pts[j]), vSc(t, -wd[j] - .3)), vAdd(W3(i, ...pts[j]), vSc(t, wd[j] + .3)), .32, STEEL, { w: .5 });
      sbPart('pads'); qBox(T, [.6, -wd[j] + .15, s > 0 ? .65 : -.95], [GP.L[j] - .5, wd[j] - .15, s > 0 ? .95 : -.65], hit ? matOf('#43c25c') : RUBBER, { w: .6 });
      if (j === 2){ sbPart('fsr'); qBox(T, [GP.L[j] - 1.6, -.5, s > 0 ? .95 : -1.05], [GP.L[j] - .6, .5, s > 0 ? 1.05 : -.95], matOf('#e2483d'), { w: .4 }); }
      // return spring across the back of the joint
      if (j > 0){ const B0 = W3(i, ...pts[j]), back = (k, Th2) => vAdd(B0, vAdd(vSc(wv(-Math.sin(Th2), Math.cos(Th2)), k), vSc(wv(Math.cos(Th2), Math.sin(Th2)), .85)));
        sbPart('springs'); qHelix(back(-.9, ang[j - 1]), back(.9, Th), .25, 5, '#c6ccce', { w: .45 }); } }
    // tendon: from the spool, down the inside of each link, to the tip
    sbPart('tendons'); const tp = [[cx + 2.4*Math.cos(GP_PHI[i] + Math.PI), cy + 2.4*Math.sin(GP_PHI[i] + Math.PI), zp + 1.1], W3(i, GP.RP - .6, -.2)];
    for (let j = 0; j < 3; j++){ const Th = ang[j], n = [-Math.cos(Th), -Math.sin(Th)]; for (const k of [.15, .85]) tp.push(W3(i, pts[j][0] + (pts[j+1][0] - pts[j][0])*k + n[0]*.45, pts[j][1] + (pts[j+1][1] - pts[j][1])*k + n[1]*.45)); }
    qWire(tp, st.T[i] > .5 ? '#e2483d' : CABLE.sig, .16, { seg: 2.5, bend: .4 }); }
  // ---- plinth and the object
  sbPart('plinth'); qRod([cx, cy, 3], [cx, cy, GP.PLT], 3.5, matOf('#9db2cc'), { w: 1, sh: tone });
  sbPart('objects'); const O = GP_OBJ[st.obj], col = matOf(st.broken ? '#f4b829' : O.c);
  if (st.obj === 'ball') qBall([cx, cy, zb + 4], 4, col, { w: 1.2 });
  else if (st.obj === 'can') qRod([cx, cy, zb], [cx, cy, zb + 9], 3, col, { w: 1.1, sh: tone, deco: () => txT(T_I, [cx, cy - 3.05, zb + 4.5], [1,0,0], [0,0,1], 'SODA', 1.6, '#fff') });
  else if (st.obj === 'box') qBox(T_I, [cx - 2.6, cy - 2.6, zb], [cx + 2.6, cy + 2.6, zb + 5.2], col, { w: 1, sh: tone });
  else if (st.broken) qRod([cx, cy, zb], [cx, cy, zb + .8], 2.6, col, { w: .9, r1: 1.6, sh: tone });
  else { sbPut(`<ellipse cx="${f(P(cx, cy, zb + 3)[0])}" cy="${f(P(cx, cy, zb + 3)[1])}" rx="${f(2.2*CAM_K)}" ry="${f(3*CAM_K)}" fill="${col.l}" stroke="${INK}" stroke-width="1"/>`, obbW([cx - 2.2, cy - 2.2, zb], [cx + 2.2, cy + 2.2, zb + 6])); }
  const pt = boardTop(0, 0, 36, 34, 3, 6); let over = '';
  if (st.crash) over += crashBurst(st.crash.at, st.crash.t);
  return { under: board(0, 0, 36, 34, 3, 6), z0: 3, clip: pt, over };
}

function gpTick(st, dt, ui, lab){
  let more = false;
  const lim = st.limit, maxT = Math.max(...st.T);
  for (let i = 0; i < 3; i++){ if (st.cmd === 'close'){ if (st.T[i] < lim) st.s[i] += GP.VS*dt; else st.s[i] -= (st.T[i] - lim)/GP.KT*.5; }
    else if (st.cmd === 'open') st.s[i] = Math.max(0, st.s[i] - GP.VS*2*dt); }
  const before = st.th.flat(); gpStep(st, dt); const changed = st.th.flat().some((v, k) => Math.abs(v - before[k]) > 1e-3);
  const O = GP_OBJ[st.obj];
  if (O.crush && !st.broken && Math.max(...st.F) > O.crush){ st.broken = true; st.att = false; st.objZ = 0; st.crash = { at: [GP.C[0], GP.C[1], GP.PLT + 3], t: 1 }; st.crashes++; lab.flash(`Crushed! ${Math.max(...st.F).toFixed(1)} N on one finger — lower the force limit`); }
  // Z slide: move toward the target; a held object comes along, a weak grasp lets it slip
  const dz = st.zpT - st.zp;
  if (Math.abs(dz) > 1e-3){ const step = Math.sign(dz)*Math.min(Math.abs(dz), 8*dt);
    if (step > 0 && !st.att && st.objZ === 0){ const g = gpGrasp(st); if (g.touching >= 2){ if (g.ok){ st.att = true; lab.flash(g.caged ? 'Lifted: the fingers cage it from below' : 'Lifted: friction holds it'); } else lab.flash(`Slipped: friction ${g.hold.toFixed(1)} N < weight ${g.W.toFixed(1)} N`); } }
    st.zp += step; if (st.att) st.objZ += step; more = true; }
  if (st.att){ const g = gpGrasp(st); if (!g.ok){ st.att = false; st.objZ = 0; lab.flash('Dropped: the grip let go'); } }
  if (st.crash){ st.crash.t -= dt/1.2; if (st.crash.t <= 0) st.crash = null; else more = true; }
  const settling = st.cmd === 'close' ? Math.abs(maxT - lim) > .3 && st.s.some(v => v < 6) : st.s.some(v => v > 0);
  return more || changed || settling || !!st.crash;
}
const gpApproach = obj => GP.PLT + GP_OBJ[obj].top + 1.2;

ROBOTS.gripper = {
  id: 'gripper', fig: '07', name: 'Three-finger gripper', pal: 'mint', kicker: '9 joints · 3 motors · tendons · force sensing',
  blurb: 'Three fingers, nine joints, only three motors. Each motor pulls one tendon; springs and contact decide how the finger bends, so it wraps itself around whatever shape is there.',
  size: [36, 34, 40], center: [18, 17], bounds: [2, 34, 2, 32, 0, 40],
  init: () => ({ obj: 'ball', zp: gpApproach('ball'), zpT: gpApproach('ball'), objZ: 0, att: false, broken: false, th: [0, 1, 2].map(() => GP_REST.slice()), s: [0, 0, 0], T: [0, 0, 0], F: [0, 0, 0],
    con: [0, 1, 2].map(() => [false, false, false]), cmd: 'open', limit: 15, crash: null, crashes: 0 }),
  thumbYaw: 0,
  steps: [
    { title: 'Mount & Z slide', parts: ['mount'], text: 'The hand hangs from a robot arm’s wrist; here a post, a boom and a vertical slide stand in for the arm.' },
    { title: 'Finger motors & spools', parts: ['motors', 'spools'], peek: true, text: 'One small gear motor per finger, axis vertical, with a spool on its shaft. Turning the spool winds in that finger’s tendon.' },
    { title: 'Controller board', parts: ['ctrl'], text: 'Reads the three pad force sensors and the motor currents, and decides when to stop pulling.' },
    { title: 'Palm shell', parts: ['palm'], text: 'The palm closes over the drives. The fingers pivot on its rim, 120 degrees apart.' },
    { title: 'Proximal phalanges & pins', parts: ['prox', 'pins'], text: 'The first link of each finger, on a steel pin through the palm rim. At rest the springs hold it splayed outward.' },
    { title: 'Middle & distal phalanges', parts: ['mid', 'dist'], text: 'Two more links per finger on their own pins. Nine joints in all, and still only three motors.' },
    { title: 'Return springs', parts: ['springs'], text: 'A spring across the back of each joint opens it. Stiffer toward the tip, so pulling the tendon bends the base joint first.' },
    { title: 'Tendons', parts: ['tendons'], text: 'A braided cord from each spool runs down the inside of its finger past every joint to the fingertip. It turns red when it is under tension.' },
    { title: 'Pads & force sensors', parts: ['pads', 'fsr'], text: 'Rubber pads give friction; a thin force-sensing resistor in each fingertip changes resistance with pressure. Pads turn green where they touch.' },
    { title: 'Plinth & test object', parts: ['plinth', 'objects'], text: 'Something to grasp. In Operate, pick a shape, set the force limit and press Close, then Lift.' },
  ],
  parts: {
    mount: { name: 'Mount & Z slide', kind: 'structure', lbl: 'Z SLIDE', key: true, short: 'stands in for the arm', info: 'Lowers the hand over the object and lifts it for the grasp test.', held: 'Bolted to the bench' },
    motors: { name: 'Finger motors (×3)', kind: 'motor', lbl: 'FINGER MOTORS', inside: true, short: 'DC gear motors, 100:1', info: 'One motor per finger, not per joint. Current through it is proportional to tendon tension, so it doubles as a force sensor.', held: 'The palm floor' },
    spools: { name: 'Tendon spools', kind: 'gear', lbl: 'SPOOLS', inside: true, short: '5 mm radius', info: 'Tension = motor torque / spool radius.', held: 'The motor shafts' },
    ctrl: { name: 'Controller board', kind: 'electronics', lbl: 'CONTROLLER', inside: true, short: 'force loop', info: 'Closes each finger until its force reaches the limit, then holds: force control rather than position control.', held: 'Inside the palm' },
    palm: { name: 'Palm', kind: 'cover', lbl: 'PALM', key: true, short: 'printed shell', info: 'Holds the three finger pivots 120 degrees apart around the axis.', held: 'The wrist flange' },
    prox: { name: 'Proximal phalanges', kind: 'structure', lbl: 'PROXIMAL', key: true, short: '44 mm', info: 'Softest spring: closes first.', held: 'Pins in the palm rim', spec: 'k = 0.10 N cm/deg' },
    mid: { name: 'Middle phalanges', kind: 'structure', lbl: 'MIDDLE', short: '36 mm', info: 'Keeps curling after the proximal link is stopped by the object.', held: 'Pins on the proximal links', spec: 'k = 0.17 N cm/deg' },
    dist: { name: 'Distal phalanges', kind: 'structure', lbl: 'DISTAL', short: '28 mm', info: 'Stiffest spring: the fingertip curls last, closing round the far side of the object.', held: 'Pins on the middle links', spec: 'k = 0.26 N cm/deg' },
    pins: { name: 'Joint pins', kind: 'structure', short: 'steel pins', info: 'Plain pivots, no motors: these joints are passive.', held: 'Press-fit in the links' },
    springs: { name: 'Return springs', kind: 'structure', lbl: 'SPRINGS', short: 'one per joint', info: 'The tendon pulls against them. Their stiffness ratio decides the closing order.', held: 'Hooks across the back of each joint' },
    tendons: { name: 'Tendons', kind: 'gear', lbl: 'TENDONS', key: true, short: 'braided cord', info: 'One tendon passes every joint at the same 6 mm moment arm, so the same tension makes the same torque at each joint.', held: 'The spool and the fingertip anchor' },
    pads: { name: 'Rubber pads', kind: 'tool', lbl: 'PADS', short: 'grip friction μ ≈ 0.7', info: 'Friction force = μ × squeeze. A fingertip grasp relies on it entirely.', held: 'Glued to every link' },
    fsr: { name: 'Fingertip force sensors', kind: 'sensor', lbl: 'FORCE SENSORS', short: 'FSR in each tip', info: 'Force-sensing resistors: pressure on the film lowers its resistance.', held: 'Under the distal pads' },
    plinth: { shadow: 'plinth', name: 'Plinth', kind: 'structure', short: 'object stand', info: 'Holds the test object at a reachable height.', held: 'The bench' },
    objects: { shadow: 'obj', name: 'Test object', kind: 'tool', lbl: 'OBJECT', short: 'ball, can, box or egg', info: 'Choose a shape in Operate. The same motor command gives a different finger shape for each.', held: 'The plinth' },
  },
  concepts: [
    ['Underactuation', 'Fewer motors than joints: 3 motors, 9 joints. The missing control comes from the springs and from contact with the object. The hand adapts its shape for free, but it cannot place each joint where you want.'],
    ['Why the stiffness gradient', 'All three joints feel the same tendon torque, so the softest joint moves most. Soft at the base, stiff at the tip: the finger closes base first, then curls its tip around, like your own fingers.'],
    ['Envelope vs fingertip grasp', 'An envelope (power) grasp wraps many links round the object and cages it: it holds even at low force. A fingertip (pinch) grasp touches with the tips only and relies on friction: squeeze harder or it slips.'],
    ['Force control', 'The controller pulls until the pad force reaches the limit and then holds it. Set the limit low for the egg; too low and the can slips when lifted.'],
    ['Friction needed to lift', 'To hold by friction alone: μ × total squeeze ≥ weight. The readout compares the two; the Lift button tests it.'],
  ],
  draw: gpDraw,
  active: st => { const a = {}; if (st.T.some(t => t > .5)) a.motors = a.spools = a.tendons = true; if (st.F.some(v => v > .2)) a.fsr = a.pads = true; return a; },
  opHTML: () => `<div class="seg" id="gpobj">${Object.entries(GP_OBJ).map(([k, o]) => `<button data-o="${k}">${o.label}</button>`).join('')}</div>
    <div class="row2"><button id="gpclose">Close</button><button id="gpopen">Open</button><button id="gplift">Lift</button><button id="gplower">Lower</button></div>
    <p class="note" id="gpnote"></p>
    ${slHTML('gplim', 'Force limit (tendon)', 2, 40, 1, ' N')}`,
  opBind(el, st, ui, lab){ const $ = id => document.getElementById(id);
    el.querySelectorAll('#gpobj button').forEach(b => b.onclick = () => { st.obj = b.dataset.o; st.broken = false; st.att = false; st.objZ = 0; st.cmd = 'open';
      st.s = [0, 0, 0]; st.th = [0, 1, 2].map(() => GP_REST.slice()); st.zp = st.zpT = gpApproach(st.obj); lab.kick(); ui.render(); });
    $('gpclose').onclick = () => { if (st.zpT > gpApproach(st.obj) + .1 && !st.att) lab.flash('Closing in the air: press Lower to reach the object'); st.cmd = 'close'; lab.kick(); };
    $('gpopen').onclick = () => { st.cmd = 'open'; lab.kick(); };
    $('gplift').onclick = () => { st.zpT = gpApproach(st.obj) + 10; lab.kick(); };
    $('gplower').onclick = () => { st.zpT = gpApproach(st.obj) + st.objZ; lab.kick(); };
    $('s_gplim').oninput = e => { st.limit = +e.target.value; lab.kick(); ui.render(); }; },
  opSync(el, st){ const $ = id => document.getElementById(id);
    el.querySelectorAll('#gpobj button').forEach(b => b.setAttribute('aria-pressed', b.dataset.o === st.obj));
    $('gpclose').setAttribute('aria-pressed', st.cmd === 'close'); $('gpopen').setAttribute('aria-pressed', st.cmd === 'open');
    $('gpnote').textContent = (st.broken ? 'Broken! Pick the egg again to reset. ' : '') + GP_OBJ[st.obj].note;
    slSync('gplim', st.limit, `${st.limit} N`); },
  readout(st){ const g = gpGrasp(st), rows = [];
    for (let i = 0; i < 3; i++) rows.push({ k: `Finger ${i + 1} (${st.con[i].filter(Boolean).length}/3 touch)`, bar: st.T[i]/40, v: `${st.T[i].toFixed(0)} / ${st.F[i].toFixed(1)} N`, tip: 'Tendon tension / pad force', warn: GP_OBJ[st.obj].crush && st.F[i] > GP_OBJ[st.obj].crush*.8 });
    rows.push({ k: 'Finger 1 joints', v: st.th[0].map(v => v.toFixed(0) + '°').join(' ') });
    rows.push({ k: 'Grasp', v: g.type.split(' ')[0] + (g.caged ? ', caged' : ''), tip: g.type });
    rows.push({ k: 'Friction capacity', bar: g.hold/Math.max(g.W*2, .1), v: `${g.hold.toFixed(1)} / ${g.W.toFixed(1)} N`, warn: g.touching >= 2 && !g.ok, tip: 'μ × total pad force vs the object’s weight' });
    rows.push({ k: 'Motor current (est.)', bar: Math.max(...st.T)/40, v: `${(Math.max(...st.T)*.005/0.02/100*1000).toFixed(0)} mA` });
    rows.push({ k: 'Lifted', v: st.att ? `yes, ${(st.objZ*10).toFixed(0)} mm` : 'no' });
    rows.push({ k: 'Broken objects', v: `${st.crashes}`, warn: st.broken });
    return rows; },
  stats: st => [{ label: 'Max finger force', value: `${Math.max(...st.F).toFixed(1)} N`, small: true }],
  tick: gpTick,
  enterOp: st => { st.cmd = st.cmd || 'open'; },
  params(st, q){ if (q.has('obj') && GP_OBJ[q.get('obj')]){ st.obj = q.get('obj'); st.zp = st.zpT = gpApproach(st.obj); }
    if (q.get('demo') === 'close'){ st.cmd = 'close'; for (let k = 0; k < 240; k++){ for (let i = 0; i < 3; i++) if (st.T[i] < st.limit) st.s[i] += GP.VS/60; gpStep(st, 1/60); } } },
};
ROBOT_ORDER.push({ id: 'gripper', name: 'Three-finger gripper', blurb: 'Tendon-driven fingers with one motor each, compliant joints and force sensing: watch them wrap a ball, pinch a box, lift a can and (gently) hold an egg.', chips: ['Underactuated', 'Tendons', 'Force sensing'] });
