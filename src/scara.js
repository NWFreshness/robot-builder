/* ============================================================
   Robot 02 — SCARA arm (Selective Compliance Assembly Robot Arm), 400 mm reach. 1 unit = 1 cm.
   Truth: a pedestal holds the J1 servo and harmonic drive (and, in compact SCARAs, the servo amplifiers). Link 1 swings
   about J1; at its tip the J2 servo + harmonic drive swing link 2. Link 2 carries two motors: the Z motor turns a ball-screw
   nut by a belt (the quill moves up and down), the R motor turns a ball-spline nut by a second belt (the quill rotates).
   Both nuts ride one combined ball-screw/spline shaft: the quill. A vacuum cup on its end picks parts. A cable duct arcs
   from the pedestal top to link 2. The arm is stiff vertically and compliant sideways: ideal for pressing parts in.
   Parts: pedestal | j1motor, j1hd, amps, psu | j1out | link1 | j2motor, j2hd | link2 | zmotor, zbelt, rmotor, rbelt, nut |
          quill | cup | duct | tray, pucks
   ============================================================ */
const SC = { BX: 16, BY: 38, L1: 22, L2: 18 };
const SC_LIM = [[-130, 130], [-145, 145], [.4, 16], [-360, 360]];
const SC_VMAX = [180, 220, 30, 360];
const SC_NAMES = ['J1 shoulder', 'J2 elbow', 'Z quill height', 'R quill roll'];
const SC_HOME = [-20, 60, 10, 0];
const SC_FEED = [[44, 22], [48, 30], [40, 15]];
const SC_TRAY = [[37, 55], [43, 55], [40, 61]];
const SC_PUCKC = ['#e2483d', '#5a82c6', '#43c25c'];

function scFK(q){ const A = tMul(tTr(SC.BX, SC.BY, 33), tRz(q[0]*DEG)), B = tCat(A, tTr(SC.L1, 0, 6), tRz(q[1]*DEG)), Q = tAp(B, [SC.L2, 0, 0]);
  return { A, B, E: tAp(A, [SC.L1, 0, 0]), tip: [Q[0], Q[1], 3 + q[2]] }; }
function scIK(x, y, cfg){ const dx = x - SC.BX, dy = y - SC.BY, c2 = (dx*dx + dy*dy - SC.L1**2 - SC.L2**2) / (2*SC.L1*SC.L2);
  if (c2 > 1) return { err: 'Out of reach: both links are in a straight line' }; if (c2 < -1) return { err: 'Too close to the column' };
  const q2 = cfg*Math.acos(c2), q1 = Math.atan2(dy, dx) - Math.atan2(SC.L2*Math.sin(q2), SC.L1 + SC.L2*Math.cos(q2)), q = [wrap180(q1/DEG), q2/DEG];
  for (let i = 0; i < 2; i++) if (q[i] < SC_LIM[i][0] || q[i] > SC_LIM[i][1]) return { err: `${SC_NAMES[i]} would pass its limit (${SC_LIM[i][q[i] < 0 ? 0 : 1]} deg)`, q };
  return { q }; }
function scCheck(st, q){ const F = scFK(q), t = F.tip;
  if (Math.abs(t[0] - SC.BX) < 10.5 && Math.abs(t[1] - SC.BY) < 10.5) return { why: 'Self-collision: the quill hit the pedestal', at: [t[0], t[1], 20] };
  const held = st.pucks.find(p => p.held);
  if (held && q[2] < 1.6 - .01 && !inTray(t)) return { why: 'Crash: the part being carried hit the table', at: [t[0], t[1], 3.4] };
  if (held && q[2] < 2.2 - .01 && inTray(t)) return { why: 'Crash: the part hit the tray', at: [t[0], t[1], 4] };
  for (const p of st.pucks) if (!p.held && Math.hypot(p.x - t[0], p.y - t[1]) < 2.8 && q[2] < p.z + 1.6 - .01) return { why: 'Crash: the vacuum cup pressed into a part', at: [t[0], t[1], 3 + p.z + 1.6] };
  if (!held && inTray(t) && q[2] < .6) return { why: 'Crash: the cup hit the tray', at: [t[0], t[1], 3.6] };
  return null; }
const inTray = t => t[0] > 33 && t[0] < 47 && t[1] > 51 && t[1] < 65;
const SC_WS = (() => { const out = []; for (let a = -130; a <= 130; a += 13) for (let b = -145; b <= 145; b += 14.5){ const A = a*DEG, B = (a + b)*DEG;
  out.push([SC.BX + SC.L1*Math.cos(A) + SC.L2*Math.cos(B), SC.BY + SC.L1*Math.sin(A) + SC.L2*Math.sin(B)]); } return out; })();

function scDraw(st, m){
  const F = scFK(st.q), { BX, BY, L1, L2 } = SC, q = st.q;
  // ---- pedestal: J1 servo, shaft, harmonic drive, servo amplifiers, PSU, under the cover
  sbGroup('ped', T_I, [BX-9, BY-9, 3], [BX+9, BY+9, 31]);
  sbPart('j1motor'); qRod([BX, BY, 5], [BX, BY, 14], 4, MOTOR, { w: 1 }); qRod([BX, BY, 14], [BX, BY, 26], 1, STEEL, { w: .7 });
  sbPart('j1hd'); qRod([BX, BY, 26], [BX, BY, 30.6], 6, REDUCER, { w: 1 });
  sbPart('amps'); qBox(T_I, [BX-7.6, BY-6, 6], [BX-7, BY+6, 24], PCB, { w: .7 }); qBox(T_I, [BX-6, BY+7, 6], [BX+6, BY+7.6, 22], PCB, { w: .7 });
  for (const z of [9, 13, 17, 21]) qBox(T_I, [BX-7, BY-4, z], [BX-6, BY-1, z+2], DARK, { w: .5 });
  sbPart('psu'); qBox(T_I, [BX+1, BY-7.5, 3.4], [BX+7, BY-2, 10], ALU, { w: .8 });
  sbPart('j1motor'); qWire([[BX-7, BY+2, 10], [BX-3.5, BY+2, 10], [BX-3.5, BY+1, 8]], CABLE.pwr, .35);
  sbPart('pedestal'); qExt(T_I, rrUV(BX-9, BY-9, BX+9, BY+9, 4), 3, 31, m, { ax: 'z', cover: true, w: 2.2 });
  sbFree(); if (facing([1,0,0])) sbPut(txT(T_I, [BX+9.05, BY, 24], [0,-1,0], [0,0,1], 'SCARA', 2.8) + dotAt([BX+9.1, BY-5, 27], .7, st.mode === 'op' ? '#43c25c' : '#8d877a'), obbW([BX+9, BY-7, 22], [BX+9.1, BY+7, 28]), { p: 'pedestal' });
  sbPart('j1out'); qRod([BX, BY, 31], [BX, BY, 33], 7, STEEL, { w: 1.2 });
  // ---- link 1 with the J2 drive inside its tip
  sbGroup('l1', F.A, [-7, -7, 0], [L1 + 7, 7, 6]);
  sbPart('j2motor'); qCyl(F.A, 'z', [L1, 0], .6, 4.4, 3.2, MOTOR, { w: .9 });
  sbPart('j2hd'); qCyl(F.A, 'z', [L1, 0], 4.4, 5.8, 4.4, REDUCER, { w: .9 });
  sbPart('j2motor'); qWire([[1, -3, 3], [L1 - 4, -3, 3], [L1 - 2.5, -1.5, 2]].map(p => tAp(F.A, p)), CABLE.pwr, .35);
  sbPart('link1'); qExt(F.A, rrUV(-7, -7, L1 + 7, 7, 7), 0, 6, m, { ax: 'z', cover: true, w: 2 });
  // ---- link 2: Z motor + belt to the ball-screw nut, R motor + belt to the spline nut, middle of the quill
  const zb = 3 + q[2] + 2.5, top = zb + 44;
  sbGroup('l2', F.B, [-6, -6, 0], [L2 + 6, 6, 7]);
  sbPart('zmotor'); qCyl(F.B, 'z', [6, 0], 1.2, 5.6, 2.2, MOTOR, { w: .8 }); qCyl(F.B, 'z', [6, 0], 5.6, 6.2, 1.1, STEEL, { w: .6 });
  sbPart('zbelt'); qBelt(tAp(F.B, [6, 0, 5.9]), tAp(F.B, [L2, 0, 5.95]), 1.15, 2.4, [0, 0, 1], '#2b2724', { w: 1.1 });
  sbPart('rmotor'); qCyl(F.B, 'z', [10.5, 3], 1.4, 5.2, 2, MOTOR, { w: .8 }); qCyl(F.B, 'z', [10.5, 3], .6, 1.4, 1, STEEL, { w: .6 });
  sbPart('rbelt'); qBelt(tAp(F.B, [10.5, 3, .95]), tAp(F.B, [L2, 0, .95]), 1.05, 2.4, [0, 0, 1], '#2b2724', { w: 1.1 });
  sbPart('nut'); qCyl(F.B, 'z', [L2, 0], 5.4, 6.5, 2.3, BRASS, { w: .8 }); qCyl(F.B, 'z', [L2, 0], .5, 1.5, 2.3, BRASS, { w: .8 });
  sbPart('quill'); const qx = F.tip[0], qy = F.tip[1];
  qRod([qx, qy, 39.05], [qx, qy, 45.95], 1.1, STEEL, { w: .8 });
  sbPart('zmotor'); qWire([[1, 3.5, 3], [5, 2.5, 3]].map(p => tAp(F.B, p)), CABLE.pwr, .3);
  sbPart('link2'); qExt(F.B, rrUV(-6, -6, L2 + 6, 6, 6), 0, 7, m, { ax: 'z', cover: true, w: 2,
    capDeco: b => b ? txT(F.B, [L2/2 - 1, 0, 7.02], [1,0,0], [0,1,0], 'S-400', 2.4) : '' });
  // ---- quill outside link 2: lower piece (ball screw thread visible), top piece with a stop collar; then the cup
  sbFree(); sbPart('quill');
  qRod([qx, qy, zb], [qx, qy, 39], 1.1, STEEL, { w: .9 }); qHelix([qx, qy, Math.max(zb + 1, 30)], [qx, qy, 38.6], 1.12, (38.6 - Math.max(zb + 1, 30))/1, '#8f989b', { w: .55 });
  qRod([qx, qy, 46], [qx, qy, top], 1.1, STEEL, { w: .9 }); qRod([qx, qy, top - 1], [qx, qy, top], 1.7, DARK, { w: .8 });
  sbPart('cup'); const ang = (q[0] + q[1] + q[3])*DEG;
  qRod([qx, qy, zb], [qx, qy, zb - 1.2], .9, DARK, { w: .8 }); qRod([qx, qy, zb - 1.2], [qx, qy, 3 + q[2]], .9, RUBBER, { r1: 2, w: .9 });
  qBox(tMul(tTr(qx, qy, zb - .6), tRz(ang)), [.9, -.3, -.4], [1.8, .3, .4], matOf('#f4b829'), { w: .5 });
  // vacuum line from the cup fitting up to link 2
  qWire([[qx + .9, qy, zb - .6], [qx + 2.2, qy, zb + 2], [qx + 2.2, qy, 37], ...[tAp(F.B, [L2 - 3, 2.8, 0])]], st.vac ? '#8af0ae' : CABLE.air, .3, { seg: 6 });
  // ---- cable duct: pedestal top to link 2
  sbPart('duct'); const d0 = [BX - 6, BY, 31], d3 = tAp(F.B, [2.5, 0, 7]), d1 = vAdd(d0, [0, 0, 16]), d2 = vAdd(d3, [0, 0, 9]);
  const curve = []; for (let i = 0; i <= 14; i++){ const t = i/14, a = vLerp(d0, d1, t), b = vLerp(d1, d2, t), c = vLerp(d2, d3, t); curve.push(vLerp(vLerp(a, b, t), vLerp(b, c, t), t)); }
  qWire(curve, '#9aa0a3', .9, { seg: 5, bend: 0 });
  // ---- tray and parts
  sbPart('tray'); qBox(T_I, [33, 51, 3], [47, 65, 3.6], matOf('#9db2cc'), { w: .9 });
  for (const [lo, hi] of [[[33, 51, 3.6], [47, 51.6, 5]], [[33, 64.4, 3.6], [47, 65, 5]], [[33, 51.6, 3.6], [33.6, 64.4, 5]], [[46.4, 51.6, 3.6], [47, 64.4, 5]]]) qBox(T_I, lo, hi, matOf('#9db2cc'), { w: .7 });
  sbPart('pucks'); st.pucks.forEach((p, i) => { const c = p.held ? [qx, qy, 3 + q[2] - 1.6] : [p.x, p.y, 3 + p.z];
    qRod(c, vAdd(c, [0, 0, 1.6]), 2.6, matOf(SC_PUCKC[i]), { w: 1, sh: tone }); });
  // ---- floor marks: feeder spots, workspace, trace
  const pt = boardTop(0, 0, 62, 76, 3, 6);
  let floor = SC_FEED.map(([x, y]) => `<path d="${pd(on(PZ(3.02), ellUV(x, y, 3, 3, 20)))}" fill="none" stroke="#8f8774" stroke-width=".7" stroke-dasharray="1.2 1"/>`).join('');
  if (st.mode === 'op' && st.ws){ let d = ''; for (const [x, y] of SC_WS){ const p = P(x, y, 3.02); d += `M${f(p[0]-.35)} ${f(p[1]-.35)}h.7v.7h-.7z`; } floor += clipTo(pt, `<path d="${d}" fill="#3d5878" opacity=".45"/>`); }
  if (st.mode === 'op' && st.trace) floor += trailSVG(st.trail.map(p => [p[0], p[1], 3.03]), '#e2483d', .9, '2 1.4');
  let over = '';
  if (st.mode === 'op'){ over += tipHandle(F.tip, 3, !!st.crash); if (st.crash) over += crashBurst(st.crash.at, st.crash.t); }
  return { under: board(0, 0, 62, 76, 3, 6), z0: 3, clip: pt, floor, over };
}

function scTick(st, dt, ui, lab){
  let moving = false; const nq = st.q.slice();
  for (let i = 0; i < 4; i++){ const d = st.qt[i] - st.q[i]; if (Math.abs(d) > 1e-4){ nq[i] += Math.sign(d)*Math.min(Math.abs(d), SC_VMAX[i]*st.speed*dt); moving = true; } }
  if (moving){ const c = scCheck(st, nq);
    if (c){ st.qt = st.q.slice(); st.qd = [0,0,0,0]; st.prog = []; st.crash = { at: c.at, t: 1 }; st.crashes++; lab.flash(c.why + ' — motion stopped'); }
    else { st.qd = nq.map((v, i) => (v - st.q[i]) / Math.max(dt, 1e-3)); st.q = nq; const tp = scFK(st.q).tip, l = st.trail[st.trail.length - 1];
      if (!l || Math.hypot(tp[0] - l[0], tp[1] - l[1]) > .5){ st.trail.push(tp); if (st.trail.length > 400) st.trail.shift(); } } }
  else st.qd = [0,0,0,0];
  if (st.prog.length){ st.cycle += dt;
    if (!moving){ const s = st.prog[0];
      if (!s.started){ s.started = true; let r = scIK(s.x, s.y, st.cfg);
        if (r.err){ const r2 = scIK(s.x, s.y, -st.cfg); if (!r2.err){ r = r2; st.cfg = -st.cfg; } }
        if (r.err){ st.prog = []; lab.flash(r.err); } else st.qt = [r.q[0], r.q[1], s.z, 0]; moving = true; }
      else { if (s.vac != null) setVac(st, s.vac, lab); st.prog.shift(); if (!st.prog.length){ st.cycles++; st.lastCycle = st.cycle; } moving = true; } }
    else moving = true; }
  if (st.crash){ st.crash.t -= dt/1.2; if (st.crash.t <= 0) st.crash = null; else moving = true; }
  return moving || st.dragging;
}
function setVac(st, on, lab){ st.vac = on; const t = scFK(st.q).tip;
  if (on){ const p = st.pucks.find(p => !p.held && Math.hypot(p.x - t[0], p.y - t[1]) < 2 && Math.abs(st.q[2] - (p.z + 1.6)) < .4);
    if (p){ p.held = true; lab.flash('Vacuum on: part picked'); } else lab.flash('Vacuum on — nothing under the cup'); }
  else { const p = st.pucks.find(p => p.held); if (p){ p.held = false; p.x = t[0]; p.y = t[1]; p.z = st.q[2] - 1.6; } } }
function scPickPlace(st, lab){
  if (st.prog.length) return lab.flash('Already running a cycle');
  if (st.pucks.some(p => p.held)) return lab.flash('Drop the part first (Vacuum off)');
  const i = st.pucks.findIndex(p => !inTray([p.x, p.y]));
  if (i < 0) return lab.flash('All parts are in the tray — press Reset parts');
  const p = st.pucks[i], dst = SC_TRAY.find(d => !st.pucks.some(u => Math.hypot(u.x - d[0], u.y - d[1]) < 3));
  st.cycle = 0; st.prog = [{ x: p.x, y: p.y, z: 8 }, { x: p.x, y: p.y, z: p.z + 1.6, vac: true }, { x: p.x, y: p.y, z: 8 }, { x: dst[0], y: dst[1], z: 8 }, { x: dst[0], y: dst[1], z: 2.2, vac: false }, { x: dst[0], y: dst[1], z: 10 }]; }

ROBOTS.scara = {
  id: 'scara', fig: '02', name: 'SCARA assembly arm', pal: 'cobalt', kicker: '4 DOF · 2 rotary + ball-screw Z + spline R',
  blurb: 'The fast pick-and-place arm of electronics factories. Two arms swing in a flat plane like a human shoulder and elbow, and a combined ball-screw / ball-spline quill moves the tool up, down and round.',
  size: [62, 76, 64], center: [31, 38], bounds: [6, 50, 20, 60, 0, 60],
  init: () => ({ q: SC_HOME.slice(), qt: SC_HOME.slice(), qd: [0,0,0,0], speed: 1, cfg: 1, trail: [], trace: true, ws: true, crash: null, crashes: 0, prog: [], cycle: 0, cycles: 0, lastCycle: 0, vac: false,
    pucks: SC_FEED.map(([x, y]) => ({ x, y, z: 0, held: false })) }),
  steps: [
    { title: 'Pedestal drive: J1 servo & harmonic drive', parts: ['j1motor', 'j1hd'], text: 'The J1 servo stands in the pedestal and drives a strain-wave gear at the top through a short shaft. All of link 1 and link 2 swing on this one joint.' },
    { title: 'Servo amplifiers & power supply', parts: ['amps', 'psu'], text: 'Compact SCARAs carry their drives inside the pedestal: boards with power transistors that turn the controller’s commands into motor currents.' },
    { title: 'Pedestal cover', parts: ['pedestal'], text: 'The cover closes the column. Turn on See-through to keep the drives in view.' },
    { title: 'J1 output flange', parts: ['j1out'], text: 'The harmonic drive’s output flange: link 1 bolts on top.' },
    { title: 'Link 1 with the J2 drive', parts: ['j2motor', 'j2hd', 'link1'], text: 'Link 1 is 220 mm between axes. At its far end sits the J2 servo and its own harmonic drive, so link 2 can swing ±145 degrees.' },
    { title: 'Link 2 drives (inside)', parts: ['zmotor', 'zbelt', 'rmotor', 'rbelt', 'nut'], peek: true, text: 'Two motors ride on link 2. One belt spins the ball-screw nut (the quill moves up and down), the other spins the ball-spline nut (the quill rotates). Turning both together rotates without lifting.' },
    { title: 'Link 2 casting', parts: ['link2'], text: 'Link 2 (180 mm) closes over its two drives. It is above link 1 so it can pass over it.' },
    { title: 'Quill (ball screw + spline shaft)', parts: ['quill'], text: 'One hardened shaft has a screw thread and spline grooves cut into it. It passes straight through link 2: thread below, stop collar on top.' },
    { title: 'Vacuum cup & line', parts: ['cup'], text: 'A rubber suction cup picks flat parts; a vacuum line runs back up to link 2. The yellow tab shows the tool’s rotation (R axis).' },
    { title: 'Cable duct', parts: ['duct'], text: 'The flexible duct carries power, encoder and air from the pedestal to link 2, arcing high so it never snags while the arm swings.' },
    { title: 'Tray & parts', parts: ['tray', 'pucks'], text: 'Three parts on the feeder spots and an empty tray. In Operate mode, run Pick & place or move parts yourself.' },
  ],
  parts: {
    j1motor: { name: 'J1 servo motor', kind: 'motor', lbl: 'J1 SERVO', inside: true, short: '400 W AC servo', info: 'Swings the whole arm. Fast SCARAs reach 400–600 degrees per second at J1, so the motor is sized for acceleration, not gravity: in the horizontal plane gravity does no work.', held: 'The pedestal floor' },
    j1hd: { name: 'J1 harmonic drive', kind: 'gear', lbl: 'J1 HARMONIC DRIVE', inside: true, short: '80:1 strain wave', info: 'Zero-backlash reduction so the tool repeats its position within ±10 micrometres.', held: 'Top of the pedestal', spec: '80:1' },
    amps: { name: 'Servo amplifier boards', kind: 'electronics', lbl: 'SERVO AMPS', inside: true, short: '4 axes of drive electronics', info: 'Each axis has an amplifier that reads its encoder and switches the motor current thousands of times a second (PWM) to hold the commanded position.', held: 'Rails inside the pedestal' },
    psu: { name: 'Power supply', kind: 'power', lbl: 'POWER SUPPLY', inside: true, short: '48 V DC bus', info: 'Turns mains AC into the DC bus the amplifiers switch.', held: 'The pedestal floor' },
    pedestal: { name: 'Pedestal', kind: 'cover', lbl: 'PEDESTAL', key: true, short: 'base column', info: 'A rigid cast column bolted to the bench. Every vibration of the column shows up magnified at the tool.', held: 'The base plate' },
    j1out: { name: 'J1 output flange', kind: 'structure', short: 'link 1 mount', info: 'Steel flange on the harmonic drive output.', held: 'The J1 harmonic drive' },
    link1: { name: 'Link 1 (inner arm)', kind: 'structure', lbl: 'LINK 1', key: true, short: '220 mm, J1 to J2', info: 'L1 = 22 cm. With L2 it sets the workspace: an annulus from |L1−L2| out to L1 + L2 = 40 cm, trimmed by the joint limits (blue dots).', held: 'The J1 output flange', spec: 'L1 = 220 mm' },
    j2motor: { name: 'J2 servo', kind: 'motor', lbl: 'J2 SERVO', inside: true, short: '200 W servo', info: 'Drives the elbow. It rides on link 1, so J1 must accelerate its mass too.', held: 'Inside the tip of link 1' },
    j2hd: { name: 'J2 harmonic drive', kind: 'gear', lbl: 'J2 HARMONIC DRIVE', inside: true, short: '50:1', info: 'A thinner strain-wave set; its output carries link 2.', held: 'Link 1', spec: '50:1' },
    link2: { name: 'Link 2 (outer arm)', kind: 'structure', lbl: 'LINK 2', key: true, short: '180 mm, J2 to quill', info: 'L2 = 18 cm. Elbow-left and elbow-right (the Elbow config buttons) reach the same point with different arm shapes.', held: 'The J2 harmonic drive', spec: 'L2 = 180 mm' },
    zmotor: { name: 'Z motor', kind: 'motor', lbl: 'Z MOTOR', inside: true, short: 'drives the ball-screw nut', info: 'A small servo with a holding brake: when power is cut the quill must not drop onto the part.', held: 'Link 2' },
    zbelt: { name: 'Z timing belt', kind: 'gear', lbl: 'Z BELT', inside: true, short: '2:1 belt reduction', info: 'A toothed belt from the Z motor to the ball-screw nut, with a bigger pulley on the nut for a 2:1 reduction.', held: 'Pulleys on the Z motor and nut' },
    rmotor: { name: 'R motor', kind: 'motor', lbl: 'R MOTOR', inside: true, short: 'drives the spline nut', info: 'Rotates the quill so parts can be turned before placing.', held: 'Link 2' },
    rbelt: { name: 'R timing belt', kind: 'gear', lbl: 'R BELT', inside: true, short: 'to the spline nut', info: 'The second belt, under the first.', held: 'Pulleys on the R motor and spline nut' },
    nut: { name: 'Ball-screw & spline nuts', kind: 'gear', lbl: 'SCREW + SPLINE NUTS', inside: true, short: 'two nuts, one shaft', info: 'Spin only the screw nut: the shaft moves up or down (20 mm per turn). Spin only the spline nut: the shaft turns and also moves (the screw nut is still), so the controller spins both to rotate without lifting.', held: 'Bearings in link 2', spec: 'Lead 20 mm' },
    quill: { name: 'Quill shaft', kind: 'structure', lbl: 'QUILL', key: true, short: 'ball screw + ball spline', info: 'The vertical axis: stiff top to bottom, compliant sideways. That “selective compliance” lets the arm push pins into holes that are slightly off.', held: 'The two nuts in link 2', spec: 'Stroke 160 mm' },
    cup: { name: 'Vacuum cup', kind: 'tool', lbl: 'VACUUM CUP', key: true, short: 'suction gripper', info: 'A venturi makes a vacuum when air is blown through it; atmospheric pressure then pushes the part onto the cup. Fast and gentle for flat parts.', held: 'The end of the quill' },
    duct: { name: 'Cable duct', kind: 'wiring', lbl: 'CABLE DUCT', key: true, short: 'pedestal to link 2', info: 'Carries the motor cables, encoder cables and the vacuum line over the arm.', held: 'Pedestal top and link 2' },
    tray: { shadow: 'tray', name: 'Parts tray', kind: 'structure', short: 'drop-off', info: 'The destination for the pick-and-place cycle.', held: 'The bench' },
    pucks: { shadow: 'pucks', name: 'Parts', kind: 'tool', short: 'to be moved', info: 'Three test parts. Pick them with the vacuum cup; lowering onto them too far is a crash.', held: 'The feeder spots' },
  },
  concepts: [
    ['Why a SCARA?', 'Two rotary joints with vertical axes move the tool in a horizontal plane; Z and R are separate. Gravity never loads J1 or J2, so the arm can be light and very fast: cycle times under half a second.'],
    ['Two-link inverse kinematics', 'For a target (x, y) the elbow angle comes from the law of cosines: cos q2 = (x² + y² − L1² − L2²) / 2 L1 L2. Its sign picks elbow-left or elbow-right. Drag the yellow ring and watch both joints solve together.'],
    ['Workspace', 'The blue dots are every tip position the joint limits allow. Near the outer edge both links line up (a singularity): a small move of the tip needs a big move of the joints.'],
    ['Ball screw', 'A screw turning in a nut full of recirculating balls: little friction, no backlash. Torque needed = force × lead / (2π × efficiency), so a tiny motor can hold the quill.'],
    ['Joint-space vs straight-line moves', 'This lab moves each joint at its own top speed (joint interpolation), so the tip follows a curve, not a straight line. Watch the red trace during Pick & place.'],
  ],
  draw: scDraw,
  active: st => { const a = {}; const nm = [['j1motor', 'j1hd'], ['j2motor', 'j2hd'], ['zmotor', 'zbelt', 'nut'], ['rmotor', 'rbelt', 'nut']];
    (st.qd || []).forEach((v, i) => { if (Math.abs(v) > .3) nm[i].forEach(p => a[p] = true); }); if (st.vac) a.cup = true; return a; },
  enterOp: st => { st.qt = st.q.slice(); },
  opHTML: () => `<div class="row2"><button id="scpp">Pick &amp; place</button><button id="scvac">Vacuum on</button><button id="screset">Reset parts</button><button id="schome">Home</button></div>
    <div class="seg" id="sccfg"><button data-c="1">Elbow right</button><button data-c="-1">Elbow left</button></div>
    <p class="note">Drag the yellow ring at the cup to move in the horizontal plane; use Z to go down onto a part.</p>
    ${SC_NAMES.map((n, i) => slHTML('sc' + i, n, SC_LIM[i][0], SC_LIM[i][1], i === 2 ? .1 : 1, i === 2 ? ' cm' : '°')).join('')}
    ${slHTML('scspd', 'Speed override', 10, 100, 5, '%')}
    <div class="row2"><button id="scws" aria-pressed="true">Workspace</button><button id="sctr" aria-pressed="true">Trace</button><button id="scclr">Clear trace</button></div>`,
  opBind(el, st, ui, lab){
    const $ = id => document.getElementById(id);
    for (let i = 0; i < 4; i++) $('s_sc' + i).oninput = e => { st.prog = []; st.qt[i] = +e.target.value; lab.kick(); };
    $('s_scspd').oninput = e => { st.speed = +e.target.value / 100; ui.render(); };
    $('scpp').onclick = () => { scPickPlace(st, lab); lab.kick(); ui.render(); };
    $('scvac').onclick = () => { setVac(st, !st.vac, lab); ui.render(); };
    $('screset').onclick = () => { st.prog = []; st.vac = false; st.pucks = SC_FEED.map(([x, y]) => ({ x, y, z: 0, held: false })); ui.render(); };
    $('schome').onclick = () => { st.prog = []; st.qt = SC_HOME.slice(); lab.kick(); };
    el.querySelectorAll('#sccfg button').forEach(b => b.onclick = () => { const c = +b.dataset.c; if (c === st.cfg) return;
      const t = scFK(st.qt).tip, r = scIK(t[0], t[1], c); if (r.err) return lab.flash('Elbow flip: ' + r.err); st.cfg = c; st.qt = [r.q[0], r.q[1], st.qt[2], st.qt[3]]; lab.kick(); ui.render(); });
    $('scws').onclick = () => { st.ws = !st.ws; ui.render(); }; $('sctr').onclick = () => { st.trace = !st.trace; ui.render(); }; $('scclr').onclick = () => { st.trail = []; ui.render(); };
  },
  opSync(el, st){
    const $ = id => document.getElementById(id);
    for (let i = 0; i < 4; i++) slSync('sc' + i, st.qt[i], i === 2 ? `${st.q[2].toFixed(1)} cm` : `${st.q[i].toFixed(1)}°`, Math.abs(st.q[i] - SC_LIM[i][0]) < .3 || Math.abs(st.q[i] - SC_LIM[i][1]) < .3);
    slSync('scspd', Math.round(st.speed*100), Math.round(st.speed*100) + '%');
    $('scvac').textContent = st.vac ? 'Vacuum off' : 'Vacuum on'; $('scvac').setAttribute('aria-pressed', st.vac);
    el.querySelectorAll('#sccfg button').forEach(b => b.setAttribute('aria-pressed', +b.dataset.c === st.cfg));
    $('scws').setAttribute('aria-pressed', st.ws); $('sctr').setAttribute('aria-pressed', st.trace);
  },
  readout(st){
    const F = scFK(st.q), r = Math.hypot(F.tip[0] - SC.BX, F.tip[1] - SC.BY), rows = [];
    SC_NAMES.forEach((n, i) => rows.push({ k: n.split(' ')[0] + ' ' + (i === 2 ? st.q[2].toFixed(1) + ' cm' : st.q[i].toFixed(0) + '°'), bar: Math.abs(st.qd[i]) / SC_VMAX[i], v: i === 2 ? `${Math.abs(st.qd[2]*10).toFixed(0)} mm/s` : `${Math.abs(st.qd[i]).toFixed(0)}°/s` }));
    rows.push({ k: 'Tip X Y', v: `${((F.tip[0]-SC.BX)*10).toFixed(0)} ${((F.tip[1]-SC.BY)*10).toFixed(0)} mm` });
    rows.push({ k: 'Radius from J1', bar: r / (SC.L1 + SC.L2), v: `${(r*10).toFixed(0)} mm`, warn: r > SC.L1 + SC.L2 - .6, tip: 'Near 400 mm the links line up: a singularity' });
    const load = .8 + (st.pucks.some(p => p.held) ? .05 : 0), tq = load*9.81*.02/(2*Math.PI*.9);
    rows.push({ k: 'Z motor holding torque', bar: tq / .08, v: `${(tq*1000).toFixed(0)} mN m`, tip: 'Force x lead / (2 pi x efficiency), before the 2:1 belt' });
    rows.push({ k: 'Vacuum', v: st.vac ? 'ON' : 'off' });
    rows.push({ k: 'Cycle time', v: st.prog.length ? st.cycle.toFixed(2) + ' s' : st.lastCycle ? st.lastCycle.toFixed(2) + ' s' : '—' });
    rows.push({ k: 'Cycles / crashes', v: `${st.cycles} / ${st.crashes}`, warn: !!st.crash });
    return rows;
  },
  stats: st => { const t = scFK(st.q).tip; return [{ label: 'Cup X Y Z mm', value: `${Math.round((t[0]-SC.BX)*10)} ${Math.round((t[1]-SC.BY)*10)} ${Math.round(st.q[2]*10)}`, small: true }]; },
  tick: scTick,
  onDrag(st, dx, dy, phase, lab){
    if (phase === 'start'){ st.prog = []; st.dragP = scFK(st.qt).tip; return; } if (phase !== 'move') return;
    const d = dragPlane(dx, dy), x = st.dragP[0] + d[0], y = st.dragP[1] + d[1], r = scIK(x, y, st.cfg);
    if (r.err) return lab.flash(r.err); st.dragP = [x, y, st.dragP[2]]; st.qt = [r.q[0], r.q[1], st.qt[2], st.qt[3]]; },
  params(st, q){ if (q.has('q')){ const v = q.get('q').split(',').map(Number); if (v.length === 4) st.q = st.qt = v.slice(); }
    if (q.get('demo') === 'pp') scPickPlace(st, { flash(){} }); },
};
ROBOT_ORDER.push({ id: 'scara', name: 'SCARA assembly arm', blurb: 'Two arms swinging in a flat plane plus a ball-screw quill. Run a pick-and-place cycle, flip the elbow, see the workspace and feel why gravity never loads J1 and J2.', chips: ['4 DOF', 'Ball screw', 'Planar IK', 'Pick & place'] });
