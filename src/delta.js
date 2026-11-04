/* ============================================================
   Robot 06 — delta picker (three-arm parallel robot, ~36 cm working circle). 1 unit = 1 cm.
   Truth: a stiff frame holds a top plate. Under it, in a sealed drive housing, three servo motors with planetary
   gearboxes sit 120 degrees apart, each turning one short, stiff upper arm. From each arm's elbow two thin carbon rods run
   down to the small moving platform; the rods end in ball joints held together by springs. Each pair of rods is a
   parallelogram, so the platform can only translate: it stays level whatever the arms do. All three motors share the
   load, and the moving parts are very light: that is how deltas reach 3–5 picks per second.
   Parts: frame | housing, motors, gearboxes | arms | balls | rods, springs | plate | cup, vac | pads, pucks
   ============================================================ */
const DL = { C: 30, R: 8, RF: 13, RE: 34, r: 4.2, ZM: 46, ZT: 52, W: 2, CUP: 2.6, ZMIN: 6, ZMAX: 21, RMAX: 16, VMAX: 400, AMAX: 6000 };
const DL_PHI = [90, 210, 330].map(a => a*DEG);
const DL_LIM = [-50, 95];
const DL_A = [18, 30], DL_B = [42, 30], DL_PAD = 4, DL_PUCK = 1.1, DL_UP = 3.5;
const dlU = i => [Math.cos(DL_PHI[i]), Math.sin(DL_PHI[i]), 0], dlT = i => [-Math.sin(DL_PHI[i]), Math.cos(DL_PHI[i]), 0];
const dlM = i => [DL.C + DL.R*Math.cos(DL_PHI[i]), DL.C + DL.R*Math.sin(DL_PHI[i]), DL.ZM];

// inverse kinematics: platform centre P -> three arm angles (deg, positive = down); null if any arm cannot reach
function dlIK(P){ const th = [];
  for (let i = 0; i < 3; i++){ const u = dlU(i), t = dlT(i), d = [P[0] - DL.C, P[1] - DL.C];
    const a = d[0]*u[0] + d[1]*u[1] + DL.r - DL.R, y0 = d[0]*t[0] + d[1]*t[1], z = P[2] - DL.ZM;
    const A = -2*a*DL.RF, B = 2*z*DL.RF, K = DL.RE**2 - DL.RF**2 - a*a - y0*y0 - z*z, h = Math.hypot(A, B);
    if (Math.abs(K) > h) return { err: 'Out of reach: a rod pair cannot stretch that far' };
    const base = Math.atan2(B, A), dd = Math.acos(K/h), s1 = base + dd, s2 = base - dd, s = Math.cos(s1) > Math.cos(s2) ? s1 : s2;
    const deg = wrap180(s/DEG); if (deg < DL_LIM[0] || deg > DL_LIM[1]) return { err: `Arm ${i + 1} would pass its limit` }; th.push(deg); }
  return { th }; }
const dlElbow = (i, th) => vAdd(dlM(i), vAdd(vSc(dlU(i), DL.RF*Math.cos(th*DEG)), [0, 0, -DL.RF*Math.sin(th*DEG)]));
function dlClamp(P){ const d = [P[0] - DL.C, P[1] - DL.C], r = Math.hypot(d[0], d[1]), k = r > DL.RMAX ? DL.RMAX/r : 1;
  return [DL.C + d[0]*k, DL.C + d[1]*k, clamp(P[2], DL.ZMIN, DL.ZMAX)]; }
const dlStackTop = n => DL_PAD + n*DL_PUCK;
const DL_PUCKC = ['#e2483d', '#5a82c6', '#43c25c', '#f4b829', '#d8869c'];

function dlDraw(st, m){
  const P = st.P, th = st.th, C = DL.C;
  // ---- frame: three columns, top beams, top plate
  sbPart('frame'); for (const a of [30, 150, 270].map(a => a*DEG)){ const x = C + 25*Math.cos(a), y = C + 25*Math.sin(a);
    qBox(T_I, [x - 1.1, y - 1.1, 3], [x + 1.1, y + 1.1, DL.ZT + 1.4], ALU, { w: 1 });
    qBox(T_I, [x - 2.2, y - 2.2, 3], [x + 2.2, y + 2.2, 3.6], STEEL, { w: .7 });
    qRod([x, y, DL.ZT + .7], [C + 13*Math.cos(a), C + 13*Math.sin(a), DL.ZT + .7], .9, ALU, { w: .9 }); }
  qExt(T_I, ellUV(C, C, 16, 16, 6), DL.ZT, DL.ZT + 1.4, ALU, { ax: 'z', w: 1.6 });
  // ---- drive housing: motors, gearboxes and encoders (shaft along the arm's tangent)
  sbGroup('drive', T_I, [C - 15, C - 15, DL.ZM - 3.4], [C + 15, C + 15, DL.ZT]);
  for (let i = 0; i < 3; i++){ const M = dlM(i), t = dlT(i);
    sbPart('gearboxes'); qRod(vAdd(M, vSc(t, -1.5)), vAdd(M, vSc(t, -4.4)), 2.3, REDUCER, { w: .9 });
    sbPart('motors'); qRod(vAdd(M, vSc(t, -4.4)), vAdd(M, vSc(t, -9.6)), 1.9, MOTOR, { w: .9 });
    sbPart('encoders'); qRod(vAdd(M, vSc(t, -9.6)), vAdd(M, vSc(t, -10.6)), 1.6, DARK, { w: .7 });
    sbPart('motors'); qBox(T_I, [M[0] - .8, M[1] - .8, DL.ZM + 2], [M[0] + .8, M[1] + .8, DL.ZT], ALU, { w: .6 }); }
  sbPart('housing'); qExt(T_I, ellUV(C, C, 14.6, 14.6, 30), DL.ZM - 3.4, DL.ZT, m, { ax: 'z', cover: true, w: 2,
    capDeco: () => '' });
  sbFree(); if (facing([0, -1, 0])) sbPut(txT(T_I, [C, C - 14.7, DL.ZM], [1,0,0], [0,0,1], 'DELTA-3', 2.6), obbW([C - 6, C - 14.8, DL.ZM - 1], [C + 6, C - 14.6, DL.ZM + 1]), { p: 'housing' });
  // ---- arms, elbow crossbars + balls, rod pairs with springs
  const pl = (i, s) => vAdd(vAdd(P, vSc(dlU(i), DL.r)), vSc(dlT(i), s*DL.W));
  for (let i = 0; i < 3; i++){ const M = dlM(i), t = dlT(i), J = dlElbow(i, th[i]);
    sbPart('arms'); qRod(vAdd(M, vSc(t, -1.5)), vAdd(M, vSc(t, 1.5)), 1.6, STEEL, { w: .8 });
    qRod(M, J, 1, m, { r1: .7, sh: tone, w: 1.1 });
    qRod(vAdd(J, vSc(t, -DL.W)), vAdd(J, vSc(t, DL.W)), .45, STEEL, { w: .6 });
    for (const s of [-1, 1]){ const a = vAdd(J, vSc(t, s*DL.W)), b = pl(i, s);
      sbPart('balls'); qBall(a, .55, STEEL); qBall(b, .5, STEEL);
      sbPart('rods'); qRod(vLerp(a, b, .03), vLerp(b, a, .03), .32, DARK, { w: .7 }); }
    sbPart('springs'); const a = vAdd(J, vSc(t, -DL.W)), b = pl(i, -1), c = vAdd(J, vSc(t, DL.W)), d = pl(i, 1);
    for (const k of [.12, .88]) qHelix(vLerp(a, b, k), vLerp(c, d, k), .22, 7, '#c6ccce', { w: .5 }); }
  // ---- platform, cup, vacuum line
  sbPart('plate'); qExt(T_I, ellUV(P[0], P[1], 5.4, 5.4, 6), P[2] - .45, P[2] + .45, m, { ax: 'z', w: 1.3, sh: tone });
  for (let i = 0; i < 3; i++) qRod(pl(i, -1), pl(i, 1), .3, STEEL, { w: .5 });
  sbPart('cup'); qRod([P[0], P[1], P[2] - .45], [P[0], P[1], P[2] - 1.6], .7, DARK, { w: .7 }); qRod([P[0], P[1], P[2] - 1.6], [P[0], P[1], P[2] - DL.CUP], .6, RUBBER, { r1: 1.4, w: .8 });
  sbPart('vac'); qWire([[P[0] + .6, P[1], P[2] - .9], [P[0] + 2, P[1] + 1, P[2] + 4], [P[0]*.5 + C*.5 + 3, P[1]*.5 + C*.5, (P[2] + DL.ZM)/2 + 4], [C + 4, C + 3, DL.ZM - 3.4]], st.vac ? '#8af0ae' : CABLE.air, .3, { seg: 6 });
  // ---- pick pads and pucks
  sbPart('pads'); for (const [x, y] of [DL_A, DL_B]) qBox(T_I, [x - 3.2, y - 3.2, 3], [x + 3.2, y + 3.2, DL_PAD], matOf('#9db2cc'), { w: .9 });
  sbPart('pucks'); let k = 0;
  for (const [key, c] of [['A', DL_A], ['B', DL_B]]) for (let j = 0; j < st.stack[key]; j++){ const z = dlStackTop(j);
    qRod([c[0], c[1], z], [c[0], c[1], z + DL_PUCK - .05], 2.3, matOf(DL_PUCKC[(k++) % 5]), { w: .9, sh: tone, p: 'pucks' }); }
  if (st.held) qRod([P[0], P[1], P[2] - DL.CUP - DL_PUCK + .05], [P[0], P[1], P[2] - DL.CUP], 2.3, matOf('#f2efe7'), { w: .9, sh: tone });
  const pt = boardTop(0, 0, 60, 60, 3, 6); let floor = '', over = '';
  if (st.mode === 'op'){ floor += `<path d="${pd(on(PZ(3.02), ellUV(C, C, DL.RMAX, DL.RMAX, 48)))}" fill="none" stroke="#3d5878" stroke-width=".7" stroke-dasharray="1.6 1.2"/>`;
    if (st.trace) over += trailSVG(st.trail, '#e2483d', .9);
    over += tipHandle([P[0], P[1], P[2] - DL.CUP], 3, !!st.crash); if (st.crash) over += crashBurst(st.crash.at, st.crash.t); }
  return { under: board(0, 0, 60, 60, 3, 6), z0: 3, clip: pt, floor, over };
}

// a pick & place cycle: the classic "adept" path, up, across, down
function dlCycle(st, lab){
  const from = st.stack.A > 0 ? 'A' : 'B'; if (!st.stack[from]) return lab.flash('No parts to move');
  const to = from === 'A' ? 'B' : 'A', fc = from === 'A' ? DL_A : DL_B, tc = to === 'A' ? DL_A : DL_B;
  if (from !== st.dir){ st.dir = from; }
  const pz = dlStackTop(st.stack[from]) + DL.CUP, dz = dlStackTop(st.stack[to]) + DL_PUCK + DL.CUP, hi = Math.max(pz, dz) + DL_UP;
  st.prog = [{ p: [fc[0], fc[1], hi] }, { p: [fc[0], fc[1], pz], vac: true }, { p: [fc[0], fc[1], hi] }, { p: [tc[0], tc[1], hi] }, { p: [tc[0], tc[1], dz], vac: false }, { p: [tc[0], tc[1], hi] }];
  st.cycle = 0; }
function dlTick(st, dt, ui, lab){
  let moving = false;
  if (!st.prog.length && st.run && !st.dragging) dlCycle(st, lab);
  const goal = st.prog.length ? st.prog[0].p : st.goal;
  if (goal){ const d = sub(goal, st.P), L = vLen(d), vmax = DL.VMAX*st.speed, amax = DL.AMAX*st.speed*st.speed;
    if (L > 1e-3){ const vstop = Math.sqrt(2*amax*L); const v = Math.min(vmax, st.vel + amax*dt, vstop); const stepL = Math.min(L, v*dt);
      const np = vAdd(st.P, vSc(d, stepL/L)), r = dlIK(np), hit = dlHit(st, np);
      if (hit){ st.prog = []; st.goal = null; st.run = false; st.vel = 0; st.crash = { at: hit.at, t: 1 }; st.crashes++; lab.flash(hit.why + ' — motion stopped'); }
      else if (r.err){ st.prog = []; st.goal = null; st.run = false; st.vel = 0; lab.flash(r.err); }
      else { st.acc = (v - st.vel)/Math.max(dt, 1e-3); st.thd = r.th.map((a, i) => (a - st.th[i])/Math.max(dt, 1e-3)); st.th = r.th; st.P = np; st.vel = v; moving = true;
        const l = st.trail[st.trail.length - 1], tp = [np[0], np[1], np[2] - DL.CUP]; if (!l || vLen(sub(tp, l)) > .4){ st.trail.push(tp); if (st.trail.length > 260) st.trail.shift(); } } }
    else { st.vel = 0; st.P = goal.slice();
      if (st.prog.length){ const s = st.prog.shift(); if (s.vac != null) dlVac(st, s.vac, lab); moving = true;
        if (!st.prog.length){ st.cycles++; st.lastCycle = st.cycle; } }
      else st.goal = null; } }
  if (!moving){ st.thd = [0, 0, 0]; st.acc = 0; }
  if (st.prog.length) st.cycle += dt;
  if (st.crash){ st.crash.t -= dt/1.2; if (st.crash.t <= 0) st.crash = null; else moving = true; }
  return moving || st.run || st.dragging || !!st.crash;
}
function dlVac(st, on, lab){ st.vac = on; const P = st.P;
  const at = Math.hypot(P[0] - DL_A[0], P[1] - DL_A[1]) < 1.5 ? 'A' : Math.hypot(P[0] - DL_B[0], P[1] - DL_B[1]) < 1.5 ? 'B' : null;
  if (on && !st.held && at && st.stack[at] && Math.abs(P[2] - DL.CUP - dlStackTop(st.stack[at])) < .5){ st.stack[at]--; st.held = true; }
  else if (on && !st.held) lab.flash('Vacuum on — nothing under the cup');
  if (!on && st.held){ st.held = false; if (at) st.stack[at]++; else lab.flash('Part dropped off the pads'); } }
function dlHit(st, P){ const cz = P[2] - DL.CUP - (st.held ? DL_PUCK : 0);
  for (const [key, c] of [['A', DL_A], ['B', DL_B]]) if (Math.abs(P[0] - c[0]) < 3.2 + 1.4 && Math.abs(P[1] - c[1]) < 3.2 + 1.4 && cz < dlStackTop(st.stack[key]) - .05) return { why: `Crash: the cup pressed into pad ${key}`, at: [P[0], P[1], dlStackTop(st.stack[key])] };
  if (cz < 3) return { why: 'Crash: the cup hit the table', at: [P[0], P[1], 3] };
  return null; }

const DL_HOME = [30, 30, 18];
ROBOTS.delta = {
  id: 'delta', fig: '06', name: 'Delta picker', pal: 'mustard', kicker: '3 DOF parallel · closed chain · platform always level',
  blurb: 'Three motors on a fixed plate swing three short arms; pairs of thin rods carry a tiny platform that can only slide, never tilt. Everything that moves is light, so it picks parts several times a second.',
  size: [60, 60, 62], center: [30, 30], bounds: [4, 56, 4, 56, 0, 60],
  init: () => { const r = dlIK(DL_HOME); return { P: DL_HOME.slice(), th: r.th, thd: [0, 0, 0], vel: 0, acc: 0, goal: null, prog: [], run: false, speed: .25, vac: false, held: false,
    stack: { A: 5, B: 0 }, dir: 'A', cycle: 0, cycles: 0, lastCycle: 0, trail: [], trace: true, crash: null, crashes: 0 }; },
  thumbYaw: 0,
  steps: [
    { title: 'Frame & top plate', parts: ['frame'], text: 'Three aluminium columns hold a stiff hexagonal plate. Everything else hangs from it, so it must not flex: any wobble shows at the platform.' },
    { title: 'Motors, gearboxes, encoders', parts: ['motors', 'gearboxes', 'encoders'], peek: true, text: 'Three servo motors hang under the plate 120 degrees apart, each through a planetary gearbox. Their shafts run sideways (tangent to the circle) so each arm swings in its own vertical plane.' },
    { title: 'Drive housing', parts: ['housing'], text: 'A sealed cover keeps the drives clean: deltas often work over food.' },
    { title: 'Upper arms', parts: ['arms'], text: 'Short, stiff arms (120 mm) clamp onto the gearbox outputs. Each ends in a crossbar for the two rod ends.' },
    { title: 'Ball joints', parts: ['balls'], text: 'Steel balls on the crossbars and on the platform. The rod ends are cups that snap over them.' },
    { title: 'Parallelogram rods & springs', parts: ['rods', 'springs'], text: 'Two carbon rods per arm, 300 mm long, kept parallel and equal: a parallelogram. Springs pull each pair of cups onto its balls.' },
    { title: 'Moving platform', parts: ['plate'], text: 'The small platform hangs on the six rod ends. Because each rod pair is a parallelogram it can move in X, Y and Z but never tilt.' },
    { title: 'Vacuum cup & line', parts: ['cup', 'vac'], text: 'A suction cup under the platform and a light hose up to the plate.' },
    { title: 'Pads & parts', parts: ['pads', 'pucks'], text: 'Five parts stacked on pad A. In Operate, press Run to move them to pad B and back.' },
  ],
  parts: {
    frame: { name: 'Frame & top plate', kind: 'structure', lbl: 'FRAME', key: true, short: 'three columns + plate', info: 'The robot pushes against its own frame at every acceleration. A delta at 10 g needs a frame as stiff as a machine tool.', held: 'Bolted to the table' },
    motors: { name: 'Servo motors (×3)', kind: 'motor', lbl: 'SERVO MOTORS', inside: true, short: '750 W AC servos', info: 'Each motor turns one arm. They never move: the heaviest parts of the robot stay still, which is the whole trick of a parallel robot.', held: 'Brackets under the top plate' },
    gearboxes: { name: 'Planetary gearboxes', kind: 'gear', lbl: 'GEARBOXES', inside: true, short: '15:1, low backlash', info: 'A low ratio keeps the arms fast; the motors run at up to 3000 rpm, the arms at 200 rpm.', held: 'The motor flanges', spec: '15:1' },
    encoders: { name: 'Motor encoders', kind: 'sensor', lbl: 'ENCODERS', inside: true, short: '20-bit absolute', info: 'The controller knows only the three arm angles; the platform position comes from the forward kinematics.', held: 'The back of each motor' },
    housing: { name: 'Drive housing', kind: 'cover', lbl: 'DRIVE HOUSING', key: true, short: 'washdown cover', info: 'Sealed and smooth so it can be cleaned.', held: 'Under the top plate' },
    arms: { name: 'Upper arms', kind: 'structure', lbl: 'UPPER ARMS', key: true, short: '120 mm, stiff', info: 'The only links the motors swing directly. Arm angle 0 is horizontal; positive is down.', held: 'The gearbox output shafts', spec: 'rf = 120 mm' },
    balls: { name: 'Ball joints', kind: 'structure', lbl: 'BALL JOINTS', short: '12 ball-and-cup joints', info: 'Each ball joint is 3 rotational DOF with no backlash because the springs preload it.', held: 'Crossbars and platform studs' },
    rods: { name: 'Carbon rods', kind: 'structure', lbl: 'PARALLELOGRAM RODS', key: true, short: '300 mm, 6 rods', info: 'Rods only push and pull, so they can be thin, hollow and light. Two equal parallel rods per arm keep the platform level.', held: 'Ball joints at both ends', spec: 're = 300 mm' },
    springs: { name: 'Preload springs', kind: 'structure', short: 'hold cups on balls', info: 'If the platform crashes, the cups pop off instead of breaking something: snap them back on.', held: 'Hooks on the rod ends' },
    plate: { name: 'Moving platform', kind: 'tool', lbl: 'PLATFORM', key: true, short: 'translation only', info: 'Moving mass is just the platform, rods and tool: under 0.5 kg.', held: 'Six rod ends' },
    cup: { name: 'Vacuum cup', kind: 'tool', lbl: 'CUP', short: 'suction gripper', info: 'Picks flat parts in a few milliseconds.', held: 'The platform' },
    vac: { name: 'Vacuum hose', kind: 'wiring', short: 'to the valve', info: 'The valve sits up on the plate, not on the platform, to keep the moving mass low.', held: 'The cup and the top plate' },
    pads: { shadow: 'pads', name: 'Pads A and B', kind: 'structure', short: 'pick and place spots', info: 'Pad A on the left, B on the right, 240 mm apart.', held: 'The table' },
    pucks: { shadow: 'pucks', name: 'Parts', kind: 'tool', short: '5 stacked parts', info: 'Each pick lowers onto the top of the stack, so every cycle has a different depth.', held: 'The pads' },
  },
  concepts: [
    ['Parallel vs serial', 'A serial arm stacks motors on motors, each carrying the next. A parallel robot puts all motors on the base and joins them at the tool through several chains. Lighter and faster, but a smaller workspace.'],
    ['Inverse kinematics is easy here', 'For a platform point, each arm’s angle solves on its own: the elbow must lie on a sphere of radius re around its platform joint and on the arm’s circle. Two answers; the robot uses the elbow-out one. Drag the yellow ring and watch three angles change together.'],
    ['Forward kinematics is the hard way round', 'From three angles, the platform is where three spheres (one around each elbow) meet: a small equation to solve. Serial arms are the opposite: forward is easy, inverse is hard.'],
    ['Always level', 'Each pair of equal parallel rods is a parallelogram. Three parallelograms at 120 degrees lock all rotation, leaving only X, Y and Z.'],
    ['Speed and acceleration', 'Picks per minute are limited by acceleration, not top speed: on a 25-305-25 mm cycle the platform barely reaches full speed. Push the speed override up and watch the g readout.'],
  ],
  draw: dlDraw,
  active: st => { const a = {}; if (st.thd.some(v => Math.abs(v) > .5)) a.motors = a.gearboxes = a.arms = true; if (st.vac) a.cup = a.vac = true; return a; },
  enterOp: st => { st.goal = null; },
  opHTML: () => `<div class="row2"><button id="dlrun">Run</button><button id="dlone">One cycle</button><button id="dlvac">Vacuum on</button><button id="dlhome">Home</button><button id="dlreset">Reset parts</button></div>
    <p class="note">Drag the yellow ring under the platform to move it; the arms follow by inverse kinematics.</p>
    ${slHTML('dlx', 'X', -17, 17, .5, ' cm')}${slHTML('dly', 'Y', -17, 17, .5, ' cm')}${slHTML('dlz', 'Z (height above table)', DL.ZMIN - 3, DL.ZMAX - 3, .5, ' cm')}
    ${slHTML('dlspd', 'Speed override', 5, 100, 5, '%')}
    <div class="row2"><button id="dltr" aria-pressed="true">Trace</button><button id="dlclr">Clear trace</button></div>`,
  opBind(el, st, ui, lab){ const $ = id => document.getElementById(id);
    const go = p => { st.run = false; st.prog = []; st.goal = dlClamp(p); const c = dlHit(st, st.goal); if (c){ st.goal = null; st.crash = { at: c.at, t: 1 }; st.crashes++; lab.flash(c.why + ' — move refused'); } lab.kick(); };
    $('s_dlx').oninput = e => go([DL.C + +e.target.value, (st.goal || st.P)[1], (st.goal || st.P)[2]]);
    $('s_dly').oninput = e => go([(st.goal || st.P)[0], DL.C + +e.target.value, (st.goal || st.P)[2]]);
    $('s_dlz').oninput = e => go([(st.goal || st.P)[0], (st.goal || st.P)[1], +e.target.value + 3]);
    $('s_dlspd').oninput = e => { st.speed = +e.target.value/100; ui.render(); };
    $('dlrun').onclick = () => { st.run = !st.run; if (st.run && st.held) { st.run = false; return lab.flash('Drop the part first (Vacuum off)'); } lab.kick(); ui.render(); };
    $('dlone').onclick = () => { if (st.prog.length) return lab.flash('Already running a cycle'); if (st.held) return lab.flash('Drop the part first (Vacuum off)'); dlCycle(st, lab); lab.kick(); };
    $('dlvac').onclick = () => { dlVac(st, !st.vac, lab); ui.render(); };
    $('dlhome').onclick = () => go(DL_HOME);
    $('dlreset').onclick = () => { st.run = false; st.prog = []; st.held = false; st.vac = false; st.stack = { A: 5, B: 0 }; ui.render(); };
    $('dltr').onclick = () => { st.trace = !st.trace; ui.render(); }; $('dlclr').onclick = () => { st.trail = []; ui.render(); }; },
  opSync(el, st){ const $ = id => document.getElementById(id), g = st.goal || st.P;
    slSync('dlx', g[0] - DL.C, `${((st.P[0] - DL.C)*10).toFixed(0)} mm`); slSync('dly', g[1] - DL.C, `${((st.P[1] - DL.C)*10).toFixed(0)} mm`); slSync('dlz', g[2] - 3, `${((st.P[2] - 3)*10).toFixed(0)} mm`, st.P[2] <= DL.ZMIN + .01 || st.P[2] >= DL.ZMAX - .01);
    slSync('dlspd', Math.round(st.speed*100), Math.round(st.speed*100) + '%');
    $('dlrun').textContent = st.run ? 'Stop' : 'Run'; $('dlrun').setAttribute('aria-pressed', st.run);
    $('dlvac').textContent = st.vac ? 'Vacuum off' : 'Vacuum on'; $('dltr').setAttribute('aria-pressed', st.trace); },
  readout(st){ const rows = [];
    st.th.forEach((a, i) => rows.push({ k: `Arm ${i + 1} ${a.toFixed(1)}°`, bar: Math.abs(st.thd[i])/1200, v: `${(Math.abs(st.thd[i])/6).toFixed(0)} rpm`, warn: Math.abs(st.thd[i])/6 > 190, tip: 'Arm speed (motor turns 15x faster)' }));
    rows.push({ k: 'Platform speed', bar: st.vel/DL.VMAX, v: `${(st.vel/100).toFixed(2)} m/s` });
    rows.push({ k: 'Acceleration', bar: Math.abs(st.acc)/DL.AMAX, v: `${(Math.abs(st.acc)/981).toFixed(1)} g` });
    rows.push({ k: 'Platform tilt', v: '0.0° (always)', tip: 'Parallelograms lock rotation' });
    rows.push({ k: 'Stacks A / B', v: `${st.stack.A} / ${st.stack.B}${st.held ? ' + 1 held' : ''}` });
    rows.push({ k: 'Cycle time', v: st.prog.length ? st.cycle.toFixed(2) + ' s' : st.lastCycle ? st.lastCycle.toFixed(2) + ' s' : '—' });
    rows.push({ k: 'Picks per minute', v: st.lastCycle ? (60/st.lastCycle).toFixed(0) : '—', tip: 'At this speed override' });
    rows.push({ k: 'Cycles / crashes', v: `${st.cycles} / ${st.crashes}`, warn: !!st.crash });
    return rows; },
  stats: st => [{ label: 'Platform X Y Z mm', value: `${Math.round((st.P[0]-DL.C)*10)} ${Math.round((st.P[1]-DL.C)*10)} ${Math.round((st.P[2]-3)*10)}`, small: true }],
  tick: dlTick,
  onDrag(st, dx, dy, phase, lab){
    if (phase === 'start'){ st.run = false; st.prog = []; st.goal = null; st.dragP = st.P.slice(); return; } if (phase !== 'move') return;
    const p = dlClamp(vAdd(st.dragP, dragWorld(dx, dy))), c = dlHit(st, p); if (c) return lab.flash(c.why.replace('Crash: ', 'Blocked: '));
    const r = dlIK(p); if (r.err) return lab.flash(r.err); st.dragP = p; st.goal = p; },
  params(st, q){ if (q.has('p')){ const v = q.get('p').split(',').map(Number), r = dlIK(v); if (r.th){ st.P = v; st.th = r.th; } } if (q.get('demo') === 'run') st.run = true; },
};
ROBOT_ORDER.push({ id: 'delta', name: 'Delta picker', blurb: 'Three motors on a fixed top plate drive three parallelogram arms down to one small platform: extremely fast, always level. Run pick & place and watch the g-forces.', chips: ['3 DOF', 'Parallel', 'Closed chain'] });
