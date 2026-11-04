/* ============================================================
   Robot 08 — CNC gantry plotter (36 x 24 cm work area, pen on a Z axis). 1 unit = 1 cm.
   Truth: two aluminium extrusion rails on feet. A beam (the gantry) rides both rails on wheeled carriages; two X steppers,
   one per rail, pull it along by GT2 timing belts so it cannot twist. A Y stepper on the beam's end drives a belt to the
   Y carriage. On the carriage a small Z stepper turns a lead screw that raises and lowers the pen holder. Limit switches
   at the rail ends find home. A controller board with three stepper drivers reads G-code and sends step pulses.
   Steppers have no position sensor: the controller only counts the steps it sent. Ask for too much speed and a motor
   slips, and from then on the machine is wrong about where it is.
   Parts: frame, endstops | bed, paper | xmotors, xbelts | gantry | ymotor, ybelt | ycarr | zmotor, leadscrew | pen |
          ctrl, drivers, psu | case | cables
   ============================================================ */
const GT = { X0: 8, Y0: 8, XW: 36, YW: 24, PZ: 4.06, PUP: .8, OFF: 3.8, ACC: 150, RAPID: 15, STALLX: 16, STALLY: 20 };
const GT_MICRO = [1, 4, 16];
const gtSPmm = (axis, mu) => axis === 2 ? 25*mu : 5*mu;       // belts: 200 steps / 40 mm; Z lead screw: 200 steps / 8 mm

// a few drawings as G-code (machine coordinates in mm from the paper's corner)
function gtProgram(kind){ const P = [], g = (code, x, y, z, txt) => P.push({ code, x, y, z, txt });
  const pt = (x, y) => [Math.round(x*10)/10, Math.round(y*10)/10];
  let paths = [];
  if (kind === 'square') paths = [[[120, 60], [240, 60], [240, 180], [120, 180], [120, 60]]];
  if (kind === 'circle') paths = [Array.from({ length: 49 }, (_, i) => { const a = i/48*2*Math.PI; return pt(180 + 80*Math.cos(a), 120 + 80*Math.sin(a)); })];
  if (kind === 'star') paths = [Array.from({ length: 11 }, (_, i) => { const a = Math.PI/2 + i*4*Math.PI/5; return pt(180 + 95*Math.cos(a), 125 + 95*Math.sin(a)); })];
  if (kind === 'spiral') paths = [Array.from({ length: 121 }, (_, i) => { const a = i/120*6*Math.PI, r = 8 + 80*i/120; return pt(180 + r*Math.cos(a), 120 + r*Math.sin(a)); })];
  if (kind === 'grid') for (let k = 0; k <= 4; k++){ paths.push([[60, 40 + k*40], [300, 40 + k*40]]); paths.push([[60 + k*60, 40], [60 + k*60, 200]]); }
  g('G21', null, null, null, 'G21 ; units: mm'); g('G90', null, null, null, 'G90 ; absolute coordinates');
  for (const path of paths){ const [x0, y0] = path[0];
    g('G0', null, null, GT.PUP*10, `G0 Z${GT.PUP*10} ; pen up`); g('G0', x0, y0, null, `G0 X${x0} Y${y0}`); g('G1', null, null, 0, 'G1 Z0 ; pen down');
    for (const [x, y] of path.slice(1)) g('G1', x, y, null, `G1 X${x} Y${y}`); }
  g('G0', null, null, GT.PUP*10, `G0 Z${GT.PUP*10} ; pen up`); g('G0', 0, 0, null, 'G0 X0 Y0 ; park');
  return P; }

function gtDraw(st, m){
  const [px, py] = [GT.X0 + st.p[0] - st.err[0], GT.Y0 + st.p[1] - st.err[1]], X = px - GT.OFF, zt = GT.PZ + st.p[2];
  // ---- frame rails on feet, endstops
  sbPart('frame'); for (const y of [3, 37]){ qBox(T_I, [5, y - 1.3, 6], [49, y + 1.3, 8.6], ALU, { w: 1.1, deco: n => n[2] === 1 ? line([[5.4, y, 8.62], [48.6, y, 8.62]], .5) : '' });
    for (const x of [6, 48]) qBox(T_I, [x - 1.5, y - 1.6, 3], [x + 1.5, y + 1.6, 6], STEEL, { w: .8 }); }
  sbPart('endstops'); for (const y of [3, 37]) qBox(T_I, [5.4, y + 1.3, 8.6], [6.4, y + 2, 9.4], matOf('#e2483d'), { w: .5 });
  qBox(T_I, [X - .6, 37.6, 17], [X + .4, 38.2, 17.8], matOf('#e2483d'), { w: .4 });
  // ---- bed and paper, with the ink laid down so far
  sbPart('bed'); qBox(T_I, [7, 7, 3], [45, 33, 4], matOf('#c9a476'), { w: 1 });
  // the ink is drawn as part of the paper so it sorts with it
  const ink = st.ink.map(s => trailSVG(s.map(([x, y]) => [GT.X0 + x, GT.Y0 + y, 4.06]), '#3f5fb0', .8)).join('');
  sbPart('paper'); sbPut(qBoxNoPut(T_I, [GT.X0, GT.Y0, 4], [GT.X0 + GT.XW, GT.Y0 + GT.YW, 4.05], CERAMIC, toneI, .6) + ink, obbW([GT.X0, GT.Y0, 4], [GT.X0 + GT.XW, GT.Y0 + GT.YW, 4.05]));
  // ---- X motors + belts (one per rail)
  for (const y of [3, 37]){ sbPart('xmotors'); qBox(T_I, [.8, y - 2.1, 4.2], [5, y + 2.1, 8.4], MOTOR, { w: .9, deco: n => n[2] === 1 ? dotAt([2.9, y, 8.42], .5, STEEL.m, .3) : '' });
    qCyl(T_I, 'y', [2.9, 9.4], y - .6, y + .6, .65, BRASS, { w: .5 });
    sbPart('xbelts'); qBelt([2.9, y, 9.4], [48.4, y, 9.4], .65, .65, [0, 1, 0], '#2b2724', { w: 1.1 }); }
  // ---- gantry: carriages on both rails, uprights, beam
  sbPart('gantry'); for (const y of [3, 37]){ qBox(T_I, [X - 2.6, y - 1.8, 8.6], [X + 2.6, y + 1.8, 10.2], ALU, { w: .8 });
    qBox(T_I, [X - 1.4, y - 1.4, 10.2], [X + 1.4, y + 1.4, 14], ALU, { w: .8 }); }
  qBox(T_I, [X - 1.3, 2, 14], [X + 1.3, 38, 16.6], m, { w: 1.2, sh: tone, deco: n => n[0] === 1 ? txT(T_I, [X + 1.32, 20, 15.3], [0,1,0], [0,0,1], 'PLOT-36', 1.5) : '' });
  // ---- Y motor and belt along the beam top
  sbPart('ymotor'); qBox(T_I, [X - 2.1, 38, 12.4], [X + 2.1, 42, 16.6], MOTOR, { w: .9 });
  sbPart('ybelt'); qBelt([X, 39.6, 17.3], [X, 2.6, 17.3], .65, .65, [1, 0, 0], '#2b2724', { w: 1.1 });
  // ---- Y carriage, Z motor + lead screw, pen holder and pen
  sbPart('ycarr'); qBox(T_I, [X + 1.3, py - 3, 10.6], [X + 1.9, py + 3, 17.8], ALU, { w: .8 }); qBox(T_I, [X - 1.3, py - 2.6, 16.6], [X + 1.9, py + 2.6, 17.8], ALU, { w: .7 });
  sbPart('zmotor'); qBox(T_I, [px - 1.8, py - 1.8, 17.8], [px + 1.8, py + 1.8, 21], MOTOR, { w: .8 }); qBox(T_I, [X + 1.9, py - 1.8, 17.2], [px + 1.8, py + 1.8, 17.8], ALU, { w: .6 });
  const zh = zt + 5;
  sbPart('leadscrew'); qRod([px, py, 17.2], [px, py, 10.4], .3, STEEL, { w: .5 }); qHelix([px, py, 17], [px, py, 10.6], .34, 14, '#8f989b', { w: .45 });
  sbPart('pen'); qBox(T_I, [X + 1.9, py - 1.5, zh], [px + 1.6, py + 1.5, zh + 1.8], m, { w: .8, sh: tone });
  qRod([px, py, zh + 3], [px, py, zt + 1], .5, matOf('#3f7fc0'), { w: .8, sh: tone }); qRod([px, py, zt + 1], [px, py, zt], .5, DARK, { w: .7, r1: .12 });
  // ---- controller case: board, three drivers, power supply
  sbGroup('case', T_I, [8, 39, 3], [24, 45, 8]);
  sbPart('psu'); qBox(T_I, [8.6, 39.6, 3], [13.6, 44.4, 6.4], ALU, { w: .8 });
  sbPart('ctrl'); qBox(T_I, [14.4, 39.6, 3.4], [23.4, 44.4, 3.8], matOf('#2f6fb0'), { w: .7 }); qBox(T_I, [21.4, 41, 3.8], [23.4, 43, 4.6], STEEL, { w: .5 });
  sbPart('drivers'); for (let k = 0; k < 3; k++){ qBox(T_I, [15 + k*2.1, 40.2, 3.8], [16.6 + k*2.1, 42.4, 4.3], PCB, { w: .5 }); qBox(T_I, [15.3 + k*2.1, 40.5, 4.3], [16.3 + k*2.1, 42, 5.2], ALU, { w: .4 }); }
  sbPart('case'); qExt(T_I, rrUV(8, 39, 24, 45, 1), 3, 8, DARK, { ax: 'z', cover: true, w: 1.6, capDeco: b => b ? txT(T_I, [16, 42, 8.02], [1,0,0], [0,1,0], 'CNC-3', 1.6, '#f2efe7') + dotAt([22, 40.5, 8.02], .5, st.mode === 'op' ? '#43c25c' : '#8d877a', .3) : '' });
  sbFree(); sbPart('cables'); qWire([[20, 39, 7], [20, 36, X > 30 ? 22 : 20], [X, 38.5, 19], [X, 38.5, 16.6]], '#3a3633', .4, { seg: 5 });
  qWire([[17, 39, 7], [(17 + X)/2, 36.5, 9.5], [1.5, 37, 8.4]], '#3a3633', .3, { seg: 6 }); qWire([[16, 39, 6], [10, 22, 4.8], [2.9, 3, 4.2]], '#3a3633', .3, { seg: 6 });
  // ---- ink on the paper and a ghost of where the controller believes the pen is
  const floor = '';
  let over = '';
  if (st.mode === 'op' && (Math.abs(st.err[0]) > .05 || Math.abs(st.err[1]) > .05)){ const g = P(GT.X0 + st.p[0], GT.Y0 + st.p[1], 4.06), r = 1.2*INK_K*2;
    over += `<g pointer-events="none"><circle cx="${f(g[0])}" cy="${f(g[1])}" r="${f(r)}" fill="none" stroke="#e2483d" stroke-width=".9" stroke-dasharray="1.2 1"/><path d="M${f(g[0]-r*1.4)} ${f(g[1])}H${f(g[0]+r*1.4)}M${f(g[0])} ${f(g[1]-r*1.4)}V${f(g[1]+r*1.4)}" stroke="#e2483d" stroke-width=".6"/></g>`; }
  if (st.mode === 'op') over += tipHandle([px, py, zt], 4.05, false);
  return { under: board(0, 0, 52, 47, 3, 6), z0: 3, clip: boardTop(0, 0, 52, 47, 3, 6), floor, over };
}

// motion: straight lines with an acceleration ramp; a stepper asked to go faster than it can slips and loses steps
function gtTick(st, dt, ui, lab){
  let goal = st.jog, cur = st.prog && st.prog[st.pc];
  if (st.running && cur){ if (!st.segGoal){ const c = cur, g = st.p.slice();
      if (c.x != null) g[0] = c.x/10; if (c.y != null) g[1] = c.y/10; if (c.z != null) g[2] = c.z/10;
      st.segGoal = g; st.segV = c.code === 'G0' ? GT.RAPID : st.feed/10; }
    goal = st.segGoal; }
  if (!goal){ st.v = [0, 0, 0]; st.speed = 0; return false; }
  const d = sub(goal, st.p), L = vLen(d), vmax = st.running ? st.segV : GT.RAPID;
  if (L < 1e-4){ st.p = goal.slice(); st.speed = 0; st.v = [0, 0, 0];
    if (st.running){ st.segGoal = null; st.pc++; if (st.pc >= st.prog.length){ st.running = false; lab.flash('Program done'); } }
    else st.jog = null;
    return true; }
  const v = Math.min(vmax, st.speed + GT.ACC*dt, Math.sqrt(2*GT.ACC*L)), step = Math.min(L, v*dt), mv = vSc(d, step/L);
  st.speed = v; st.v = vSc(mv, 1/dt);
  // lost steps: above its pull-out speed a motor falls behind its pulses; the error never comes back
  const stall = [GT.STALLX, GT.STALLY];
  for (let k = 0; k < 2; k++){ const va = Math.abs(st.v[k]); if (va > stall[k]){ const e = Math.sign(st.v[k])*(va - stall[k])*dt*.35; st.err[k] += e; st.lost[k] += Math.abs(e)*10*gtSPmm(k, st.micro);
      if (!st.slipT) lab.flash(`${'XY'[k]} motor is skipping steps: too fast for its torque`); st.slipT = 1.5; } }
  if (st.slipT) st.slipT = Math.max(0, st.slipT - dt);
  const was = st.p[2] <= .02; st.p = vAdd(st.p, mv); const down = st.p[2] <= .02;
  const ap = [st.p[0] - st.err[0], st.p[1] - st.err[1]];
  if (down){ if (!was || !st.ink.length) st.ink.push([ap]); else { const s = st.ink[st.ink.length - 1], l = s[s.length - 1]; if (Math.hypot(ap[0] - l[0], ap[1] - l[1]) > .15) s.push(ap); }
    if (st.ink.length > 80) st.ink.shift(); }
  return true;
}
const gtClampP = p => [clamp(p[0], 0, GT.XW), clamp(p[1], 0, GT.YW), clamp(p[2], 0, 1.6)];

ROBOTS.gantry = {
  id: 'gantry', fig: '08', name: 'CNC gantry plotter', pal: 'cobalt', kicker: '3 axes · steppers · belts + lead screw · G-code',
  blurb: 'Linear rails, timing belts, a lead screw and three stepper motors: the Cartesian machine inside every 3D printer, laser cutter and CNC router. Run a G-code drawing and push the speed until the motors start skipping steps.',
  size: [52, 47, 24], center: [26, 23.5], bounds: [2, 50, 2, 45, 0, 24],
  init: () => ({ p: [0, 0, GT.PUP], err: [0, 0], lost: [0, 0], v: [0, 0, 0], speed: 0, jog: null, prog: gtProgram('square'), kind: 'square', pc: 0, running: false, segGoal: null, segV: 0,
    feed: 80*60/60, micro: 16, ink: [], slipT: 0 }),
  thumbState: { p: [18, 12, 0], ink: [[[12, 6], [24, 6], [24, 18], [12, 18], [12, 6]]] }, thumbYaw: 0,
  steps: [
    { title: 'Rails, feet & limit switches', parts: ['frame', 'endstops'], text: 'Two aluminium extrusion rails on steel feet. A red limit switch at the end of each axis tells the controller where zero is.' },
    { title: 'Bed & paper', parts: ['bed', 'paper'], text: 'A flat MDF bed with the paper taped down: 360 × 240 mm of drawing area.' },
    { title: 'X motors & belts', parts: ['xmotors', 'xbelts'], text: 'One stepper and one GT2 belt per rail. Both motors get the same step pulses, so both ends of the gantry move together and it cannot rack.' },
    { title: 'Gantry beam', parts: ['gantry'], text: 'The beam rides both rails on wheeled carriages clamped to the belts. It is the heaviest moving part, so X has the lowest top speed.' },
    { title: 'Y motor & belt', parts: ['ymotor', 'ybelt'], text: 'The Y stepper rides on the end of the beam; its belt runs along the beam’s top.' },
    { title: 'Y carriage', parts: ['ycarr'], text: 'Slides along the beam, carrying the Z axis.' },
    { title: 'Z motor & lead screw', parts: ['zmotor', 'leadscrew'], text: 'A small stepper turns an 8 mm-lead screw: slow, strong, and it does not back-drive, so the pen stays put with the power off.' },
    { title: 'Pen holder & pen', parts: ['pen'], text: 'The holder rides the lead screw nut. Pen down is Z = 0, the paper.' },
    { title: 'Controller, drivers & power', parts: ['ctrl', 'drivers', 'psu'], peek: true, text: 'The board turns G-code into step and direction pulses; each driver chops current through its motor’s two coils in sequence, with microstepping for smoothness.' },
    { title: 'Case', parts: ['case'], text: 'A vented cover over the electronics; the drivers need airflow.' },
    { title: 'Motor cables', parts: ['cables'], text: 'Four wires per stepper, one pair per coil. The Y and Z cables arc over to the moving gantry.' },
  ],
  parts: {
    frame: { name: 'Rails & feet', kind: 'structure', lbl: 'RAILS', key: true, short: '20 × 40 extrusion', info: 'The rails define straightness: any bow in them shows up in every line drawn.', held: 'Steel feet on the bench' },
    endstops: { name: 'Limit switches', kind: 'sensor', lbl: 'ENDSTOPS', short: 'home sensors', info: 'Steppers do not know where they are at power-up. Homing drives each axis slowly into its switch and calls that zero.', held: 'Rail ends and the beam end' },
    bed: { name: 'Bed', kind: 'structure', short: 'MDF board', info: 'Flat and stiff; the pen needs a constant height.', held: 'The rails' },
    paper: { name: 'Paper', kind: 'tool', lbl: 'PAPER', short: '360 × 240 mm', info: 'Origin (X0 Y0) is the front-left corner.', held: 'Tape' },
    xmotors: { name: 'X steppers (×2)', kind: 'motor', lbl: 'X STEPPERS', key: true, short: 'NEMA 17, 200 steps/rev', info: 'A stepper has a toothed rotor and two coils; energising them in sequence pulls the rotor one 1.8 degree step at a time. Torque falls as speed rises.', held: 'Brackets at the rail ends', spec: '0.45 N m holding' },
    xbelts: { name: 'X belts', kind: 'gear', lbl: 'X BELTS', short: 'GT2, 20-tooth pulleys', info: '20 teeth × 2 mm pitch = 40 mm of travel per motor turn: 200 full steps → 5 steps per mm.', held: 'Pulleys and the carriage clamps', spec: '40 mm/rev' },
    gantry: { name: 'Gantry beam', kind: 'structure', lbl: 'GANTRY', key: true, short: 'moves in X', info: 'Moving mass about 1.5 kg. At 1.5 m/s² the X motors already pull hard.', held: 'Carriages on both rails' },
    ymotor: { name: 'Y stepper', kind: 'motor', lbl: 'Y STEPPER', short: 'drives the carriage', info: 'Lighter load than X, so Y can go faster before skipping.', held: 'The beam end' },
    ybelt: { name: 'Y belt', kind: 'gear', short: 'GT2 along the beam', info: 'Same 40 mm per turn as X.', held: 'Pulleys on the beam' },
    ycarr: { name: 'Y carriage', kind: 'structure', lbl: 'Y CARRIAGE', short: 'moves in Y', info: 'Wheels either side of the beam hold it square.', held: 'The beam' },
    zmotor: { name: 'Z stepper', kind: 'motor', lbl: 'Z STEPPER', short: 'small NEMA 14', info: 'Only lifts the pen holder.', held: 'The Y carriage' },
    leadscrew: { name: 'Lead screw', kind: 'gear', lbl: 'LEAD SCREW', short: '8 mm per turn', info: '200 steps / 8 mm = 25 steps per mm: five times finer than the belts, but slower.', held: 'Coupler on the Z motor', spec: 'T8, 8 mm lead' },
    pen: { name: 'Pen holder & pen', kind: 'tool', lbl: 'PEN', key: true, short: 'the tool', info: 'Swap it for a laser, a drag knife or a spindle and this is a laser cutter, a vinyl cutter or a CNC router.', held: 'The lead screw nut' },
    ctrl: { name: 'Controller board', kind: 'electronics', lbl: 'CONTROLLER', inside: true, short: 'G-code interpreter', info: 'Plans each line with acceleration ramps, then emits step pulses at up to tens of thousands per second.', held: 'Standoffs in the case' },
    drivers: { name: 'Stepper drivers (×3)', kind: 'electronics', lbl: 'DRIVERS', inside: true, short: 'current chopping, microstepping', info: 'Microstepping splits each step by setting the two coil currents to sine and cosine fractions. 16 microsteps: 80 steps per mm on the belts.', held: 'Sockets on the board' },
    psu: { name: 'Power supply', kind: 'power', lbl: 'PSU', inside: true, short: '24 V, 5 A', info: 'Higher voltage lets current rise faster in the coils, so torque holds up better at speed.', held: 'The case floor' },
    case: { name: 'Electronics case', kind: 'cover', lbl: 'CASE', key: true, short: 'vented cover', info: 'Status LED is green while it runs.', held: 'The bench' },
    cables: { name: 'Motor cables', kind: 'wiring', short: '4-wire stepper cables', info: 'Two coils, two wires each.', held: 'Clips along the frame' },
  },
  concepts: [
    ['Cartesian motion', 'One motor per axis, axes at right angles: X, Y and Z are simply motor positions times a constant. No inverse kinematics at all, which is why CNC machines and printers look like this.'],
    ['G-code', 'G0 = move fast (rapid), G1 = cut or draw at the feed rate F, Z for pen height. The controller reads a line, plans a straight move with ramps, and moves on. The program list shows the line running now.'],
    ['Steps per mm', 'Belts: 200 steps × microsteps / 40 mm. Lead screw: 200 × microsteps / 8 mm. More microsteps are smoother and finer, but not stronger.'],
    ['Open loop and lost steps', 'The controller never measures position: it only counts pulses. Above a motor’s pull-out speed it slips, and every line after is offset (the red cross is where it thinks the pen is). Drop the feed, then press Home to re-zero on the switches.'],
    ['Acceleration ramps', 'A stepper cannot jump to full speed. Each move ramps up and down at 1.5 m/s²; on short segments (the circle) it never reaches the feed rate.'],
  ],
  draw: gtDraw,
  active: st => { const a = {}; if (Math.abs(st.v[0]) > .1) a.xmotors = a.xbelts = true; if (Math.abs(st.v[1]) > .1) a.ymotor = a.ybelt = true; if (Math.abs(st.v[2]) > .05) a.zmotor = a.leadscrew = true;
    if (st.running) a.ctrl = a.drivers = true; return a; },
  opHTML: () => `<div class="seg" id="gtprog">${['square', 'circle', 'star', 'spiral', 'grid'].map(k => `<button data-k="${k}">${k[0].toUpperCase() + k.slice(1)}</button>`).join('')}</div>
    <div class="row2"><button id="gtrun">Run program</button><button id="gtpen">Pen down</button><button id="gthome">Home</button><button id="gtclr">New paper</button></div>
    ${slHTML('gtfeed', 'Feed rate F', 10, 300, 5, ' mm/s')}
    <div class="seg" id="gtmu">${GT_MICRO.map(u => `<button data-u="${u}">1/${u} step</button>`).join('')}</div>
    <p class="note">Drag the yellow ring (or use the sliders) to jog; with the pen down it draws.</p>
    ${slHTML('gtx', 'Jog X', 0, 360, 1, ' mm')}${slHTML('gty', 'Jog Y', 0, 240, 1, ' mm')}
    <h3 style="margin-top:8px">Program</h3><pre id="gtcode" style="font-size:10.5px;line-height:1.45;margin:0;max-height:118px;overflow:hidden;background:#fff;border:1.2px solid var(--ink);border-radius:8px;padding:5px 8px"></pre>`,
  opBind(el, st, ui, lab){ const $ = id => document.getElementById(id);
    const jog = p => { if (st.running) return lab.flash('Program running — press Stop to jog'); st.jog = gtClampP(p); lab.kick(); };
    el.querySelectorAll('#gtprog button').forEach(b => b.onclick = () => { if (st.running) return lab.flash('Stop the program first'); st.kind = b.dataset.k; st.prog = gtProgram(st.kind); st.pc = 0; ui.render(); });
    el.querySelectorAll('#gtmu button').forEach(b => b.onclick = () => { st.micro = +b.dataset.u; ui.render(); });
    $('gtrun').onclick = () => { if (st.running){ st.running = false; st.segGoal = null; ui.render(); return; } if (st.pc >= st.prog.length) st.pc = 0; st.jog = null; st.running = true; lab.kick(); ui.render(); };
    $('gtpen').onclick = () => jog([st.p[0], st.p[1], st.p[2] > .02 ? 0 : GT.PUP]);
    $('gthome').onclick = () => { if (st.running) return lab.flash('Stop the program first'); st.p = [st.p[0] - st.err[0], st.p[1] - st.err[1], st.p[2]]; st.err = [0, 0]; st.lost = [0, 0]; st.jog = [0, 0, GT.PUP]; lab.flash('Homing: each axis runs into its switch and resets to zero'); lab.kick(); };
    $('gtclr').onclick = () => { st.ink = []; ui.render(); };
    $('s_gtfeed').oninput = e => { st.feed = +e.target.value; ui.render(); };
    $('s_gtx').oninput = e => jog([+e.target.value/10, (st.jog || st.p)[1], (st.jog || st.p)[2]]);
    $('s_gty').oninput = e => jog([(st.jog || st.p)[0], +e.target.value/10, (st.jog || st.p)[2]]); },
  opSync(el, st){ const $ = id => document.getElementById(id);
    el.querySelectorAll('#gtprog button').forEach(b => b.setAttribute('aria-pressed', b.dataset.k === st.kind));
    el.querySelectorAll('#gtmu button').forEach(b => b.setAttribute('aria-pressed', +b.dataset.u === st.micro));
    $('gtrun').textContent = st.running ? 'Stop' : st.pc > 0 && st.pc < st.prog.length ? 'Resume' : 'Run program'; $('gtrun').setAttribute('aria-pressed', st.running);
    $('gtpen').textContent = st.p[2] > .02 ? 'Pen down' : 'Pen up';
    slSync('gtfeed', st.feed, `${st.feed} mm/s`, st.feed > GT.STALLX*10); slSync('gtx', (st.jog || st.p)[0]*10, `${(st.p[0]*10).toFixed(1)} mm`); slSync('gty', (st.jog || st.p)[1]*10, `${(st.p[1]*10).toFixed(1)} mm`);
    const i0 = Math.max(0, Math.min(st.pc - 2, st.prog.length - 7));
    $('gtcode').innerHTML = st.prog.slice(i0, i0 + 7).map((c, k) => { const i = i0 + k, cur = i === st.pc && st.pc < st.prog.length;
      return `<div style="${cur ? 'background:var(--accent,#f4b829);font-weight:800;' : i < st.pc ? 'opacity:.45;' : ''}">${String(i + 1).padStart(3, ' ')}  ${c.txt}</div>`; }).join('') + `<div class="note">${st.prog.length} lines</div>`; },
  readout(st){ const rows = [], spm = k => gtSPmm(k, st.micro);
    ['X', 'Y', 'Z'].forEach((a, k) => rows.push({ k: `${a} ${(st.p[k]*10).toFixed(1)} mm`, bar: Math.abs(st.v[k])/GT.RAPID, v: `${Math.round(st.p[k]*10*spm(k))} steps`, tip: `${spm(k)} steps/mm` }));
    rows.push({ k: 'Speed', bar: st.speed/30, v: `${(st.speed*10).toFixed(0)} mm/s` });
    rows.push({ k: 'Step rate X', bar: Math.abs(st.v[0])*10*spm(0)/40000, v: `${(Math.abs(st.v[0])*10*spm(0)/1000).toFixed(1)} kHz` });
    rows.push({ k: 'Resolution', v: `${(1/spm(0)*1000).toFixed(1)} µm (belts)` });
    rows.push({ k: 'Lost steps X / Y', v: `${Math.round(st.lost[0])} / ${Math.round(st.lost[1])}`, warn: st.lost[0] + st.lost[1] > 0, tip: 'Steps sent that the motor never made' });
    rows.push({ k: 'Position error', bar: Math.hypot(...st.err)/1, v: `${(Math.hypot(...st.err)*10).toFixed(1)} mm`, warn: Math.hypot(...st.err) > .05 });
    rows.push({ k: 'Line', v: st.pc < st.prog.length ? `${st.pc + 1} / ${st.prog.length}` : 'done' });
    return rows; },
  stats: st => [{ label: 'Pen X Y mm', value: `${(st.p[0]*10).toFixed(0)} ${(st.p[1]*10).toFixed(0)}`, small: true }],
  tick: gtTick,
  enterOp: st => { st.jog = null; },
  onDrag(st, dx, dy, phase, lab){ if (phase === 'start'){ if (st.running){ st.running = false; st.segGoal = null; lab.flash('Program paused: jogging by hand'); } st.dragP = st.p.slice(); return; } if (phase !== 'move') return;
    const d = dragPlane(dx, dy); st.dragP = gtClampP([st.dragP[0] + d[0], st.dragP[1] + d[1], st.dragP[2]]); st.jog = st.dragP.slice(); },
  params(st, q){ if (q.has('prog')){ st.kind = q.get('prog'); st.prog = gtProgram(st.kind); }
    if (q.has('feed')) st.feed = +q.get('feed');
    if (q.get('demo') === 'run'){ st.running = true; for (let k = 0; k < 6000 && st.running; k++) gtTick(st, 1/60, null, { flash(){} }); } },
};
ROBOT_ORDER.push({ id: 'gantry', name: 'CNC gantry plotter', blurb: 'Steppers, lead screw and timing belts on linear rails: run G-code drawings, change microstepping, and over-drive the motors until they lose steps.', chips: ['3 axes', 'Steppers', 'Lead screw + belts', 'G-code'] });
