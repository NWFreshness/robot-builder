/* ============================================================
   Robot 03 — differential-drive rover (a 28 cm research rover). 1 unit = 1 cm.
   Truth: an aluminium chassis plate carries two DC gear motors hung below it on brackets; each drives one wheel directly,
   with a magnetic quadrature encoder on the back of the motor. A ball caster at the rear makes three points of contact.
   On the plate: a battery pack, a dual H-bridge motor driver with heat sink, a microcontroller board and an IMU; a shell
   covers them and a 2D lidar spins on top. A sprung bumper bar presses a microswitch on contact.
   Steering is done only by wheel speed difference: v = (vR + vL)/2, omega = (vR - vL)/track.
   Parts: plate | motors, encoders | wheels | caster | battery | driver | mcu, imu | wires | bumper | shell | lidar
   ============================================================ */
const RV = { C: 35, R0: 33, TRACK: 24.6, RW: 4.5, RW_R: 4.43, VMAX: 70, WMAX: 2.4, TPR: 1632 };
const RV_OBST = [
  { x: 52, y: -4, w: 10, d: 10, h: 20, c: '#d49c1f' }, { x: -30, y: 34, w: 14, d: 9, h: 22, c: '#cf5038' },
  { x: 24, y: 52, w: 9, d: 22, h: 20, c: '#71b693' }, { x: -38, y: -32, w: 12, d: 12, h: 24, c: '#5a82c6' },
  { x: 80, y: 40, w: 10, d: 30, h: 20, c: '#d8869c' }, { x: 10, y: -55, w: 32, d: 7, h: 20, c: '#d49c1f' },
  { x: -70, y: 0, w: 8, d: 40, h: 22, c: '#71b693' }, { x: 40, y: -70, w: 14, d: 14, h: 20, c: '#cf5038' } ];

const rvScene = (st, w) => [RV.C + w[0] - st.pos[0], RV.C + w[1] - st.pos[1]];
function rvHit(px, py, r){ for (const o of RV_OBST){ const cx = clamp(px, o.x - o.w/2, o.x + o.w/2), cy = clamp(py, o.y - o.d/2, o.y + o.d/2); if (Math.hypot(px - cx, py - cy) < r) return o; } return null; }
function rvRay(px, py, a, max = 45){ const dx = Math.cos(a), dy = Math.sin(a); let best = max;
  for (const o of RV_OBST){ let t0 = 0, t1 = max;
    for (const [p, d, lo, hi] of [[px, dx, o.x - o.w/2, o.x + o.w/2], [py, dy, o.y - o.d/2, o.y + o.d/2]]){
      if (Math.abs(d) < 1e-9){ if (p < lo || p > hi){ t0 = 1e9; } continue; }
      let a1 = (lo - p)/d, a2 = (hi - p)/d; if (a1 > a2) [a1, a2] = [a2, a1]; t0 = Math.max(t0, a1); t1 = Math.min(t1, a2); }
    if (t0 <= t1 && t0 < best) best = t0; }
  return best; }

function rvDraw(st, m){
  const TR = tMul(tTr(RV.C, RV.C, 0), tRz(st.th)), wheelsAt = ROBOTS.rover._step.wheels;
  // ---- chassis plate (and the build stand until the wheels go on)
  if (st.mode === 'build' && st.built <= wheelsAt){ sbFree(); sbPut(qBoxNoPut(TR, [-6, -6, 0], [6, 6, 6.6], matOf('#c9bfa9')), obbT(TR, [-6, -6, 0], [6, 6, 6.6]), { p: null }); }
  sbFree(); sbPart('plate'); qBox(TR, [-13, -10.5, 6.6], [13, 10.5, 7.2], ALU, { w: 1.1 });
  // ---- drive motors: can, gearbox, shaft, bracket; encoders on the inner ends
  for (const s of [1, -1]){
    const y = v => s*v, lo = (a, b) => Math.min(y(a), y(b)), hi = (a, b) => Math.max(y(a), y(b));
    sbPart('motors'); qBox(TR, [.6, lo(4.2, 5), 2.6], [3.4, hi(4.2, 5), 6.6], ALU, { w: .7 });
    qCyl(TR, 'y', [2, 4.5], y(.6), y(4.2), 1.7, MOTOR, { w: .9 }); qCyl(TR, 'y', [2, 4.5], y(5), y(7.6), 1.9, matOf('#e8c45a'), { w: .9 });
    qCyl(TR, 'y', [2, 4.5], y(7.6), y(11), .4, STEEL, { w: .5 });
    sbPart('encoders'); qCyl(TR, 'y', [2, 4.5], y(.15), y(.6), 1.9, matOf('#3c6fb4'), { w: .6 });
    qBox(TR, [3.6, lo(.1, .7), 3.4], [4.4, hi(.1, .7), 5.6], PCB, { w: .5 });
    // wheel: tyre, hub, three spokes turning with the wheel angle
    sbPart('wheels'); const wa = st.wa[s > 0 ? 0 : 1];
    qCyl(TR, 'y', [2, 4.5], y(11), y(13.6), RV.RW, TYRE, { w: 1.4, n: 28 });
    const out = tVec(TR, [0, s, 0]), face = vis(out) ? y(13.65) : y(10.95);
    qCyl(TR, 'y', [2, 4.5], y(13.6), y(13.95), 2, ALU, { w: .8, deco: () => vis(out) ? [0, 1, 2].map(k => { const a = wa + k*2*Math.PI/3;
      return line([tAp(TR, [2, y(13.97), 4.5]), tAp(TR, [2 + 1.7*Math.cos(a), y(13.97), 4.5 + 1.7*Math.sin(a)])], .7); }).join('') : '' });
    if (!vis(out)) sbPut([0, 1, 2, 3, 4, 5].map(k => { const a = wa + k*Math.PI/3; return line([tAp(TR, [2 + 2.6*Math.cos(a), face, 4.5 + 2.6*Math.sin(a)]), tAp(TR, [2 + 4.1*Math.cos(a), face, 4.5 + 4.1*Math.sin(a)])], .5); }).join(''),
      obbT(TR, [-2.5, lo(10.9, 11), 0], [6.5, hi(10.9, 11), 9]), { p: 'wheels' });
  }
  // ---- rear ball caster
  sbPart('caster'); qCyl(TR, 'z', [-10, 0], 3.2, 6.6, 1.6, ALU, { w: .8 }); qBall(tAp(TR, [-10, 0, 1.5]), 1.5, STEEL);
  // ---- inside the shell: battery, motor driver, MCU + IMU, wiring
  sbGroup('shell', TR, [-13.6, -10, 7.2], [13.6, 10, 14.4]);
  sbPart('battery'); qBox(TR, [-9, -4.5, 7.2], [1, 4.5, 10.2], BATT, { w: .9, deco: n => n[2] === 1 ? txT(TR, [-4, 0, 10.22], [1,0,0], [0,1,0], '3S LiPo', 1.5, '#fff') : '' });
  sbPart('driver'); qBox(TR, [3, -7, 7.2], [10, -1, 7.7], PCB, { w: .7 }); qBox(TR, [4, -6, 7.7], [9, -2, 9.6], ALU, { w: .7,
    deco: n => n[2] === 1 ? [4.6, 5.6, 6.6, 7.6, 8.4].map(x => line([tAp(TR, [x, -5.8, 9.62]), tAp(TR, [x, -2.2, 9.62])], .5)).join('') : '' });
  qBox(TR, [8.6, -6.9, 7.7], [9.9, -5.4, 8.6], matOf('#3f7fc0'), { w: .5 }); qBox(TR, [8.6, -2.6, 7.7], [9.9, -1.1, 8.6], matOf('#3f7fc0'), { w: .5 });
  sbPart('mcu'); qBox(TR, [3, 1.2, 7.2], [10, 7.4, 7.6], matOf('#2f6fb0'), { w: .7 }); qBox(TR, [5, 3, 7.6], [7.4, 5.4, 8.1], DARK, { w: .5 }); qBox(TR, [8.6, 2, 7.6], [9.8, 3.2, 8.4], STEEL, { w: .4 });
  sbPart('imu'); qBox(TR, [-3, 6, 7.2], [-.6, 8.4, 7.5], PCB, { w: .5 }); qBox(TR, [-2.2, 6.8, 7.5], [-1.4, 7.6, 7.8], DARK, { w: .4 });
  sbPart('wires'); qWire([[1, -2, 9], [3.5, -3.5, 8.4], [8.8, -6.2, 8.8]].map(p => tAp(TR, p)), CABLE.pwr, .28); qWire([[1, -1, 8.6], [3.2, -2.4, 8.2], [9.2, -2, 8.8]].map(p => tAp(TR, p)), CABLE.gnd, .28);
  qWire([[6, 1.4, 7.8], [6, -.8, 7.9], [5, -1.4, 7.8]].map(p => tAp(TR, p)), CABLE.sig, .22);
  qWire([[-1.5, 6.4, 7.6], [1, 5, 7.8], [3.2, 4.6, 7.8]].map(p => tAp(TR, p)), CABLE.enc, .2);
  qWire([[7, 6, 7.8], [4, 2, 11], [1, 0, 14.2]].map(p => tAp(TR, p)), CABLE.sig, .25);
  sbPart('shell'); qExt(TR, rrUV(-13.6, -10, 13.6, 10, 4), 7.2, 14.4, m, { ax: 'z', cover: true, w: 2,
    capDeco: b => b ? txT(TR, [8, 0, 14.42], [0,-1,0], [1,0,0], 'R-1', 2.6) : '' });
  sbFree(); sbPart('shell'); qBox(TR, [-13.9, -2, 10], [-13.6, 2, 12.4], DARK, { w: .6, deco: n => n[0] === -1 ? dotAt(tAp(TR, [-13.95, 1.2, 11.2]), .4, st.mode === 'op' ? '#43c25c' : '#8d877a', .3) : '' });
  // ---- bumper bar on two sprung brackets, microswitch behind it
  sbPart('bumper'); const bx = st.bumpT > 0 ? -.5 : 0;
  qBox(TR, [14 + bx, -8.5, 3.4], [15.2 + bx, 8.5, 6], matOf('#ef6a4c'), { w: 1, sh: tone });
  for (const s of [1, -1]) qBox(TR, [12.4, s > 0 ? 5 : -6, 5], [14 + bx, s > 0 ? 6 : -5, 6.6], ALU, { w: .6 });
  qBox(TR, [12.6, -1, 5.2], [13.6, 1, 6.6], st.bumpT > 0 ? matOf('#e2483d') : DARK, { w: .5 });
  // ---- lidar: base, spinning head with a window band
  sbPart('lidar'); qRod(tAp(TR, [0, 0, 14.4]), tAp(TR, [0, 0, 16.2]), 3.4, DARK, { w: 1.1, sh: tone });
  const la = st.lidarA, LT = tMul(TR, tRz(la));
  qRod(tAp(TR, [0, 0, 16.2]), tAp(TR, [0, 0, 19]), 3, matOf('#3a4652'), { w: 1.1, sh: tone,
    deco: () => { const pts = []; for (let i = 0; i <= 12; i++){ const a = -.6 + 1.2*i/12; const p = tAp(LT, [3.02*Math.cos(a), 3.02*Math.sin(a), 17.6]); pts.push(p); }
      return vis(tVec(LT, [1, 0, 0])) ? `<path d="${pd(pts, false)}" fill="none" stroke="#e2483d" stroke-width="1.6" stroke-linecap="round"/>` : ''; } });
  // ---- world: obstacles in reach of the window
  for (const o of RV_OBST){ const [sx, sy] = rvScene(st, [o.x, o.y]), far = Math.max(...[[-1,-1],[1,-1],[1,1],[-1,1]].map(([a, b]) => Math.hypot(sx + a*o.w/2 - RV.C, sy + b*o.d/2 - RV.C)));
    if (far > RV.R0 + 1) continue; const op = 1;
    sbFree(); const s = qBoxNoPut(T_I, [sx - o.w/2, sy - o.d/2, 0], [sx + o.w/2, sy + o.d/2, o.h], matOf(o.c), tone, 1.4);
    sbPut(`<g opacity="${f(op)}">${s}</g>`, obbW([sx - o.w/2, sy - o.d/2, 0], [sx + o.w/2, sy + o.d/2, o.h]), { p: null }); }
  // ---- floor: trails, waypoints
  const rim = on(PZ(0), ellUV(RV.C, RV.C, RV.R0, RV.R0, 64)); let floor = '';
  if (st.mode === 'op'){
    const toS = arr => arr.map(w => [...rvScene(st, w), .05]);
    floor += clipTo(rim, trailSVG(toS(st.trail), '#e2483d', 1.1) + trailSVG(toS(st.odoTrail), '#3d5878', .9, '2 1.5'));
    if (st.wps.length){ const pts = [[RV.C, RV.C, .1], ...st.wps.map(w => [...rvScene(st, w), .1])]; floor += clipTo(rim, trailSVG(pts, '#231e16', .7, '1 1.2'));
      st.wps.forEach((w, i) => { const [x, y] = rvScene(st, w); if (Math.hypot(x - RV.C, y - RV.C) > RV.R0 - 1) return;
        sbFree(); qRod([x, y, 0], [x, y, 7], .25, STEEL, { w: .6, p: null });
        sbPut(shape([[x, y, 7], [x, y, 4.6], [x + 0.01, y - 3.4, 5.8]], i ? '#f4b829' : '#e2483d', .7) + txT(T_I, [x, y + .1, 7.8], [0,-1,0], [0,0,1], String(i + 1), 1.8), obbW([x - .2, y - 3.5, 4], [x + .2, y + .2, 8.5]), { p: null }); }); } }
  let over = '';
  if (st.mode === 'op' && st.scan){ const L0 = tAp(TR, [0, 0, 17.6]); let d = '', dots = '';
    for (const h of st.hits){ const [x, y] = rvScene(st, [h[0], h[1]]); if (Math.hypot(x - RV.C, y - RV.C) > RV.R0) continue; const q = P(x, y, 17.6), q0 = P(...L0);
      d += `M${f(q0[0])} ${f(q0[1])}L${f(q[0])} ${f(q[1])}`; if (h[2]) dots += `<circle cx="${f(q[0])}" cy="${f(q[1])}" r="${f(.9*INK_K*1.6)}" fill="#e2483d"/>`; }
    over += `<g pointer-events="none"><path d="${d}" stroke="#e2483d" stroke-width=".4" opacity=".35" fill="none"/>${dots}</g>`; }
  if (st.crash) over += crashBurst(st.crash.at, st.crash.t);
  return { under: floorDisc(RV.C, RV.C, RV.R0, st.pos[0], st.pos[1]), z0: 0, clip: rim, floor, over };
}
// box drawn without registering (for wrapped parts)
function qBoxNoPut(T, lo, hi, M, sh = toneI, w = .9){ const saved = SB.items.length; const s = qBox(T, lo, hi, M, { sh, w }); SB.items.length = saved; return s; }

function rvTick(st, dt, ui, lab){
  const j = st.joy || [0, 0], manual = Math.hypot(j[0], j[1]) > .05;
  let vC = 0, wC = 0;
  if (manual){ vC = j[1]*RV.VMAX*st.vscale; wC = -j[0]*RV.WMAX; st.wps = st.wps.length && manual ? st.wps : st.wps; }
  else if (st.wps.length){ const g = st.wps[0], dx = g[0] - st.pos[0], dy = g[1] - st.pos[1], dist = Math.hypot(dx, dy), e = wrap180((Math.atan2(dy, dx) - st.th)/DEG)*DEG;
    if (dist < 2.5){ st.wps.shift(); if (!st.wps.length) lab.flash('Arrived at the last waypoint'); }
    else { wC = clamp(3*e, -RV.WMAX, RV.WMAX); vC = RV.VMAX*.6*st.vscale*Math.max(0, Math.cos(e))*Math.min(1, dist/12); } }
  // acceleration limits (the motors cannot change speed instantly)
  const dv = clamp(vC - st.v, -90*dt, 90*dt), dw = clamp(wC - st.w, -6*dt, 6*dt); st.acc = dt > 1e-4 ? dv/dt : 0; st.v += dv; st.w += dw;
  // wheel speeds; a wheel past its top speed saturates and the turn rate drops
  let vL = st.v - st.w*RV.TRACK/2, vR = st.v + st.w*RV.TRACK/2; const k = Math.max(1, Math.abs(vL)/RV.VMAX, Math.abs(vR)/RV.VMAX); vL /= k; vR /= k; st.sat = k > 1.001;
  st.vL = vL; st.vR = vR;
  const wl = vL/RV.RW, wr = vR/RV.RW;                                       // commanded wheel angular speeds
  st.wa[0] -= wl*dt; st.wa[1] -= wr*dt;                                       // [0] = the +y wheel, the robot's left
  // true motion uses the real (slightly worn) right tyre; odometry assumes both are nominal
  const tvL = wl*RV.RW, tvR = wr*RV.RW_R, tv = (tvL + tvR)/2, tw = (tvR - tvL)/RV.TRACK;
  const ov = (vL + vR)/2, ow = (vR - vL)/RV.TRACK;
  const np = [st.pos[0] + tv*Math.cos(st.th)*dt, st.pos[1] + tv*Math.sin(st.th)*dt], nth = st.th + tw*dt;
  const front = [np[0] + 9*Math.cos(nth), np[1] + 9*Math.sin(nth)];
  if (rvHit(np[0], np[1], 13) || rvHit(front[0], front[1], 6.5)){ st.v = st.w = 0; st.bumpT = .6; st.crashes++;
    st.crash = { at: tAp(tMul(tTr(RV.C, RV.C, 0), tRz(st.th)), [15, 0, 5]), t: 1 }; lab.flash('Bump! The bumper switch fired and the motors stopped'); }
  else { st.pos = np; st.th = nth;
    st.odo[0] += ov*Math.cos(st.odo[2])*dt; st.odo[1] += ov*Math.sin(st.odo[2])*dt; st.odo[2] += ow*dt;
    st.ticks[0] += vL/RV.RW*dt/(2*Math.PI)*RV.TPR; st.ticks[1] += vR/RV.RW*dt/(2*Math.PI)*RV.TPR; st.dist += Math.abs(tv*dt);
    const l = st.trail[st.trail.length - 1]; if (!l || Math.hypot(l[0] - st.pos[0], l[1] - st.pos[1]) > 1){ st.trail.push(st.pos.slice()); st.odoTrail.push(st.odo.slice(0, 2)); if (st.trail.length > 500){ st.trail.shift(); st.odoTrail.shift(); } } }
  // lidar: 48 beams, one sweep per frame for the drawing
  st.lidarA += dt*8; const lp = st.pos; st.hits = [];
  for (let i = 0; i < 48; i++){ const a = st.th + i/48*2*Math.PI, r = rvRay(lp[0], lp[1], a); st.hits.push([lp[0] + r*Math.cos(a), lp[1] + r*Math.sin(a), r < 44.9, r, a - st.th]); }
  st.near = Math.min(...st.hits.map(h => h[3]));
  if (st.bumpT > 0) st.bumpT -= dt; if (st.crash){ st.crash.t -= dt/1.2; if (st.crash.t <= 0) st.crash = null; }
  st.batt = Math.max(10.5, 12.6 - st.dist*.0006);
  return manual || st.wps.length > 0 || Math.abs(st.v) > .05 || Math.abs(st.w) > .005 || st.bumpT > 0 || !!st.crash;
}

ROBOTS.rover = {
  id: 'rover', fig: '03', name: 'Differential-drive rover', pal: 'mint', kicker: '2 driven wheels · encoders · lidar · odometry',
  blurb: 'The classic mobile robot: two motor-driven wheels and a caster. It steers only by spinning its wheels at different speeds. Drive it with the joystick or click the floor to set waypoints, and watch the lidar scan and the odometry drift.',
  size: [70, 70, 24], center: [35, 35], bounds: [20, 50, 20, 50, 0, 20],
  init: () => ({ pos: [0, 0], th: 0, v: 0, w: 0, vL: 0, vR: 0, acc: 0, wa: [0, 0], joy: [0, 0], vscale: 1, wps: [], trail: [], odoTrail: [], odo: [0, 0, 0], ticks: [0, 0], dist: 0,
    lidarA: 0, hits: [], near: 45, scan: true, bumpT: 0, crash: null, crashes: 0, sat: false, batt: 12.6 }),
  thumbYaw: 0,
  steps: [
    { title: 'Chassis plate', parts: ['plate'], text: 'A 3 mm aluminium plate: everything hangs from or stands on it. It rests on an assembly stand until the wheels are on.' },
    { title: 'Drive motors & encoders', parts: ['motors', 'encoders'], text: 'Two brushed DC gear motors hang below the plate on brackets, one per side. On the back of each motor a magnet disc spins past two Hall sensors: a quadrature encoder.' },
    { title: 'Wheels', parts: ['wheels'], text: 'Each wheel sits straight on its gearbox shaft. The distance between the wheels (the track, 246 mm) sets how fast the robot turns for a given speed difference.' },
    { title: 'Rear ball caster', parts: ['caster'], text: 'A steel ball in a socket: the third point of contact. It rolls any direction, so it never fights the steering.' },
    { title: 'Battery pack', parts: ['battery'], text: 'A 3-cell lithium polymer pack (11.1 V nominal) sits low and central to keep the centre of mass between the wheels.' },
    { title: 'Motor driver', parts: ['driver'], text: 'A dual H-bridge: four transistors per motor switch the battery across the motor either way round, and PWM sets the average voltage. The heat sink carries away the switching losses.' },
    { title: 'Microcontroller & IMU', parts: ['mcu', 'imu'], text: 'The microcontroller reads both encoders, runs a speed loop for each wheel and integrates odometry. The IMU (gyro + accelerometer) measures turning rate independently.' },
    { title: 'Wiring', parts: ['wires'], peek: true, text: 'Battery to driver (red/black), driver to motors, encoder lines and PWM signals back to the microcontroller. Thick wires for amps, thin for signals.' },
    { title: 'Bumper & microswitch', parts: ['bumper'], text: 'A sprung bar: on contact it slides back and presses a microswitch that stops the motors in hardware, whatever the software is doing.' },
    { title: 'Shell', parts: ['shell'], text: 'A moulded cover protects the electronics; the power switch and status LED are on the back.' },
    { title: 'Lidar', parts: ['lidar'], text: 'A spinning laser rangefinder on top: it measures the distance to the nearest surface in 360 directions per turn. The red dots in Operate mode are its hits.' },
  ],
  parts: {
    plate: { name: 'Chassis plate', kind: 'structure', lbl: 'CHASSIS', key: true, short: '3 mm aluminium', info: 'The backbone: motor brackets bolt underneath, electronics on top.', held: 'The two wheels and the caster' },
    motors: { name: 'DC gear motors', kind: 'motor', lbl: 'GEAR MOTORS', key: true, short: '12 V, 34:1 gearbox', info: 'A brushed DC motor spins fast with little torque; the 34:1 spur gearbox trades speed for torque. Speed is roughly proportional to voltage, torque to current.', held: 'Aluminium brackets under the plate', spec: '12 V, 150 rpm out, 0.5 N m' },
    encoders: { name: 'Quadrature encoders', kind: 'sensor', lbl: 'ENCODERS', inside: true, short: '1632 ticks per wheel turn', info: 'A 12-pole magnet on the motor shaft, two Hall sensors 90 degrees apart. 12 x 4 edges x 34:1 = 1632 ticks per wheel revolution. The order of the two signals tells direction.', held: 'The back of each motor' },
    wheels: { name: 'Wheels', kind: 'structure', lbl: 'WHEELS', key: true, short: '90 mm rubber tyres', info: 'Wheel radius turns rotation into distance: one turn = 2π × 45 mm = 283 mm. The right tyre here is slightly worn (44.3 mm), which is exactly the kind of error that makes odometry drift.', held: 'The gearbox output shafts' },
    caster: { name: 'Ball caster', kind: 'structure', lbl: 'CASTER', short: 'passive third wheel', info: 'Supports the rear without steering it.', held: 'Under the plate, at the back' },
    battery: { name: 'LiPo battery', kind: 'power', lbl: 'BATTERY', inside: true, short: '3S 2200 mAh', info: 'Voltage sags as it empties (12.6 V full, about 10.5 V empty), so the same PWM gives less speed: one reason wheel speed is closed-loop controlled with the encoders.', held: 'Velcro strap on the plate', spec: '11.1 V, 24 Wh' },
    driver: { name: 'Dual H-bridge driver', kind: 'electronics', lbl: 'MOTOR DRIVER', inside: true, short: 'PWM power stage', info: 'Turns 3.3 V logic signals into amps. Each H-bridge can drive forward, reverse or brake by shorting the motor.', held: 'Standoffs on the plate', spec: '2 x 10 A' },
    mcu: { name: 'Microcontroller board', kind: 'electronics', lbl: 'MICROCONTROLLER', inside: true, short: 'speed loops + odometry', info: 'Every 10 ms: read encoder counts, compare each wheel’s speed with its target, adjust PWM (a PI loop), and add the motion to the pose estimate.', held: 'Standoffs on the plate' },
    imu: { name: 'IMU', kind: 'sensor', lbl: 'IMU', inside: true, short: 'gyro + accelerometer', info: 'The gyro measures turning rate directly. Fusing it with the encoders corrects heading errors from wheel slip.', held: 'The plate, near the centre' },
    wires: { name: 'Wiring loom', kind: 'wiring', lbl: 'WIRING', inside: true, short: 'power + signal', info: 'Power (red/black), motor leads, encoder (blue) and PWM (yellow).', held: 'Cable ties to the plate' },
    bumper: { name: 'Bumper + microswitch', kind: 'sensor', lbl: 'BUMPER', key: true, short: 'last-resort contact sensor', info: 'Drive into a crate: the bar slides back, the switch opens, and the motors stop.', held: 'Two sprung brackets on the plate' },
    shell: { name: 'Shell', kind: 'cover', lbl: 'SHELL', key: true, short: 'protective cover', info: 'Keeps fingers and dust out. See-through shows what is under it.', held: 'Four screws into the plate' },
    lidar: { name: '2D lidar', kind: 'sensor', lbl: 'LIDAR', key: true, short: '360° laser scanner', info: 'A laser pulse and a timer measure distance; a motor spins the head 10 times a second. The scan feeds mapping (SLAM) and obstacle avoidance. It sees one horizontal slice only: obstacles lower than the beam are invisible to it.', held: 'The top of the shell', spec: '12 m range, 8000 samples/s' },
  },
  concepts: [
    ['Differential drive', 'Both wheels the same speed: straight. Different speeds: an arc. Opposite speeds: spin on the spot. v = (vR + vL)/2 and ω = (vR − vL)/track. Push the joystick to full forward and full turn together and a wheel hits its top speed (saturation): the turn rate drops.'],
    ['Encoders & odometry', 'Counting ticks gives each wheel’s distance; adding up small arcs gives the pose. The blue dashed path is the odometry estimate, the red line where the robot really went. The worn right tyre makes them drift apart.'],
    ['Nonholonomic motion', 'The rover cannot slide sideways, only forward/back and turn. To reach a point beside it, it must turn first: try a waypoint just to one side.'],
    ['Go-to-goal control', 'For a waypoint the controller turns in proportion to the heading error and slows its forward speed when the error is large (v ∝ cos error). Click the floor to queue waypoints.'],
    ['Lidar', 'Time-of-flight distance in many directions. The scan plot shows the robot’s-eye view: everything it knows about the world right now.'],
  ],
  draw: rvDraw,
  active: st => { const a = {}; if (Math.abs(st.vL) > 1 || Math.abs(st.vR) > 1) a.motors = a.wheels = a.encoders = a.driver = true; if (st.bumpT > 0) a.bumper = true; if (st.mode === 'op') a.lidar = true; return a; },
  opHTML: () => `<div class="joyw"><div class="joy" id="rvjoy"><i></i></div><small>Drag the knob (or W A S D / arrow keys).<br>Up: forward. Sideways: turn.<br>Click the floor to drop waypoints.</small></div>
    ${slHTML('rvsp', 'Top speed', 20, 100, 5, '%')}
    <div class="row2"><button id="rvclr">Clear waypoints</button><button id="rvtr">Clear trails</button><button id="rvodo">Reset odometry</button><button id="rvscan" aria-pressed="true">Lidar rays</button></div>
    <h3 style="margin-top:8px">Lidar scan (robot's view)</h3><svg class="gaitc" id="rvplot" viewBox="-50 -50 100 100" style="max-width:190px;margin:0 auto"></svg>`,
  opBind(el, st, ui, lab){ const $ = id => document.getElementById(id);
    joyBind($('rvjoy'), st, lab);
    $('s_rvsp').oninput = e => { st.vscale = +e.target.value / 100; ui.render(); };
    $('rvclr').onclick = () => { st.wps = []; ui.render(); }; $('rvtr').onclick = () => { st.trail = []; st.odoTrail = []; ui.render(); };
    $('rvodo').onclick = () => { st.odo = [st.pos[0], st.pos[1], st.th]; st.trail = []; st.odoTrail = []; st.ticks = [0, 0]; ui.render(); };
    $('rvscan').onclick = () => { st.scan = !st.scan; ui.render(); }; },
  opSync(el, st){ const $ = id => document.getElementById(id);
    slSync('rvsp', Math.round(st.vscale*100), Math.round(st.vscale*100) + '%'); $('rvscan').setAttribute('aria-pressed', st.scan);
    let s = '<circle r="45" fill="#fff" stroke="#231e16" stroke-width="1"/><circle r="22.5" fill="none" stroke="#ddd6c8" stroke-width=".6"/><path d="M0 -45V45M-45 0H45" stroke="#ddd6c8" stroke-width=".6"/>';
    for (const h of st.hits) if (h[2]){ const x = -Math.sin(h[4])*h[3], y = -Math.cos(h[4])*h[3]; s += `<circle cx="${f(x)}" cy="${f(y)}" r="1.6" fill="#e2483d"/>`; }
    s += '<path d="M0 -6L4 4L-4 4Z" fill="#71b693" stroke="#231e16" stroke-width="1"/>';
    $('rvplot').innerHTML = s; },
  readout(st){ const rpm = v => v/(2*Math.PI*RV.RW)*60;
    return [
      { k: 'Speed v', bar: Math.abs(st.v)/RV.VMAX, v: `${st.v.toFixed(1)} cm/s` },
      { k: 'Turn rate ω', bar: Math.abs(st.w)/RV.WMAX, v: `${(st.w/DEG).toFixed(0)}°/s` },
      { k: 'Left wheel', bar: Math.abs(st.vL)/RV.VMAX, v: `${rpm(st.vL).toFixed(0)} rpm`, warn: st.sat && Math.abs(st.vL) >= RV.VMAX - .1 },
      { k: 'Right wheel', bar: Math.abs(st.vR)/RV.VMAX, v: `${rpm(st.vR).toFixed(0)} rpm`, warn: st.sat && Math.abs(st.vR) >= RV.VMAX - .1, tip: 'Red = at its top speed (saturated)' },
      { k: 'Encoder ticks L / R', v: `${Math.round(st.ticks[0])} / ${Math.round(st.ticks[1])}` },
      { k: 'Heading', v: `${(((st.th/DEG) % 360 + 360) % 360).toFixed(0)}°` },
      { k: 'Odometry error', bar: Math.hypot(st.odo[0] - st.pos[0], st.odo[1] - st.pos[1]) / 30, v: `${(Math.hypot(st.odo[0] - st.pos[0], st.odo[1] - st.pos[1])*10).toFixed(0)} mm`, tip: 'Distance between where it thinks it is and where it is' },
      { k: 'Nearest obstacle', bar: 1 - st.near/45, v: st.near > 44.9 ? '> 45 cm' : `${st.near.toFixed(0)} cm`, warn: st.near < 18 },
      { k: 'Motor current (est.)', bar: (0.4 + Math.abs(st.acc)*.012 + Math.abs(st.w)*.3) / 3, v: `${(0.4 + Math.abs(st.acc)*.012 + Math.abs(st.w)*.3).toFixed(1)} A` },
      { k: 'Battery', bar: (st.batt - 10.5)/2.1, v: `${st.batt.toFixed(2)} V` },
      { k: 'Distance / bumps', v: `${(st.dist/100).toFixed(1)} m / ${st.crashes}`, warn: st.bumpT > 0 } ]; },
  stats: st => [{ label: 'Speed', value: `${Math.abs(st.v).toFixed(0)} cm/s`, small: true }],
  tick: rvTick,
  enterOp: st => { st.hits = []; },
  onFloor(st, x, y, lab){ if (Math.hypot(x - RV.C, y - RV.C) > RV.R0) return; const w = [x - RV.C + st.pos[0], y - RV.C + st.pos[1]];
    if (rvHit(w[0], w[1], 12)) return lab.flash('Too close to an obstacle for a waypoint'); if (st.wps.length >= 8) return lab.flash('Up to 8 waypoints');
    st.wps.push(w); lab.flash(`Waypoint ${st.wps.length} set`, 900); },
  params(st, q){ if (q.has('wps')) st.wps = q.get('wps').split(';').map(s => s.split(',').map(Number)); if (q.has('th')) st.th = +q.get('th')*DEG; if (q.has('pos')){ st.pos = q.get('pos').split(',').map(Number); st.odo = [st.pos[0], st.pos[1], st.th]; } },
};
ROBOT_ORDER.push({ id: 'rover', name: 'Differential-drive rover', blurb: 'Two wheels, two gear motors, two encoders and a caster. Steer by speed difference, queue waypoints, read the lidar and watch dead-reckoning drift away from the truth.', chips: ['2 wheels', 'DC gear motors', 'Odometry', 'Lidar'] });
