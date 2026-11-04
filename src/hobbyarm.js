/* ============================================================
   Robot 05 — desktop hobby arm (5 DOF + gripper, serial bus servos, 3D-printed links). 1 unit = 1 cm.
   Truth: six smart servos on one daisy-chained 3-wire bus (power, ground, half-duplex data). Each servo has its own
   magnetic encoder (4096 counts per turn), a 345:1 gear train and a small controller that answers "where are you, how
   hot, how loaded" on the bus. A USB bus adapter and a 7.4 V brick sit on the table. The base yaw servo stands in a
   printed housing; the shoulder servo rides the turntable; elbow, wrist pitch and wrist roll servos sit at their joints;
   a sixth servo opens the jaws. A smaller leader arm with the same joints (servos unpowered, read only) is moved by hand;
   the follower copies its angles every bus cycle: the way teleoperation data for robot learning is collected.
   Parts: plate, yawS | housing | turn, shS | upper, elS | fore, wpS | wrS | grip | bus, adapter, psu | leader | cube
   ============================================================ */
const HA = { BX: 15, BY: 28, Z0: 3, SH: 10, L1: 11.6, L2: 10.4, W: 4.4, G: 6, LK: .62, LX: 48, LY: 36 };
const HA_LIM = [[-150, 150], [5, 175], [0, 165], [-105, 105], [-180, 180], [0, 1]];
const HA_VMAX = [160, 120, 140, 200, 280, 2.2];
const HA_NAMES = ['Base yaw', 'Shoulder', 'Elbow', 'Wrist pitch', 'Wrist roll', 'Gripper'];
const HA_HOME = [0, 80, 115, 55, 0, .6];
const HA_CUBE0 = [30, 18];
const HA_SCALE = k => [k,0,0,0, 0,k,0,0, 0,0,k,0];

function haFK(q, base = tTr(HA.BX, HA.BY, HA.Z0 + HA.SH)){
  const Ty = tMul(base, tRz(q[0]*DEG)), Tu = tMul(Ty, tRy(-q[1]*DEG)), Te = tCat(Tu, tTr(HA.L1, 0, 0), tRy(q[2]*DEG));
  const Tw = tCat(Te, tTr(HA.L2, 0, 0), tRy(q[3]*DEG)), Tr = tCat(Tw, tTr(HA.W, 0, 0), tRx(q[4]*DEG));
  return { Ty, Tu, Te, Tw, Tr, E: tAp(Te, [0,0,0]), W: tAp(Tw, [0,0,0]), tip: tAp(Tr, [HA.G, 0, 0]) }; }
const haLeadBase = () => tCat(tTr(HA.LX, HA.LY, HA.Z0 + HA.SH*HA.LK), HA_SCALE(HA.LK));
// tool tip p (follower frame), tool pitch phi (deg from horizontal), elbow up
function haIK(p, phi, roll, grip){
  const dx = p[0] - HA.BX, dy = p[1] - HA.BY, r = Math.hypot(dx, dy), yaw = Math.atan2(dy, dx)/DEG, z = p[2] - HA.Z0 - HA.SH, ph = phi*DEG;
  const wr = r - (HA.W + HA.G)*Math.cos(ph), wz = z - (HA.W + HA.G)*Math.sin(ph), D = Math.hypot(wr, wz);
  if (D > HA.L1 + HA.L2 - .05) return { err: 'Out of reach: the arm is fully stretched' };
  if (D < Math.abs(HA.L1 - HA.L2) + 1) return { err: 'Too close to the shoulder' };
  const a1 = Math.atan2(wz, wr) + Math.acos(clamp((HA.L1**2 + D*D - HA.L2**2) / (2*HA.L1*D), -1, 1));
  const be = Math.atan2(wz - HA.L1*Math.sin(a1), wr - HA.L1*Math.cos(a1));
  const q = [yaw, a1/DEG, (a1 - be)/DEG, (be - ph)/DEG, roll, grip];
  for (let i = 0; i < 5; i++) if (q[i] < HA_LIM[i][0] - 1e-6 || q[i] > HA_LIM[i][1] + 1e-6) return { err: `${HA_NAMES[i]} would pass its limit`, q };
  return { q }; }
const haToolPitch = q => q[1] - q[2] - q[3];
const haGap = g => .5 + g*4.4;
function haCheck(q){ const F = haFK(q);
  for (const [nm, p, r] of [['elbow', F.E, 1.8], ['wrist', F.W, 1.6], ['gripper', F.tip, .2], ['gripper', tAp(F.Tr, [1, 0, 0]), 1.6]]){
    if (p[2] - r < HA.Z0) return { why: `Crash: the ${nm} hit the table`, at: [p[0], p[1], HA.Z0] };
    if (nm !== 'elbow' && Math.hypot(p[0] - HA.BX, p[1] - HA.BY) < 5 + r && p[2] < HA.Z0 + HA.SH + 2) return { why: 'Self-collision: the gripper hit the base', at: p }; }
  return null; }
// gravity torque (N m) at shoulder, elbow, wrist pitch: masses in kg at points along the arm
function haTorques(q, held){ const F = haFK(q), g = 9.81, S = tAp(F.Tu, [0,0,0]);
  const M = [[tAp(F.Tu, [HA.L1/2, 0, 0]), .03], [F.E, .055], [tAp(F.Te, [HA.L2/2, 0, 0]), .025], [F.W, .055], [tAp(F.Tw, [HA.W/2, 0, 0]), .055], [tAp(F.Tr, [3, 0, 0]), .07 + (held ? .03 : 0)]];
  const hz = (a, b) => Math.hypot(b[0] - a[0], b[1] - a[1]) * Math.sign((b[0] - a[0])*Math.cos(q[0]*DEG) + (b[1] - a[1])*Math.sin(q[0]*DEG) || 1);
  return [S, F.E, F.W].map((o, j) => Math.abs(M.slice(j*2).reduce((a, [p, m]) => a + m*g*hz(o, p)/100, 0))); }

// one arm: full detail for the follower; the leader uses the same joints, plain printed links and no motors' wiring
function haArm(st, q, m, lead){
  const F = haFK(q, lead ? haLeadBase() : undefined), B = lead ? haLeadBase() : tTr(HA.BX, HA.BY, HA.Z0 + HA.SH);
  const part = n => sbPart(lead ? 'leader' : n), Sv = lead ? matOf('#6f6a63') : SERVO, Lk = lead ? matOf('#efe9da') : m;
  const servo = (T, lo, hi, ax, c, a0, a1) => { qBox(T, lo, hi, Sv, { w: .8 }); qCyl(T, ax, c, a0, a1, 1.1, HORN, { w: .6 }); };
  // base: plate, yaw servo, housing
  part('plate'); qBox(B, [-5, -5, -HA.SH], [5, 5, -HA.SH + .8], Lk, { w: 1 });
  for (const [x, y] of [[-4, -4], [4, -4], [4, 4], [-4, 4]]) qCyl(B, 'z', [x, y], -HA.SH + .8, -HA.SH + 1.1, .45, STEEL, { w: .4, n: 8 });
  sbGroup(lead ? 'lhous' : 'hous', B, [-4.2, -4.2, -HA.SH + .8], [4.2, 4.2, -3.6]);
  part('yawS'); servo(B, [-2, -1.2, -HA.SH + .8], [2, 1.2, -4.3], 'z', [0, 0], -4.3, -3.7);
  part('housing'); qExt(B, rrUV(-4.2, -4.2, 4.2, 4.2, 2), -HA.SH + .8, -3.6, Lk, { ax: 'z', cover: !lead, w: 1.6,
    capDeco: b => b && !lead ? txT(B, [0, -2.6, -3.58], [1,0,0], [0,1,0], 'ARM-5', 1.3) : '' });
  sbFree();
  // turntable bracket + shoulder servo (axis along local y)
  part('turn'); qCyl(F.Ty, 'z', [0, 0], -3.6, -3, 3.4, Lk, { w: 1, sh: tone }); for (const y of [-2.6, 2.1]) qBox(F.Ty, [-1.6, y, -3], [1.6, y + .5, .4], Lk, { w: .8, sh: tone });
  part('shS'); servo(F.Ty, [-1.6, -2.1, -3], [1.8, 2.1, 1.4], 'y', [0, 0], 2.1, 2.5);
  // upper link + elbow servo
  part('upper'); qExt(F.Tu, rrUV(-1.5, -1.1, 1.5, 1.1, .6), 1.4, HA.L1 - 1.6, Lk, { w: 1.4 });
  part('elS'); servo(F.Tu, [HA.L1 - 1.6, -1.6, -1.5], [HA.L1 + 1.6, 1.6, 2.2], 'y', [HA.L1, 0], 1.6, 2);
  // forearm + wrist pitch servo
  part('fore'); qExt(F.Te, rrUV(-1.3, -1, 1.3, 1, .5), 1.6, HA.L2 - 1.6, Lk, { w: 1.3 });
  part('wpS'); servo(F.Te, [HA.L2 - 1.5, -1.4, -1.4], [HA.L2 + 1.5, 1.4, 1.9], 'y', [HA.L2, 0], 1.4, 1.8);
  // wrist roll servo (axis along local x) and the gripper
  part('wrS'); servo(F.Tw, [.9, -1.3, -1.3], [HA.W - .4, 1.3, 1.3], 'x', [0, 0], HA.W - .4, HA.W);
  part('grip'); const gap = haGap(q[5]);
  qBox(F.Tr, [0, -1.8, -1.1], [.9, 1.8, 1.1], Lk, { w: .8 }); qBox(F.Tr, [.2, -1.1, 1.1], [3.2, 1.1, 2.7], Sv, { w: .7 });
  for (const s of [1, -1]){ const y0 = s*gap/2, y1 = s*(gap/2 + .45);
    qBox(F.Tr, [.9, Math.min(y0, y1), -.9], [HA.G + .3, Math.max(y0, y1), .9], lead ? Lk : matOf('#f4b829'), { w: .7 }); }
  if (lead){ part('leader'); qRod(tAp(F.Tr, [1.4, 0, -1.1]), tAp(F.Tr, [1.4, 0, -3.4]), .55, DARK, { w: .6 }); }
  return F; }

function haDraw(st, m){
  const F = haArm(st, st.q, m, false);
  // bus: daisy chain from the adapter up the arm, servo to servo
  sbPart('adapter'); qBox(T_I, [3, 6, 3], [9, 10, 3.5], PCB, { w: .7 }); qBox(T_I, [3.2, 7.2, 3.5], [4.6, 8.8, 4.2], STEEL, { w: .5 });
  sbPart('psu'); qBox(T_I, [3, 13, 3], [10, 17, 5.2], DARK, { w: .9, deco: n => n[2] === 1 ? txT(T_I, [6.5, 15, 5.22], [1,0,0], [0,1,0], '7.4V 5A', 1.1, '#f2efe7') : '' });
  sbPart('bus'); qWire([[9.5, 15, 4], [11, 12, 3.4], [8.5, 9.2, 3.6]], CABLE.pwr, .3);
  qWire([[8.8, 8, 3.6], [12, 14, 3.3], [HA.BX - 4.6, HA.BY - 2, 3.4], [HA.BX - 4.4, HA.BY - 1, 7]], '#3a3633', .3);
  const lp = (T, p) => tAp(T, p);
  qWire([lp(F.Ty, [-1.8, 0, -2.4]), lp(F.Ty, [-2.4, -2.8, -1]), lp(F.Tu, [3, -1.4, 1.4]), lp(F.Tu, [HA.L1 - 2, -1.6, 1.8])], '#3a3633', .25, { seg: 3 });
  qWire([lp(F.Tu, [HA.L1 - .6, -1.7, 2]), lp(F.Te, [2.5, -1.2, 1.2]), lp(F.Te, [HA.L2 - 2, -1.4, 1.4])], '#3a3633', .25, { seg: 3 });
  qWire([lp(F.Te, [HA.L2 - .5, -1.5, 1.6]), lp(F.Tw, [1.2, -1.4, 1.4]), lp(F.Tr, [1.4, -.6, 2.4])], '#3a3633', .25, { seg: 3 });
  if (partShown(st, ROBOTS.hobbyarm, 'leader')) haArm(st, st.lead, m, true);
  // cube
  sbFree(); sbPart('cube'); const c = st.cube, held = st.held, s = 1.25;
  const Tc = held ? tMul(F.Tr, tTr(HA.G - 1.2, 0, 0)) : tMul(tTr(c[0], c[1], HA.Z0 + c[2] + s), tRz(c[3]));
  qBox(Tc, [-s, -s, -s], [s, s, s], matOf('#5a82c6'), { w: 1, sh: tone });
  const pt = boardTop(0, 0, 60, 50, 3, 6); let floor = '', over = '';
  if (st.mode === 'op'){ if (st.trace) floor += trailSVG(st.trail, '#e2483d', .9, '2 1.4');
    const L = haFK(st.lead, haLeadBase()); over += tipHandle(L.tip, 3, !!st.crash); if (st.crash) over += crashBurst(st.crash.at, st.crash.t); }
  return { under: board(0, 0, 60, 50, 3, 6), z0: 3, clip: pt, floor, over };
}

function haTick(st, dt, ui, lab){
  let more = false;
  if (st.play != null){ st.play += dt*30; const i = Math.floor(st.play);
    if (i >= st.rec.length){ st.play = null; lab.flash('Replay finished'); } else { st.qt = st.rec[i].slice(); more = true; } }
  else st.qt = st.lead.slice();
  if (st.recording){ st.recT += dt; more = true;
    if (st.recT >= 1/30){ st.recT = 0; st.rec.push(st.lead.slice()); if (st.rec.length >= 600){ st.recording = false; lab.flash('Recorder full (20 s)'); } } }
  const nq = st.q.slice(); let moving = false;
  for (let i = 0; i < 6; i++){ const d = st.qt[i] - st.q[i]; if (Math.abs(d) > 1e-4){ nq[i] += Math.sign(d)*Math.min(Math.abs(d), HA_VMAX[i]*dt); moving = true; } }
  if (st.held && nq[5] < st.gripHold) nq[5] = st.gripHold;          // the cube stops the jaws
  moving = nq.some((v, i) => Math.abs(v - st.q[i]) > 1e-6);
  if (moving){ const c = haCheck(nq);
    if (c){ if (!st.crash){ st.crashes++; lab.flash(c.why + ' — follower holds position'); } st.crash = { at: c.at, t: 1 }; }
    else { st.qd = nq.map((v, i) => (v - st.q[i]) / Math.max(dt, 1e-3)); st.q = nq; const tp = haFK(st.q).tip, l = st.trail[st.trail.length - 1];
      if (!l || vLen(sub(tp, l)) > .4){ st.trail.push(tp); if (st.trail.length > 300) st.trail.shift(); } } }
  if (!moving || st.crash) st.qd = [0,0,0,0,0,0];
  // grasp: closing jaws around the cube take it; opening drops it to the table
  const F = haFK(st.q), gc = tAp(F.Tr, [HA.G - 1.2, 0, 0]);
  if (!st.held && st.q[5] < .55 && st.qd[5] < 0 && vLen(sub(gc, [st.cube[0], st.cube[1], HA.Z0 + st.cube[2] + 1.25])) < 1.6){ st.held = true; st.gripHold = (2.5 - .5)/4.4; lab.flash('Cube gripped'); }
  if (st.held && st.qt[5] > st.gripHold + .05){ st.held = false; st.cube = [gc[0], gc[1], 0, (st.q[0] + 0)*DEG]; lab.flash('Cube released'); }
  const tq = haTorques(st.q, st.held); st.tq = tq;
  st.temp = st.temp.map((T, i) => T + dt*((tq[i] || 0)/1.6*30 - (T - 26)*.04));
  if (st.crash){ st.crash.t -= dt/1.2; if (st.crash.t <= 0) st.crash = null; else more = true; }
  return more || moving || st.dragging || !!st.crash;
}

ROBOTS.hobbyarm = {
  id: 'hobbyarm', fig: '05', name: 'Desktop hobby arm', pal: 'blush', kicker: '5 DOF + gripper · bus servos · leader / follower teleop',
  blurb: 'A small arm of smart bus servos on 3D-printed links. Move the leader arm by hand and the follower copies it: the way people record demonstrations to teach robots new skills.',
  size: [60, 50, 34], center: [30, 25], bounds: [4, 56, 4, 46, 0, 32],
  init: () => ({ q: HA_HOME.slice(), qt: HA_HOME.slice(), lead: HA_HOME.slice(), qd: [0,0,0,0,0,0], tq: [0,0,0], temp: [26, 26, 26], trail: [], trace: true, crash: null, crashes: 0,
    cube: [HA_CUBE0[0], HA_CUBE0[1], 0, 0], held: false, gripHold: 0, rec: [], recording: false, recT: 0, play: null }),
  thumbYaw: 0,
  steps: [
    { title: 'Base plate & yaw servo', parts: ['plate', 'yawS'], text: 'A printed plate is clamped to the desk. The base servo stands in it with its output horn facing up: it turns the whole arm.' },
    { title: 'Base housing', parts: ['housing'], text: 'A printed shell closes over the yaw servo. See-through keeps it in view.' },
    { title: 'Turntable & shoulder servo', parts: ['turn', 'shS'], text: 'A bracket on the yaw horn carries the shoulder servo with its shaft horizontal. The shoulder works hardest: it lifts everything beyond it.' },
    { title: 'Upper arm & elbow servo', parts: ['upper', 'elS'], text: 'The 116 mm printed upper arm bolts to the shoulder horn and holds the elbow servo at its tip.' },
    { title: 'Forearm & wrist pitch servo', parts: ['fore', 'wpS'], text: 'The forearm (104 mm) ends in the wrist pitch servo, which keeps the gripper pointing where you want while the arm moves.' },
    { title: 'Wrist roll servo', parts: ['wrS'], text: 'Its shaft points along the forearm, so it spins the gripper about its own axis.' },
    { title: 'Gripper', parts: ['grip'], text: 'A sixth servo drives a rack between two jaws. Its load reading tells you how hard it is squeezing.' },
    { title: 'Bus cable, adapter & power', parts: ['bus', 'adapter', 'psu'], text: 'One 3-wire cable hops servo to servo: power, ground and a shared data line. Each servo has an ID (1 to 6); the USB adapter asks each in turn for its position, load and temperature.' },
    { title: 'Leader arm', parts: ['leader'], text: 'A smaller copy with the same joints. Its servos are left unpowered and only read their encoders, so a person can move it freely by hand.' },
    { title: 'Test cube', parts: ['cube'], text: 'Something to pick. In Operate, drag the leader’s ring, close the gripper on the cube and record the motion.' },
  ],
  parts: {
    plate: { name: 'Base plate', kind: 'structure', short: 'printed, clamped to the desk', info: 'Spreads the reaction forces of a moving arm into the desk. A light arm that is not clamped tips over when it reaches out.', held: 'Desk clamps' },
    yawS: { name: 'Base yaw servo (ID 1)', kind: 'motor', lbl: 'YAW SERVO', inside: true, short: 'bus servo, 4096-count encoder', info: 'Inside every bus servo: a coreless DC motor, a 345:1 metal gear train, a 12-bit magnetic encoder on the output and a controller running a position loop.', held: 'The base plate', spec: '1.6 N m stall at 7.4 V' },
    housing: { name: 'Base housing', kind: 'cover', lbl: 'BASE', key: true, short: 'printed shell', info: 'PLA, 2 mm walls. Printed links are light and cheap; their weakness is creep under a constant load.', held: 'Screws into the plate' },
    turn: { name: 'Turntable bracket', kind: 'structure', short: 'yaw horn to shoulder', info: 'Two printed cheeks either side of the shoulder servo.', held: 'The yaw servo horn' },
    shS: { name: 'Shoulder servo (ID 2)', kind: 'motor', lbl: 'SHOULDER', key: true, short: 'carries the most load', info: 'Gravity torque here = weight × horizontal reach. Stretch the arm out flat and watch its load bar climb toward stall.', held: 'The turntable bracket' },
    upper: { name: 'Upper arm', kind: 'structure', lbl: 'UPPER ARM', key: true, short: '116 mm printed link', info: 'Hollow, ribbed and printed in one piece.', held: 'The shoulder servo horn', spec: 'L1 = 116 mm' },
    elS: { name: 'Elbow servo (ID 3)', kind: 'motor', lbl: 'ELBOW', short: 'bus servo', info: 'Its own weight sits at the tip of the upper arm, so it adds to the shoulder load.', held: 'The tip of the upper arm' },
    fore: { name: 'Forearm', kind: 'structure', lbl: 'FOREARM', short: '104 mm printed link', info: 'Shorter and lighter than the upper arm: weight far out costs the most torque.', held: 'The elbow servo horn', spec: 'L2 = 104 mm' },
    wpS: { name: 'Wrist pitch servo (ID 4)', kind: 'motor', lbl: 'WRIST PITCH', short: 'keeps the tool angle', info: 'Shoulder + elbow + wrist pitch together decide the gripper angle: pitch = q2 − q3 − q4.', held: 'The tip of the forearm' },
    wrS: { name: 'Wrist roll servo (ID 5)', kind: 'motor', lbl: 'WRIST ROLL', short: 'spins the gripper', info: 'Turns the jaws to line up with an object’s edges.', held: 'The wrist pitch horn' },
    grip: { name: 'Gripper (ID 6)', kind: 'tool', lbl: 'GRIPPER', key: true, short: 'two jaws, 0–49 mm', info: 'The jaw servo is position controlled. When a jaw meets the cube it stalls short of its target, and the load reading rises: the simplest grip-force sensor there is.', held: 'The wrist roll horn' },
    bus: { name: 'Servo bus cable', kind: 'wiring', lbl: 'BUS', inside: true, short: 'daisy chain, 1 Mbit/s', info: 'Half-duplex serial: the adapter talks, then listens. At 1 Mbit/s it can read and write all six servos a few hundred times a second.', held: 'Clips on each servo' },
    adapter: { name: 'USB bus adapter', kind: 'electronics', lbl: 'BUS ADAPTER', short: 'USB to servo bus', info: 'Turns the computer’s USB serial into the servo bus and injects power from the brick.', held: 'The desk' },
    psu: { name: 'Power brick', kind: 'power', lbl: '7.4 V SUPPLY', short: '7.4 V, 5 A', info: 'Six stalled servos can pull over 10 A, so the supply sets a real limit on how hard the arm can push.', held: 'The desk' },
    leader: { shadow: 'leader', name: 'Leader arm', kind: 'tool', lbl: 'LEADER ARM', key: true, short: 'hand-moved, read only', info: 'Same joint layout at a smaller scale, so joint angles copy straight across with no inverse kinematics. Drag its yellow ring to move it.', held: 'Its own base plate' },
    cube: { shadow: 'cube', name: 'Test cube', kind: 'tool', short: '25 mm, 20 g', info: 'Close the gripper with the jaws either side of it to pick it.', held: 'The desk' },
  },
  concepts: [
    ['Bus servos', 'Hobby servos used to take one PWM signal each and report nothing. Bus servos share one cable, have IDs and send back position, speed, load, voltage and temperature: a robot arm that can feel.'],
    ['Leader / follower teleoperation', 'The human moves the leader; every bus cycle its joint angles become the follower’s targets. Because both arms have identical joints, no inverse kinematics is needed. The follower lags when its speed limit is reached: see Tracking error.'],
    ['Recording demonstrations', 'Press Record and move the leader: each frame stores the joint angles (30 per second). Replay runs the follower from the recording alone. A learned policy does the same job, predicting the next angles from camera images instead of replaying.'],
    ['Gravity load', 'Torque = mass × g × horizontal distance. With the arm folded up the shoulder is nearly unloaded; stretched out flat it works hardest. Printed-arm servos are close to their stall torque, which is why these arms carry only a few hundred grams.'],
    ['Heat', 'A servo holding a load draws current all the time and warms up. Hobby servos cut out around 70 °C; keep the arm folded when idle.'],
  ],
  draw: haDraw,
  active: st => { const a = {}, nm = ['yawS', 'shS', 'elS', 'wpS', 'wrS', 'grip']; (st.qd || []).forEach((v, i) => { if (Math.abs(v) > .5) a[nm[i]] = true; }); if (st.recording) a.leader = true; return a; },
  enterOp: st => { st.qt = st.q.slice(); },
  opHTML: () => `<div class="row2"><button id="harec">Record</button><button id="haplay">Replay</button><button id="hahome">Home</button><button id="hacube">Reset cube</button></div>
    <div class="row2"><button id="haopen">Open gripper</button><button id="haclose">Close gripper</button></div>
    <p class="note">Drag the yellow ring on the leader arm (right): the follower copies its joints. Sliders move the leader too.</p>
    ${HA_NAMES.map((n, i) => slHTML('ha' + i, n, i === 5 ? 0 : HA_LIM[i][0], i === 5 ? 100 : HA_LIM[i][1], 1, i === 5 ? '%' : '°')).join('')}
    <div class="row2"><button id="hatr" aria-pressed="true">Trace</button><button id="haclr">Clear trace</button></div>`,
  opBind(el, st, ui, lab){ const $ = id => document.getElementById(id);
    const stopPlay = () => { if (st.play != null){ st.play = null; lab.flash('Replay stopped: the leader has control'); } };
    for (let i = 0; i < 6; i++) $('s_ha' + i).oninput = e => { stopPlay(); st.lead[i] = i === 5 ? +e.target.value/100 : +e.target.value; lab.kick(); };
    $('haopen').onclick = () => { stopPlay(); st.lead[5] = 1; lab.kick(); }; $('haclose').onclick = () => { stopPlay(); st.lead[5] = 0; lab.kick(); };
    $('harec').onclick = () => { if (st.recording){ st.recording = false; lab.flash(`Recorded ${(st.rec.length/30).toFixed(1)} s (${st.rec.length} frames)`); }
      else { stopPlay(); st.rec = []; st.recording = true; lab.flash('Recording: move the leader arm'); } lab.kick(); ui.render(); };
    $('haplay').onclick = () => { if (!st.rec.length) return lab.flash('Record something first');
      st.recording = false; st.play = 0; lab.flash('Replaying: the follower runs from the recording, the leader is ignored'); lab.kick(); ui.render(); };
    $('hahome').onclick = () => { stopPlay(); st.lead = HA_HOME.slice(); lab.kick(); };
    $('hacube').onclick = () => { st.held = false; st.cube = [HA_CUBE0[0], HA_CUBE0[1], 0, 0]; ui.render(); };
    $('hatr').onclick = () => { st.trace = !st.trace; ui.render(); }; $('haclr').onclick = () => { st.trail = []; ui.render(); }; },
  opSync(el, st){ const $ = id => document.getElementById(id);
    for (let i = 0; i < 6; i++) slSync('ha' + i, i === 5 ? st.lead[5]*100 : st.lead[i], i === 5 ? `${(haGap(st.lead[5])*10).toFixed(0)} mm` : `${st.lead[i].toFixed(0)}°`, i < 5 && (Math.abs(st.lead[i] - HA_LIM[i][0]) < .5 || Math.abs(st.lead[i] - HA_LIM[i][1]) < .5));
    $('harec').textContent = st.recording ? `Stop (${(st.rec.length/30).toFixed(1)} s)` : 'Record'; $('harec').setAttribute('aria-pressed', st.recording);
    $('haplay').setAttribute('aria-pressed', st.play != null); $('hatr').setAttribute('aria-pressed', st.trace); },
  readout(st){ const tick = a => Math.round(2048 + a/360*4096), err = Math.max(...st.q.slice(0, 5).map((v, i) => Math.abs(v - st.qt[i]))), rows = [];
    HA_NAMES.slice(0, 5).forEach((n, i) => rows.push({ k: `${n} ${st.q[i].toFixed(0)}°`, bar: i >= 1 && i <= 3 ? st.tq[i - 1]/1.6 : Math.abs(st.qd[i])/HA_VMAX[i], v: i >= 1 && i <= 3 ? `${(st.tq[i - 1]/1.6*100).toFixed(0)}% load` : `${tick(st.q[i])}`, warn: i >= 1 && i <= 3 && st.tq[i - 1] > 1.2, tip: i >= 1 && i <= 3 ? 'Gravity torque / 1.6 N m stall' : 'Encoder count (0–4095)' }));
    rows.push({ k: 'Jaw gap', v: `${(haGap(st.q[5])*10).toFixed(0)} mm${st.held ? ' · holding' : ''}` });
    rows.push({ k: 'Tracking error', bar: err/30, v: `${err.toFixed(1)}°`, warn: err > 15, tip: 'Largest gap between leader and follower joints' });
    rows.push({ k: 'Shoulder temp', bar: (st.temp[0] - 20)/50, v: `${st.temp[0].toFixed(1)} °C`, warn: st.temp[0] > 55 });
    rows.push({ k: 'Recording', v: st.recording ? `REC ${(st.rec.length/30).toFixed(1)} s` : st.play != null ? `replay ${(st.play/30).toFixed(1)} / ${(st.rec.length/30).toFixed(1)} s` : st.rec.length ? `${(st.rec.length/30).toFixed(1)} s stored` : '—' });
    rows.push({ k: 'Crashes', v: `${st.crashes}`, warn: !!st.crash });
    return rows; },
  stats: st => { const t = haFK(st.q).tip; return [{ label: 'Tip X Y Z mm', value: `${Math.round((t[0]-HA.BX)*10)} ${Math.round((t[1]-HA.BY)*10)} ${Math.round((t[2]-HA.Z0)*10)}`, small: true }]; },
  tick: haTick,
  onDrag(st, dx, dy, phase, lab){
    if (phase === 'start'){ st.play = null; st.dragP = haFK(st.lead).tip; return; } if (phase !== 'move') return;
    const d = vSc(dragWorld(dx, dy), 1/HA.LK), p = vAdd(st.dragP, d), ph0 = haToolPitch(st.lead);
    // keep the gripper angle if possible; otherwise tilt it as little as needed to stay inside the joint limits
    let r = haIK(p, ph0, st.lead[4], st.lead[5]);
    for (let k = 1; r.err && k <= 12; k++) for (const sg of [1, -1]){ const r2 = haIK(p, ph0 + sg*k*8, st.lead[4], st.lead[5]); if (!r2.err){ r = r2; break; } }
    if (r.err) return lab.flash(r.err); st.dragP = p; st.lead = r.q; },
  params(st, q){ if (q.has('q')){ const v = q.get('q').split(',').map(Number); if (v.length === 6){ st.q = v.slice(); st.qt = v.slice(); st.lead = v.slice(); } } },
};
ROBOT_ORDER.push({ id: 'hobbyarm', name: 'Desktop hobby arm', blurb: 'A 5-DOF + gripper arm of serial bus servos on 3D-printed links. Move the leader arm, the follower copies it; record a demonstration and replay it.', chips: ['5 DOF', 'Bus servos', 'Leader / follower'] });
