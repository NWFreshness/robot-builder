/* ============================================================
   Robot 09 — mobile manipulator (a differential-drive base with a 5-DOF arm). 1 unit = 1 cm.
   Truth: a low, heavy base keeps the centre of mass between the wheels: two hub motors with encoders, front and rear
   casters, a big battery low down, motor drivers and an onboard computer, a lidar at bumper height and an RGB-D camera
   on a rear mast. The arm turret is bolted near the front edge. Joints use compact actuators (motor + strain-wave gear +
   encoder in one housing). A camera on the wrist looks between the gripper fingers.
   Whole-body: when the target is outside the arm's reach, the base drives to a spot where it is inside, turns to face it,
   then the arm finishes the job. Base + arm = 3 + 5 DOF.
   Parts: chassis | motors, wheels, casters | battery, drivers, pc | lidar | shell | mast, headcam | turret, j1 | j2, upper |
          j3, fore | wrist | grip, wcam | harness | cube, bin
   ============================================================ */
const MM = { C: 60, R0: 58, TRACK: 30, RW: 6, VMAX: 45, WMAX: 1.6, AX: 9, SH: 31, L1: 24, L2: 22, W: 6, G: 8, RMIN: 20, RMAX: 40, STAND: 30 };
const MM_LIM = [[-160, 160], [0, 175], [0, 165], [-110, 110], [-180, 180], [0, 1]];
const MM_VMAX = [70, 50, 60, 90, 120, 1.5];
const MM_NAMES = ['Arm yaw', 'Shoulder', 'Elbow', 'Wrist pitch', 'Wrist roll', 'Gripper'];
const MM_STOW = [0, 95, 150, 40, 0, .7];
const MM_BIN = { x: -30, y: 30, s: 9, h: 11 };
const MM_CUBE = 2.6;

const mmScene = (st, w) => [MM.C + w[0] - st.pos[0], MM.C + w[1] - st.pos[1]];
const mmTR = st => tMul(tTr(MM.C, MM.C, 0), tRz(st.th));
function mmFK(st, q){ const B = tCat(mmTR(st), tTr(MM.AX, 0, MM.SH)), Ty = tMul(B, tRz(q[0]*DEG)), Tu = tMul(Ty, tRy(-q[1]*DEG));
  const Te = tCat(Tu, tTr(MM.L1, 0, 0), tRy(q[2]*DEG)), Tw = tCat(Te, tTr(MM.L2, 0, 0), tRy(q[3]*DEG)), Tr = tCat(Tw, tTr(MM.W, 0, 0), tRx(q[4]*DEG));
  return { B, Ty, Tu, Te, Tw, Tr, E: tAp(Te, [0,0,0]), W: tAp(Tw, [0,0,0]), tip: tAp(Tr, [MM.G, 0, 0]) }; }
// world point -> robot frame, measured from the arm's yaw axis
function mmLocal(st, w){ const dx = w[0] - st.pos[0], dy = w[1] - st.pos[1], c = Math.cos(-st.th), s = Math.sin(-st.th);
  return [dx*c - dy*s - MM.AX, dx*s + dy*c, (w[2] ?? 0) - MM.SH]; }
// arm IK, tool pointing straight down; target in world coordinates
function mmIK(st, w, grip){ const [x, y, z] = mmLocal(st, w), r = Math.hypot(x, y), yaw = Math.atan2(y, x)/DEG, wz = z + MM.W + MM.G, D = Math.hypot(r, wz);
  if (D > MM.L1 + MM.L2 - .1) return { err: 'Out of the arm’s reach' }; if (D < Math.abs(MM.L1 - MM.L2) + 2) return { err: 'Too close to the shoulder' };
  const a1 = Math.atan2(wz, r) + Math.acos(clamp((MM.L1**2 + D*D - MM.L2**2) / (2*MM.L1*D), -1, 1)), be = Math.atan2(wz - MM.L1*Math.sin(a1), r - MM.L1*Math.cos(a1));
  const q = [yaw, a1/DEG, (a1 - be)/DEG, (be + Math.PI/2)/DEG, 0, grip];
  for (let i = 0; i < 4; i++) if (q[i] < MM_LIM[i][0] || q[i] > MM_LIM[i][1]) return { err: `${MM_NAMES[i]} would pass its limit`, q };
  return { q }; }
// can the arm reach this world point without the base moving?
function mmReachable(st, w){ const [x, y] = mmLocal(st, w), r = Math.hypot(x, y); return r >= MM.RMIN && r <= MM.RMAX && Math.abs(Math.atan2(y, x)/DEG) < 150; }
function mmHit(px, py, st){ const b = MM_BIN, cx = clamp(px, b.x - b.s, b.x + b.s), cy = clamp(py, b.y - b.s, b.y + b.s);
  if (Math.hypot(px - cx, py - cy) < 23) return 'the bin';
  if (!st.held && !st.inBin && Math.hypot(px - st.cube[0], py - st.cube[1]) < 23) return 'the cube';
  return null; }

function mmDraw(st, m){
  const TR = mmTR(st), F = mmFK(st, st.q), wheelsAt = ROBOTS.mobman._step.wheels;
  if (st.mode === 'build' && st.built <= wheelsAt){ sbFree(); sbPut(qBoxNoPut(TR, [-8, -8, 0], [8, 8, 5], matOf('#c9bfa9')), obbT(TR, [-8, -8, 0], [8, 8, 5]), { p: null }); }
  // ---- chassis, drive, casters
  sbPart('chassis'); qBox(TR, [-21, -14, 5], [21, 14, 7], ALU, { w: 1.2 });
  for (const s of [1, -1]){ const y = v => s*v, lo = (a, b) => Math.min(y(a), y(b)), hi = (a, b) => Math.max(y(a), y(b));
    sbPart('motors'); qCyl(TR, 'y', [0, 6], y(9), y(13.4), 4.2, MOTOR, { w: 1 }); qBox(TR, [-3, lo(9, 12), 5], [3, hi(9, 12), 7], ALU, { w: .7 });
    sbPart('wheels'); qCyl(TR, 'y', [0, 6], y(13.4), y(17), MM.RW, TYRE, { w: 1.4, n: 28 });
    const out = tVec(TR, [0, s, 0]); qCyl(TR, 'y', [0, 6], y(17), y(17.3), 3, ALU, { w: .8, deco: () => vis(out) ? [0, 1, 2].map(k => { const a = st.wa[s > 0 ? 0 : 1] + k*2*Math.PI/3;
      return line([tAp(TR, [0, y(17.32), 6]), tAp(TR, [2.6*Math.cos(a), y(17.32), 6 + 2.6*Math.sin(a)])], .7); }).join('') : '' }); }
  sbPart('casters'); for (const x of [17, -17]){ qCyl(TR, 'z', [x, 0], 2.6, 5, 1.5, ALU, { w: .7 }); qBall(tAp(TR, [x, 0, 1.3]), 1.3, STEEL); }
  // ---- inside the shell
  sbGroup('shell', TR, [-21, -14, 7], [21, 14, 18]);
  sbPart('battery'); qBox(TR, [-17, -7, 7], [-2, 7, 12], BATT, { w: 1, deco: n => n[2] === 1 ? txT(TR, [-9.5, 0, 12.02], [1,0,0], [0,1,0], '48V 20Ah', 1.8, '#fff') : '' });
  sbPart('drivers'); qBox(TR, [0, -11, 7], [8, -4, 7.6], PCB, { w: .7 }); qBox(TR, [1, -10, 7.6], [7, -5, 10], ALU, { w: .7, deco: n => n[2] === 1 ? [2, 3.5, 5, 6.5].map(x => line([tAp(TR, [x, -9.6, 10.02]), tAp(TR, [x, -5.4, 10.02])], .5)).join('') : '' });
  sbPart('pc'); qBox(TR, [0, 3, 7], [11, 11, 10.5], DARK, { w: .9, deco: n => n[2] === 1 ? txT(TR, [5.5, 7, 10.52], [1,0,0], [0,1,0], 'COMPUTE', 1.3, '#93d3b0') : '' });
  sbPart('harness'); qWire([[-3, 0, 10], [1, -4, 9.5], [4, -7, 10]].map(p => tAp(TR, p)), CABLE.pwr, .4); qWire([[5, 3, 9], [8, 1, 13], [MM.AX, 0, 17.5]].map(p => tAp(TR, p)), CABLE.sig, .35);
  sbPart('shell'); qExt(TR, rrUV(-21, -14, 21, 14, 5), 7, 18, m, { ax: 'z', cover: true, w: 2.2, capDeco: b => b ? txT(TR, [-6, 0, 18.02], [0,-1,0], [1,0,0], 'MOMA-8', 3) : '' });
  sbFree(); sbPart('lidar'); qRod(tAp(TR, [19, 0, 12]), tAp(TR, [19, 0, 15]), 2.6, DARK, { w: 1, sh: tone });
  qBox(TR, [21, -9, 8], [21.6, 9, 11], matOf('#ef6a4c'), { w: .8, sh: tone });
  // ---- mast with the head camera
  sbPart('mast'); qBox(TR, [-17, -1.2, 18], [-14.6, 1.2, 46], ALU, { w: .9 });
  sbPart('headcam'); const Hc = tCat(TR, tTr(-15.8, 0, 47.5), tRz(st.cam), tRy(28*DEG)); qBox(Hc, [-2, -6, -1.6], [2.4, 6, 1.6], DARK, { w: 1, sh: tone,
    deco: n => n[0] === 1 ? [-3.6, 0, 3.6].map(y => dotAt(tAp(Hc, [2.42, y, 0]), y ? .55 : .8, y ? '#3a3633' : '#5a82c6', .35)).join('') : '' });
  // ---- arm: turret + J1 actuator, shoulder, upper arm, elbow, forearm, wrist, gripper with camera
  sbPart('turret'); qRod(tAp(TR, [MM.AX, 0, 18]), tAp(TR, [MM.AX, 0, 20]), 6, STEEL, { w: 1.1 });
  sbPart('j1'); qRod(tAp(TR, [MM.AX, 0, 20]), tAp(TR, [MM.AX, 0, 25]), 4.6, MOTOR, { w: 1.1, sh: tone });
  qCyl(F.Ty, 'z', [0, 0], -6, -3.4, 4.2, m, { w: 1, sh: tone }); for (const y of [-4.6, 3.4]) qBox(F.Ty, [-3.4, y, -3.4], [3.4, y + 1.2, 1], m, { w: .9, sh: tone });
  sbPart('j2'); qCyl(F.Ty, 'y', [0, 0], -3.4, 3.4, 3.6, MOTOR, { w: 1.1, sh: tone });
  sbPart('upper'); qExt(F.Tu, rrUV(-2.4, -2, 2.4, 2, 1.2), 3, MM.L1 - 3, m, { w: 1.6 });
  sbPart('j3'); qCyl(F.Tu, 'y', [MM.L1, 0], -3, 3, 3.1, MOTOR, { w: 1, sh: tone });
  sbPart('fore'); qExt(F.Te, rrUV(-2, -1.7, 2, 1.7, 1), 2.8, MM.L2 - 2.4, m, { w: 1.4 });
  sbPart('wrist'); qCyl(F.Te, 'y', [MM.L2, 0], -2.4, 2.4, 2.4, MOTOR, { w: .9, sh: tone }); qCyl(F.Tw, 'x', [0, 0], 2.2, MM.W, 2, MOTOR, { w: .9, sh: tone });
  sbPart('grip'); const gap = 1 + st.q[5]*6;
  qBox(F.Tr, [0, -3, -1.6], [1.4, 3, 1.6], ALU, { w: .8 });
  for (const s of [1, -1]){ const y0 = s*gap/2, y1 = s*(gap/2 + .6); qBox(F.Tr, [1.4, Math.min(y0, y1), -1.2], [MM.G + .4, Math.max(y0, y1), 1.2], DARK, { w: .8 }); }
  sbPart('wcam'); qBox(F.Tr, [.2, -1.2, 1.6], [2.6, 1.2, 3], DARK, { w: .7, deco: n => n[0] === 1 ? dotAt(tAp(F.Tr, [2.62, 0, 2.3]), .45, '#5a82c6', .3) : '' });
  sbPart('harness'); qWire([tAp(F.Ty, [-3, 3.8, -1]), tAp(F.Tu, [6, 2.4, 2.2]), tAp(F.Tu, [MM.L1 - 4, 2.4, 2.2])], '#3a3633', .35, { seg: 4 });
  qWire([tAp(F.Tu, [MM.L1 + 1, 3.1, 2]), tAp(F.Te, [5, 2, 1.8]), tAp(F.Te, [MM.L2 - 3, 2, 1.8])], '#3a3633', .3, { seg: 4 });
  // ---- world: the cube and the bin
  sbFree(); sbPart('cube'); const s = MM_CUBE;
  if (st.held) qBox(tMul(F.Tr, tTr(MM.G - 2.4, 0, 0)), [-s, -s, -s], [s, s, s], matOf('#f4b829'), { w: 1, sh: tone });
  else { const [cx, cy] = mmScene(st, st.cube); if (Math.hypot(cx - MM.C, cy - MM.C) < MM.R0 - 3) qBox(tMul(tTr(cx, cy, st.cube[2] + s), tRz(st.cube[3] || 0)), [-s, -s, -s], [s, s, s], matOf('#f4b829'), { w: 1, sh: tone }); }
  sbPart('bin'); { const [bx, by] = mmScene(st, [MM_BIN.x, MM_BIN.y]), b = MM_BIN, far = Math.hypot(bx - MM.C, by - MM.C);
    if (far < MM.R0 - b.s*1.5){ const M = matOf('#71b693'); qBox(T_I, [bx - b.s, by - b.s, 0], [bx + b.s, by + b.s, .8], M, { w: .9 });
      for (const [lo, hi] of [[[bx - b.s, by - b.s, .8], [bx + b.s, by - b.s + .8, b.h]], [[bx - b.s, by + b.s - .8, .8], [bx + b.s, by + b.s, b.h]], [[bx - b.s, by - b.s + .8, .8], [bx - b.s + .8, by + b.s - .8, b.h]], [[bx + b.s - .8, by - b.s + .8, .8], [bx + b.s, by + b.s - .8, b.h]]]) qBox(T_I, lo, hi, M, { w: .9, sh: tone });
      if (st.binN) sbPut(txT(T_I, [bx, by - b.s - .05, b.h*.55], [1,0,0], [0,0,1], String(st.binN), 3.2), obbW([bx - 2, by - b.s - .1, 3], [bx + 2, by - b.s, 8]), { p: 'bin' }); } }
  // ---- floor: reach ring around the arm, path, trail, target marker
  const rim = on(PZ(0), ellUV(MM.C, MM.C, MM.R0, MM.R0, 64)); let floor = '', over = '';
  if (st.mode === 'op'){ const ax = tAp(TR, [MM.AX, 0, 0]), ring = r => `<path d="${pd(on(PZ(.05), ellUV(ax[0], ax[1], r, r, 60)))}" fill="none" stroke="#3d5878" stroke-width=".7" stroke-dasharray="1.6 1.2"/>`;
    floor += clipTo(rim, ring(MM.RMIN) + ring(MM.RMAX) + trailSVG(st.trail.map(w => [...mmScene(st, w), .05]), '#e2483d', 1));
    if (st.goalW){ const [gx, gy] = mmScene(st, st.goalW); floor += clipTo(rim, `<path d="${pd(on(PZ(.06), ellUV(gx, gy, 2, 2, 16)))}" fill="#e2483d" fill-opacity=".3" stroke="#e2483d" stroke-width=".8"/>`); }
    if (!st.held && !st.inBin){ const [cx, cy] = mmScene(st, st.cube), c0 = tAp(Hc, [2.4, 0, 0]), q0 = P(...c0), q1 = P(cx, cy, 2.6);
      over += `<path d="M${f(q0[0])} ${f(q0[1])}L${f(q1[0])} ${f(q1[1])}" stroke="#5a82c6" stroke-width=".7" stroke-dasharray="1.4 1.2" opacity=".7" pointer-events="none"/>`; }
    if (st.crash) over += crashBurst(st.crash.at, st.crash.t); }
  return { under: floorDisc(MM.C, MM.C, MM.R0, st.pos[0], st.pos[1]), z0: 0, clip: rim, floor, over };
}

// ---- task planner: base go-to, turn-to-face, arm moves and gripper actions, run in order
function mmPlan(st, kind, lab){
  if (st.inBin && kind !== 'stow') return lab.flash('The cube is in the bin — press New cube');
  const P = [], c = st.cube, B = [MM_BIN.x, MM_BIN.y];
  const visit = (pt) => { if (mmReachable(st, [pt[0], pt[1]]) && !P.length) return;
    const dx = pt[0] - st.pos[0], dy = pt[1] - st.pos[1], d = Math.hypot(dx, dy), u = [dx/d, dy/d];
    P.push({ ph: 'drive', to: [pt[0] - u[0]*(MM.STAND + MM.AX), pt[1] - u[1]*(MM.STAND + MM.AX)], face: pt }); };
  if (!st.held && (kind === 'reach' || kind === 'fetch')){ visit(c);
    P.push({ ph: 'arm', w: [c[0], c[1], c[2] + MM_CUBE + 9], grip: 1 }, { ph: 'arm', w: [c[0], c[1], c[2] + MM_CUBE], grip: 1 }, { ph: 'grip', grip: 0 }, { ph: 'q', q: MM_STOW, keep: true }); }
  if (kind === 'fetch' || (kind === 'reach' && st.held)){ P.push({ ph: 'q', q: MM_STOW, keep: true }); P.push({ ph: 'drive', to: null, face: B, bin: true });
    P.push({ ph: 'arm', w: [B[0], B[1], MM_BIN.h + MM_CUBE + 6], grip: null }, { ph: 'grip', grip: 1 }, { ph: 'q', q: MM_STOW }); }
  if (kind === 'stow') P.push({ ph: 'q', q: MM_STOW, keep: true });
  st.task = P; st.taskT = 0; lab.flash(kind === 'stow' ? 'Stowing the arm' : P[0] && P[0].ph === 'drive' ? 'Out of the arm’s reach: the base drives first' : 'In reach: the arm alone does it'); }
function mmTick(st, dt, ui, lab){
  const j = st.joy || [0, 0], manual = Math.hypot(j[0], j[1]) > .05; let vC = 0, wC = 0, armBusy = false;
  if (manual && st.task){ st.task = null; st.goalW = null; lab.flash('Task cancelled: manual driving'); }
  if (manual){ vC = j[1]*MM.VMAX; wC = -j[0]*MM.WMAX; }
  const T = st.task && st.task[0];
  if (T){ st.taskT += dt;
    if (T.ph === 'drive'){ if (T.bin && !T.to){ const dx = T.face[0] - st.pos[0], dy = T.face[1] - st.pos[1], d = Math.hypot(dx, dy), u = [dx/d, dy/d]; T.to = [T.face[0] - u[0]*(MM.STAND + MM.AX), T.face[1] - u[1]*(MM.STAND + MM.AX)]; }
      st.goalW = T.to; const dx = T.to[0] - st.pos[0], dy = T.to[1] - st.pos[1], dist = Math.hypot(dx, dy);
      if (dist > 2.5){ const e = wrap180((Math.atan2(dy, dx) - st.th)/DEG)*DEG; wC = clamp(3*e, -MM.WMAX, MM.WMAX); vC = MM.VMAX*.8*Math.max(0, Math.cos(e))*Math.min(1, dist/15); }
      else { const ef = wrap180((Math.atan2(T.face[1] - st.pos[1], T.face[0] - st.pos[0]) - st.th)/DEG); wC = clamp(3*ef*DEG, -MM.WMAX, MM.WMAX);
        if (Math.abs(ef) < 2 && Math.abs(st.w) < .1){ st.task.shift(); st.goalW = null; } } }
    else if (T.ph === 'arm'){ if (!T.started){ T.started = true; const r = mmIK(st, T.w, T.grip ?? st.q[5]);
        if (r.err){ st.task = null; lab.flash(r.err + ' — task stopped'); } else st.qt = r.q; } armBusy = true; }
    else if (T.ph === 'q'){ if (!T.started){ T.started = true; st.qt = [...T.q.slice(0, 5), T.keep ? st.qt[5] : T.q[5]]; } armBusy = true; }
    else if (T.ph === 'grip'){ if (!T.started){ T.started = true; st.qt = [...st.qt.slice(0, 5), T.grip]; } armBusy = true; } }
  // base dynamics: acceleration limits, wheel speeds, collisions
  const dv = clamp(vC - st.v, -60*dt, 60*dt), dw = clamp(wC - st.w, -3*dt, 3*dt); st.v += dv; st.w += dw;
  const np = [st.pos[0] + st.v*Math.cos(st.th)*dt, st.pos[1] + st.v*Math.sin(st.th)*dt], nth = st.th + st.w*dt, hit = mmHit(np[0], np[1], st);
  if (hit && (Math.abs(st.v) > .01)){ st.v = st.w = 0; st.task = null; st.goalW = null; st.crashes++; st.crash = { at: tAp(mmTR(st), [22, 0, 8]), t: 1 }; lab.flash(`Bump: the base hit ${hit}`); }
  else { st.pos = np; st.th = nth; }
  st.wa[0] -= (st.v - st.w*MM.TRACK/2)/MM.RW*dt; st.wa[1] -= (st.v + st.w*MM.TRACK/2)/MM.RW*dt; st.dist += Math.abs(st.v*dt);
  const l = st.trail[st.trail.length - 1]; if (!l || Math.hypot(l[0] - st.pos[0], l[1] - st.pos[1]) > 1.5){ st.trail.push(st.pos.slice()); if (st.trail.length > 300) st.trail.shift(); }
  // head camera turns toward the cube
  const tc = st.held || st.inBin ? 0 : wrap180((Math.atan2(st.cube[1] - st.pos[1], st.cube[0] - st.pos[0]) - st.th)/DEG)*DEG; st.cam += clamp(tc - st.cam, -2*dt, 2*dt);
  // arm joints
  let moving = false; const nq = st.q.slice();
  for (let i = 0; i < 6; i++){ const d = st.qt[i] - st.q[i]; if (Math.abs(d) > 1e-4){ nq[i] += Math.sign(d)*Math.min(Math.abs(d), MM_VMAX[i]*dt); moving = true; } }
  if (st.held && nq[5] < .45) nq[5] = .45;
  moving = nq.some((v, i) => Math.abs(v - st.q[i]) > 1e-6);
  st.qd = nq.map((v, i) => (v - st.q[i]) / Math.max(dt, 1e-3)); st.q = nq;
  const F = mmFK(st, st.q), tipW = [F.tip[0] - MM.C + st.pos[0], F.tip[1] - MM.C + st.pos[1], F.tip[2]];
  if (F.tip[2] < .5 || F.W[2] < 2){ st.qt = st.q.slice(); }
  if (!st.held && !st.inBin && st.q[5] < .55 && st.qd[5] < 0 && Math.hypot(tipW[0] - st.cube[0], tipW[1] - st.cube[1]) < 2 && Math.abs(tipW[2] - st.cube[2] - MM_CUBE) < 2){ st.held = true; lab.flash('Cube gripped'); }
  if (st.held && st.qt[5] > .6){ st.held = false; const b = MM_BIN, inB = Math.abs(tipW[0] - b.x) < b.s - 2 && Math.abs(tipW[1] - b.y) < b.s - 2;
    st.cube = [tipW[0], tipW[1], inB ? .8 : 0, st.th + st.q[0]*DEG]; if (inB){ st.inBin = true; st.binN++; lab.flash('Delivered to the bin'); } else lab.flash('Cube put down'); }
  if (armBusy && !moving && st.task && st.task[0] && st.task[0].started){ st.task.shift(); if (!st.task.length){ st.task = null; } }
  if (st.task && !st.task.length) st.task = null;
  if (st.crash){ st.crash.t -= dt/1.2; if (st.crash.t <= 0) st.crash = null; }
  return manual || !!st.task || moving || Math.abs(st.v) > .05 || Math.abs(st.w) > .005 || !!st.crash || Math.abs(tc - st.cam) > .01;
}
const mmNewCube = st => { const a = Math.random()*2*Math.PI, r = 45 + Math.random()*40; let c = [st.pos[0] + r*Math.cos(a), st.pos[1] + r*Math.sin(a)];
  if (Math.hypot(c[0] - MM_BIN.x, c[1] - MM_BIN.y) < 30) c = [st.pos[0] + 55, st.pos[1] - 20]; st.cube = [c[0], c[1], 0, a]; st.inBin = false; st.held = false; };

ROBOTS.mobman = {
  id: 'mobman', fig: '09', name: 'Mobile manipulator', pal: 'tomato', kicker: 'diff-drive base + 5-DOF arm · whole-body reach',
  blurb: 'A rover with an arm on top. When the cube is out of the arm’s reach the base drives to it, turns to face it and parks; then the arm picks it. Fetch carries it to the bin.',
  size: [120, 120, 66], center: [60, 60], bounds: [20, 100, 20, 100, 0, 62],
  init: () => ({ pos: [0, 0], th: 0, v: 0, w: 0, wa: [0, 0], joy: [0, 0], dist: 0, trail: [], cam: 0, q: MM_STOW.slice(), qt: MM_STOW.slice(), qd: [0,0,0,0,0,0],
    cube: [30, -42, 0, .3], held: false, inBin: false, binN: 0, task: null, taskT: 0, goalW: null, crash: null, crashes: 0 }),
  thumbYaw: 0, thumbState: { q: [-20, 70, 100, 30, 0, .7], cube: [36, -6, 0, .3] },
  steps: [
    { title: 'Chassis', parts: ['chassis'], text: 'A thick aluminium plate. It sits on a stand until the wheels go on.' },
    { title: 'Hub motors, wheels & casters', parts: ['motors', 'wheels', 'casters'], text: 'Two brushless hub motors with encoders drive the wheels; casters front and back keep it level. Track 300 mm.' },
    { title: 'Battery, drivers & computer', parts: ['battery', 'drivers', 'pc'], peek: true, text: 'A 48 V pack sits low and central so the base stays stable when the arm reaches out. Motor drivers for the wheels, and a GPU computer that runs perception and planning.' },
    { title: 'Lidar & bumper', parts: ['lidar'], text: 'A 2D lidar at the front for mapping and obstacle detection, behind a soft bumper strip.' },
    { title: 'Shell', parts: ['shell'], text: 'A moulded cover with the arm turret opening near the front.' },
    { title: 'Mast & head camera', parts: ['mast', 'headcam'], text: 'An RGB-D camera (colour + depth) high on a rear mast sees the floor ahead and finds the cube. It turns to keep the cube in view.' },
    { title: 'Turret & J1 actuator', parts: ['turret', 'j1'], text: 'The arm’s yaw joint, bolted near the front edge so the arm reaches past the bumper.' },
    { title: 'Shoulder & upper arm', parts: ['j2', 'upper'], text: 'The shoulder actuator: motor, strain-wave gear and encoder in one housing. 240 mm upper arm.' },
    { title: 'Elbow & forearm', parts: ['j3', 'fore'], text: 'The elbow actuator and a 220 mm forearm.' },
    { title: 'Wrist', parts: ['wrist'], text: 'Wrist pitch and wrist roll actuators: five arm joints in all.' },
    { title: 'Gripper & wrist camera', parts: ['grip', 'wcam'], text: 'Parallel jaws, with a small camera on top for the last few centimetres of the approach.' },
    { title: 'Arm harness', parts: ['harness'], text: 'Power and an EtherCAT bus daisy-chained through the actuators.' },
    { title: 'Cube & bin', parts: ['cube', 'bin'], text: 'A cube on the floor and a bin to deliver it to. In Operate, press Fetch.' },
  ],
  parts: {
    chassis: { name: 'Chassis', kind: 'structure', lbl: 'CHASSIS', key: true, short: '420 × 280 mm plate', info: 'A wide, heavy base: when the arm reaches out with a load, the centre of mass must stay inside the wheel and caster footprint.', held: 'Wheels and casters' },
    motors: { name: 'Hub motors', kind: 'motor', lbl: 'HUB MOTORS', key: true, short: 'brushless, encoders', info: 'Motors built into the wheel hubs: no gearbox to wear out, and quiet.', held: 'Brackets under the chassis' },
    wheels: { name: 'Drive wheels', kind: 'structure', lbl: 'WHEELS', short: '120 mm', info: 'Differential drive: steer by speed difference, cannot slide sideways.', held: 'The hub motors' },
    casters: { name: 'Casters', kind: 'structure', short: 'front and rear', info: 'Two casters: the base is stable whichever way the arm leans.', held: 'Under the chassis' },
    battery: { name: 'Battery', kind: 'power', lbl: 'BATTERY', inside: true, short: '48 V, 1 kWh', info: 'Heavy on purpose, and low: it is the counterweight for the arm.', held: 'Straps on the chassis' },
    drivers: { name: 'Wheel drivers', kind: 'electronics', lbl: 'DRIVERS', inside: true, short: 'field-oriented control', info: 'Three-phase inverters running speed loops for each hub motor.', held: 'The chassis' },
    pc: { name: 'Onboard computer', kind: 'electronics', lbl: 'COMPUTER', inside: true, short: 'perception + planning', info: 'Finds the cube in the camera images, plans where to park, solves the arm IK and sequences the task.', held: 'The chassis' },
    lidar: { name: 'Lidar & bumper', kind: 'sensor', lbl: 'LIDAR', short: '2D scanner at the front', info: 'Keeps the base from driving into things it cannot see from the mast.', held: 'Front of the chassis' },
    shell: { name: 'Shell', kind: 'cover', lbl: 'SHELL', key: true, short: 'cover', info: 'See-through shows the battery, drivers and computer.', held: 'Screws to the chassis' },
    mast: { name: 'Mast', kind: 'structure', short: 'camera mount', info: 'High up so the camera looks down on the floor ahead.', held: 'Rear of the chassis' },
    headcam: { name: 'Head RGB-D camera', kind: 'sensor', lbl: 'HEAD CAMERA', key: true, short: 'colour + depth', info: 'Depth comes from projected infrared dots seen by two cameras. The dashed blue line shows what it is looking at.', held: 'Pan joint on the mast' },
    turret: { name: 'Arm turret', kind: 'structure', short: 'arm mount', info: 'Bolted through the shell to the chassis.', held: 'The chassis' },
    j1: { name: 'J1 yaw actuator', kind: 'motor', lbl: 'J1', short: 'arm yaw', info: 'Swings the arm left and right; with the base turning too, yaw is redundant.', held: 'The turret' },
    j2: { name: 'J2 shoulder actuator', kind: 'motor', lbl: 'SHOULDER', short: 'highest torque', info: 'Motor + strain-wave gear + encoder + brake in one housing.', held: 'The J1 bracket' },
    upper: { name: 'Upper arm', kind: 'structure', lbl: 'UPPER ARM', short: '240 mm', info: 'Carbon tube with aluminium end fittings.', held: 'The shoulder actuator', spec: 'L1 = 240 mm' },
    j3: { name: 'J3 elbow actuator', kind: 'motor', lbl: 'ELBOW', short: 'elbow', info: 'Smaller than the shoulder: less load out here.', held: 'The end of the upper arm' },
    fore: { name: 'Forearm', kind: 'structure', lbl: 'FOREARM', short: '220 mm', info: 'Arm reach on its own: 200 to 400 mm from the yaw axis (the dashed rings).', held: 'The elbow actuator', spec: 'L2 = 220 mm' },
    wrist: { name: 'Wrist actuators', kind: 'motor', lbl: 'WRIST', short: 'pitch + roll', info: 'Keeps the gripper pointing straight down while the arm moves.', held: 'The forearm' },
    grip: { name: 'Gripper', kind: 'tool', lbl: 'GRIPPER', key: true, short: 'parallel jaws', info: 'Opens to 70 mm.', held: 'The wrist roll flange' },
    wcam: { name: 'Wrist camera', kind: 'sensor', short: 'close-up view', info: 'Corrects the last few millimetres of error that the head camera and the base odometry leave.', held: 'Top of the gripper' },
    harness: { name: 'Harness', kind: 'wiring', short: 'power + EtherCAT', info: 'One bus cable chains every actuator.', held: 'Clips along the arm' },
    cube: { shadow: 'cube', name: 'Cube', kind: 'tool', short: '52 mm', info: 'Click the floor in Operate to move it.', held: 'The floor' },
    bin: { shadow: 'bin', name: 'Bin', kind: 'structure', short: 'delivery target', info: 'Count of delivered cubes on its side.', held: 'The floor' },
  },
  concepts: [
    ['Why put an arm on wheels?', 'A fixed arm’s workspace is a hollow sphere around its base. Wheels move that sphere anywhere on the floor. The dashed rings are the arm’s reach right now; anything outside them needs the base.'],
    ['Whole-body planning', 'Plan where to park so the target lands inside the reach rings, turn to face it, then let the arm finish. Real systems also move base and arm at the same time; here they take turns, which is easier to follow.'],
    ['Redundancy', 'Base yaw and arm yaw both turn the gripper about a vertical axis: 8 DOF for a 3D point + gripper angle. The planner uses the base to aim and keeps arm yaw near zero, the most stable pose.'],
    ['Nonholonomic base', 'The base cannot slide sideways, so parking beside the cube means turning first. Try clicking the floor right next to the robot and pressing Reach.'],
    ['Stability', 'Reaching out with a load moves the centre of mass forward. A heavy, low battery and front and rear casters keep it from tipping.'],
  ],
  draw: mmDraw,
  active: st => { const a = {}; if (Math.abs(st.v) > .5 || Math.abs(st.w) > .02) a.motors = a.wheels = a.drivers = true;
    const nm = ['j1', 'j2', 'j3', 'wrist', 'wrist', 'grip']; st.qd.forEach((v, i) => { if (Math.abs(v) > .5) a[nm[i]] = true; }); if (st.task) a.pc = true; return a; },
  opHTML: () => `<div class="row2"><button id="mmfetch">Fetch to bin</button><button id="mmreach">Reach cube</button><button id="mmstow">Stow arm</button><button id="mmnew">New cube</button></div>
    <div class="joyw"><div class="joy" id="mmjoy"><i></i></div><small>Drive the base by hand (or W A S D).<br>Click the floor to move the cube there.</small></div>
    ${MM_NAMES.map((n, i) => slHTML('mm' + i, n, i === 5 ? 0 : MM_LIM[i][0], i === 5 ? 100 : MM_LIM[i][1], 1, i === 5 ? '%' : '°')).join('')}`,
  opBind(el, st, ui, lab){ const $ = id => document.getElementById(id);
    joyBind($('mmjoy'), st, lab);
    for (let i = 0; i < 6; i++) $('s_mm' + i).oninput = e => { st.task = null; st.qt[i] = i === 5 ? +e.target.value/100 : +e.target.value; lab.kick(); };
    $('mmfetch').onclick = () => { mmPlan(st, 'fetch', lab); lab.kick(); ui.render(); };
    $('mmreach').onclick = () => { mmPlan(st, 'reach', lab); lab.kick(); ui.render(); };
    $('mmstow').onclick = () => { mmPlan(st, 'stow', lab); lab.kick(); };
    $('mmnew').onclick = () => { st.task = null; mmNewCube(st); lab.kick(); ui.render(); }; },
  opSync(el, st){ for (let i = 0; i < 6; i++) slSync('mm' + i, i === 5 ? st.qt[5]*100 : st.qt[i], i === 5 ? `${((1 + st.q[5]*6)*10).toFixed(0)} mm` : `${st.q[i].toFixed(0)}°`); },
  readout(st){ const L = mmLocal(st, [st.cube[0], st.cube[1], 0]), r = Math.hypot(L[0], L[1]), reach = mmReachable(st, st.cube), F = mmFK(st, st.q), rows = [];
    rows.push({ k: 'Base speed', bar: Math.abs(st.v)/MM.VMAX, v: `${st.v.toFixed(0)} cm/s` });
    rows.push({ k: 'Base turn', bar: Math.abs(st.w)/MM.WMAX, v: `${(st.w/DEG).toFixed(0)}°/s` });
    rows.push({ k: 'Base pose', v: `${(st.pos[0]*10).toFixed(0)}, ${(st.pos[1]*10).toFixed(0)} mm, ${(((st.th/DEG) % 360 + 360) % 360).toFixed(0)}°` });
    MM_NAMES.slice(0, 4).forEach((n, i) => rows.push({ k: n, bar: Math.abs(st.qd[i])/MM_VMAX[i], v: `${st.q[i].toFixed(0)}°` }));
    rows.push({ k: 'Cube from arm axis', bar: r/80, v: st.held ? 'held' : st.inBin ? 'in the bin' : `${(r*10).toFixed(0)} mm`, warn: !st.held && !st.inBin && !reach, tip: 'Arm alone reaches 200–400 mm, in front' });
    rows.push({ k: 'Who does it', v: st.held || st.inBin ? '—' : reach ? 'arm alone' : 'base + arm' });
    rows.push({ k: 'Gripper height', v: `${(F.tip[2]*10).toFixed(0)} mm` });
    rows.push({ k: 'Task', v: st.task && st.task[0] ? { drive: 'driving', arm: 'arm moving', q: 'arm moving', grip: 'gripper' }[st.task[0].ph] + ` (${st.task.length} steps left)` : 'idle' });
    rows.push({ k: 'Delivered / bumps', v: `${st.binN} / ${st.crashes}`, warn: !!st.crash });
    return rows; },
  stats: st => [{ label: 'Cubes delivered', value: String(st.binN), small: true }],
  tick: mmTick,
  enterOp: st => { st.qt = st.q.slice(); },
  onFloor(st, x, y, lab){ if (Math.hypot(x - MM.C, y - MM.C) > MM.R0 - 3) return; if (st.held) return lab.flash('Put the cube down first');
    const w = [x - MM.C + st.pos[0], y - MM.C + st.pos[1]];
    if (Math.hypot(w[0] - st.pos[0], w[1] - st.pos[1]) < 24) return lab.flash('That spot is under the robot');
    if (Math.hypot(w[0] - MM_BIN.x, w[1] - MM_BIN.y) < 16) return lab.flash('That spot is in the bin');
    st.task = null; st.cube = [w[0], w[1], 0, 0]; st.inBin = false; lab.flash(mmReachable(st, st.cube) ? 'Cube moved: in the arm’s reach' : 'Cube moved: out of reach, the base will have to drive', 1200); },
  params(st, q){ if (q.has('cube')){ const v = q.get('cube').split(',').map(Number); st.cube = [v[0], v[1], 0, 0]; }
    if (q.has('q')){ const v = q.get('q').split(',').map(Number); if (v.length === 6){ st.q = v.slice(); st.qt = v.slice(); } } },
};
ROBOT_ORDER.push({ id: 'mobman', name: 'Mobile manipulator', blurb: 'The rover with an arm on top: when the cube is out of reach the base drives and parks, then the arm picks it and carries it to the bin.', chips: ['Base + arm', '8 DOF', 'Whole-body'] });
