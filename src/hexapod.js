/* ============================================================
   Robot 04 — hexapod walker (18 hobby servos, ~45 cm across). 1 unit = 1 cm.
   Truth: two plates sandwich six coxa servos (vertical axes) on standoffs, with the battery, a 5-6 V regulator (UBEC),
   a 32-channel servo controller, a small computer and an IMU between them. Each coxa servo horn carries a U-bracket with
   the femur servo (horizontal axis); the femur is a pair of side plates ending at the knee servo; the tibia hangs from the
   knee horn and ends in a rubber foot. Each leg = 3 DOF, so the foot can be put anywhere in a volume; the body stays level
   while stance feet stay planted on the ground and swing feet arc forward.
   Parts: plates | battery, ubec | ctrlb, sbc, imu | coxaS | cbr, femurS | femur, tibiaS | tibia, foot | leads | top | head
   ============================================================ */
const HX = { C: 40, R0: 38, COXA: 4.4, FEM: 7.5, TIB: 11, MASS: 2.0, VMAX: 9, WMAX: .55, LIFT: 3.2 };
const HX_LEGS = [ { n: 'LF', m: [9.6, 5.8], a: 40 }, { n: 'LM', m: [0, 8.6], a: 90 }, { n: 'LR', m: [-9.6, 5.8], a: 140 },
                  { n: 'RF', m: [9.6, -5.8], a: -40 }, { n: 'RM', m: [0, -8.6], a: -90 }, { n: 'RR', m: [-9.6, -5.8], a: -140 } ];
HX_LEGS.forEach(L => { const a = L.a*DEG, r = HX.COXA + 9.6; L.N = [L.m[0] + r*Math.cos(a), L.m[1] + r*Math.sin(a)]; });
const HX_GAITS = {
  tripod: { label: 'Tripod', duty: .5, off: [0, .5, 0, .5, 0, .5], freq: 1.1, note: 'Three legs up, three down: a stable triangle always under the body. The fastest statically stable gait.' },
  ripple: { label: 'Ripple', duty: 2/3, off: [0, 1/3, 2/3, .5, 5/6, 1/6], freq: .9, note: 'Two legs swing at a time, offset left to right: four feet always down, smoother and stronger.' },
  wave:   { label: 'Wave', duty: 5/6, off: [2/6, 1/6, 0, 5/6, 4/6, 3/6], freq: .75, note: 'One leg at a time, rear to front, left side then right: five feet down, slow but very stable, good for rough ground.' } };
const HX_OCT = [[12, 4], [5, 9.2], [-5, 9.2], [-12, 4], [-12, -4], [-5, -9.2], [5, -9.2], [12, -4]];
const HX_LIM = { g: 55, a: [-60, 95], b: [15, 165] };

const hxRot = (th, v) => [v[0]*Math.cos(th) - v[1]*Math.sin(th), v[0]*Math.sin(th) + v[1]*Math.cos(th)];
const hxBodyW = st => { const l = hxRot(st.th, st.lean); return [st.pos[0] + l[0], st.pos[1] + l[1]]; };
const hxScene = (st, w) => [HX.C + w[0] - st.pos[0], HX.C + w[1] - st.pos[1]];
// leg IK in the body frame (origin on the body axis at femur-axis height)
function hxIK(L, fb){
  const dx = fb[0] - L.m[0], dy = fb[1] - L.m[1], h = fb[2], g = wrap180(Math.atan2(dy, dx)/DEG - L.a), rad = Math.hypot(dx, dy) - HX.COXA, D = Math.hypot(rad, h);
  if (D > HX.FEM + HX.TIB - .15) return { err: 'cannot reach that far' }; if (D < HX.TIB - HX.FEM + .4) return { err: 'foot too close to the body' };
  const al = Math.atan2(h, rad) + Math.acos(clamp((HX.FEM**2 + D*D - HX.TIB**2) / (2*HX.FEM*D), -1, 1));
  const ph = Math.atan2(h - HX.FEM*Math.sin(al), rad - HX.FEM*Math.cos(al)), be = al - ph, ang = [g, al/DEG, be/DEG];
  if (Math.abs(g) > HX_LIM.g) return { err: 'coxa servo at its limit', ang };
  if (ang[1] < HX_LIM.a[0] || ang[1] > HX_LIM.a[1]) return { err: 'femur servo at its limit', ang };
  if (ang[2] < HX_LIM.b[0] || ang[2] > HX_LIM.b[1]) return { err: 'tibia servo at its limit', ang };
  return { ang }; }
function hxFootBody(st, i){ const b = hxBodyW(st), w = st.feet[i], d = hxRot(-st.th, [w[0] - b[0], w[1] - b[1]]); return [d[0], d[1], w[2] - st.zb]; }
function hxSolve(st){ st.legErr = [];
  HX_LEGS.forEach((L, i) => { const r = hxIK(L, hxFootBody(st, i)); if (r.ang && !r.err) st.ang[i] = r.ang; else { st.legErr.push(i); if (r.ang) st.ang[i] = r.ang; } }); }
function hxSupport(st){
  const down = st.feet.map((p, i) => p[2] < .3 ? i : -1).filter(i => i >= 0), pts = down.map(i => st.feet[i].slice(0, 2)), c = hxBodyW(st);
  if (pts.length < 3) return { down, poly: pts, margin: -1 };
  const hl = hull(pts.map(p => p.slice())); let m = Infinity, pos = 0, neg = 0;
  for (let i = 0; i < hl.length; i++){ const a = hl[i], b = hl[(i + 1) % hl.length], ex = b[0] - a[0], ey = b[1] - a[1], l = Math.hypot(ex, ey) || 1;
    const cr = (ex*(c[1] - a[1]) - ey*(c[0] - a[0])) / l; m = Math.min(m, Math.abs(cr)); if (cr > 1e-9) pos++; else if (cr < -1e-9) neg++; }
  const inside = !(pos && neg);
  return { down, poly: hl, margin: inside ? m : -m }; }

function hxDraw(st, m){
  const bw = hxBodyW(st), [bx, by] = hxScene(st, bw), TB = tMul(tTr(bx, by, st.zb), tRz(st.th)), tibAt = ROBOTS.hexapod._step.tibia;
  if (st.mode === 'build' && st.built <= tibAt){ sbFree(); sbPut(qBoxNoPut(TB, [-5, -5, -st.zb], [5, 5, -2.4], matOf('#c9bfa9')), obbT(TB, [-5, -5, -st.zb], [5, 5, -2.4]), { p: null }); }
  // ---- body sandwich: bottom plate, standoffs, battery, UBEC, boards, coxa servos, leads, top plate
  sbGroup('body', TB, [-12, -9.2, -2.4], [12, 9.2, 2.4]);
  sbPart('plates'); qExt(TB, HX_OCT, -2.4, -1.8, ALU, { ax: 'z', w: 1.4 });
  for (const [x, y] of [[7.5, 0], [-7.5, 0], [4, 7], [-4, 7], [4, -7], [-4, -7]]) qCyl(TB, 'z', [x, y], -1.8, 1.8, .35, STEEL, { w: .5, n: 8 });
  sbPart('battery'); qBox(TB, [-6.5, -3, -1.8], [2, 3, -.4], BATT, { w: .8, deco: n => n[2] === 1 ? txT(TB, [-2.2, 0, -.38], [1,0,0], [0,1,0], '2S 5000', 1.3, '#fff') : '' });
  sbPart('ubec'); qBox(TB, [3, -3, -1.8], [6, -1, -1], matOf('#e8c45a'), { w: .6 });
  sbPart('ctrlb'); qBox(TB, [-6, -3.4, .4], [1.5, 3.4, .8], matOf('#2f6fb0'), { w: .7, deco: n => n[2] === 1 ? Array.from({length:9}, (_, k) => line([tAp(TB, [-5.5 + k*.8, 2.6, .82]), tAp(TB, [-5.5 + k*.8, 3.2, .82])], .5)).join('') : '' });
  for (const y of [-3.4, 3.4]) for (let k = 0; k < 2; k++) qCyl(TB, 'z', [-5 + k*5, y*.85], -.4, .4, .3, STEEL, { w: .4, n: 8 });
  sbPart('sbc'); qBox(TB, [2.4, -3, .4], [8.4, 3, .8], PCB, { w: .7 }); qBox(TB, [4.2, -1.2, .8], [6.4, 1.2, 1.3], DARK, { w: .5 }); qBox(TB, [7.2, -2.6, .8], [8.4, -.4, 1.6], STEEL, { w: .5 });
  sbPart('imu'); qBox(TB, [-8.2, -1, .4], [-6.8, 1, .7], PCB, { w: .5 });
  sbPart('leads'); HX_LEGS.forEach((L, i) => { const a = L.a*DEG, s = [L.m[0] - 2.2*Math.cos(a), L.m[1] - 2.2*Math.sin(a)];
    qWire([[s[0], s[1], 1.2], [s[0]*.6, s[1]*.55, 1.4], [-5.2 + i*.75, 3.4*Math.sign(L.m[1]), 1]].map(p => tAp(TB, p)), ['#e2483d', '#f4b829', '#3a3633'][i % 3], .22, { seg: 3 }); });
  sbPart('coxaS'); HX_LEGS.forEach(L => { const Tm = tCat(TB, tTr(L.m[0], L.m[1], 0), tRz(L.a*DEG)); qBox(Tm, [-3.4, -1, -1.8], [.8, 1, 1.6], SERVO, { w: .8, deco: n => n[2] === 1 ? dotAt(tAp(Tm, [-2.4, 0, 1.62]), .3, '#f2efe7', .25) : '' }); });
  sbPart('top'); qExt(TB, HX_OCT, 1.8, 2.4, m, { ax: 'z', cover: true, w: 1.8, capDeco: b => b ? txT(TB, [-3, 0, 2.42], [0,-1,0], [1,0,0], 'HEX-6', 2.6) : '' });
  // ---- head: ultrasonic eyes
  sbFree(); sbPart('head'); qBox(TB, [12, -3, -1.2], [14.6, 3, 1.4], DARK, { w: 1 });
  for (const y of [1.6, -1.6]) qCyl(TB, 'x', [y, .1], 14.6, 15.4, 1, STEEL, { w: .7, deco: () => vis(tAx(TB, 0)) ? `<path d="${pd(on(PX(0), ellUV(0, 0, 1, 1, 12)).map(p => tAp(TB, [15.42, y + p[1]*.7, .1 + p[2]*.7])))}" fill="#3a3633"/>` : '' });
  // ---- legs
  HX_LEGS.forEach((L, i) => { const [g, al, be] = st.ang[i], bad = st.legErr.includes(i);
    const Tc = tCat(TB, tTr(L.m[0], L.m[1], 0), tRz((L.a + g)*DEG)), Tf = tCat(Tc, tTr(HX.COXA, 0, 0), tRy(-al*DEG)), Tk = tMul(Tf, tTr(HX.FEM, 0, 0)), Tt = tMul(Tk, tRy(be*DEG));
    sbPart('cbr'); qCyl(Tc, 'z', [0, 0], -2.8, -2.4, 1.1, HORN, { w: .6 }); qBox(Tc, [-1, -1.4, -3.5], [HX.COXA + 1.3, 1.4, -2.8], ALU, { w: .7 });
    sbPart('femurS'); qBox(Tc, [HX.COXA - 1.2, -1.5, -2.8], [HX.COXA + 1.2, 1.3, 1.3], SERVO, { w: .8 }); qCyl(Tc, 'y', [HX.COXA, 0], 1.3, 1.6, 1.1, HORN, { w: .6 });
    sbPart('femur'); for (const [y0, y1] of [[1.6, 2.05], [-2.05, -1.6]]) qBox(Tf, [-1, y0, -.9], [HX.FEM + .8, y1, .9], ALU, { w: .7 });
    sbPart('tibiaS'); qBox(Tk, [-1.2, -1.5, -1.3], [1.2, 1.3, 2.8], SERVO, { w: .8 }); qCyl(Tk, 'y', [0, 0], 1.3, 1.6, 1.1, HORN, { w: .6 });
    sbPart('tibia'); qRod(tAp(Tt, [.2, 0, 0]), tAp(Tt, [HX.TIB - 1.2, 0, 0]), 1.2, bad ? matOf('#e2483d') : m, { r1: .55, sh: tone, w: 1.1 });
    qBox(Tt, [-.9, 1.6, -1.2], [2.6, 2.0, 1.2], ALU, { w: .6 });
    sbPart('foot'); qBall(tAp(Tt, [HX.TIB - .7, 0, 0]), .75, RUBBER); });
  // ---- floor: footprints, support polygon, centre of mass; world props
  const rim = on(PZ(0), ellUV(HX.C, HX.C, HX.R0, HX.R0, 64)); let floor = '';
  if (st.mode === 'op'){
    const fp = st.prints.map(([x, y]) => { const [sx, sy] = hxScene(st, [x, y]); return `<path d="${pd(on(PZ(.02), ellUV(sx, sy, 1, 1, 10)))}" fill="#c9bfa9"/>`; }).join('');
    const sup = hxSupport(st); let poly = '';
    if (sup.poly.length >= 3) poly = `<path d="${pd(sup.poly.map(p => [...hxScene(st, p), .05]))}" fill="${sup.margin > 0 ? '#43c25c' : '#e2483d'}" fill-opacity=".22" stroke="${sup.margin > 0 ? '#2f8a4a' : '#e2483d'}" stroke-width=".9" stroke-dasharray="1.6 1"/>`;
    floor += clipTo(rim, fp + poly); }
  let over = '';
  if (st.mode === 'op'){ const sup = hxSupport(st), cm = hxScene(st, bw), q = P(cm[0], cm[1], .05), q2 = P(cm[0], cm[1], st.zb - 2.4), r = 1.6*INK_K*2, c = sup.margin > 0 ? '#231e16' : '#e2483d';
    over += `<g pointer-events="none"><path d="M${f(q[0])} ${f(q[1])}L${f(q2[0])} ${f(q2[1])}" stroke="${c}" stroke-width=".7" stroke-dasharray="1 1"/><circle cx="${f(q[0])}" cy="${f(q[1])}" r="${f(r)}" fill="#fff" stroke="${c}" stroke-width="1.2"/><circle cx="${f(q[0])}" cy="${f(q[1])}" r="${f(r*.45)}" fill="${c}"/></g>`; }
  for (const [x, y, r, c] of HX_ROCKS){ const [sx, sy] = hxScene(st, [x, y]); if (Math.hypot(sx - HX.C, sy - HX.C) > HX.R0 - r - .5) continue;
    sbFree(); qRod([sx, sy, 0], [sx, sy, r*.7], r, matOf(c), { w: 1, r1: r*.6, sh: tone, p: null }); }
  return { under: floorDisc(HX.C, HX.C, HX.R0, st.pos[0], st.pos[1]), z0: 0, clip: rim, floor, over: over + (st.crash ? crashBurst(st.crash.at, st.crash.t) : '') };
}
const HX_ROCKS = [[45, 20, 3, '#bfb390'], [-30, -25, 4, '#a9b1b5'], [20, -45, 2.6, '#bfb390'], [-50, 30, 3.4, '#a9b1b5'], [70, -10, 3, '#bfb390']];

function hxNeutralW(st, i, pos = st.pos, th = st.th){ const n = hxRot(th, HX_LEGS[i].N); return [pos[0] + n[0], pos[1] + n[1], 0]; }
function hxTick(st, dt, ui, lab){
  const G = HX_GAITS[st.gait], j = st.joy || [0, 0], cmd = st.lift < 0 && Math.hypot(j[0], j[1]) > .05;
  if (st.lift >= 0 && Math.hypot(j[0], j[1]) > .05) lab.flash('Put the lifted leg down first (All feet down)');
  const vx = cmd ? j[1]*HX.VMAX*st.vscale : 0, w = cmd ? -j[0]*HX.WMAX : 0;
  st.v = vx; st.w = w;
  const far = st.feet.some((p, i) => p[2] > .05 || Math.hypot(p[0] - hxNeutralW(st, i)[0], p[1] - hxNeutralW(st, i)[1]) > .8);
  if (st.lift >= 0){ st.walking = false;
    st.feet.forEach((p, i) => { if (i !== st.lift && p[2] > 0){ p[2] = Math.max(0, p[2] - dt*12); } });
    const L = HX_LEGS[st.lift], lr = hxRot(st.th, [L.m[0] + (L.N[0] - L.m[0])*.78, L.m[1] + (L.N[1] - L.m[1])*.78]), p = st.feet[st.lift], tgt = [st.pos[0] + lr[0], st.pos[1] + lr[1], 4.2]; for (let k = 0; k < 3; k++) p[k] += (tgt[k] - p[k])*Math.min(1, dt*6);
    st.settling = vLen(sub(tgt, p)) > .02 || st.feet.some((q, i) => i !== st.lift && q[2] > 0); }
  else if (cmd || far){ if (!st.walking){ st.walking = true; st.phase = 0; st.swing = st.feet.map(() => null); }
    const freq = G.freq*st.rate, Ts = G.duty/freq, Tsw = (1 - G.duty)/freq;
    st.phase = (st.phase + dt*freq) % 1;
    const c = Math.cos(st.th), s = Math.sin(st.th); st.pos = [st.pos[0] + (c*vx)*dt, st.pos[1] + (s*vx)*dt]; st.th += w*dt;
    HX_LEGS.forEach((L, i) => { const p = (st.phase + G.off[i]) % 1, stance = p < G.duty;
      if (stance){ if (st.swing[i]){ st.feet[i] = st.swing[i].to.slice(); st.feet[i][2] = 0; st.prints.push(st.feet[i].slice(0, 2)); if (st.prints.length > 60) st.prints.shift(); st.swing[i] = null; } }
      else { if (!st.swing[i]) st.swing[i] = { from: st.feet[i].slice(), to: null };
        const u = (p - G.duty)/(1 - G.duty), rem = (1 - u)*Tsw, th2 = st.th + w*rem, pos2 = [st.pos[0] + Math.cos(st.th)*vx*rem, st.pos[1] + Math.sin(st.th)*vx*rem];
        const vf = [vx - w*L.N[1], w*L.N[0]], tl = [L.N[0] + vf[0]*Ts/2, L.N[1] + vf[1]*Ts/2], r = hxRot(th2, tl), to = [pos2[0] + r[0], pos2[1] + r[1], 0];
        st.swing[i].to = to; const e = ease(u), fr = st.swing[i].from;
        st.feet[i] = [fr[0] + (to[0] - fr[0])*e, fr[1] + (to[1] - fr[1])*e, HX.LIFT*Math.sin(Math.PI*u)]; } }); }
  else { st.walking = false; st.feet.forEach(p => { p[2] = 0; }); }
  hxSolve(st);
  if (st.legErr.length){ const n = st.legErr.map(i => HX_LEGS[i].n).join(', ');
    lab.flash(`Leg ${n} cannot reach: ${hxIK(HX_LEGS[st.legErr[0]], hxFootBody(st, st.legErr[0])).err || 'limit'} — slow down or change body height`); }
  const sup = hxSupport(st); st.margin = sup.margin; st.nDown = sup.down.length;
  if (sup.margin < 0 && !st.tipWarn){ st.tipWarn = true; st.crashes++; lab.flash('Tipping! The centre of mass left the support polygon'); }
  if (sup.margin >= 0) st.tipWarn = false;
  if (st.crash){ st.crash.t -= dt/1.2; if (st.crash.t <= 0) st.crash = null; }
  return st.lift >= 0 ? !!st.settling || !!st.crash : st.walking || cmd || far || !!st.crash;
}
function hxInit(){ const st = { pos: [0, 0], th: 0, zb: 8, lean: [0, 0], gait: 'tripod', rate: 1, vscale: 1, joy: [0, 0], phase: 0, walking: false, swing: [], prints: [], lift: -1,
  ang: HX_LEGS.map(() => [0, 30, 90]), legErr: [], margin: 1, nDown: 6, v: 0, w: 0, crashes: 0, crash: null, tipWarn: false };
  st.feet = HX_LEGS.map((_, i) => hxNeutralW(st, i)); hxSolve(st); return st; }
function hxGaitSVG(st){ const G = HX_GAITS[st.gait], W = 200, H = 96, rowH = 13, x0 = 24, w = W - x0 - 4; let s = '';
  HX_LEGS.forEach((L, i) => { const y = 6 + i*rowH, off = G.off[i];
    s += `<text x="2" y="${y + 9}" font-size="8.5" font-weight="800" font-family="ui-monospace,monospace" fill="#231e16">${L.n}</text><rect x="${x0}" y="${y}" width="${w}" height="10" fill="#fff" stroke="#231e16" stroke-width=".8"/>`;
    // stance where (t + off) % 1 < duty, for t in 0..1
    for (let k = 0; k < 2; k++){ const a = (k - off), s0 = Math.max(0, a), s1 = Math.min(1, a + G.duty); if (s1 > s0) s += `<rect x="${f(x0 + s0*w)}" y="${y}" width="${f((s1 - s0)*w)}" height="10" fill="#231e16"/>`; }
    if (st.lift === i) s += `<rect x="${x0}" y="${y}" width="${w}" height="10" fill="#e2483d" opacity=".6"/>`; });
  const cx = x0 + st.phase*w; s += `<path d="M${f(cx)} 2V${6 + 6*rowH}" stroke="#e2483d" stroke-width="1.6"/>`;
  s += `<text x="${x0}" y="${H - 2}" font-size="7.5" font-family="ui-monospace,monospace" fill="#857d6e">dark = stance, white = swing</text>`;
  return s; }

ROBOTS.hexapod = {
  id: 'hexapod', fig: '04', name: 'Hexapod walker', pal: 'tomato', kicker: '18 servos · 3-DOF legs · tripod / ripple / wave gaits',
  blurb: 'Six legs of three servos each. Each foot is placed by its own little inverse-kinematics solver while a gait decides which feet are on the ground. Walk it, change gait, lean the body and lift a leg to see when it would tip.',
  size: [80, 80, 22], center: [40, 40], bounds: [24, 56, 24, 56, 0, 16],
  init: hxInit, thumbYaw: -10,
  steps: [
    { title: 'Bottom plate & standoffs', parts: ['plates'], text: 'The lower half of the body sandwich. Six standoffs set the gap for the coxa servos. It rests on an assembly stand until the legs are on.' },
    { title: 'Battery & regulator', parts: ['battery', 'ubec'], text: 'A 2-cell LiPo (7.4 V) sits low in the middle. 18 servos can pull 10+ A together, so a switching regulator (UBEC) supplies a steady 6 V to them.' },
    { title: 'Servo controller, computer, IMU', parts: ['ctrlb', 'sbc', 'imu'], text: 'A 32-channel servo board makes 18 PWM signals (a 1–2 ms pulse 50 times a second sets each angle). The small computer runs the gait and the leg IK; the IMU senses body tilt.' },
    { title: 'Coxa (hip) servos', parts: ['coxaS'], text: 'Six servos with vertical shafts, clamped between the plates. They swing the legs forward and back.' },
    { title: 'Coxa brackets & femur servos', parts: ['cbr', 'femurS'], text: 'A U-bracket on each coxa horn carries the femur servo with its shaft horizontal: this lifts and lowers the leg.' },
    { title: 'Femurs & knee servos', parts: ['femur', 'tibiaS'], text: 'Two aluminium side plates form each femur (75 mm) and hold the knee servo at their end.' },
    { title: 'Tibias & feet', parts: ['tibia', 'foot'], text: 'The 110 mm tibias hang from the knee horns, each ending in a rubber foot. Now the robot can stand: the stand comes away.' },
    { title: 'Servo leads', parts: ['leads'], peek: true, text: 'Every servo has a 3-wire lead (power, ground, signal) back to the controller: 18 leads, bundled along the plates.' },
    { title: 'Top plate', parts: ['top'], text: 'The top plate closes the sandwich and stiffens the body.' },
    { title: 'Sensor head', parts: ['head'], text: 'An ultrasonic rangefinder: one transducer chirps, the other listens, and the echo time gives the distance ahead.' },
  ],
  parts: {
    plates: { name: 'Body plates & standoffs', kind: 'structure', lbl: 'BOTTOM PLATE', key: true, short: '2 mm aluminium', info: 'The body is a sandwich: the coxa servos are clamped between two plates, which makes a stiff box from flat parts.', held: 'The six legs' },
    battery: { name: 'LiPo battery', kind: 'power', lbl: 'BATTERY', inside: true, short: '2S 5000 mAh', info: 'The heaviest single part, placed at the centre so the centre of mass stays in the middle of the support polygon.', held: 'Strap on the bottom plate', spec: '7.4 V, 37 Wh' },
    ubec: { name: 'UBEC regulator', kind: 'power', lbl: 'UBEC 6 V', inside: true, short: 'servo power supply', info: 'A switching regulator: steps 7.4 V down to 6 V for the servos at up to 15 A, without wasting the difference as heat.', held: 'The bottom plate' },
    ctrlb: { name: '32-channel servo controller', kind: 'electronics', lbl: 'SERVO CONTROLLER', inside: true, short: 'makes 18 PWM signals', info: 'A hobby servo reads a pulse every 20 ms: 1.0 ms = one end, 1.5 ms = centre, 2.0 ms = the other end. This board generates all 18 pulses at once.', held: 'Standoffs over the battery' },
    sbc: { name: 'Single-board computer', kind: 'electronics', lbl: 'COMPUTER', inside: true, short: 'gait + IK, 50 Hz', info: 'Fifty times a second: advance the gait phase, decide each foot’s target, solve 6 leg IKs (18 angles), and send them to the servo controller.', held: 'Standoffs on the bottom plate' },
    imu: { name: 'IMU', kind: 'sensor', lbl: 'IMU', inside: true, short: 'tilt sensor', info: 'Measures body roll and pitch; advanced gaits use it to level the body on slopes.', held: 'The bottom plate' },
    coxaS: { name: 'Coxa servos (x6)', kind: 'motor', lbl: 'COXA SERVO', inside: true, short: 'hip yaw, ±55°', info: 'A hobby servo: DC motor, gear train, potentiometer and a control board in one case. It drives toward whatever angle the pulse asks for.', held: 'Clamped between the plates', spec: '11 kg cm (1.1 N m) at 6 V' },
    cbr: { name: 'Coxa brackets', kind: 'structure', short: 'U-brackets', info: 'Bolted to the coxa servo horns; they carry the femur servos.', held: 'The coxa servo horns' },
    femurS: { name: 'Femur servos (x6)', kind: 'motor', lbl: 'FEMUR SERVO', short: 'hip pitch: lifts the leg', info: 'The hardest-working servos: in stance they hold up the body. See the femur load bar in Operate mode.', held: 'The coxa brackets', spec: '1.1 N m' },
    femur: { name: 'Femurs', kind: 'structure', lbl: 'FEMUR', short: '75 mm side plates', info: 'Upper leg: femur length and tibia length are the two sides of each leg’s IK triangle.', held: 'Femur servo horns' },
    tibiaS: { name: 'Knee servos (x6)', kind: 'motor', lbl: 'KNEE SERVO', short: 'bends the tibia', info: 'Sets the knee angle; with the femur servo it sets the foot’s reach and height.', held: 'The ends of the femurs' },
    tibia: { name: 'Tibias', kind: 'structure', lbl: 'TIBIA', key: true, short: '110 mm', info: 'Lower legs. A leg turns red when its foot target is out of reach or past a servo limit.', held: 'The knee servo horns' },
    foot: { name: 'Rubber feet', kind: 'structure', short: 'grip pads', info: 'Point contacts with high friction, so stance feet do not slip while the body moves over them.', held: 'Tibia tips' },
    leads: { name: 'Servo leads', kind: 'wiring', lbl: 'SERVO LEADS', inside: true, short: '18 x 3-wire', info: 'Brown = ground, red = 6 V, yellow/orange = signal.', held: 'Ties along the plates' },
    top: { name: 'Top plate', kind: 'cover', lbl: 'TOP PLATE', key: true, short: 'closes the body', info: 'Clamps the coxa servos from above. See-through fades it to show the electronics.', held: 'The standoffs' },
    head: { name: 'Ultrasonic head', kind: 'sensor', lbl: 'SONAR', key: true, short: 'distance ahead', info: 'Sound at 40 kHz; distance = speed of sound x echo time / 2. Wide beam, a few metres range.', held: 'The front of the body' },
  },
  concepts: [
    ['Leg inverse kinematics', 'Each leg is a 3-joint arm. The coxa angle points it at the foot target; the femur and knee then solve a two-link triangle in that vertical plane. 6 legs x 3 = 18 angles, recomputed 50 times a second.'],
    ['Gaits', 'A gait is a timing pattern: which feet are on the ground (stance) and which are moving forward (swing). The chart shows it; the red cursor is now. More feet down = slower, stronger and more stable.'],
    ['Support polygon & static stability', 'The shape joining the feet on the ground is the support polygon (green). If the centre of mass (the dot) stays inside it, the robot cannot fall even if it froze mid-step. Use Lean and Lift a leg to push it out and see the warning.'],
    ['Stance and swing', 'Stance feet are planted in the world while the body moves over them, so in body coordinates they slide backwards. A swing foot lifts, arcs forward and lands ahead, at a spot predicted for where the body will be.'],
    ['Servo load', 'Load on a femur servo = body weight / feet down x horizontal distance to the foot. Wave gait spreads the weight over five legs; tripod over three, so each femur works almost twice as hard.'],
  ],
  draw: hxDraw,
  active: st => { const a = {}; if (st.walking || st.lift >= 0) a.coxaS = a.femurS = a.tibiaS = a.ctrlb = a.sbc = true; return a; },
  opHTML: () => `<div class="seg" id="hxgait">${Object.entries(HX_GAITS).map(([k, g]) => `<button data-g="${k}">${g.label}</button>`).join('')}</div>
    <p class="note" id="hxnote"></p>
    <div class="joyw"><div class="joy" id="hxjoy"><i></i></div><small>Drag the knob (or W A S D).<br>Up: walk forward.<br>Sideways: turn on the spot.</small></div>
    <svg class="gaitc" id="hxgc" viewBox="0 0 200 96"></svg>
    ${slHTML('hxz', 'Body height', 5, 12, .1, ' cm', 'Height of the femur axes above the floor')}
    ${slHTML('hxrate', 'Step rate', 40, 160, 5, '%')}
    ${slHTML('hxlx', 'Lean forward / back', -7, 7, .1, ' cm')}
    ${slHTML('hxly', 'Lean left / right', -7, 7, .1, ' cm')}
    <h3 style="margin-top:6px">Lift a leg</h3><div class="row2" id="hxlift">${HX_LEGS.map((L, i) => `<button data-l="${i}">${L.n}</button>`).join('')}<button data-l="-1">All feet down</button></div>`,
  opBind(el, st, ui, lab){ const $ = id => document.getElementById(id);
    joyBind($('hxjoy'), st, lab);
    el.querySelectorAll('#hxgait button').forEach(b => b.onclick = () => { st.gait = b.dataset.g; st.walking = false; lab.kick(); ui.render(); });
    $('s_hxz').oninput = e => { st.zb = +e.target.value; hxSolve(st); lab.kick(); ui.render(); };
    $('s_hxrate').oninput = e => { st.rate = +e.target.value / 100; ui.render(); };
    $('s_hxlx').oninput = e => { st.lean[0] = +e.target.value; hxSolve(st); lab.kick(); ui.render(); };
    $('s_hxly').oninput = e => { st.lean[1] = +e.target.value; hxSolve(st); lab.kick(); ui.render(); };
    el.querySelectorAll('#hxlift button').forEach(b => b.onclick = () => { const l = +b.dataset.l; st.lift = st.lift === l ? -1 : l; lab.kick(); ui.render(); }); },
  opSync(el, st){ const $ = id => document.getElementById(id);
    el.querySelectorAll('#hxgait button').forEach(b => b.setAttribute('aria-pressed', b.dataset.g === st.gait));
    $('hxnote').textContent = HX_GAITS[st.gait].note;
    slSync('hxz', st.zb, st.zb.toFixed(1) + ' cm'); slSync('hxrate', Math.round(st.rate*100), Math.round(st.rate*100) + '%');
    slSync('hxlx', st.lean[0], st.lean[0].toFixed(1) + ' cm', Math.abs(st.lean[0]) > 5); slSync('hxly', st.lean[1], st.lean[1].toFixed(1) + ' cm', Math.abs(st.lean[1]) > 5);
    el.querySelectorAll('#hxlift button').forEach(b => b.setAttribute('aria-pressed', +b.dataset.l === st.lift && st.lift >= 0));
    $('hxgc').innerHTML = hxGaitSVG(st); },
  readout(st){ const rows = [], down = Math.max(1, st.nDown);
    rows.push({ k: 'Speed', bar: Math.abs(st.v)/HX.VMAX, v: `${st.v.toFixed(1)} cm/s` });
    rows.push({ k: 'Turn rate', bar: Math.abs(st.w)/HX.WMAX, v: `${(st.w/DEG).toFixed(0)}°/s` });
    rows.push({ k: 'Feet on ground', bar: st.nDown/6, v: String(st.nDown), warn: st.nDown < 3 });
    rows.push({ k: 'Stability margin', bar: clamp(st.margin, 0, 10)/10, v: st.margin < 0 ? 'TIPPING' : `${(st.margin*10).toFixed(0)} mm`, warn: st.margin < 1.5, tip: 'Distance from the centre of mass to the nearest edge of the support polygon' });
    HX_LEGS.forEach((L, i) => { const p = st.feet[i], fb = hxFootBody(st, i), r = Math.hypot(fb[0] - L.m[0], fb[1] - L.m[1]) - HX.COXA, load = p[2] < .3 ? HX.MASS*9.81/down*r/100 : 0;
      rows.push({ k: `${L.n} ${st.ang[i].map(a => a.toFixed(0)).join(' / ')}°`, bar: load/1.1, v: p[2] < .3 ? `${load.toFixed(2)} N m` : 'swing', warn: st.legErr.includes(i) || load > 1.1, tip: 'Coxa / femur / knee angles, and femur servo load (rated 1.1 N m)' }); });
    rows.push({ k: 'Tip warnings', v: String(st.crashes), warn: st.margin < 0 });
    return rows; },
  stats: st => [{ label: 'Gait', value: HX_GAITS[st.gait].label, small: true }],
  tick: hxTick,
  enterOp: st => { hxSolve(st); },
  params(st, q){ if (HX_GAITS[q.get('gait')]) st.gait = q.get('gait'); if (q.has('lift')) st.lift = +q.get('lift'); if (q.has('lean')) st.lean = q.get('lean').split(',').map(Number); if (q.has('zb')) st.zb = +q.get('zb');
    if (q.has('lift')){ const i = st.lift, L = HX_LEGS[i], lr = [L.m[0] + (L.N[0] - L.m[0])*.78, L.m[1] + (L.N[1] - L.m[1])*.78]; st.feet[i] = [lr[0], lr[1], 4.2]; } hxSolve(st); },
};
ROBOT_ORDER.push({ id: 'hexapod', name: 'Hexapod walker', blurb: 'Eighteen hobby servos, six 3-DOF legs and a gait engine. Switch tripod, ripple and wave gaits, lean the body, lift a leg and watch the support polygon decide whether it tips.', chips: ['18 servos', 'Leg IK', 'Gaits', 'Stability'] });
