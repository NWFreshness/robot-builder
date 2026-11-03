/* ============================================================
   Robotics Lab shell: mounts one robot (assembly + operation + panel), or the hub when no ?robot= is given.
   ============================================================ */
const ROBOTS = {};            // id -> robot definition (filled by the robot files)
const ROBOT_ORDER = [];       // hub order: { id, built:bool, ...meta }
const KIND_NAMES = { structure:'Structure', motor:'Actuator', gear:'Transmission', sensor:'Sensor', electronics:'Electronics', wiring:'Wiring', cover:'Cover', tool:'End effector', power:'Power' };

const LAB_CSS = `
  header, .controls, .made-with{max-width:1200px}
  header{align-items:center} header .back{color:var(--ink);text-decoration:none;border:1.5px solid var(--ink);border-radius:999px;padding:3px 12px;margin-right:10px;font-size:12px}
  header .back:hover{background:#0000000d}
  .labwrap{width:100%;max-width:1200px;display:grid;grid-template-columns:minmax(0,1fr) 370px;gap:22px;align-items:start}
  .labwrap .stage{max-width:none} #fig{max-width:none;max-height:80vh!important}
  @media (max-width:1020px){.labwrap{grid-template-columns:1fr}}
  .lab{display:flex;flex-direction:column;gap:12px;padding-top:4px}
  .lab h3{margin:0 0 6px;font-size:11px;letter-spacing:.12em;text-transform:uppercase;color:var(--muted)}
  .lab .card{border:1.5px solid var(--ink);border-radius:12px;background:#fbf7ea;box-shadow:3px 3px 0 var(--ink);padding:11px 13px}
  .lab p{margin:4px 0;font-size:12.2px;line-height:1.5}
  .tabs{display:grid;grid-template-columns:1fr 1fr;gap:8px}
  .tabs button{border-radius:10px;padding:8px 6px;font-weight:700;letter-spacing:.04em;background:#fbf7ea;box-shadow:2px 2px 0 var(--ink)}
  .tabs button[aria-pressed=true]{background:var(--ink);color:var(--paper)}
  .tabs button:disabled{opacity:.45}
  .stepno{font-size:11px;letter-spacing:.12em;color:var(--muted);text-transform:uppercase}
  .steptitle{font-size:15px;font-weight:800;margin:2px 0 4px}
  .prog{display:flex;gap:3px;margin:8px 0 6px}
  .prog i{flex:1;height:9px;border:1.2px solid var(--ink);border-radius:3px;background:#fff;cursor:pointer}
  .prog i.done{background:var(--accent,#f4b829)} .prog i.cur{background:var(--ink)}
  .chipr{display:flex;flex-wrap:wrap;gap:5px;margin-top:6px}
  .pchip{font:inherit;font-size:11px;font-weight:700;border:1.3px solid var(--ink);border-radius:7px;padding:2px 7px;background:#fff;cursor:pointer;box-shadow:1.5px 1.5px 0 var(--ink)}
  .pchip.sel{background:var(--ink);color:var(--paper)} .pchip.todo{opacity:.45}
  .kind{display:inline-block;font-size:10px;font-weight:800;letter-spacing:.1em;text-transform:uppercase;border:1.2px solid var(--ink);border-radius:999px;padding:1px 8px;background:var(--accent,#f4b829)}
  .pname{font-size:15px;font-weight:800;margin:6px 0 2px}
  dl.spec{display:grid;grid-template-columns:auto 1fr;gap:3px 10px;margin:8px 0 0;font-size:11.5px} dl.spec dt{color:var(--muted);text-transform:uppercase;letter-spacing:.06em;font-size:10.5px;padding-top:1px} dl.spec dd{margin:0}
  details{border-top:1px solid var(--rule);padding:6px 0} details:first-of-type{border-top:0}
  summary{cursor:pointer;font-weight:700;font-size:12.5px} details p{color:#3b352b}
  .bom{max-height:230px;overflow:auto;margin:0;padding:0;list-style:none;font-size:11.5px}
  .bom li{display:flex;gap:8px;align-items:baseline;padding:2px 0;cursor:pointer} .bom li:hover{text-decoration:underline}
  .bom li b{min-width:22px;color:var(--muted)} .bom li.ok::before{content:"\\2713";font-weight:900} .bom li:not(.ok)::before{content:"\\00b7";opacity:.5}
  .sl{margin:6px 0 9px} .slh{display:flex;justify-content:space-between;font-size:12px;font-weight:700} .slh b{font-variant-numeric:tabular-nums}
  .sl.lim .slh b{color:#e2483d} .sl input{margin:3px 0 0}
  .sl .rng{display:flex;justify-content:space-between;font-size:10px;color:var(--muted)}
  table.ro{width:100%;border-collapse:collapse;font-size:11.5px;font-variant-numeric:tabular-nums;margin-top:4px}
  table.ro td{padding:2px 4px;border-top:1px solid var(--rule)} table.ro td:first-child{color:var(--muted);white-space:nowrap} table.ro td.v{text-align:right;font-weight:700;white-space:nowrap}
  table.ro .bar{height:7px;border:1px solid var(--ink);border-radius:3px;background:#fff;position:relative;min-width:60px}
  table.ro .bar i{position:absolute;left:0;top:0;bottom:0;background:#43c25c;border-radius:2px}
  table.ro tr.warn td.v{color:#e2483d} table.ro tr.warn .bar i{background:#e2483d}
  .row2{display:flex;gap:6px;flex-wrap:wrap;margin:6px 0}
  .row2 button{font-size:11.5px;padding:4px 10px}
  .joyw{display:flex;gap:12px;align-items:center;margin:6px 0}
  .joy{width:118px;height:118px;border:1.5px solid var(--ink);border-radius:50%;background:radial-gradient(circle,#fff 0 30%,#f2ecdc 31% 100%);position:relative;touch-action:none;flex:none;box-shadow:2px 2px 0 var(--ink);cursor:grab}
  .joy::before,.joy::after{content:"";position:absolute;background:var(--rule)} .joy::before{left:50%;top:8px;bottom:8px;width:1px} .joy::after{top:50%;left:8px;right:8px;height:1px}
  .joy i{position:absolute;width:40px;height:40px;margin:-20px 0 0 -20px;left:50%;top:50%;border-radius:50%;background:var(--accent,#f4b829);border:1.5px solid var(--ink);box-shadow:0 2px 0 var(--ink);z-index:1}
  .joyw small{font-size:11px;color:var(--muted);line-height:1.45}
  .seg{display:flex;border:1.5px solid var(--ink);border-radius:9px;overflow:hidden;margin:4px 0 8px}
  .seg button{flex:1;border:0;border-radius:0;padding:5px 4px;font-size:11.5px} .seg button+button{border-left:1.5px solid var(--ink)}
  .ptip{position:absolute;z-index:5;pointer-events:none;background:#fff4d6;border:1.3px solid var(--ink);border-radius:8px;padding:3px 8px;font-size:11px;font-weight:700;box-shadow:2px 2px 0 var(--ink);opacity:0;transition:opacity .12s;max-width:240px}
  .ptip.show{opacity:1} .ptip span{display:block;font-weight:400;color:#4a4337}
  #fig .hl path[stroke="#231e16"]{stroke:#e2483d} #fig .on path[stroke="#231e16"]{stroke:#c76a12}
  .gaitc{width:100%;height:auto;display:block;margin:4px 0}
  .note{font-size:11px;color:var(--muted)}`;

function mountLab(R){
  R._step = {}; R.steps.forEach((s, i) => s.parts.forEach(p => R._step[p] = i));
  const N = R.steps.length;
  const st = Object.assign({ mode:'build', built:0, pt:1, sel:null }, R.init());
  const q0 = new URLSearchParams(location.search);
  let lastFlash = 0, lastMsg = '';
  const lab = {
    kick(){ if (st.loop) return; st.loop = true; ui.animate(dt => { const more = st.mode === "op" && !!R.tick(st, dt, ui, lab); if (!more) st.loop = false; return more; }); },
    flash(msg, ms = 1700){ const t = performance.now(); if (msg === lastMsg && t - lastFlash < 1200) return; lastMsg = msg; lastFlash = t; ui.flash(msg, ms); },
  };
  const place = () => { st.built++; st.pt = 0; const step = R.steps[st.built - 1]; st.sel = step.parts[0];
    if (step.peek && !st.xrT) setXR(1);
    ui.animate(dt => { st.pt = Math.min(1, st.pt + dt/.75); return st.pt < 1; }); };
  const setXR = on => { st.xrT = on; const b = document.getElementById('xrb'); if (b) b.setAttribute('aria-pressed', !!on);
    ui.animate(dt => { const g = Math.sign(st.xrT - st.xr); st.xr = clamp(st.xr + g*dt/.7, 0, 1); return st.xr !== st.xrT; }); };
  const toOp = () => { if (st.built < N){ st.built = N; st.pt = 1; lab.flash('Finished the build for you — now drive it'); }
    st.mode = 'op'; st.sel = null; if (R.enterOp) R.enterOp(st); lab.kick(); };
  const ui = mountFigure({
    fig: R.fig, title: R.name, size: R.size, center: R.center, palette: R.pal, turn: 'full', credit: false, state: st,
    scene: (st, m) => labScene(st, m, R),
    buttons: [
      { label: 'Next part', onClick: (st) => { if (st.mode === 'op') return lab.flash('Already built — press Rebuild to start over');
          if (st.built >= N) return lab.flash('Every part is on — press Operate'); place(); } },
      { label: 'Back', onClick: (st) => { if (st.mode === 'op'){ st.mode = 'build'; return; } if (!st.built) return lab.flash('Nothing placed yet');
          st.built--; st.pt = 1; st.sel = st.built ? R.steps[st.built - 1].parts[0] : null; } },
      { label: 'Build all', onClick: (st) => { if (st.built >= N) return lab.flash('Already fully built');
          const start = st.built; let t = 0; st.mode = 'build';
          ui.animate(dt => { t += dt; const want = Math.min(N, start + 1 + Math.floor(t / .22)); if (want > st.built){ st.built = want; st.pt = 0; }
            st.pt = Math.min(1, st.pt + dt/.3); return st.built < N || st.pt < 1; }); } },
      { label: 'Operate', onClick: () => { if (st.mode === 'op') return lab.flash('Already operating — use the controls on the right'); toOp(); } },
      { label: 'Rebuild', onClick: (st) => { st.built = 0; st.pt = 1; st.mode = 'build'; st.sel = null; Object.assign(st, R.init()); } },
    ],
    stats: st => st.mode === 'op' && R.stats ? R.stats(st) : [{ label: 'Parts placed', value: `${R.steps.slice(0, st.built).reduce((a, s) => a + s.parts.length, 0)}/${Object.keys(R._step).length}`, small: true }],
    params: (st, q) => {
      if (q.get('built') === 'all') st.built = N; else if (q.has('built')) st.built = clamp(+q.get('built') || 0, 0, N);
      if (q.get('mode') === 'op'){ st.built = N; st.mode = 'op'; }
      if (q.has('sel')) st.sel = q.get('sel');
      if (q.has('joy')) st.joy = q.get('joy').split(',').map(Number);
      if (R.params) R.params(st, q); },
  });
  xrMount(() => ui, () => '');
  window.__lab = lab;

  // ---- layout: figure left, lab panel right; a link back to the hub
  const css = document.createElement('style'); css.textContent = LAB_CSS; document.head.appendChild(css);
  document.title = R.name + ' — Robotics Lab';
  const hdr = document.querySelector('header');
  hdr.innerHTML = `<h1><a class="back" href="?">&larr; All robots</a>Robotics Lab &middot; ${R.fig} &mdash; ${R.name}</h1><span>${R.kicker}</span>`;
  const stage = document.querySelector('.stage'), wrapEl = document.createElement('div'); wrapEl.className = 'labwrap';
  stage.parentNode.insertBefore(wrapEl, stage); wrapEl.appendChild(stage);
  const panel = document.createElement('aside'); panel.className = 'lab'; wrapEl.appendChild(panel);
  const kinds = p => KIND_NAMES[R.parts[p].kind] || R.parts[p].kind;
  panel.innerHTML = `
    <div class="tabs"><button id="tBuild">1 &middot; Assemble</button><button id="tOp">2 &middot; Operate</button></div>
    <section class="card" id="cBuild"></section>
    <section class="card" id="cOp" hidden><h3>Controls</h3>${R.opHTML()}<h3 style="margin-top:10px">Live feedback</h3><table class="ro" id="ro"></table></section>
    <section class="card" id="cPart"></section>
    <section class="card"><h3>Ideas behind it</h3>${R.concepts.map(([t, b], i) => `<details${i ? '' : ' open'}><summary>${t}</summary><p>${b}</p></details>`).join('')}</section>
    <section class="card"><h3>Parts list (${Object.keys(R._step).length})</h3><ul class="bom" id="bom"></ul></section>`;
  const $ = id => document.getElementById(id);
  $('tBuild').onclick = () => { st.mode = 'build'; ui.render(); };
  $('tOp').onclick = () => { if (st.mode !== 'op'){ toOp(); ui.render(); } };
  panel.addEventListener('click', e => { const c = e.target.closest('[data-pick]'); if (c){ st.sel = c.dataset.pick === st.sel ? null : c.dataset.pick; ui.render(); }
    const s = e.target.closest('[data-step]'); if (s && st.mode === 'build'){ st.built = +s.dataset.step + 1; st.pt = 1; st.sel = R.steps[st.built - 1].parts[0]; ui.render(); } });
  R.opBind(panel, st, ui, lab);

  // ---- picking parts on the figure (a click that did not turn into a drag), hover tooltips, tip drag
  const svg = $('fig'), tip = document.createElement('div'); tip.className = 'ptip'; stage.appendChild(tip);
  let down = null, drag = null;
  const vbPt = e => { const m = svg.getScreenCTM(); if (!m) return [0, 0]; const q = new DOMPoint(e.clientX, e.clientY).matrixTransform(m.inverse()); return [q.x, q.y]; };
  svg.addEventListener('pointerdown', e => { const g = e.target.closest('[data-p]'), fl = e.target.closest('[data-floor]');
    down = { x: e.clientX, y: e.clientY, p: g && g.dataset.p, floor: !!fl, vb: vbPt(e) };
    if (e.target.closest('[data-act="tip"]') && R.onDrag){ drag = { last: vbPt(e) }; st.dragging = true; R.onDrag(st, 0, 0, 'start', lab); lab.kick(); } }, true);
  addEventListener('pointermove', e => { if (!drag) return; const p = vbPt(e); R.onDrag(st, p[0] - drag.last[0], p[1] - drag.last[1], 'move', lab); drag.last = p; lab.kick(); ui.invalidate(); });
  addEventListener('pointerup', e => { if (drag){ drag = null; st.dragging = false; R.onDrag(st, 0, 0, 'end', lab); ui.render(); return; }
    if (!down) return; const moved = Math.hypot(e.clientX - down.x, e.clientY - down.y) > 5, d = down; down = null; if (moved) return;
    if (d.p && R.parts[d.p]){ st.sel = st.sel === d.p ? null : d.p; ui.render(); }
    else if (d.floor && st.mode === 'op' && R.onFloor){ const s = screenToPlane(d.vb[0], d.vb[1], 0); if (s){ R.onFloor(st, s[0], s[1], lab); lab.kick(); ui.render(); } }
    else if (!d.p && st.sel){ st.sel = null; ui.render(); } });
  svg.addEventListener('mousemove', e => { const g = e.target.closest('[data-p]'), p = g && g.dataset.p, d = p && R.parts[p];
    if (!d || drag){ tip.classList.remove('show'); return; }
    const r = stage.getBoundingClientRect(); tip.innerHTML = `${d.name}<span>${kinds(p)} &middot; ${d.short || ''}</span>`;
    tip.style.left = Math.min(r.width - 200, e.clientX - r.left + 14) + 'px'; tip.style.top = (e.clientY - r.top + 12) + 'px'; tip.classList.add('show'); });
  svg.addEventListener('mouseleave', () => tip.classList.remove('show'));

  // ---- panel sync after every render
  let bomSig = '';
  const sync = () => {
    $('tBuild').setAttribute('aria-pressed', st.mode === 'build'); $('tOp').setAttribute('aria-pressed', st.mode === 'op');
    $('cBuild').hidden = st.mode !== 'build'; $('cOp').hidden = st.mode !== 'op';
    if (st.mode === 'build'){
      const i = Math.max(0, st.built - 1), s = R.steps[i], cur = st.built ? s : null;
      const html = `<div class="stepno">${st.built ? `Step ${st.built} of ${N}` : `${N} steps &middot; nothing placed yet`}</div>
        <div class="steptitle">${cur ? cur.title : 'Press Next part to begin'}</div>
        <p>${cur ? cur.text : R.blurb}</p>
        <div class="prog">${R.steps.map((x, j) => `<i data-step="${j}" title="${j + 1}. ${x.title}" class="${j < st.built - 1 ? 'done' : j === st.built - 1 ? 'cur' : ''}"></i>`).join('')}</div>
        ${cur ? `<div class="chipr">${cur.parts.map(p => `<button class="pchip${st.sel === p ? ' sel' : ''}" data-pick="${p}">${R.parts[p].name}</button>`).join('')}</div>` : ''}
        ${st.built < N ? `<p class="note" style="margin-top:8px">Next: ${R.steps[st.built].title}${R.steps[st.built].peek ? ' (goes inside &mdash; See-through turns on)' : ''}</p>` : `<p class="note" style="margin-top:8px">All ${N} steps done. Turn on See-through to inspect the inside, then press Operate.</p>`}`;
      if ($('cBuild')._h !== html){ $('cBuild')._h = html; $('cBuild').innerHTML = html; } }
    else { R.opSync(panel, st);
      const rows = R.readout(st), html = rows.map(r => `<tr${r.warn ? ' class="warn"' : ''}${r.tip ? ` title="${r.tip}"` : ''}><td>${r.k}</td>${r.bar != null ? `<td><div class="bar"><i style="width:${f(clamp(r.bar, 0, 1)*100)}%"></i></div></td>` : '<td></td>'}<td class="v">${r.v}</td></tr>`).join('');
      if ($('ro')._h !== html){ $('ro')._h = html; $('ro').innerHTML = html; } }
    const p = st.sel && R.parts[st.sel];
    const ph = p ? `<span class="kind">${kinds(st.sel)}</span><div class="pname">${p.name}</div><p>${p.info}</p><dl class="spec">${p.spec ? `<dt>Spec</dt><dd>${p.spec}</dd>` : ''}${p.held ? `<dt>Held by</dt><dd>${p.held}</dd>` : ''}<dt>Step</dt><dd>${R._step[st.sel] + 1}. ${R.steps[R._step[st.sel]].title}</dd></dl>`
      : `<h3>Part inspector</h3><p>Click any part on the robot (or in the parts list) to see what it is, what it does and what holds it. Hover for a quick name. Drag empty space to turn the robot.</p>`;
    if ($('cPart')._h !== ph){ $('cPart')._h = ph; $('cPart').innerHTML = ph; }
    const bs = st.built + '|' + st.sel;
    if (bs !== bomSig){ bomSig = bs; $('bom').innerHTML = Object.keys(R._step).map(id => `<li data-pick="${id}" class="${partShown(st, R, id) ? 'ok' : ''}"${st.sel === id ? ' style="font-weight:800"' : ''}><b>${R._step[id] + 1}</b>${R.parts[id].name}</li>`).join(''); }
  };
  const r0 = ui.render; ui.render = () => { r0(); sync(); }; ui.render();
  if (st.mode === 'op'){ if (R.enterOp) R.enterOp(st); lab.kick(); }
  if (q0.get('xr') === '1'){ const b = $('xrb'); if (b) b.setAttribute('aria-pressed', true); }
  return ui;
}

/* ---------- small control builders used by the robot panels ---------- */
const slHTML = (id, label, min, max, step, unit, tip = '') => `<div class="sl" id="w_${id}"><div class="slh"><span title="${tip}">${label}</span><b id="v_${id}"></b></div><input type="range" id="s_${id}" min="${min}" max="${max}" step="${step}" aria-label="${label}"><div class="rng"><span>${min}${unit}</span><span>${max}${unit}</span></div></div>`;
function slSync(id, v, txt, atLim){ const s = document.getElementById('s_' + id); if (!s) return; if (document.activeElement !== s) s.value = v;
  document.getElementById('v_' + id).textContent = txt; document.getElementById('w_' + id).classList.toggle('lim', !!atLim); }
// virtual joystick: writes st.joy = [x (right), y (up)] in -1..1, springs back on release; WASD / arrows drive it too
function joyBind(el, st, lab){
  const knob = el.querySelector('i'); let id = null;
  const set = (x, y) => { const l = Math.hypot(x, y); if (l > 1){ x /= l; y /= l; } st.joy = [x, y]; knob.style.left = (50 + x*36) + '%'; knob.style.top = (50 - y*36) + '%'; lab.kick(); };
  const fromE = e => { const r = el.getBoundingClientRect(); set((e.clientX - r.left - r.width/2) / (r.width*.36), -(e.clientY - r.top - r.height/2) / (r.height*.36)); };
  el.addEventListener('pointerdown', e => { id = e.pointerId; el.setPointerCapture(id); fromE(e); e.preventDefault(); });
  el.addEventListener('pointermove', e => { if (e.pointerId === id) fromE(e); });
  const up = e => { if (e.pointerId !== id) return; id = null; set(0, 0); };
  el.addEventListener('pointerup', up); el.addEventListener('pointercancel', up);
  const keys = new Set(), map = { w:[0,1], s:[0,-1], a:[-1,0], d:[1,0], ArrowUp:[0,1], ArrowDown:[0,-1], ArrowLeft:[-1,0], ArrowRight:[1,0] };
  const kset = () => { let x = 0, y = 0; for (const k of keys){ x += map[k][0]; y += map[k][1]; } set(x, y); };
  addEventListener('keydown', e => { if (!map[e.key] || st.mode !== 'op' || /input|textarea/i.test(e.target.tagName)) return; e.preventDefault(); e.stopImmediatePropagation(); keys.add(e.key); kset(); }, true);
  addEventListener('keyup', e => { if (!map[e.key]) return; keys.delete(e.key); kset(); }, true);
}

/* ---------- hub ---------- */
function thumbSVG(R, i){
  R._step = {}; R.steps.forEach((s, j) => s.parts.forEach(p => R._step[p] = j));
  const st = Object.assign({ mode:'build', built:R.steps.length, pt:1, sel:null, xr:0 }, R.init(), R.thumbState || {});
  CAM_LIM = null; setCam(YAW0 + (R.thumbYaw || 0), PITCH0 + (R.thumbPitch || 0)); setLight(105);
  const [W, D, H] = R.size; RC = R.center || [W/2, D/2]; RCZ = H/2; OX = OY = 0;
  const pts = []; for (const x of [0, W]) for (const y of [0, D]) for (const z of [0, H]) pts.push(P(x, y, z));
  const xs = pts.map(p => p[0]), ys = pts.map(p => p[1]), bw = Math.max(...xs) - Math.min(...xs), bh = Math.max(...ys) - Math.min(...ys);
  const vbW = Math.max(bw/.9, bh/.9/.78), vbH = vbW*.78;
  OX = (vbW - bw)/2 - Math.min(...xs); OY = (vbH - bh)/2 - Math.min(...ys);
  INK_K = vbW / INK_REF_VB;
  const body = inkScale(labScene(st, PALS[R.pal], R)).replace(/clip-path="url\(#/g, 'clip-path="url(#').trim();
  return `<svg viewBox="0 0 ${f(vbW)} ${f(vbH)}" role="img" aria-label="${R.name}"><defs>${filterFor(INK_K).replace('id="hand"', `id="hand${i}"`).replace('id="wob"', '')}</defs><g filter="url(#hand${i})">${body}</g></svg>`;
}
function renderHub(){
  document.title = 'Robotics Lab';
  const style = document.createElement('style');
  style.textContent = CORE_CSS + `
    .hub{width:100%;max-width:1200px}
    .hero{display:grid;grid-template-columns:1.3fr 1fr;gap:28px;align-items:end;margin:6px 0 22px}
    .hero h1{font-size:30px;line-height:1.1;margin:0 0 8px;letter-spacing:-.01em} .hero p{margin:0;font-size:13px;line-height:1.6;color:#3b352b}
    .how{display:grid;grid-template-columns:repeat(3,1fr);gap:10px} .how div{border:1.5px solid var(--ink);border-radius:12px;padding:9px 11px;background:#fbf7ea;box-shadow:2px 2px 0 var(--ink);font-size:11.5px;line-height:1.45}
    .how b{display:block;font-size:11px;letter-spacing:.1em;text-transform:uppercase;margin-bottom:3px}
    .grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(270px,1fr));gap:16px}
    .rc{border:1.5px solid var(--ink);border-radius:14px;background:#fbf7ea;box-shadow:3px 3px 0 var(--ink);display:flex;flex-direction:column;overflow:hidden;color:var(--ink);text-decoration:none;transition:transform .12s}
    a.rc:hover{transform:translate(-2px,-2px);box-shadow:5px 5px 0 var(--ink)}
    .rc .th{aspect-ratio:1/.78;border-bottom:1.5px solid var(--ink);background:var(--paper);display:flex;align-items:center;justify-content:center}
    .rc .th svg{width:100%;height:100%;display:block}
    .rc .bp{width:100%;height:100%;display:flex;flex-direction:column;align-items:center;justify-content:center;gap:6px;background-image:linear-gradient(#c9d6e6 1px,transparent 1px),linear-gradient(90deg,#c9d6e6 1px,transparent 1px);background-size:16px 16px;background-color:#e9eff5;color:#3d5878;font-size:11px;letter-spacing:.14em;text-transform:uppercase;font-weight:800}
    .rc .bp big{font-size:42px;letter-spacing:0;color:#9db2cc}
    .rc .bd{padding:11px 13px 13px;display:flex;flex-direction:column;gap:6px;flex:1}
    .rc .no{font-size:10.5px;letter-spacing:.14em;color:var(--muted);text-transform:uppercase;display:flex;justify-content:space-between}
    .rc h2{font-size:16px;margin:0} .rc p{margin:0;font-size:12px;line-height:1.5;color:#3b352b}
    .chips2{display:flex;flex-wrap:wrap;gap:5px;margin-top:auto;padding-top:6px} .chips2 span{font-size:10.5px;font-weight:700;border:1.2px solid var(--ink);border-radius:999px;padding:1px 8px;background:#fff}
    .go{font-weight:800;font-size:12px;margin-top:6px} .rc.planned{opacity:.82}
    .gloss{margin-top:26px;display:grid;grid-template-columns:repeat(auto-fill,minmax(250px,1fr));gap:10px}
    .gloss div{border-top:1.5px solid var(--ink);padding:8px 2px;font-size:12px;line-height:1.5} .gloss b{display:block;font-size:12.5px}
    .sect{font-size:11px;letter-spacing:.14em;text-transform:uppercase;color:var(--muted);margin:26px 0 8px}
    @media (max-width:760px){.hero{grid-template-columns:1fr}.how{grid-template-columns:1fr}.hero h1{font-size:24px}}`;
  document.head.appendChild(style);
  const cards = ROBOT_ORDER.map((m, i) => { const R = ROBOTS[m.id];
    let th = '';
    if (R){ try { th = thumbSVG(R, i); } catch (e) { th = `<div class="bp">${e.message}</div>`; document.documentElement.dataset.errors = (document.documentElement.dataset.errors || '') + ' thumb ' + m.id + ': ' + e.message + ';'; } }
    else th = `<div class="bp"><big>${String(i + 1).padStart(2, '0')}</big>Blueprint &middot; in the workshop</div>`;
    const inner = `<div class="th">${th}</div><div class="bd"><div class="no"><span>Robot ${String(i + 1).padStart(2, '0')}</span><span>${R ? `${R.steps.length} steps &middot; ${Object.keys(R._step).length} parts` : 'coming next'}</span></div>
      <h2>${m.name}</h2><p>${m.blurb}</p><div class="chips2">${m.chips.map(c => `<span>${c}</span>`).join('')}</div>
      <div class="go">${R ? 'Build it &amp; drive it &rarr;' : 'Spec sheet only for now'}</div></div>`;
    return R ? `<a class="rc" href="?robot=${m.id}">${inner}</a>` : `<div class="rc planned">${inner}</div>`; }).join('');
  document.body.innerHTML = `<div class="hub">
    <div class="hero"><div><div class="sect" style="margin-top:0">Robotics Lab &middot; no hardware needed</div><h1>Build a robot part by part, look through its skin, then make it move.</h1>
      <p>Each machine here is modelled on how the real thing is built: the motors, gearboxes, encoders, bearings, belts, boards and cables are all in the right place. Assemble it step by step, switch on See-through to watch the mechanism, then drive it with real kinematics, joint limits and loads.</p></div>
      <div class="how"><div><b>1 &middot; Assemble</b>Place one part at a time, base to tool. Click any part to learn what it is and what holds it.</div><div><b>2 &middot; See-through</b>Covers turn to glass so motors, gears, belts and wiring show while it is built.</div><div><b>3 &middot; Operate</b>Sliders, drag-the-tip IK, joystick or gaits, with live angles, torques, traces and crashes.</div></div></div>
    <div class="grid">${cards}</div>
    <div class="sect">Parts you will meet</div>
    <div class="gloss">${HUB_GLOSSARY.map(([t, b]) => `<div><b>${t}</b>${b}</div>`).join('')}</div>
    <svg width="0" height="0" style="position:absolute"><g id="art"></g></svg></div>`;
}
const HUB_GLOSSARY = [
  ['Servo motor', 'A motor plus a position sensor plus a control loop. You command an angle; it measures where it is and drives until the error is zero. Industrial servos are brushless AC motors with high-resolution encoders; hobby servos hide a DC motor, gears and a potentiometer in one plastic case.'],
  ['Stepper motor', 'Moves in fixed steps (usually 200 per turn) as its coils are switched in sequence. No sensor needed for position, which is why CNC machines and 3D printers use them, but if it is overloaded it silently skips steps.'],
  ['Encoder', 'Counts rotation. An optical or magnetic disc with fine marks passes a sensor; quadrature (two channels 90 degrees apart) gives direction. Absolute encoders remember position through a power cut.'],
  ['Gear reduction', 'Trades speed for torque. A 100:1 reducer makes the joint turn 100 times slower than the motor and up to 100 times stronger (minus friction). Strain wave (harmonic) and cycloidal (RV) reducers give big ratios with almost no backlash.'],
  ['Motor driver', 'The power stage between the low-power controller and the motor: H-bridges for DC motors, step/direction drivers for steppers, three-phase inverters for brushless servos. It turns logic signals into amps.'],
  ['Kinematics', 'Forward kinematics: from joint angles, where is the tool? Inverse kinematics: for a wanted tool position, which joint angles? IK can have several answers (elbow up or down) or none (out of reach).'],
];
