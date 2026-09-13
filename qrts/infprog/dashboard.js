'use strict';
/* dashboard.js - UI for the DE2-115 NPU.
 *
 * Every board action ends in one quartus_stp run on the server and the cable is
 * exclusive, so board buttons lock while one is in flight and long transfers are
 * a polled job. The LED panel is computed from the STATUS register the same way
 * rtl/platform/dvcon_status.sv drives the real lamps.
 */

const $ = (s, r = document) => r.querySelector(s);
const $$ = (s, r = document) => [...r.querySelectorAll(s)];
const esc = s => String(s ?? '').replace(/[&<>"]/g, c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));
const PALETTE = ['#ff4040', '#40c8ff', '#78ff78', '#ffc83c', '#dc78ff', '#ff8c28', '#50ffdc', '#ff5aaa'];

let busy = false;
let defaults = {};
let transport = 'eth';
let board = null;              // last decoded status, null = never read

/* ---------------------------------------------------------------- plumbing */
function say(msg) { $('#foot-msg').textContent = msg; }

function setBusy(on, msg) {
  busy = on;
  $$('button').forEach(b => { if (!b.classList.contains('nav') && !b.classList.contains('link')) b.disabled = on; });
  $('#cable-dot').classList.toggle('busy', on);
  if (msg) say(msg);
}

async function api(path, opts) {
  try {
    const res = await fetch(path, opts);
    const txt = await res.text();
    try { return JSON.parse(txt); } catch { return { ok: false, error: `unexpected reply: ${txt.slice(0, 300)}` }; }
  } catch (e) { return { ok: false, error: `server not reachable: ${e}` }; }
}
const post = (p, body) => api(p, { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body) });

async function call(path, msg) {
  if (busy) return { ok: false, error: 'another board operation is running' };
  setBusy(true, msg || 'talking to the board');
  try { return await api(path); } finally { setBusy(false); }
}

/* ---------------------------------------------------------------- views */
const TITLES = { 'v-dash': 'Dashboard', 'v-xfer': 'Transfer', 'v-run': 'Inference',
                 'v-board': 'Board and LEDs', 'v-mem': 'Memory', 'v-eth': 'Ethernet' };
function show(id) {
  $$('.nav').forEach(n => n.classList.toggle('active', n.dataset.view === id));
  $$('.view').forEach(v => v.classList.toggle('active', v.id === id));
  $('#view-title').textContent = TITLES[id];
  history.replaceState(null, '', '#' + id.slice(2));
}
$$('.nav').forEach(n => n.addEventListener('click', () => show(n.dataset.view)));
$$('[data-goto]').forEach(b => b.addEventListener('click', () => show(b.dataset.goto)));

/* ---------------------------------------------------------------- board LEDs */
const RED = ['Idle', 'Fetch', 'Conv input', 'Conv weights', 'Conv array', 'Conv store', 'Add',
             'Upsample', 'Maxpool', 'Softmax', 'Pack', 'Detect', 'Done', 'Error',
             'DMA read', 'DMA write', 'MAC slot', 'Found boxes'];
const GREEN = ['Heartbeat', 'Model loaded', 'Image loaded', 'Eth RX', 'Eth TX', 'RX error', 'Transfer', 'NPU busy', 'Reset off'];
const SEG = { 0: 0x3F, 1: 0x06, 2: 0x5B, 3: 0x4F, 4: 0x66, 5: 0x6D, 6: 0x7D, 7: 0x07, 8: 0x7F, 9: 0x6F,
              a: 0x77, b: 0x7C, c: 0x39, d: 0x5E, e: 0x79, f: 0x71,
              '-': 0x40, ' ': 0, n: 0x54, o: 0x5C, r: 0x50, E: 0x79, C: 0x39, A: 0x77, U: 0x3E,
              P: 0x73, S: 0x6D, I: 0x30 };
const OPL = { 1: 'C', 2: 'A', 3: 'U', 4: 'P', 5: 'S', 6: 'I', 7: 'd' };

function ledState(s) {
  const r = Array(18).fill(false), g = Array(9).fill(false);
  g[8] = true;                                // reset released
  let digits = '--------';
  if (s) {
    const conv = s.fsm === 2, elem = s.fsm === 3;
    r[0] = s.fsm === 0 && !s.busy;
    r[1] = s.fsm === 1 || s.fsm === 6;
    r[2] = conv && s.conv_phase === 2;
    r[3] = conv && (s.conv_phase === 1 || s.conv_phase === 3);
    r[4] = conv && s.conv_phase === 4;
    r[5] = conv && s.conv_phase === 5;
    for (let op = 2; op <= 7; op++) r[4 + op] = elem && s.op === op;
    r[12] = s.done && !s.error;
    r[13] = !!s.error;
    r[17] = s.done && s.boxes > 0;
    g[7] = !!s.busy;
    g[1] = !!(s.flags & 1);
    g[2] = !!(s.flags & 2);
    const hx = (v, n) => v.toString(16).padStart(n, '0').slice(-n);
    if (s.busy) {
      const t = s.tag % 100;
      digits = `${Math.floor(t / 10)}${t % 10}${OPL[s.op] || '-'} ${hx(s.idx, 4)}`;
    } else if (s.error) digits = `Err ${hx(s.idx, 4)}`;
    else if (s.done) digits = `donE${hx(s.boxes, 4)}`;
  }
  return { r, g, digits };
}

function digitHTML(ch) {
  const key = /[0-9]/.test(ch) ? +ch : (SEG[ch] !== undefined ? ch : ch.toLowerCase());
  const m = SEG[key] ?? 0x40;
  return `<div class="digit">${'abcdefg'.split('').map((k, i) =>
    `<i class="${k}${m >> i & 1 ? ' on' : ''}"></i>`).join('')}</div>`;
}

function renderBoard(el, full) {
  const { r, g, digits } = ledState(board);
  const lamp = (on, id, label, extra = '') =>
    `<div class="lamp${on ? ' on' : ''}${extra}"><i></i><b>${id}</b>${full ? `<span>${esc(label)}</span>` : ''}</div>`;
  el.innerHTML =
    `<div class="lamps r">${r.map((on, i) => lamp(on, 'R' + i, RED[i])).join('')}</div>` +
    `<div class="boardrow">` +
      `<div class="lamps g" style="flex:1 1 360px">${g.map((on, i) =>
          lamp(i === 0 || on, 'G' + i, GREEN[i], i === 0 ? ' blink' : '')).join('')}</div>` +
      `<div><div class="hex" role="img" aria-label="7-segment display ${esc(digits)}">` +
        digits.split('').map(digitHTML).join('') + `</div>` +
      `<div class="hexcap"><span>7</span><span>6</span><span>5</span><span>4</span><span>3</span><span>2</span><span>1</span><span>0</span></div></div>` +
    `</div>`;
}
function renderBoards() { renderBoard($('#board-mini'), false); renderBoard($('#board-full'), true); }

function applyStatus(regs, st) {
  const R = k => parseInt(regs[k] || '0', 16);
  board = { ...st, op: R('OP') & 0xff, tag: R('TAG'), idx: R('LAYER_IDX'), boxes: R('NUM_BOXES'), flags: R('FLAGS') };
  renderBoards();
  const pill = $('#state-pill');
  pill.className = 'pill ' + (st.error ? 'bad' : st.busy ? 'busy' : 'ok');
  $('#state-text').textContent = st.error ? `Error: ${st.err_text || st.state}` :
    st.busy ? `Running descriptor ${board.idx}` : st.done ? `Done, ${board.boxes} boxes` : 'Idle';
  $('#led-src').textContent = `STATUS ${regs.STATUS} read at ${new Date().toLocaleTimeString()}.`;
}

async function loadLedGuide() {
  const r = await api('/api/leds');
  if (!r.ok) return;
  const all = [...r.leds.red, ...r.leds.green, ...(r.leds.hex || [])];
  $('#led-guide').innerHTML = all.map(l =>
    `<div><b>${esc(l.id)}</b>${esc(l.name)}<span>${esc(l.meaning)}</span></div>`).join('');
}

/* ---------------------------------------------------------------- health */
function markStep(name, ok, text) {
  const li = $(`.steps li[data-step="${name}"]`);
  li.classList.toggle('ok', ok === true);
  li.classList.toggle('bad', ok === false);
  $('.out', li).textContent = text;
}
function healthRow(id, ok, text) {
  const b = $(id); b.className = ok ? 'ok' : 'bad'; b.textContent = text;
}

const CHECKS = {
  async cable() {
    const r = await call('/api/cable', 'looking for the USB-Blaster');
    if (!r.ok || !r.device_present) {
      const t = r.ok ? `cable seen, no EP4CE115 (saw ${(r.devices || []).join(', ') || 'nothing'})` : r.error;
      markStep('cable', false, t); healthRow('#h-cable', false, 'not found');
      $('#chip-title').textContent = 'Board not found'; $('#chip-sub').textContent = 'Check the USB-Blaster';
      $('#cable-dot').className = 'dot bad';
      return false;
    }
    markStep('cable', true, r.devices.join(', ')); healthRow('#h-cable', true, 'connected');
    $('#cable-dot').className = 'dot ok';
    $('#chip-title').textContent = 'DE2-115 connected'; $('#chip-sub').textContent = 'USB-Blaster';
    return true;
  },
  async ident() {
    const r = await call('/api/ident', 'reading IDENT');
    if (!r.ok) { markStep('ident', false, r.error); healthRow('#h-ident', false, 'no answer'); return false; }
    const good = r.magic_ok && r.is_npu && r.array_size === defaults.expected_array_size;
    const t = good ? `${r.ident}: NPU, 16 x 16 array` : `${r.ident}: not the NPU bitstream, program quartus/output_files/dvcon.sof`;
    markStep('ident', good, t); healthRow('#h-ident', good, good ? 'NPU loaded' : 'wrong bitstream');
    if (good) $('#chip-sub').textContent = `NPU ${r.ident}`;
    return good;
  },
  async memtest() {
    const r = await call('/api/memtest', 'testing SDRAM');
    const ok = r.ok && r.passed;
    markStep('memtest', ok, r.ok ? (ok ? `${r.pairs.length} of ${r.pairs.length} words read back` : `${r.bad} words differ`) : r.error);
    healthRow('#h-mem', ok, ok ? 'passes' : 'fails');
    return ok;
  },
  async regs() {
    const r = await call('/api/regs', 'reading registers');
    if (!r.ok) { markStep('regs', false, r.error); return false; }
    renderRegs(r); markStep('regs', true, `state ${r.status.state}`);
    return true;
  },
};
$$('.steps [data-act]').forEach(b => b.addEventListener('click', () => CHECKS[b.dataset.act]()));
$('#run-all-checks').addEventListener('click', async () => {
  for (const n of ['cable', 'ident', 'memtest', 'regs']) {
    if (!await CHECKS[n]()) { say(`Stopped at "${n}": fix that before the rest.`); return; }
  }
  say('All checks passed.');
});

function renderRegs(r) {
  $('#reg-rows').innerHTML = Object.entries(r.registers || {})
    .map(([k, v]) => `<li><span>${esc(k)}</span><b>${esc(v)}</b></li>`).join('');
  applyStatus(r.registers, r.status);
}
$('#btn-refresh').addEventListener('click', async () => {
  const r = await call('/api/regs', 'reading STATUS');
  if (!r.ok) { say(r.error); return; }
  renderRegs(r); say('Board state read.');
});

/* ---------------------------------------------------------------- transfer */
$$('.seg button').forEach(b => b.addEventListener('click', () => {
  transport = b.dataset.tp;
  $$('.seg button').forEach(x => { x.classList.toggle('on', x === b); x.setAttribute('aria-checked', x === b); });
  $('#eth-opts').hidden = transport !== 'eth';
  $('#tp-note').textContent = transport === 'eth'
    ? 'Ethernet moves the frame in about 0.1 s. It needs JP2 on pins 2-3, the cable in J5 and the NIC at 100 Mbps.'
    : 'JTAG needs only the USB-Blaster: about a minute for a frame and two for the model.';
}));

const drop = $('#drop');
['dragenter', 'dragover'].forEach(e => drop.addEventListener(e, ev => { ev.preventDefault(); drop.classList.add('over'); }));
['dragleave', 'drop'].forEach(e => drop.addEventListener(e, ev => { ev.preventDefault(); drop.classList.remove('over'); }));
drop.addEventListener('drop', ev => {
  const f = ev.dataTransfer.files[0];
  if (f) { $('#img-file').files = ev.dataTransfer.files; $('#drop-name').textContent = f.name; }
});
$('#img-file').addEventListener('change', () => {
  const f = $('#img-file').files[0]; if (f) $('#drop-name').textContent = f.name;
});

const toB64 = f => new Promise((ok, bad) => {
  const r = new FileReader(); r.onload = () => ok(String(r.result).split(',')[1] || ''); r.onerror = () => bad(r.error); r.readAsDataURL(f);
});

async function prepare() {
  const f = $('#img-file').files[0];
  if (!f) { say('Choose an image first.'); return null; }
  setBusy(true, `Preparing ${f.name}`);
  const r = await post('/api/image', { name: f.name, data: await toB64(f) });
  setBusy(false);
  if (!r.ok) { say(r.error); return null; }
  const lb = r.letterbox;
  $('#img-preview').src = '/preview.png?t=' + Date.now(); $('#img-preview').hidden = false;
  $('#img-rows').hidden = false;
  $('#img-rows').innerHTML =
    `<li><span>Original</span><b>${r.original[0]} x ${r.original[1]}</b></li>` +
    `<li><span>Frame</span><b>3 x ${r.imgsz} x ${r.imgsz} INT8, ${r.bytes.toLocaleString()} bytes</b></li>` +
    `<li><span>Letterbox</span><b>scale ${lb.scale}, bars ${lb.pad_x} x ${lb.pad_y} px</b></li>`;
  $('#frame-path').value = r.bin;
  $('#det-pill-text').textContent = f.name; $('#det-pill').className = 'pill soft ok';
  say(`Frame ready: ${r.bin}`);
  return r;
}
$('#btn-img').addEventListener('click', prepare);
$('#btn-img-send').addEventListener('click', async () => { if (await prepare()) load('frame'); });

let jobTimer = null;
function watchJob(done) {
  clearInterval(jobTimer);
  $('#job-bar').parentElement.classList.add('run');
  $('#job-pill').className = 'pill soft busy';
  jobTimer = setInterval(async () => {
    const j = await api('/api/job');
    $('#job-name').textContent = j.name || 'Idle';
    if (j.log && j.log.length) { const l = $('#job-log'); l.textContent = j.log.slice(-300).join('\n'); l.scrollTop = l.scrollHeight; }
    if (!j.running) {
      clearInterval(jobTimer);
      $('#job-bar').parentElement.classList.remove('run');
      $('#job-bar').style.width = '100%';
      const ok = (j.result || {}).ok;
      $('#job-pill').className = 'pill soft ' + (ok ? 'ok' : 'bad');
      setBusy(false);
      done && done(j.result || {});
    }
  }, 700);
}

async function load(which) {
  const path = $(`#${which}-path`).value.trim();
  if (!path) { say(`Give the ${which} file first.`); return; }
  const base = which === 'model' ? defaults.model_base : defaults.frame_base;
  setBusy(true, `Sending the ${which} over ${transport === 'eth' ? 'Ethernet' : 'JTAG'}`);
  const r = await post('/api/load', { path, base, verify: $(`#${which}-verify`).checked, transport,
                                      iface: $('#eth-iface').value.trim(), gap_us: $('#eth-gap').value.trim() });
  if (!r.ok) { setBusy(false, r.error); return; }
  $('#job-bar').style.width = '0';
  watchJob(res => {
    if (!res.ok) { say(`Sending the ${which} failed: ${res.error}`); return; }
    const size = res.words !== undefined && res.words !== null ? `${res.words.toLocaleString()} words` : `${(res.bytes || 0).toLocaleString()} bytes`;
    say(`Sent the ${which}: ${size} in ${res.seconds} s${res.verify_bad === 0 ? ', verified' : ''}.`);
    if (which === 'model') healthRow('#h-model', true, 'in SDRAM');
  });
}
$$('[data-load]').forEach(b => b.addEventListener('click', () => load(b.dataset.load)));

/* ---------------------------------------------------------------- inference */
$('#conf').addEventListener('input', () => { $('#conf-val').textContent = (+$('#conf').value).toFixed(2); });
const conf = () => +$('#conf').value;

function showStage(boxes, note) {
  $('#stage-img').src = '/render.png?t=' + Date.now();
  $('#stage-img').hidden = false; $('#stage-empty').hidden = true;
  const count = {};
  (boxes || []).forEach(b => { count[b.cls] = count[b.cls] || { name: b.name, n: 0 }; count[b.cls].n++; });
  const keys = Object.keys(count);
  $('#legend').hidden = !keys.length;
  $('#legend').innerHTML = keys.map(k =>
    `<div><i style="background:${PALETTE[k % PALETTE.length]}"></i>${esc(count[k].name)}<span>${count[k].n}</span></div>`).join('');
  $('#det-meta').textContent = note;
  renderTable(boxes);
}

function renderTable(boxes) {
  const tb = $('#box-table tbody');
  if (!boxes || !boxes.length) { tb.innerHTML = '<tr><td colspan="7" class="muted">No detections above the threshold.</td></tr>'; return; }
  tb.innerHTML = boxes.map(b =>
    `<tr><td><i style="background:${PALETTE[b.cls % PALETTE.length]}"></i>${esc(b.name)}</td>` +
    `<td>${(b.conf * 100).toFixed(1)}%</td><td>${b.x1.toFixed(1)}</td><td>${b.y1.toFixed(1)}</td>` +
    `<td>${b.x2.toFixed(1)}</td><td>${b.y2.toFixed(1)}</td><td>${b.level < 0 ? 'FP32' : 'P' + (b.level + 3)}</td></tr>`).join('');
}

function setMetrics(ms, boxes, util, dma, stamp = true) {
  $('#m-ms').textContent = ms;
  $('#m-boxes').textContent = stamp ? `${boxes} box${boxes === 1 ? '' : 'es'}` : '';
  if (stamp) $('#run-when').textContent = new Date().toLocaleTimeString();
  const u = util === null ? null : Math.round(util * 100);
  $('#m-util').textContent = u === null ? '--' : u;
  $('#m-dma').textContent = dma === null ? 'SDRAM traffic --' : `SDRAM traffic ${dma} MB`;
  const on = u === null ? 0 : Math.round(u / 100 * 32);
  $('#segbar').innerHTML = Array.from({ length: 32 }, (_, i) => `<i class="${i < on ? 'on' : ''}"></i>`).join('');
}

async function drawBoard() {
  const r = await call('/api/render' + ($('#frame-path').value ? '?frame=' + encodeURIComponent($('#frame-path').value) : ''), 'Reading the box list');
  if (!r.ok) { say(r.error); return; }
  showStage(r.boxes_list, `Board result: ${r.count} detection(s), ${r.drawn} drawn.`);
}

async function runBoard() {
  setBusy(true, 'Inference running on the board');
  const r = await post('/api/run', { conf: conf(), model: $('#model-path').value.trim() });
  if (!r.ok) { setBusy(false, r.error); return; }
  watchJob(async res => {
    if (!res.ok) { say(`Run failed: ${res.error}`); return; }
    $('#run-rows').innerHTML =
      `<li><span>Status</span><b class="${res.error ? 'bad' : 'ok'}">${esc(res.state)} ${esc(res.status)}</b></li>` +
      `<li><span>Descriptors run</span><b>${res.layer_idx} of ${res.n_desc}</b></li>` +
      `<li><span>NPU time</span><b>${res.npu_ms} ms, ${res.cycles.toLocaleString()} cycles</b></li>` +
      `<li><span>Array utilisation</span><b>${(res.array_util * 100).toFixed(1)}%</b></li>` +
      `<li><span>SDRAM traffic</span><b>${res.dma_mb} MB</b></li>` +
      `<li><span>Detections</span><b>${res.num_boxes}</b></li>`;
    $('#run-notes').innerHTML = (res.notes || []).map(n => `<p class="note">${esc(n)}</p>`).join('');
    setMetrics(res.npu_ms, res.num_boxes, res.array_util, res.dma_mb);
    board = { busy: res.busy, done: res.done, error: res.error, fsm: res.fsm, conv_phase: 0,
              op: 0, tag: 0, idx: res.layer_idx, boxes: res.num_boxes };
    renderBoards();
    if (res.done && !res.error) await drawBoard();
  });
}
$('#btn-run').addEventListener('click', runBoard);
$('#btn-quickrun').addEventListener('click', () => { show('v-run'); runBoard(); });
$('#btn-draw').addEventListener('click', drawBoard);

async function simulate() {
  setBusy(true, 'Running the NPU model on this PC');
  const r = await post('/api/simulate', { conf: conf(), model: $('#model-path').value.trim(), frame: $('#frame-path').value.trim() });
  setBusy(false);
  if (!r.ok) { say(r.error); return; }
  showStage(r.boxes_list, `Simulated on this PC in ${r.seconds} s: ${r.count} detection(s). The board must return exactly these.`);
  setMetrics('--', r.count, null, null);
  $('#run-sub').textContent = 'Simulation result: time is measured only on the board.';
  say(`Simulation found ${r.count} detection(s).`);
}
$('#btn-sim').addEventListener('click', simulate);
$('#btn-sim-dash').addEventListener('click', simulate);

/* The unquantised network on the same frame. Simulate checks the hardware
 * (bit-exact by design); this measures what INT8 quantisation costs. */
async function floatRef() {
  setBusy(true, 'Running the original FP32 model on this PC');
  const r = await post('/api/float', { conf: conf(), frame: $('#frame-path').value.trim() });
  setBusy(false);
  if (!r.ok) { say(`FP32 reference failed: ${r.error}`); return; }
  const c = r.compare;
  const vs = !c ? ' Run the board or Simulate first to compare against the NPU.' :
    ` Against the ${c.against}: ${c.matched} matched (mean IoU ${c.mean_iou ?? '--'}, ` +
    `mean confidence difference ${c.mean_conf_diff ?? '--'}), ${c.float_only} only in FP32, ` +
    `${c.npu_only} only on the NPU.` + (c.missed.length ? ` NPU missed: ${c.missed.join(', ')}.` : '');
  showStage(r.boxes_list, `Original FP32 model: ${r.count} detection(s) in ${r.seconds} s.` + vs);
  setMetrics('--', r.count, null, null);
  $('#run-sub').textContent = 'FP32 reference result: no board timing.';
  say(`FP32 reference found ${r.count} detection(s).`);
}
$('#btn-float').addEventListener('click', floatRef);
$('#btn-float-dash').addEventListener('click', floatRef);

$('#btn-abort').addEventListener('click', async () => {
  const r = await post('/api/abort', {});
  say(r.ok ? 'Abort sent.' : `Abort failed: ${r.error}`);
});
$('#btn-boxes').addEventListener('click', async () => {
  const r = await call('/api/boxes', 'Reading the box list');
  if (!r.ok) { say(r.error); return; }
  renderTable(r.boxes); say(`${r.count} box(es) on the board.`);
});

/* ---------------------------------------------------------------- program */
async function programFpga() {
  const sof = $('#sof-path').value.trim();
  setBusy(true, 'Programming the FPGA');
  const r = await post('/api/program', { sof });
  if (!r.ok) { setBusy(false, r.error); return; }
  watchJob(res => {
    const out = $('#program-out');
    if (!res.ok) { out.textContent = `Programming failed: ${res.error}`; say(out.textContent); return; }
    out.textContent = `Programmed at ${new Date().toLocaleTimeString()} with ${res.sof}. ` +
      'SDRAM is empty now: send the model and the frame.';
    board = null; renderBoards();
    healthRow('#h-model', false, 'send again');
    say('FPGA programmed. Run the health checks, then send the model.');
  });
}
$('#btn-program').addEventListener('click', programFpga);
$('#btn-program-top').addEventListener('click', () => { show('v-board'); programFpga(); });

/* ---------------------------------------------------------------- memory */
$$('.chip[data-addr]').forEach(c => c.addEventListener('click', () => { $('#mem-addr').value = c.dataset.addr; $('#btn-memread').click(); }));
$('#btn-memread').addEventListener('click', async () => {
  const addr = $('#mem-addr').value.trim(), n = $('#mem-n').value.trim();
  const r = await call(`/api/memread?addr=${encodeURIComponent(addr)}&n=${encodeURIComponent(n)}`, 'Reading SDRAM');
  if (!r.ok) { $('#mem-out').textContent = r.error; return; }
  const base = parseInt(addr, 16) || 0, lines = [];
  for (let i = 0; i < r.words.length; i += 4)
    lines.push(`0x${(base + i * 4).toString(16).padStart(8, '0').toUpperCase()}  ` + r.words.slice(i, i + 4).map(w => w.slice(2).toUpperCase()).join(' '));
  $('#mem-out').textContent = lines.join('\n') || '(no data)';
  decodeMem(base, r.words.map(x => parseInt(x, 16)));
});

function decodeMem(base, w) {
  const el = $('#mem-decode');
  if (base === 0 && w.length >= 7) {
    const ok = w[0] >>> 0 === 0x504E5644;
    el.hidden = false;
    el.innerHTML = ok ? `Model blob: version ${w[1]}, ${w[2]} descriptors at 0x${(w[3] >>> 0).toString(16)}, input ${w[6]} x ${w[6]}.`
                      : 'No model blob here (the header magic should be DVNP).';
    return;
  }
  const OPS = ['END', 'CONV', 'ADD', 'UPSAMPLE', 'MAXPOOL', 'SOFTMAX', 'PACK', 'DETECT'];
  if (base >= 0x100 && base < 0x400000 && (base & 63) === 0 && w.length >= 16) {
    const op = w[0] & 0xff, fl = (w[0] >> 8) & 0xff;
    const flags = [fl & 1 && 'depthwise', fl & 2 && 'runtime weights', fl & 4 && 'transposed', fl & 8 && 'weights resident'].filter(Boolean).join(', ');
    el.hidden = false;
    el.innerHTML = `Descriptor: ${OPS[op] || 'unknown op ' + op}${flags ? ' (' + flags + ')' : ''}, model layer ${w[15] & 0xffff}.<br>` +
      `In ${w[7] & 0xffff} x ${w[7] >>> 16} x ${(w[8] & 0xffff) * 16} ch, out ${w[9] & 0xffff} x ${w[9] >>> 16} x ${(w[8] >>> 16) * 16} ch, ` +
      `kernel ${w[10] & 0xf}, stride ${(w[10] >> 4) & 0xf}, pad ${(w[10] >> 8) & 0xf}, band ${w[10] >>> 20} rows.`;
    return;
  }
  el.hidden = true;
}

/* ---------------------------------------------------------------- ethernet */
$('#btn-eth').addEventListener('click', async () => {
  const r = await call('/api/eth', 'Reading the MAC counters');
  if (!r.ok) { say(r.error); return; }
  const names = { GOOD: 'Good frames', BAD_FCS: 'Bad FCS', CMD: 'Command frames', FILTERED: 'Not for us', BITMAP: 'ACK bitmap', DROPS: 'Words dropped' };
  $('#eth-kpis').innerHTML = Object.entries(r.counters).map(([k, v]) =>
    `<div><b>${k === 'BITMAP' ? '0x' + v.toString(16) : v.toLocaleString()}</b><span>${names[k] || k}</span></div>`).join('');
  const v = $('#eth-verdict'); v.hidden = false;
  v.className = 'verdict ' + (r.counters.CMD > 0 && !r.counters.DROPS ? 'ok' : 'bad');
  v.textContent = r.verdict;
});
$('#btn-ethdiag').addEventListener('click', async () => {
  const r = await call('/api/ethdiag', 'Checking this PC');
  $('#eth-adapters').innerHTML = (r.adapters || []).map(a =>
    `<li><span>${esc(a.Name)}</span><b>${esc(a.Status)}, ${esc(a.LinkSpeed)}</b></li>`).join('') || '<li><span class="muted">No adapters reported.</span></li>';
  const d = $('#eth-diag'); d.hidden = false;
  d.className = 'verdict ' + (r.ok ? 'ok' : 'bad');
  d.textContent = r.verdict + (r.ok ? '' : ` Fix: ${r.fix.jumper} NIC: ${r.fix.nic}`);
});

/* ---------------------------------------------------------------- start */
(async function init() {
  defaults = await api('/api/defaults');
  if (defaults.model) $('#model-path').value = defaults.model;
  if (defaults.sof) $('#sof-path').value = defaults.sof;
  if (defaults.frame) $('#frame-path').value = defaults.frame;
  const b = defaults.blob;
  $('#blob-info').textContent = b
    ? `Model ${b.path.split(/[\\/]/).pop()}: ${(b.bytes / 1e6).toFixed(2)} MB, ${b.n_desc} descriptors, input ${b.imgsz} x ${b.imgsz}.`
    : 'No compiled model found. Run compiler/npu_compile.py first.';
  $('#h-model').textContent = b ? `${b.n_desc} descriptors on disk` : 'missing';
  setMetrics('--', 0, null, null, false);
  renderBoards();
  const v = 'v-' + location.hash.slice(1);
  if (TITLES[v]) show(v);
  loadLedGuide();
  if (defaults.frame) { $('#det-pill-text').textContent = defaults.frame.split(/[\\/]/).pop(); }
})();
