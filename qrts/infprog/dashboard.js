'use strict';
/* dashboard.js -- UI for the DE2-115 accelerator bench tools.
 *
 * Every action here ends in a quartus_stp invocation on the server, so the UI
 * is built around one rule: the cable is exclusive. Buttons disable while an
 * operation is in flight, and long transfers become a polled job rather than a
 * held-open request -- a stalled Quartus and a dead board look identical to a
 * hanging fetch, and only one of those is worth waiting on.
 */

const $ = (sel, root = document) => root.querySelector(sel);
const $$ = (sel, root = document) => [...root.querySelectorAll(sel)];

let busy = false;
let defaults = {};

/* ---- plumbing ---------------------------------------------------------- */

function setBusy(on, msg) {
  busy = on;
  $('#cable-dot').classList.toggle('busy', on);
  $$('button').forEach(b => { if (!b.classList.contains('tab')) b.disabled = on; });
  if (msg) $('#foot-msg').textContent = msg;
}

async function api(path, opts) {
  const res = await fetch(path, opts);
  const txt = await res.text();
  try { return JSON.parse(txt); }
  catch { return { ok: false, error: `non-JSON reply: ${txt.slice(0, 400)}` }; }
}

async function call(path) {
  if (busy) return { ok: false, error: 'another cable operation is running' };
  setBusy(true, `running ${path} ...`);
  try { return await api(path); }
  catch (e) { return { ok: false, error: String(e) }; }
  finally { setBusy(false, 'idle'); }
}

async function post(path, body) {
  return api(path, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(body),
  });
}

/* ---- tabs -------------------------------------------------------------- */

$$('.tab').forEach(t => t.addEventListener('click', () => {
  $$('.tab').forEach(x => x.classList.remove('active'));
  $$('.panel').forEach(x => x.classList.remove('active'));
  t.classList.add('active');
  $('#' + t.dataset.panel).classList.add('active');
}));

/* ---- health checks ----------------------------------------------------- */

function stepEl(name) { return $(`.steps li[data-step="${name}"]`); }

function markStep(name, ok, text) {
  const li = stepEl(name);
  if (!li) return;
  li.classList.toggle('ok', ok === true);
  li.classList.toggle('bad', ok === false);
  $('.out', li).textContent = text;
}

const CHECKS = {
  async cable() {
    const r = await call('/api/cable');
    if (!r.ok) { markStep('cable', false, r.error); return false; }
    const dev = (r.devices || []).join(', ') || 'none';
    if (!r.device_present) {
      markStep('cable', false,
        `cable seen but no device with IDCODE 0x020F70DD. Saw: ${dev}`);
      return false;
    }
    markStep('cable', true, dev);
    $('#cable-dot').classList.add('ok');
    return true;
  },

  async ident() {
    const r = await call('/api/ident');
    if (!r.ok) { markStep('ident', false, r.error); return false; }
    const want = defaults.expected_array_size;
    const strip = $('#ident-strip');
    strip.innerHTML =
      `IDENT <b>${r.ident}</b> &nbsp; array <b>${r.array_size}</b> &nbsp; build <b>${r.build_id}</b>`;
    if (!r.magic_ok) {
      markStep('ident', false,
        `magic ${r.magic}, expected 0xDC — this is not the dvcon bitstream`);
      return false;
    }
    if (r.array_size !== want) {
      markStep('ident', false,
        `ARRAY_SIZE ${r.array_size}, tools expect ${want}. A blob exported for ` +
        `a different array reads the wrong weight tile for every conv, silently.`);
      return false;
    }
    markStep('ident', true, `magic ${r.magic}, ARRAY_SIZE ${r.array_size}, build ${r.build_id}`);
    return true;
  },

  async memtest() {
    const r = await call('/api/memtest');
    if (!r.ok) { markStep('memtest', false, r.error); return false; }
    markStep('memtest', r.passed,
      r.passed ? `${r.pairs.length}/${r.pairs.length} words match (incl. all-zero and all-ones)`
               : `${r.bad} word(s) differ`);
    return r.passed;
  },

  async regs() {
    const r = await call('/api/regs');
    if (!r.ok) { markStep('regs', false, r.error); return false; }
    renderRegs(r);
    markStep('regs', true, 'read back');
    return true;
  },
};

function renderRegs(r) {
  const body = $('#reg-table tbody');
  body.innerHTML = '';
  for (const [k, v] of Object.entries(r.registers || {})) {
    const tr = document.createElement('tr');
    const td1 = document.createElement('td'); td1.textContent = k;
    const td2 = document.createElement('td'); td2.textContent = v ?? '—';
    tr.append(td1, td2); body.append(tr);
  }
  const s = r.status || {};
  $('#status-bits').innerHTML = '';
  [['busy', s.busy], ['done', s.done], ['error', s.error]].forEach(([n, on]) => {
    const d = document.createElement('span');
    d.className = 'bit' + (on ? ' on' : '') + (n === 'error' && on ? ' err' : '');
    d.textContent = n;
    $('#status-bits').append(d);
  });
  const f = document.createElement('span');
  f.className = 'bit'; f.textContent = 'fsm ' + (s.fsm ?? '?');
  $('#status-bits').append(f);
}

$$('.steps .go').forEach(b =>
  b.addEventListener('click', () => CHECKS[b.dataset.act]()));

$('#run-all-checks').addEventListener('click', async () => {
  // Sequential and short-circuiting: a failed cable check makes the ident
  // result meaningless, and running on would just produce noise to read past.
  for (const name of ['cable', 'ident', 'memtest', 'regs']) {
    const ok = await CHECKS[name]();
    if (!ok) { $('#foot-msg').textContent = `stopped at "${name}" — fix that before the rest`; return; }
  }
  $('#foot-msg').textContent = 'all checks passed';
});


/* ---- image loading ------------------------------------------------------ */

// Read the file in the browser and post it base64-encoded. FileReader gives a
// data: URL, so the header up to the comma is stripped off.
function fileToBase64(file) {
  return new Promise((resolve, reject) => {
    const fr = new FileReader();
    fr.onload = () => resolve(String(fr.result).split(',')[1] || '');
    fr.onerror = () => reject(fr.error);
    fr.readAsDataURL(file);
  });
}

$('#btn-img').addEventListener('click', async () => {
  const f = $('#img-file').files[0];
  const out = $('#img-result');
  if (!f) { $('#foot-msg').textContent = 'choose an image first'; return; }

  setBusy(true, `preparing ${f.name} ...`);
  let r;
  try {
    r = await post('/api/image', { name: f.name, data: await fileToBase64(f) });
  } catch (e) {
    r = { ok: false, error: String(e) };
  }
  setBusy(false, 'idle');

  out.hidden = false;
  out.className = 'result ' + (r.ok ? 'ok' : 'bad');
  if (!r.ok) { out.textContent = r.error; return; }

  const lb = r.letterbox;
  out.innerHTML =
    `<h3>frame ready</h3>` +
    `<table class="kv">` +
    row('source', `${r.original[0]}×${r.original[1]}`) +
    row('frame', `${r.imgsz}×${r.imgsz}×3 INT8 CHW, ${r.bytes} bytes`) +
    row('letterbox', `scale ${lb.scale}, pad ${lb.pad_x},${lb.pad_y}`) +
    row('decoder', r.decoder) +
    row('bin', r.bin) +
    `</table>` +
    `<p class="hint">Boxes come back in 640-space. To put one back on the ` +
    `original: x_orig = (x_640 − ${lb.pad_x}) / ${lb.scale}</p>`;

  // Cache-bust: the preview is always written to the same path.
  $('#img-preview').src = '/preview.png?t=' + Date.now();
  $('#img-preview-wrap').hidden = false;
  $('#frame-path').value = r.bin;
  $('#foot-msg').textContent = 'frame prepared — now Transfer frame';
});

/* ---- transport ---------------------------------------------------------- */

function transport() {
  const el = document.querySelector('input[name="transport"]:checked');
  return el ? el.value : 'jtag';
}

$$('input[name="transport"]').forEach(r =>
  r.addEventListener('change', () => { $('#eth-opts').hidden = transport() !== 'eth'; }));

/* ---- transfer ---------------------------------------------------------- */

let jobTimer = null;

function watchJob(onDone) {
  clearInterval(jobTimer);
  jobTimer = setInterval(async () => {
    const j = await api('/api/job');
    $('#job-name').textContent = j.name || 'idle';
    $('#job-spin').hidden = !j.running;
    if (j.log && j.log.length) $('#job-log').textContent = j.log.slice(-400).join('\n');
    $('#job-log').scrollTop = $('#job-log').scrollHeight;
    if (!j.running) {
      clearInterval(jobTimer);
      setBusy(false, 'idle');
      if (onDone) onDone(j.result || {});
    }
  }, 700);
}

$$('[data-load]').forEach(btn => btn.addEventListener('click', async () => {
  const which = btn.dataset.load;
  const path = $(`#${which}-path`).value.trim();
  const base = $(`#${which}-base`).value.trim();
  const verify = $(`#${which}-verify`).checked;
  if (!path) { $('#foot-msg').textContent = 'give a file path first'; return; }

  const via = transport();
  setBusy(true, `transferring ${which} over ${via} ...`);
  const r = await post('/api/load', {
    path, base, verify, transport: via,
    iface: $('#eth-iface').value.trim(),
    gap_us: $('#eth-gap').value.trim(),
  });
  if (!r.ok) { setBusy(false, r.error); return; }
  watchJob(res => {
    if (res.ok) {
      const size = res.words !== undefined ? `${res.words} words`
                                          : `${res.bytes} bytes`;
      $('#foot-msg').textContent =
        `${which} loaded over ${via}: ${size} in ${res.seconds}s` +
        (res.mbytes_per_s ? ` (${res.mbytes_per_s} MB/s)` : '') +
        (res.verify_bad === 0 ? ', verify clean' : '');
    } else {
      $('#foot-msg').textContent = `${which} transfer FAILED: ${res.error}`;
    }
  });
}));

/* ---- inference --------------------------------------------------------- */

$('#btn-run').addEventListener('click', async () => {
  setBusy(true, 'inference running ...');
  const r = await post('/api/run', {
    conf: $('#conf').value.trim(),
    desc_base: $('#desc-base').value.trim(),
  });
  if (!r.ok) { setBusy(false, r.error); return; }
  watchJob(res => renderRun(res));
});

function renderRun(res) {
  const el = $('#run-result');
  el.hidden = false;
  if (!res.ok) {
    el.className = 'result bad';
    el.innerHTML = `<h3>Failed</h3><pre class="log">${esc(res.error || 'unknown')}</pre>`;
    return;
  }
  const notes = res.notes || [];
  el.className = 'result ' + (res.error ? 'bad' : notes.length ? 'warn' : 'ok');
  el.innerHTML =
    `<h3>STATUS ${esc(res.status)}</h3>` +
    `<table class="kv"><tbody>` +
    row('busy / done / error', `${res.busy} / ${res.done} / ${res.error}`) +
    row('fsm', res.fsm) +
    row('layer_idx', res.layer_idx) +
    row('num_boxes', res.num_boxes) +
    row('elapsed', `${res.ms} ms over ${res.polls} polls`) +
    `</tbody></table>` +
    notes.map(n => `<p class="note">${esc(n)}</p>`).join('');
  $('#foot-msg').textContent = `inference finished: ${res.num_boxes} box(es)`;
}

const esc = s => String(s).replace(/[&<>]/g, c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;' }[c]));
const row = (k, v) => `<tr><td>${esc(k)}</td><td>${esc(v)}</td></tr>`;

$('#btn-boxes').addEventListener('click', async () => {
  const r = await call('/api/boxes');
  const wrap = $('#boxes-wrap');
  if (!r.ok) { wrap.innerHTML = `<p class="muted">${esc(r.error)}</p>`; return; }
  if (!r.boxes || !r.boxes.length) {
    wrap.innerHTML =
      `<p class="muted">NUM_BOXES reads ${r.count}. Zero here is only a real
       negative if the run reached the last layer — check layer_idx on the
       Inference panel first.</p>`;
    return;
  }
  wrap.innerHTML =
    `<div class="scrollx"><table class="boxes"><thead><tr>
       <th>#</th><th>x1</th><th>y1</th><th>x2</th><th>y2</th><th>score</th><th>class</th>
     </tr></thead><tbody>` +
    r.boxes.map((b, i) =>
      `<tr><td>${i}</td><td>${b.x1.toFixed(2)}</td><td>${b.y1.toFixed(2)}</td>
       <td>${b.x2.toFixed(2)}</td><td>${b.y2.toFixed(2)}</td>
       <td>${b.score}</td><td>${b.cls}</td></tr>`).join('') +
    `</tbody></table></div>`;
});

/* ---- render detections onto the frame ---------------------------------- */

$('#btn-render').addEventListener('click', async () => {
  const frame = $('#frame-path').value.trim();
  const r = await call('/api/render' + (frame ? '?frame=' + encodeURIComponent(frame) : ''));
  const wrap = $('#render-wrap');
  if (!r.ok) {
    wrap.hidden = false;
    $('#render-note').textContent = r.error;
    $('#render-img').removeAttribute('src');
    return;
  }
  wrap.hidden = false;
  // Cache-bust: the file is rewritten in place at a fixed URL, so without this
  // the browser shows the previous run's picture.
  $('#render-img').src = '/render.png?t=' + Date.now();
  $('#render-note').textContent =
    r.drawn === 0
      ? `${r.count} box(es) reported, none drawable — zero-area boxes are what a run that never reached the detect stage leaves behind.`
      : `${r.drawn} of ${r.count} box(es) drawn.`;
});

/* ---- board LED guide --------------------------------------------------- */

async function loadLedGuide() {
  const r = await api('/api/leds');
  const host = $('#led-guide');
  if (!r.ok) { host.innerHTML = `<p class="muted">${esc(r.error || 'unavailable')}</p>`; return; }
  const group = (title, cls, rows) =>
    `<div class="ledgroup"><h3>${title}</h3>` +
    rows.map(l => `<div class="ledrow">
        <span class="ledid"><span class="lamp ${cls}"></span>${esc(l.id)}</span>
        <span class="ledname">${esc(l.name)}</span>
        <span class="ledwhy">${esc(l.meaning)}</span>
      </div>`).join('') + `</div>`;
  host.innerHTML = group('Green — host and system', 'g', r.leds.green) +
                   group('Red — accelerator internals', 'r', r.leds.red);
}

/* ---- memory ------------------------------------------------------------ */

$$('.chip[data-addr]').forEach(c => c.addEventListener('click', () => {
  $('#mem-addr').value = c.dataset.addr;
  $('#btn-memread').click();
}));

$('#btn-memread').addEventListener('click', async () => {
  const addr = $('#mem-addr').value.trim();
  const n = $('#mem-n').value.trim();
  const r = await call(`/api/memread?addr=${encodeURIComponent(addr)}&n=${encodeURIComponent(n)}`);
  if (!r.ok) { $('#mem-out').textContent = r.error; return; }

  const base = parseInt(addr, 16) || 0;
  const lines = [];
  for (let i = 0; i < r.words.length; i += 4) {
    const a = (base + i * 4).toString(16).padStart(8, '0').toUpperCase();
    lines.push(`0x${a}  ` + r.words.slice(i, i + 4).map(w => w.slice(2).toUpperCase()).join(' '));
  }
  $('#mem-out').textContent = lines.join('\n') || '(no data)';
  decodeMem(base, r.words);
});

/* A raw hex dump of a descriptor is unreadable, and the two fields that decide
 * whether a run does anything -- the opcode and NEXT -- are the ones worth
 * calling out by name. */
function decodeMem(base, words) {
  const el = $('#mem-decode');
  const w = words.map(x => parseInt(x, 16));

  if (base === 0x00000000 && w.length >= 4) {
    const magic = w[0] >>> 0;
    const ok = magic === 0x594f4c4f;
    el.hidden = false;
    el.innerHTML =
      `<b>Blob header</b><br>magic 0x${magic.toString(16).toUpperCase()} ` +
      (ok ? '= "YOLO" ✓' : '✗ expected 0x594F4C4F') +
      `<br>version ${w[1]}, descriptors ${w[2]}, stride ${w[3]} bytes` +
      (ok ? `<br><span class="muted">descriptor table starts at 0x00000020</span>` : '');
    return;
  }

  const OPS = { 0: 'END', 1: 'CONV', 2: 'ADD', 3: 'CONCAT', 4: 'UPSAMPLE',
                5: 'MAXPOOL', 6: 'SPLIT', 7: 'SOFTMAX', 8: 'DETECT', 9: 'TOPK' };
  if (base >= 0x20 && base < 0x400000 && w.length >= 16) {
    const op = w[0], next = w[15];
    el.hidden = false;
    el.innerHTML =
      `<b>Descriptor</b> op ${op} (${OPS[op] ?? '?'}), flags 0x${(w[1] >>> 0).toString(16)}<br>` +
      `src0 0x${(w[2] >>> 0).toString(16)}, dst 0x${(w[4] >>> 0).toString(16)}, ` +
      `wgt 0x${(w[5] >>> 0).toString(16)}<br>` +
      `in ${w[7] & 0xffff}×${w[7] >>> 16}×${w[8] & 0xffff} → ` +
      `out ${w[9] & 0xffff}×${w[9] >>> 16}×${w[10] & 0xffff}, ` +
      `k${w[11] & 0xf} s${(w[11] >> 4) & 0xf}<br>` +
      `NEXT 0x${(next >>> 0).toString(16)}` +
      (next === 0 ? ' <span class="note">— zero ends the walk here</span>' : '');
    return;
  }
  el.hidden = true;
}

/* ---- ethernet ---------------------------------------------------------- */

$('#btn-eth').addEventListener('click', async () => {
  const r = await call('/api/eth');
  const body = $('#eth-table tbody');
  if (!r.ok) { body.innerHTML = `<tr><td colspan="2">${esc(r.error)}</td></tr>`; return; }
  body.innerHTML = Object.entries(r.counters)
    .map(([k, v]) => row(k, v)).join('');
  const v = $('#eth-verdict');
  v.hidden = false;
  const allZero = Object.values(r.counters).every(x => x === 0);
  v.className = 'verdict ' + (allZero ? 'bad' : 'ok');
  v.textContent = r.verdict;
});

$('#btn-ethdiag').addEventListener('click', async () => {
  const d = $('#eth-diag');
  const r = await call('/api/ethdiag');
  d.hidden = false;
  d.className = 'verdict ' + (r.ok ? 'ok' : 'bad');
  const rows = (r.adapters || []).map(a =>
    row(a.Name, `${a.Status} · ${a.LinkSpeed}`)).join('');
  d.innerHTML =
    `<p>${esc(r.verdict || '')}</p>` +
    (rows ? `<table class="kv">${rows}</table>` : '') +
    (r.ok ? '' :
      `<p class="note">JP1: ${esc(r.fix.jumper)}</p>` +
      `<p class="note">NIC (Administrator PowerShell):</p>` +
      `<pre class="log">${esc(r.fix.nic)}</pre>` +
      `<p class="hint">${esc(r.fix.nic_note)}</p>`) +
    ((r.notes || []).length ? `<p class="hint">${esc(r.notes.join('; '))}</p>` : '');
});

/* ---- boot -------------------------------------------------------------- */

(async function init() {
  defaults = await api('/api/defaults');
  if (defaults.model) $('#model-path').value = defaults.model;
  if (defaults.frame) $('#frame-path').value = defaults.frame;
  if (defaults.model_base) $('#model-base').value = defaults.model_base;
  if (defaults.frame_base) $('#frame-base').value = defaults.frame_base;
  if (defaults.desc_base) $('#desc-base').value = defaults.desc_base;
  loadLedGuide();
  $('#foot-msg').textContent = 'idle — start with Health → Run all checks';
})();
