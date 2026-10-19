// CubeSat Mission Simulator front end.
//
// Performance model: only the open tab renders. Heavy Plotly figures are drawn
// once per simulation run; the moving parts (time cursor, satellite on the
// groundtrack) are cheap overlays, so animation never triggers a Plotly redraw.
// The 3D views are three.js and render at display rate.
import { AttitudeView } from './attitude.js';
import { OrbitView } from './orbit3d.js';

const MODE_COLORS = ['#ffd400', '#00ff66', '#ff00ff', '#ff3030', '#00e5ff', '#ff66cc', '#9a9a9a'];
const SLEW_COLOR = '#ff8c00';
const PHASE_NAME = ['SEPARATION / INHIBIT', 'DETUMBLING (B-dot, MTQ only)', 'MISSION'];
const MODE_TEXT = {
  0: 'charging: +Z panel side → Sun', 1: 'imaging: imager → target strip', 2: 'downlink: antenna → ground station',
  3: 'safe: Sun-pointing, payload off', 4: 'nadir: imager → Earth', 5: 'B-dot: magnetorquers damp the tumble',
  6: 'free tumble: deployables + RF inhibited',
};
const R_E = 6378.1363;
const EARTH_IMG = 'https://cdn.jsdelivr.net/npm/three-globe@2.31.0/example/img/earth-blue-marble.jpg';
const FACE_COLORS = ['#ff6b6b', '#a33', '#6bff6b', '#3a3', '#6bb5ff', '#357', '#ffb86b', '#a73'];
const PANEL_COLORS = ['#ff6b6b', '#6bff6b', '#6bb5ff', '#ffb86b'];
const AX = ['+X', '-X', '+Y', '-Y', '+Z', '-Z'];
const $ = id => document.getElementById(id);
const params = new URLSearchParams(location.search);
const SOLO = params.get('view') || (params.has('attitude') ? 'attitude' : null);
const WIN_ID = Math.random().toString(36).slice(2);
const bc = 'BroadcastChannel' in window ? new BroadcastChannel('cubesat-sim') : null;

// ================================================================== inputs
const TORQ = ['CR0002', 'CR0003', 'CR0004'];
const ROWS = [
  { name: 'ORBIT', color: 'cyan', fields: [
    ['Altitude km', 'orbit.altitude_km'], ['ECC', 'orbit.eccentricity'], ['SSO', 'orbit.sun_synchronous', 'check'],
    ['INC °', 'orbit.inclination_deg'], ['LTAN h', 'orbit.ltan_hours'], ['Arg lat °', 'orbit.arg_latitude_deg'],
    ['Epoch (UTC)', 'simulation.start_utc', 'wide'], ['Hours', 'simulation.duration_hours'], ['Step s', 'simulation.dt_s']] },
  { name: 'SOLAR ARRAY', color: 'magenta', fields: [
    ['+X panel °', 'geometry.panel_angles_deg', 0], ['+Y panel °', 'geometry.panel_angles_deg', 1],
    ['−X panel °', 'geometry.panel_angles_deg', 2], ['−Y panel °', 'geometry.panel_angles_deg', 3],
    ['Hinge end', 'geometry.hinge_end', 'sel', ['+Z', '-Z']], ['Cells front', 'geometry.cells_front'],
    ['Cells back', 'geometry.cells_back'], ['Cell eff', 'solar.cell_efficiency'], ['Albedo', 'solar.include_albedo', 'check']] },
  { name: 'POWER', color: 'red', fields: [
    ['Battery Wh', 'battery.capacity_wh'], ['SoC₀', 'battery.initial_soc'], ['SOM W', 'loads.som_base_w'], ['Accel +W', 'loads.obc_accelerator_extra_w'],
    ['EPS MCU W', 'loads.eps_mcu_w'], ['Imager W', 'loads.imager_imaging_w'], ['Idle W', 'loads.imager_idle_w'],
    ['Beacon s', 'loads.beacon_period_s'], ['PA eff', 'loads.pa_efficiency'], ['Heater W', 'loads.eclipse_heater_w']] },
  { name: 'ATT/PAYLOAD', color: 'lime', fields: [
    ['Rate °/s', 'attitude.max_rate_deg_s'], ['Acc °/s²', 'attitude.max_accel_deg_s2'], ['Idle point', 'attitude.idle_mode', 'sel', ['sun', 'nadir']],
    ['Imaging', 'payload.imaging_mode', 'sel', ['targets', 'continuous']], ['Sun axis', 'attitude.sun_axis', 'sel', AX],
    ['Imager axis', 'attitude.imager_axis', 'sel', AX], ['Antenna axis', 'attitude.antenna_axis', 'sel', AX], ['Max roll °', 'payload.max_off_nadir_deg'],
    ['Compress', 'payload.compression_ratio'], ['Data₀ GB', 'payload.initial_storage_gb']] },
  { name: 'ADCS/LEOP', color: '#ff66cc', fields: [
    ['Wheels', 'adcs.wheel_model', 'sel', ['CW0017', 'CW0057', 'CW0162']], ['MTQ X', 'adcs.torquer_x', 'sel', TORQ],
    ['MTQ Y', 'adcs.torquer_y', 'sel', TORQ], ['MTQ Z', 'adcs.torquer_z', 'sel', TORQ], ['OBC in LEOP', 'flightplan.obc_on_during_detumble', 'check'],
    ['ω₀x °/s', 'flightplan.initial_omega_deg_s', 0], ['ω₀y °/s', 'flightplan.initial_omega_deg_s', 1], ['ω₀z °/s', 'flightplan.initial_omega_deg_s', 2],
    ['Exit °/s', 'flightplan.detumble_exit_deg_s'], ['Seed', 'flightplan.release_seed'], ['Comm. h', 'flightplan.commissioning_hours']] },
  { name: 'LINK/GROUND', color: '#3d9bff', fields: [
    ['PA RF W', 'link.tx_power_w'], ['Coding', 'link.coding', 'sel', ['turbo_1_2', 'turbo_1_3', 'ldpc_7_8', 'conv_rs', 'uncoded']],
    ['Msps', 'link.max_symbol_rate_msps'], ['Gain dBi', 'link.antenna_gain_dbi'], ['G/T dB/K', 'link.gs_g_over_t_dbk'],
    ['Min el °', 'link.min_elevation_deg'], ['Rates Mbps', 'link.rates_mbps', 'wide'], ['GS lat', 'ground_station.lat_deg'],
    ['GS lon', 'ground_station.lon_deg'], ['Priority', 'link.priority', 'sel', ['imaging', 'downlink']]] },
];

function iniGet(text, sec, key) {
  let cur = '';
  for (const raw of text.split('\n')) {
    const line = raw.replace(/#.*/, '').trim();
    if (!line) continue;
    if (line[0] === '[') { cur = line.slice(1, line.indexOf(']')).trim(); continue; }
    const eq = line.indexOf('=');
    if (eq > 0 && cur === sec && line.slice(0, eq).trim() === key) return line.slice(eq + 1).trim();
  }
  return null;
}
function iniSet(text, sec, key, val) {
  const lines = text.split('\n');
  let cur = '', secLine = -1, lastInSec = -1;
  for (let i = 0; i < lines.length; i++) {
    const l = lines[i].replace(/#.*/, '').trim();
    if (l.startsWith('[')) { cur = l.slice(1, l.indexOf(']')).trim(); if (cur === sec) { secLine = i; lastInSec = i; } continue; }
    if (cur !== sec) continue;
    if (l) lastInSec = i;
    const eq = l.indexOf('=');
    if (eq > 0 && l.slice(0, eq).trim() === key) {
      const h = lines[i].indexOf('#');
      lines[i] = `${key} = ${val}` + (h >= 0 ? '   ' + lines[i].slice(h) : '');
      return lines.join('\n');
    }
  }
  if (secLine < 0) lines.push('', `[${sec}]`, `${key} = ${val}`);
  else lines.splice(lastInSec + 1, 0, `${key} = ${val}`);
  return lines.join('\n');
}

let cfgText = '', cfgDefault = '';
const fieldEls = [];
function buildInputs() {
  const host = $('inputs');
  host.innerHTML = '';
  ROWS.forEach((row, ri) => {
    const div = document.createElement('div');
    div.className = 'inrow';
    div.innerHTML = `<span class="rowname" style="color:${row.color}">${row.name}</span>`;
    row.fields.forEach(([label, key, kind, opts], fi) => {
      const id = `f${ri}_${fi}`;
      const f = document.createElement('span');
      f.className = 'field';
      let input;
      if (kind === 'check') input = `<input type="checkbox" id="${id}">`;
      else if (kind === 'sel') input = `<select id="${id}">${opts.map(o => `<option>${o}</option>`).join('')}</select>`;
      else input = `<input type="text" id="${id}" ${kind === 'wide' ? 'class="wide"' : ''}>`;
      f.innerHTML = `<label for="${id}" style="color:${row.color}">${label}</label>${input}`;
      div.appendChild(f);
      fieldEls.push({ id, key, kind, opts });
    });
    host.appendChild(div);
  });
}
function inputsFromConfig(text) {
  for (const fe of fieldEls) {
    const [sec, key] = fe.key.split('.');
    let v = iniGet(text, sec, key);
    const el = $(fe.id);
    if (typeof fe.kind === 'number') v = v == null ? '' : (v.split(',')[fe.kind] || '').trim();
    if (fe.kind === 'check') el.checked = v == null ? false : ['true', '1', 'yes', 'on'].includes(v);
    else if (fe.kind === 'sel' && v != null && !fe.opts.includes(v)) { el.add(new Option(v)); el.value = v; }
    else el.value = v ?? '';
  }
  const sso = fieldEls.find(f => f.key === 'orbit.sun_synchronous');
  const inc = fieldEls.find(f => f.key === 'orbit.inclination_deg');
  $(inc.id).disabled = $(sso.id).checked;
}
function configFromInputs(text) {
  const lists = {};
  for (const fe of fieldEls) {
    const [sec, key] = fe.key.split('.');
    const el = $(fe.id);
    if (typeof fe.kind === 'number') { (lists[fe.key] ||= [])[fe.kind] = el.value.trim(); continue; }
    if (fe.key === 'orbit.inclination_deg' && el.disabled) continue;
    const v = fe.kind === 'check' ? (el.checked ? 'true' : 'false') : el.value.trim();
    if (v !== '') text = iniSet(text, sec, key, v);
  }
  for (const [k, arr] of Object.entries(lists)) {
    const [sec, key] = k.split('.');
    text = iniSet(text, sec, key, arr.join(', '));
  }
  return text;
}

// ==================================================================== data
let D = null;
let simT = 0, playing = false, speed = 60, serverMode = false;

function geo2ecef(latDeg, lonDeg) {
  const f = 1 / 298.257223563, e2 = f * (2 - f);
  const la = latDeg * Math.PI / 180, lo = lonDeg * Math.PI / 180;
  const N = R_E / Math.sqrt(1 - e2 * Math.sin(la) ** 2);
  return [N * Math.cos(la) * Math.cos(lo), N * Math.cos(la) * Math.sin(lo), N * (1 - e2) * Math.sin(la)];
}
const rotZ = (v, g) => [Math.cos(g) * v[0] - Math.sin(g) * v[1], Math.sin(g) * v[0] + Math.cos(g) * v[1], v[2]];

function prep(sim) {
  const c = sim.cols, m = sim.meta, n = c.t.length;
  const d = { sim, c, m, n, dt: m.dt_out, tmax: c.t[n - 1] };
  d.hours = c.t.map(t => t / 3600);
  d.colors = c.mode.map((mo, i) => (c.slew[i] ? SLEW_COLOR : MODE_COLORS[mo] || '#fff'));
  d.v = new Array(n);
  for (let i = 0; i < n; i++) {
    const a = Math.max(0, i - 1), b = Math.min(n - 1, i + 1);
    d.v[i] = [c.x[b] - c.x[a], c.y[b] - c.y[a], c.z[b] - c.z[a]];
  }
  d.faceKeys = m.faces.map((_, f) => `face_p_${f}`);
  d.facePmax = Math.max(...m.faces.map(f => f.cell_area)) * 1361 * 0.3 || 1;
  d.gsEcef = geo2ecef(m.gs.lat, m.gs.lon);
  return d;
}

function sampleAt(fi) {
  const { c, n } = D;
  fi = Math.max(0, Math.min(n - 1, fi));
  const i0 = Math.floor(fi), i1 = Math.min(n - 1, i0 + 1), f = fi - i0;
  const L = k => c[k][i0] + (c[k][i1] - c[k][i0]) * f;
  let q0 = [c.qw[i0], c.qx[i0], c.qy[i0], c.qz[i0]], q1 = [c.qw[i1], c.qx[i1], c.qy[i1], c.qz[i1]];
  if (q0[0] * q1[0] + q0[1] * q1[1] + q0[2] * q1[2] + q0[3] * q1[3] < 0) q1 = q1.map(x => -x);
  let q = q0.map((x, k) => x + (q1[k] - x) * f);
  const qn = Math.hypot(...q); q = q.map(x => x / qn);
  let dg = c.gmst[i1] - c.gmst[i0];
  if (dg < -Math.PI) dg += 2 * Math.PI;
  const gmst = c.gmst[i0] + dg * f;
  const r = [L('x'), L('y'), L('z')];
  const gsEci = rotZ(D.gsEcef, gmst);
  const i = f < 0.5 ? i0 : i1;
  return {
    i, fi, r, v: D.v[i], q, gmst, sun: [c.sx[i], c.sy[i], c.sz[i]], sunfrac: L('sunfrac'),
    gsDir: [gsEci[0] - r[0], gsEci[1] - r[1], gsEci[2] - r[2]], gsEl: L('gs_el'), linkRate: c.link_rate[i],
    mode: c.mode[i], imager: c.imager[i], color: D.colors[i],
    faceP: D.faceKeys.map(k => c[k][i]), facePmax: D.facePmax,
    deployed: !D.m.launch || c.t[i] >= D.m.t_deploy, mtq: [c.mtq_x[i], c.mtq_y[i], c.mtq_z[i]],
  };
}

const utc = t => new Date((D.m.jd0 - 2440587.5) * 86400000 + t * 1000).toISOString().slice(0, 19).replace('T', ' ');
const fmt = (x, d = 1) => (x == null || !isFinite(x) ? '—' : Number(x).toFixed(d));
const modeLabel = i => (D.c.slew[i] ? 'SLEW → ' : '') + D.m.modes[D.c.mode[i]];

// ------------------------------------------------------------ geometry util
function groundCircle(latDeg, lonDeg, lam, step = 4) {
  const la = latDeg * Math.PI / 180, lo = lonDeg * Math.PI / 180, pts = [];
  for (let a = 0; a <= 360; a += step) {
    const az = a * Math.PI / 180;
    const la2 = Math.asin(Math.sin(la) * Math.cos(lam) + Math.cos(la) * Math.sin(lam) * Math.cos(az));
    const lo2 = lo + Math.atan2(Math.sin(az) * Math.sin(lam) * Math.cos(la), Math.cos(lam) - Math.sin(la) * Math.sin(la2));
    pts.push([((lo2 * 180 / Math.PI + 540) % 360) - 180, la2 * 180 / Math.PI]);
  }
  return pts;
}
function subsolar(i) {
  const c = D.c, s = rotZ([c.sx[i], c.sy[i], c.sz[i]], -c.gmst[i]);
  return [Math.asin(s[2]) * 180 / Math.PI, Math.atan2(s[1], s[0]) * 180 / Math.PI];
}
// Min/max decimation: keeps spikes (passes, imaging) visible with few points.
function decimate(x, y, maxBuckets = 2500) {
  const n = x.length;
  if (n <= maxBuckets * 2) return [x, y];
  const b = Math.ceil(n / maxBuckets), X = [], Y = [];
  for (let s = 0; s < n; s += b) {
    let lo = s, hi = s;
    const e = Math.min(n, s + b);
    for (let k = s; k < e; k++) { if (y[k] < y[lo]) lo = k; if (y[k] > y[hi]) hi = k; }
    const [a, z] = lo < hi ? [lo, hi] : [hi, lo];
    X.push(x[a], x[z]); Y.push(y[a], y[z]);
  }
  return [X, Y];
}

// =================================================================== views
const DARK = { paper_bgcolor: '#050505', plot_bgcolor: '#050505', font: { color: '#ccc', family: 'Times New Roman, serif', size: 12 } };
const PCFG = { responsive: true, displaylogo: false, modeBarButtonsToRemove: ['lasso2d', 'select2d'] };
const views = {};

// ---- 3D orbit
views.orbit = {
  init() {
    this.v = new OrbitView($('orbitview'));
    ['tg-full', 'tg-sun', 'tg-eq', 'tg-follow'].forEach(id => $(id).onchange = () => this.opts());
  },
  opts() { this.v.setOptions({ full: $('tg-full').checked, sun: $('tg-sun').checked, eq: $('tg-eq').checked, follow: $('tg-follow').checked }); },
  data() { this.v.setData(D); this.opts(); },
  frame(s) {
    this.v.update(s);
    this.v.render();
    if (s.i !== this.lastI) {
      this.lastI = s.i;
      const c = D.c, i = s.i;
      $('orbithud').innerHTML = `<div class="mode" style="color:${D.colors[i]}">${modeLabel(i)}</div>` +
        `<div>alt ${fmt(c.alt[i], 1)} km · lat ${fmt(c.lat[i], 2)}° · lon ${fmt(c.lon[i], 2)}°</div>` +
        `<div>i ${fmt(D.m.inclination_deg, 3)}° · LTAN ${fmt(D.m.ltan_h, 2)} h · T ${fmt(D.m.period_s / 60, 2)} min</div>` +
        `<div>β ${fmt(c.beta[i], 1)}° · ${c.sunfrac[i] < 0.5 ? '<span style="color:#88f">eclipse</span>' : 'sunlit'}` +
        (c.gs_el[i] > 0 ? ` · <span style="color:magenta">GS el ${fmt(c.gs_el[i], 1)}°</span>` : '') + '</div>';
    }
  },
};

// ---- attitude
views.attitude = {
  init() {
    this.v = new AttitudeView($('att'));
    $('frame-btn').onclick = () => {
      const f = this.v.frame === 'lvlh' ? 'eci' : 'lvlh';
      this.v.setFrame(f);
      $('frame-btn').textContent = f === 'lvlh' ? 'Frame: LVLH' : 'Frame: Inertial';
    };
    $('cam-btn').onclick = () => this.v.resetCamera();
  },
  data() { this.v.build(D.m); },
  frame(s) {
    this.v.update(s);
    this.v.render();
    if (s.i !== this.lastI) { this.lastI = s.i; updateHud(s); }
  },
};
function updateHud(s) {
  const { c, m } = D, i = s.i;
  const task = c.task[i] >= 0 ? m.tasks[c.task[i]] : null;
  let what = MODE_TEXT[c.mode[i]];
  if (c.mode[i] === 1 && task) what = `imaging ${task.label} (roll ${fmt(task.roll, 1)}°)`;
  if (c.mode[i] === 2) what = `downlink → ${m.gs.name}`;
  $('hud').innerHTML =
    `<div class="mode" style="color:${D.colors[i]}">${c.slew[i] ? 'SLEWING → ' : ''}${m.modes[c.mode[i]]}</div><div>${what}</div>` +
    (m.launch ? `<div style="color:#ff66cc">${PHASE_NAME[c.phase[i]]} · |ω| ${fmt(c.wmag[i], 2)} °/s</div>` : '') +
    `<div>err ${fmt(c.perr[i], 1)}° · ${c.sunfrac[i] < 0.5 ? '<span style="color:#88f">eclipse</span>' : 'sunlit'} · β ${fmt(c.beta[i], 1)}°</div>` +
    `<div style="color:#ffd400">solar ${fmt(c.p_solar[i], 1)} W · load ${fmt(c.p_load[i], 1)} W · SoC ${fmt(c.soc[i] * 100, 0)}%</div>` +
    (c.gs_el[i] > 0 ? `<div style="color:magenta">GS el ${fmt(c.gs_el[i], 1)}° · ${c.link_rate[i] > 0 ? fmt(c.link_rate[i], 1) + ' Mbps' : 'no link'}</div>` : '');
}

// ---- groundtrack: static Plotly map + canvas overlay for everything that moves
views.ground = {
  init() {
    this.canvas = $('gtover');
    this.ctx = this.canvas.getContext('2d');
    ['tg-gscone', 'tg-term', 'tg-foot'].forEach(id => $(id).onchange = () => {
      if (id === 'tg-gscone' && this.built) Plotly.restyle('gt', { visible: $(id).checked }, [1]);
      this.lastKey = null;
    });
  },
  data() {
    const { c, m } = D;
    const lon = [], lat = [], col = [];
    for (let i = 0; i < D.n; i += 2) {
      if (i > 1 && Math.abs(c.lon[i] - c.lon[i - 2]) > 180) { lon.push(null); lat.push(null); col.push('#000'); }
      lon.push(c.lon[i]); lat.push(c.lat[i]); col.push(D.colors[i]);
    }
    const e = m.gs_mask_deg * Math.PI / 180;
    const lamGs = Math.acos(R_E / (R_E + m.altitude_km) * Math.cos(e)) - e;
    const gsc = groundCircle(m.gs.lat, m.gs.lon, lamGs, 3);
    Plotly.react('gt', [
      { type: 'scattergl', mode: 'markers', x: lon, y: lat, marker: { size: 1.6, color: col, opacity: 0.35 }, hoverinfo: 'skip' },
      { type: 'scatter', mode: 'lines', x: gsc.map(p => p[0]), y: gsc.map(p => p[1]), line: { color: 'magenta', width: 1, dash: 'dot' }, hoverinfo: 'skip', visible: $('tg-gscone').checked },
      { type: 'scatter', mode: 'markers+text', x: m.targets.map(t => t.lon), y: m.targets.map(t => t.lat), text: m.targets.map(t => t.name),
        textposition: 'middle right', textfont: { size: 10, color: '#7dff9e' }, marker: { symbol: 'diamond-open', size: 8, color: '#00ff66' }, hovertemplate: '%{text}<extra></extra>' },
      { type: 'scatter', mode: 'markers+text', x: [m.gs.lon], y: [m.gs.lat], text: [m.gs.name], textposition: 'top center',
        textfont: { color: 'magenta', size: 11 }, marker: { symbol: 'star', size: 13, color: 'magenta' }, hoverinfo: 'text' },
    ], {
      ...DARK, paper_bgcolor: '#000', plot_bgcolor: '#000', showlegend: false, margin: { l: 44, r: 10, t: 34, b: 38 },
      title: { text: 'Groundtracks', font: { color: 'magenta', family: 'monospace', size: 16 } },
      xaxis: { range: [-180, 180], dtick: 30, gridcolor: 'rgba(255,255,255,.18)', zeroline: false, title: 'Longitude' },
      yaxis: { range: [-90, 90], dtick: 30, gridcolor: 'rgba(255,255,255,.18)', zeroline: false, title: 'Latitude' },
      images: [{ source: EARTH_IMG, xref: 'x', yref: 'y', x: -180, y: 90, sizex: 360, sizey: 180, sizing: 'stretch', layer: 'below', opacity: 0.85 }],
    }, PCFG);
    this.built = true;
    this.lastKey = null;
  },
  frame(s) {
    const gd = $('gt');
    const fl = gd._fullLayout;
    if (!fl || !fl.xaxis) return;
    const xa = fl.xaxis, ya = fl.yaxis;
    const W = gd.clientWidth, H = gd.clientHeight;
    const key = `${s.i}|${xa.range}|${ya.range}|${W}x${H}`;
    if (key === this.lastKey) return;  // nothing moved
    this.lastKey = key;
    const cv = this.canvas, dpr = Math.min(2, devicePixelRatio);
    if (cv.width !== Math.round(W * dpr) || cv.height !== Math.round(H * dpr)) {
      cv.width = Math.round(W * dpr); cv.height = Math.round(H * dpr); cv.style.width = W + 'px'; cv.style.height = H + 'px';
    }
    const g = this.ctx;
    g.setTransform(dpr, 0, 0, dpr, 0, 0);
    g.clearRect(0, 0, W, H);
    g.save();
    g.beginPath(); g.rect(xa._offset, ya._offset, xa._length, ya._length); g.clip();
    const X = lon => xa._offset + xa.l2p(lon), Y = lat => ya._offset + ya.l2p(lat);
    const poly = (pts, color, width, dash = []) => {
      g.strokeStyle = color; g.lineWidth = width; g.setLineDash(dash); g.beginPath();
      pts.forEach((p, k) => {
        if (k && Math.abs(p[0] - pts[k - 1][0]) > 180) g.moveTo(X(p[0]), Y(p[1]));
        else if (k) g.lineTo(X(p[0]), Y(p[1]));
        else g.moveTo(X(p[0]), Y(p[1]));
      });
      g.stroke(); g.setLineDash([]);
    };
    const { c, m } = D, i = s.i;
    if ($('tg-term').checked) {
      const ss = subsolar(i);
      poly(groundCircle(ss[0], ss[1], Math.PI / 2, 3), '#ffd400', 1, [5, 4]);
      g.fillStyle = '#ffd400'; g.beginPath(); g.arc(X(ss[1]), Y(ss[0]), 6, 0, 7); g.fill();
    }
    const per = Math.round(m.period_s / D.dt);
    g.lineWidth = 2.5;
    for (let k = Math.max(1, i - per); k <= i; k++) {
      if (Math.abs(c.lon[k] - c.lon[k - 1]) > 180) continue;
      g.strokeStyle = D.colors[k];
      g.beginPath(); g.moveTo(X(c.lon[k - 1]), Y(c.lat[k - 1])); g.lineTo(X(c.lon[k]), Y(c.lat[k])); g.stroke();
    }
    if ($('tg-foot').checked) poly(groundCircle(c.lat[i], c.lon[i], Math.acos(R_E / (R_E + c.alt[i])), 4), 'rgba(255,255,255,.8)', 1);
    if (c.gs_el[i] > 0) poly([[c.lon[i], c.lat[i]], [m.gs.lon, m.gs.lat]], c.link_rate[i] > 0 ? '#ff00ff' : '#663366', 2);
    g.fillStyle = '#fff'; g.strokeStyle = D.colors[i]; g.lineWidth = 3;
    g.beginPath(); g.arc(X(c.lon[i]), Y(c.lat[i]), 6, 0, 7); g.fill(); g.stroke();
    g.restore();
  },
};

// ---- plots: independent small figures, built lazily, with DOM cursors
const PLOTS = [
  { id: 'power', title: 'Power (W)', on: true, s: [['Solar available', 'p_solar', '#ffd400'], ['Load', 'p_load', '#ff3030'], ['Albedo part', 'p_albedo', '#3d9bff'], ['Curtailed', 'p_curtail', '#888', 'dot']] },
  { id: 'soc', title: 'Battery state of charge (%)', on: true, s: [['SoC %', 'soc', 'cyan', null, 100]], range: [0, 102] },
  { id: 'loads', title: 'Load by subsystem (W, stacked)', on: true, stack: [['OBC+TTC SOM', 'load_obc', '#ff3030'], ['ADCS', 'load_adcs', '#ff66cc'], ['PA + beacon', 'load_ttc', '#ff00ff'], ['Payload', 'load_imager', '#00ff66'], ['EPS + hk', 'load_eps', '#3d9bff'], ['Heater', 'load_heater', '#ff8c00']] },
  { id: 'point', title: 'Pointing angles (°)', on: true, s: [['Sun ∠ sun axis', 'sun_axis_angle', '#ffd400'], ['GS ∠ antenna', 'antenna_gs_angle', 'magenta'], ['Nadir ∠ imager', 'imager_nadir_angle', '#00ff66'], ['Sun ∠ imager', 'sun_imager_angle', '#ff8c00'], ['Pointing error', 'perr', '#fff', 'dot']] },
  { id: 'faces', title: 'Solar array face power (W)', faces: true },
  { id: 'rates', title: 'Body rate (°/s)', s: [['ωx', 'wx', '#ff6b6b'], ['ωy', 'wy', '#6bff6b'], ['ωz', 'wz', '#6bb5ff'], ['|ω|', 'wmag', '#fff']] },
  { id: 'adcs', title: 'ADCS actuators', s: [['MTQ power W', 'mtq_w', '#ff66cc'], ['Wheel power W', 'wheel_w', '#ffd400']], s2: [['wheel h x mNms', 'hw_x', '#ff6b6b'], ['wheel h y mNms', 'hw_y', '#6bff6b'], ['wheel h z mNms', 'hw_z', '#6bb5ff']], y2: 'mNms' },
  { id: 'link', title: 'Ground station & link', s: [['GS elevation °', 'gs_el', 'magenta']], s2: [['Link rate Mbps', 'link_rate', '#ff8c00']], y2: 'Mbps', range: [0, 90] },
  { id: 'data', title: 'Data (GB)', s: [['Camera flash', 'camera_gb', 'cyan'], ['OBC storage', 'obc_gb', '#3d9bff'], ['Downlinked', 'downlinked_gb', 'magenta'], ['Imaged', 'imaged_gb', '#00ff66']] },
  { id: 'temps', title: 'Panel temperature (°C)', s: [0, 1, 2, 3].map(p => [['+X', '+Y', '−X', '−Y'][p] + ' panel', `panel_temp_${p}`, PANEL_COLORS[p]]) },
];
views.plots = {
  init() {
    let saved = null;
    try { saved = JSON.parse(localStorage.getItem('cubesat-plots')); } catch (e) { /* storage unavailable */ }
    $('plotpick').innerHTML = PLOTS.map(p => `<label><input type="checkbox" data-plot="${p.id}" ${(saved ? saved.includes(p.id) : p.on) ? 'checked' : ''}>${p.title}</label>`).join('');
    $('plotpick').querySelectorAll('input').forEach(cb => cb.onchange = () => {
      try { localStorage.setItem('cubesat-plots', JSON.stringify([...$('plotpick').querySelectorAll('input:checked')].map(x => x.dataset.plot))); } catch (e) { /* ignore */ }
      this.layout();
    });
    this.cards = {};
  },
  data() { for (const k in this.cards) this.cards[k].built = false; this.layout(); },
  layout() {
    const on = new Set([...$('plotpick').querySelectorAll('input:checked')].map(x => x.dataset.plot));
    for (const p of PLOTS) {
      let card = this.cards[p.id];
      if (on.has(p.id)) {
        if (!card) {
          const el = document.createElement('div');
          el.className = 'pcard';
          el.innerHTML = '<div class="pl"></div><div class="cursor"></div>';
          card = this.cards[p.id] = { el, pl: el.firstChild, cur: el.lastChild, built: false, def: p };
        }
        card.el.style.display = '';
        $('plotgrid').appendChild(card.el);  // keeps the picker order
        if (!card.built && D) {
          try { this.build(card); } catch (e) { console.error('plot', p.id, e); }
        }
      } else if (card) card.el.style.display = 'none';
    }
    this.lastI = -1;
  },
  build(card) {
    const p = card.def, c = D.c, x = D.hours;
    const tr = [];
    const line = (name, key, color, dash, scale, yaxis = 'y') => {
      const y = scale ? c[key].map(v => v * scale) : c[key];
      const [X, Y] = decimate(x, y);
      tr.push({ type: 'scattergl', mode: 'lines', x: X, y: Y, name, yaxis, line: { color, width: 1.3, dash: dash || 'solid' }, hovertemplate: `${name}: %{y:.3f}<extra></extra>` });
    };
    (p.s || []).forEach(a => line(...a));
    (p.s2 || []).forEach(([n, k, col]) => line(n, k, col, 'dot', null, 'y2'));
    if (p.faces) D.m.faces.forEach((f, k) => line(f.name, `face_p_${k}`, FACE_COLORS[k % FACE_COLORS.length]));
    if (p.stack) {
      const acc = new Float64Array(D.n);
      p.stack.forEach(([name, key, color], j) => {
        for (let i = 0; i < D.n; i++) acc[i] += c[key][i];
        const [X, Y] = decimate(x, Array.from(acc), 2000);
        tr.push({ type: 'scatter', mode: 'lines', x: X, y: Y, name, fill: j ? 'tonexty' : 'tozeroy', line: { color, width: 0.8 }, fillcolor: color + '55', hovertemplate: `≤ ${name}: %{y:.2f} W<extra></extra>` });
      });
    }
    const shapes = [];
    let a = null;
    for (let i = 0; i <= D.n; i++) {
      const ecl = i < D.n && c.sunfrac[i] < 0.5;
      if (ecl && a == null) a = i;
      if (!ecl && a != null) { shapes.push({ type: 'rect', xref: 'x', yref: 'paper', x0: x[a], x1: x[i - 1], y0: 0, y1: 1, fillcolor: 'rgba(90,90,160,.15)', line: { width: 0 }, layer: 'below' }); a = null; }
    }
    let s0 = 0;
    for (let i = 1; i <= D.n; i++) {
      if (i === D.n || D.colors[i] !== D.colors[s0]) {
        shapes.push({ type: 'rect', xref: 'x', yref: 'paper', x0: x[s0], x1: x[i - 1] + D.dt / 3600, y0: 1.0, y1: 1.035, fillcolor: D.colors[s0], line: { width: 0 } });
        s0 = i;
      }
    }
    const layout = {
      ...DARK, margin: { l: 52, r: p.y2 ? 52 : 14, t: 30, b: 58 }, hovermode: 'x unified', showlegend: true,
      legend: { orientation: 'h', x: 0, y: -0.28, font: { size: 10 } },
      title: { text: p.title, font: { size: 13, color: 'magenta', family: 'monospace' }, x: 0.01, y: 0.99, yanchor: 'top' },
      xaxis: { title: { text: 'hours since epoch', font: { size: 10 } }, gridcolor: '#1c1c1c', zeroline: false },
      yaxis: { gridcolor: '#1c1c1c', zeroline: false, ...(p.range ? { range: p.range } : {}) },
      shapes,
    };
    if (p.y2) layout.yaxis2 = { overlaying: 'y', side: 'right', title: { text: p.y2, font: { size: 10 } }, showgrid: false };
    Plotly.newPlot(card.pl, tr, layout, PCFG).catch(e => console.error('plot', p.id, e)).then(() => {
      card.pl.on('plotly_click', ev => { if (ev.points && ev.points[0]) seek(ev.points[0].x * 3600); });
      card.built = true;
      this.lastI = -1;
    });
  },
  frame(s) {
    if (s.i === this.lastI) return;
    this.lastI = s.i;
    const h = D.hours[s.i];
    for (const k in this.cards) {
      const card = this.cards[k];
      if (!card.built || card.el.style.display === 'none') continue;
      const fl = card.pl._fullLayout;
      if (!fl || !fl.xaxis) continue;
      const xa = fl.xaxis, ya = fl.yaxis;
      const px = xa._offset + xa.l2p(h);
      const vis = px >= xa._offset && px <= xa._offset + xa._length;
      card.cur.style.display = vis ? '' : 'none';
      card.cur.style.transform = `translateX(${px}px)`;
      card.cur.style.top = ya._offset + 'px';
      card.cur.style.height = ya._length + 'px';
    }
  },
};

// ---- telemetry (text, 4 Hz)
function kv(k, v) { return `<div class="kv"><span>${k}</span><b>${v}</b></div>`; }
views.telemetry = {
  init() {},
  data() { this.lastI = -1; },
  frame(s, now) {
    if (s.i === this.lastI || now - (this.lastT || 0) < 250) return;
    this.lastI = s.i; this.lastT = now;
    const { c, m } = D, i = s.i;
    let h = '';
    h += `<h3>Time</h3>` + kv('UTC', utc(c.t[i])) + kv('T+', `${fmt(c.t[i] / 3600, 3)} h`);
    h += `<h3>Mode</h3>` + kv('mode', `<span style="color:${D.colors[i]}">${modeLabel(i)}</span>`) + kv('pointing error', `${fmt(c.perr[i], 2)}°`)
      + (m.launch ? kv('flight phase', PHASE_NAME[c.phase[i]]) : '');
    h += `<h3>Orbit</h3>` + kv('altitude', `${fmt(c.alt[i], 2)} km`) + kv('lat / lon', `${fmt(c.lat[i], 2)}°, ${fmt(c.lon[i], 2)}°`)
      + kv('beta angle', `${fmt(c.beta[i], 2)}°`) + kv('illumination', c.sunfrac[i] >= 0.999 ? 'sunlit' : c.sunfrac[i] <= 0.001 ? '<span style="color:#88f">ECLIPSE</span>' : `penumbra ${fmt(c.sunfrac[i] * 100, 0)}%`)
      + kv('Sun elevation at nadir', `${fmt(c.sun_el_nadir[i], 1)}°`);
    h += `<h3>Power</h3>` + kv('solar available', `${fmt(c.p_solar[i], 2)} W`) + kv('  of which albedo', `${fmt(c.p_albedo[i], 2)} W`)
      + kv('load', `${fmt(c.p_load[i], 2)} W`) + kv('net', `${fmt(c.p_net[i], 2)} W`) + kv('curtailed', `${fmt(c.p_curtail[i], 2)} W`)
      + kv('battery', `${fmt(c.soc[i] * 100, 1)} % (${fmt(c.battery_wh[i], 1)} Wh)`) + `<div class="bar"><i style="width:${c.soc[i] * 100}%"></i></div>`;
    h += `<h3>Loads (W)</h3>` + kv('OBC+TTC SOM (ADRV9364-Z7020)', fmt(c.load_obc[i], 2)) + kv('ADCS', fmt(c.load_adcs[i], 3))
      + kv('S-band PA + UHF beacon', fmt(c.load_ttc[i], 2)) + kv('payload', fmt(c.load_imager[i], 2))
      + kv('EPS + housekeeping', fmt(c.load_eps[i], 2)) + kv('heater', fmt(c.load_heater[i], 2));
    h += `<h3>Solar array (incidence / lit / W)</h3>`;
    m.faces.forEach((f, k) => { h += kv(f.name.replace(' panel', ''), `${fmt(c[`face_inc_${k}`][i], 0)}° / ${fmt(c[`face_lit_${k}`][i] * 100, 0)}% / ${fmt(c[`face_p_${k}`][i], 2)}`); });
    h += kv('panel temps °C', [0, 1, 2, 3].map(p => fmt(c[`panel_temp_${p}`][i], 0)).join(' / '));
    h += `<h3>ADCS</h3>` + kv('body rate', `${fmt(c.wx[i], 2)}, ${fmt(c.wy[i], 2)}, ${fmt(c.wz[i], 2)} °/s`) + kv('|ω|', `${fmt(c.wmag[i], 3)} °/s`)
      + kv('|B|', `${fmt(c.bmag[i], 1)} µT`) + kv('MTQ dipole', `${fmt(c.mtq_x[i], 3)}, ${fmt(c.mtq_y[i], 3)}, ${fmt(c.mtq_z[i], 3)} Am²`)
      + kv('MTQ power', `${fmt(c.mtq_w[i], 3)} W`) + kv('wheel rpm', `${fmt(c.rpm_x[i], 0)}, ${fmt(c.rpm_y[i], 0)}, ${fmt(c.rpm_z[i], 0)}`)
      + kv('wheel |h| / rated', `${fmt(Math.max(Math.abs(c.hw_x[i]), Math.abs(c.hw_y[i]), Math.abs(c.hw_z[i])), 2)} / ${fmt(m.wheel?.h_rated_mnms, 2)} mNms`)
      + kv('wheel power', `${fmt(c.wheel_w[i], 2)} W`) + kv('τ grav-grad / aero', `${fmt(c.tau_gg[i], 3)} / ${fmt(c.tau_aero[i], 3)} µNm`)
      + kv('Sun ∠ sun axis', `${fmt(c.sun_axis_angle[i], 1)}°`) + kv('GS ∠ antenna', `${fmt(c.antenna_gs_angle[i], 1)}°`) + kv('nadir ∠ imager', `${fmt(c.imager_nadir_angle[i], 1)}°`);
    h += `<h3>Link — ${m.gs.name}</h3>` + kv('elevation / azimuth', `${fmt(c.gs_el[i], 1)}° / ${fmt(c.gs_az[i], 0)}°`) + kv('range', `${fmt(c.gs_range[i], 0)} km`)
      + kv('C/N0', c.cn0[i] > -90 ? `${fmt(c.cn0[i], 1)} dBHz` : '—') + kv('rate / margin', c.link_rate[i] > 0 ? `${fmt(c.link_rate[i], 2)} Mbps / ${fmt(c.link_margin[i], 1)} dB` : '—');
    h += `<h3>Data</h3>` + kv('camera flash', `${fmt(c.camera_gb[i], 3)} GB`) + kv('OBC storage', `${fmt(c.obc_gb[i], 3)} GB`) + kv('downlinked', `${fmt(c.downlinked_gb[i], 3)} GB`)
      + kv('imaged', `${fmt(c.imaged_gb[i], 3)} GB`) + kv('imager', ['off', 'warm-up / idle', '<span style="color:#00ff66">IMAGING</span>', 'readout → OBC'][c.imager[i]]);
    $('telemetry').innerHTML = h;
  },
};

// ---- report (static per run)
views.report = {
  init() {},
  data() {
    const { m } = D;
    $('tasks').innerHTML = `<table><tr><th>type</th><th>target</th><th>start UTC</th><th>window</th><th></th><th>status</th><th>data</th><th>note</th></tr>` +
      m.tasks.map(t => {
        const col = t.type === 'imaging' ? '#00ff66' : 'magenta';
        const st = t.status === 'done' ? t.status : `<span style="color:#ff8c00">${t.status}</span>`;
        const extra = t.type === 'imaging' ? `roll ${fmt(t.roll, 1)}°` : `max el ${fmt(t.max_el, 1)}°`;
        return `<tr data-t="${t.t0}"><td style="color:${col}">${t.type}</td><td>${t.label}</td><td>${utc(t.t0)}</td><td>${(t.t1 - t.t0 + 1) | 0} s</td><td>${extra}</td><td>${st}</td><td>${fmt(t.gb, 3)} GB</td><td>${t.reason || ''}</td></tr>`;
      }).join('') + '</table>';
    const keep = new Set(['DEPLOY', 'ADCS', 'TTC', 'WHEEL', 'SAFE', 'SKIP', 'TASK', 'AOS', 'LOS', 'IMG']);
    $('events').innerHTML = `<table><tr><th>UTC</th><th>kind</th><th>event</th></tr>` +
      m.events.filter(e => keep.has(e[1])).map(e => `<tr data-t="${e[0]}"><td>${utc(e[0])}</td><td>${e[1]}</td><td>${e[2]}</td></tr>`).join('') + '</table>';
    document.querySelectorAll('#tab-report tr[data-t]').forEach(tr => tr.onclick = () => seek(Math.max(0, +tr.dataset.t - 60)));
    $('console').textContent = D.sim.console || D.m.console || '';
  },
  frame() {},
};

// ---- setup tab: summary cards
function buildSummary() {
  const s = D.m.summary, m = D.m;
  const stat = (k, v) => `<div class="stat"><span>${k}</span><b>${v}</b></div>`;
  $('summary').innerHTML = [
    stat('run', `${fmt(m.duration_s / 3600, 1)} h from ${m.epoch_utc.replace('T', ' ')}`),
    stat('orbit', `${fmt(m.altitude_km, 0)} km · i ${fmt(m.inclination_deg, 2)}° · LTAN ${fmt(m.ltan_h, 2)} h`),
    stat('mass', `${fmt(s.mass_kg, 2)} kg`),
    stat('array available / used', `${fmt(s.avg_avail_w, 2)} / ${fmt(s.avg_gen_w, 2)} W`),
    stat('mean load', `${fmt(s.avg_load_w, 2)} W`),
    stat('battery minimum', `${fmt(s.soc_min * 100, 1)} %`),
    stat('eclipse', `${fmt(s.eclipse_frac * 100, 1)} % of the run`),
    stat('imaged / downlinked', `${fmt(s.imaged_gb, 2)} / ${fmt(s.downlinked_gb, 2)} GB`),
    ...(m.launch ? [stat('detumble', s.t_det_end > 0 ? `T+${fmt(s.t_det_start / 60, 0)} → T+${fmt(s.t_det_end / 60, 0)} min` : 'not complete')] : []),
    stat('peak wheel momentum', `${fmt(s.hw_max_mnms, 2)} / ${fmt(m.wheel?.h_rated_mnms, 2)} mNms`),
    stat('engine time', `${fmt(m.wall_s, 2)} s`),
  ].join('');
}

// ================================================================ tabs
let active = null, dataGen = 0;
function showTab(name) {
  if (!views[name] && name !== 'setup') name = 'orbit';
  active = name;
  document.querySelectorAll('#tabs [data-tab]').forEach(b => b.setAttribute('aria-selected', b.dataset.tab === name));
  document.querySelectorAll('main .tab').forEach(s => s.classList.toggle('active', s.id === 'tab-' + name));
  const v = views[name];
  if (v) {
    if (!v.inited) { v.init(); v.inited = true; }
    if (D && v.dataGen !== dataGen) { v.data(); v.dataGen = dataGen; }
    v.lastI = -1; v.lastKey = null;
    if (name === 'ground') requestAnimationFrame(() => { Plotly.Plots.resize('gt'); views.ground.lastKey = null; });
    if (name === 'plots') requestAnimationFrame(() => {
      for (const k in views.plots.cards) if (views.plots.cards[k].built) Plotly.Plots.resize(views.plots.cards[k].pl);
      views.plots.lastI = -1;
    });
  }
  if (!SOLO) { try { history.replaceState(null, '', '#' + name); } catch (e) { /* ignore */ } }
}

// =============================================================== engine
function load(sim, consoleText) {
  if (consoleText) sim.console = consoleText;
  window.__SIM = sim;
  D = prep(sim);
  dataGen++;
  simT = Math.min(simT, D.tmax);
  buildLegend();
  if (!SOLO) buildSummary();
  const v = views[active];
  if (v && v.inited) { v.data(); v.dataGen = dataGen; v.lastI = -1; v.lastKey = null; }
  if (!SOLO) bc?.postMessage({ type: 'sim', src: WIN_ID });
  lastUi = 0;
}
function buildLegend() {
  const items = D.m.modes.map((n, k) => [n, MODE_COLORS[k]]).concat([['SLEWING', SLEW_COLOR], ['eclipse (plots)', '#5a5aa0']]);
  $('legend').innerHTML = items.map(([t, c]) => `<span style="--c:${c}">${t}</span>`).join('');
}
function setStatus(msg, err = false) { $('status').textContent = msg; $('status').className = err ? 'err' : ''; }

async function runEngine() {
  cfgText = configFromInputs($('adv').value || cfgText);
  $('adv').value = cfgText;
  if (!serverMode) { setStatus('Opened as a file: start the engine with  ./build/cubesat_sim --serve  and open http://127.0.0.1:8080', true); return; }
  $('run').disabled = true;
  setStatus('Running the C++ engine…');
  const t0 = performance.now();
  try {
    const r = await fetch('api/run', { method: 'POST', body: cfgText });
    const j = await r.json();
    if (!j.ok) throw new Error(j.error);
    load(j.sim, j.console);
    setStatus(`Done: ${D.n} samples · engine ${fmt(j.sim.meta.wall_s, 2)} s · total ${fmt((performance.now() - t0) / 1000, 2)} s`);
  } catch (e) {
    console.error(e && e.stack ? e.stack : e);
    setStatus('Error: ' + (e && e.message ? e.message : String(e)), true);
  } finally {
    $('run').disabled = false;
  }
}

// =============================================================== player
let lastUi = 0, lastBc = 0, following = 0;
function seek(t, fromRemote = false) {
  if (!D) return;
  simT = Math.max(0, Math.min(D.tmax, t));
  lastUi = 0;
  if (!fromRemote) broadcastTime(true);
}
function setPlaying(p) {
  playing = p;
  $('animate').innerHTML = p ? '&#10074;&#10074; Pause' : '&#9654; Play';
  if (p && D && simT >= D.tmax) simT = 0;
  broadcastTime(true);
}
function broadcastTime(force) {
  const now = performance.now();
  if (!bc || (!force && now - lastBc < 50)) return;
  lastBc = now;
  bc.postMessage({ type: 't', src: WIN_ID, t: simT, playing, speed });
}
function jump(kind) {
  if (!D) return;
  const { c, m } = D;
  let t = null;
  if (kind === 'pass' || kind === 'imaging') {
    const want = kind === 'pass' ? 'downlink' : 'imaging';
    const tk = m.tasks.find(x => x.type === want && x.t0 - 90 > simT + 1);
    if (tk) t = tk.t0 - 90;
  } else if (kind === 'phase') {
    for (let i = Math.floor(simT / D.dt) + 1; i < D.n; i++)
      if ((c.phase[i] !== c.phase[i - 1] || (m.launch && c.t[i] >= m.t_deploy && c.t[i - 1] < m.t_deploy)) && c.t[i] - 30 > simT + 1) { t = c.t[i] - 30; break; }
  } else {
    for (let i = Math.floor(simT / D.dt) + 1; i < D.n; i++)
      if (c.sunfrac[i] < 0.5 && c.sunfrac[i - 1] >= 0.5 && c.t[i] - 60 > simT + 1) { t = c.t[i] - 60; break; }
  }
  if (t != null) seek(t);
}

let lastFrame = performance.now();
function frame(now) {
  const dtw = Math.min(0.1, (now - lastFrame) / 1000);
  lastFrame = now;
  if (D) {
    if (playing && now - following > 300) {
      simT += dtw * speed;
      if (simT >= D.tmax) { simT = D.tmax; setPlaying(false); }
      broadcastTime(false);
    }
    const s = sampleAt(simT / D.dt);
    const v = views[active];
    if (v && v.inited) v.frame(s, now);
    if (now - lastUi > 100) {
      lastUi = now;
      $('clock').textContent = `${utc(D.c.t[s.i])} UTC  T+${fmt(D.c.t[s.i] / 3600, 2)} h`;
      $('slider').value = Math.round(simT / D.tmax * 10000);
      const mb = $('modebadge');
      mb.textContent = modeLabel(s.i);
      mb.style.color = D.colors[s.i];
      mb.style.borderColor = D.colors[s.i];
    }
  }
  requestAnimationFrame(frame);
}

// =============================================================== startup
async function init() {
  if (SOLO) { document.body.classList.add('solo'); document.title = `CubeSat — ${SOLO}`; }
  buildInputs();
  try {
    const r = await fetch('api/config', { cache: 'no-store' });
    if (r.ok) { cfgText = await r.text(); serverMode = true; }
  } catch (e) { /* file:// or static server */ }
  if (!serverMode && window.SIM) cfgText = window.SIM.meta.config || '';
  cfgDefault = cfgText;
  $('adv').value = cfgText;
  inputsFromConfig(cfgText);
  if (!serverMode) setStatus('Viewing saved run (sim_data.js). For live runs: ./build/cubesat_sim --serve  →  http://127.0.0.1:8080');

  document.querySelectorAll('#tabs [data-tab]').forEach(b => b.onclick = () => showTab(b.dataset.tab));
  $('popout').onclick = () => window.open(`index.html?view=${active}`, `cubesat_${active}`, 'width=1100,height=760');

  $('run').onclick = runEngine;
  const setFlight = start => {
    cfgText = iniSet(configFromInputs($('adv').value || cfgText), 'flightplan', 'start', start);
    $('adv').value = cfgText;
    document.querySelectorAll('[data-flight]').forEach(b => b.classList.toggle('on', b.dataset.flight === start));
  };
  document.querySelectorAll('[data-flight]').forEach(b => b.onclick = () => { setFlight(b.dataset.flight); simT = 0; runEngine(); });
  const fs0 = iniGet(cfgText, 'flightplan', 'start') || 'mission';
  document.querySelectorAll('[data-flight]').forEach(b => b.classList.toggle('on', b.dataset.flight === fs0));
  $('reset').onclick = () => { cfgText = cfgDefault; $('adv').value = cfgText; inputsFromConfig(cfgText); setStatus('Inputs reset to the config file'); };
  $('advbtn').onclick = () => { $('adv').value = configFromInputs($('adv').value || cfgText); $('adv').hidden = !$('adv').hidden; };
  $('adv').onchange = () => { cfgText = $('adv').value; inputsFromConfig(cfgText); };
  $('download').onclick = () => {
    const text = configFromInputs($('adv').value || cfgText);
    const a = Object.assign(document.createElement('a'), { href: URL.createObjectURL(new Blob([text], { type: 'text/plain' })), download: 'sim.cfg' });
    a.click();
    URL.revokeObjectURL(a.href);
  };
  const sso = fieldEls.find(f => f.key === 'orbit.sun_synchronous');
  const inc = fieldEls.find(f => f.key === 'orbit.inclination_deg');
  $(sso.id).onchange = () => { $(inc.id).disabled = $(sso.id).checked; };

  $('animate').onclick = () => setPlaying(!playing);
  $('speed').onchange = e => { speed = +e.target.value; broadcastTime(true); };
  $('slider').oninput = e => { if (D) seek(e.target.value / 10000 * D.tmax); };
  document.querySelectorAll('[data-jump]').forEach(b => b.onclick = () => jump(b.dataset.jump));
  document.addEventListener('keydown', e => {
    if (['INPUT', 'TEXTAREA', 'SELECT'].includes(e.target.tagName)) return;
    if (e.code === 'Space') { e.preventDefault(); setPlaying(!playing); }
    if (e.code === 'ArrowRight' && D) seek(simT + 60);
    if (e.code === 'ArrowLeft' && D) seek(simT - 60);
  });

  // cross-window sync: every window follows whichever one last moved the clock
  if (bc) bc.onmessage = e => {
    const msg = e.data;
    if (msg.src === WIN_ID) return;
    if (msg.type === 't' && D) {
      following = performance.now();
      simT = Math.max(0, Math.min(D.tmax, msg.t));
      if (msg.playing !== playing) { playing = msg.playing; $('animate').innerHTML = playing ? '&#10074;&#10074; Pause' : '&#9654; Play'; }
      speed = msg.speed; $('speed').value = String(speed);
    }
    if (msg.type === 'sim' && SOLO && window.opener?.__SIM) load(window.opener.__SIM);
  };

  const src = (SOLO && window.opener?.__SIM) || window.SIM;
  showTab(SOLO || location.hash.slice(1) || 'orbit');
  if (src) load(src, src.meta.console || '');
  else if (serverMode) runEngine();
  requestAnimationFrame(frame);
}
// debugging / benchmarking handle
window.__cubesat = { views, sampleAt: fi => sampleAt(fi), get D() { return D; } };
init();
