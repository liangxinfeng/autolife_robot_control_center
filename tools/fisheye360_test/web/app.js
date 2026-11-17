import {groundRay, previewGeometry, previewPoint, projectLens} from './projection.js?v=347-sync-1';
import {createRenderer} from './projection_renderer.js?v=347-sync-1';
import {createStream} from './jpeg_stream.js?v=347-sync-1';

const $ = id => document.getElementById(id);
const cameras = ['front', 'rear'];
const STORE = 'fisheye360-345-park-v3';
const fields = [
  ['cx', '圆心 X / px', 260, 375, 1], ['cy', '圆心 Y / px', 135, 220, 1],
  ['radius', '有效半径 / px', 130, 187, 1], ['focal', '径向焦距 / px', 75, 130, 0.5],
  ['pitch', '向下俯仰 / °', -35, 45, 0.5], ['yaw', '朝向偏差 / °', -40, 40, 0.5],
  ['roll', '画面横滚 / °', -20, 20, 0.5], ['height', '离地高度 / m', 0.8, 1.8, 0.01],
  ['lowerCut', '底部遮挡边界 / px', 80, 185, 1],
];
function drawGuides(cam, map, isRear, geometry, texW, texH, fresh) {
  if (!fresh) { $('guide').replaceChildren(); $('guide-note').hidden = true; return; }
  const range = [0.5, 1, 2];
  const colors = ['#ff7474', '#ffd577', '#90f0b1'];
  const lanes = range.map(d => [-1, 1].map(side => {
    const ray = groundRay([side * (map.bodyHalfWidth + 0.035), 0,
      (map.bodyHalfDepth + d) * (isRear ? -1 : 1)], cam);
    if (!projectLens(ray, cam, texW, texH)) return null;
    const xy = previewPoint(ray, geometry);
    if (!xy || xy.some(t => t < 0.02 || t > 0.98)) return null;
    return [xy[0]*1000, xy[1]*625].map(x => x.toFixed(2));
  }));
  const svg = [];
  function line(a, b, color, width) {
    if (!a || !b) return;
    svg.push(`<path d="M${a.join(' ')} L${b.join(' ')}" fill="none" stroke="#0a1322" stroke-width="${width+4}" stroke-linecap="round"/>`);
    svg.push(`<path d="M${a.join(' ')} L${b.join(' ')}" fill="none" stroke="${color}" stroke-width="${width}" stroke-linecap="round"/>`);
  }
  for (const [i, pair] of lanes.entries()) {
    if (!pair.every(Boolean)) continue;
    line(pair[0], pair[1], colors[i], 4);
    if (i > 0) for (const side of [0,1]) line(lanes[i-1][side], pair[side], colors[i], 5);
    const right = pair.reduce((a,b) => Number(a[0]) > Number(b[0]) ? a : b);
    svg.push(`<text x="${Number(right[0])+15}" y="${Number(right[1])+7}" fill="${colors[i]}" stroke="#0a1322" stroke-width="4" paint-order="stroke" font-size="21" font-weight="700">≈${range[i]} m</text>`);
  }
  $('guide').innerHTML = svg.join('');
  $('guide-note').hidden = !svg.length;
}

function boot(config) {
  const defaults = config.profile;
  if (!defaults || config.width !== 640 || config.height !== 360) throw new Error('服务版本与页面不一致');
  let profile = structuredClone(config.vr_profile || defaults);
  try {
    const stored = JSON.parse(localStorage.getItem(STORE) || 'null');
    if (stored?.revision === defaults.revision) {
      profile = structuredClone(defaults);
      for (const name of cameras) {
      for (const [key,, min,max] of fields) {
        const n = stored[name]?.[key];
        if (typeof n !== 'number' || !Number.isFinite(n)) continue;
        const value = key === 'yaw' ? n-defaults[name].yaw : n;
        if (value >= min && value <= max) profile[name][key] = n;
      }
      }
    }
  } catch { /* Invalid storage: use the robot defaults. */ }
  let mode = 'parking', direction = 'rear', mirror = defaults.preview.mirrorRear, lost = false;
  let dirty = false, running = 0, signature = '', guideKey = '', drawCount = 0;
  const canvas = $('surface'), board = $('board'), renderer = createRenderer(canvas, config);
  function requestDraw() {
    dirty = true;
    if (!running && !document.hidden && !lost) running = requestAnimationFrame(draw);
  }
  const streams = Object.fromEntries(cameras.map(name => [
    name, createStream(config[name].websocket_path, bmp => renderer.upload(name, bmp), requestDraw)
  ]));
  function draw(now) {
    running = 0;
    if (lost || document.hidden || !dirty) return;
    dirty = false;
    const origin = board.getBoundingClientRect();
    if (!origin.width || !origin.height) return;
    // Native inputs are 640×360. Bound fill-rate even on HiDPI/4K screens.
    const scale = Math.min(1, 1500/origin.width, Math.sqrt(980000/(origin.width*origin.height)));
    const w = Math.max(1, Math.round(origin.width*scale)), h = Math.max(1, Math.round(origin.height*scale));
    if (canvas.width !== w || canvas.height !== h) { canvas.width = w; canvas.height = h; }
    const front = streams.front.status(now), rear = streams.rear.status(now);
    const skew = Math.abs(front.epoch-rear.epoch);
    renderer.begin(profile, {front:front.fresh,rear:rear.fresh}, front.fresh && rear.fresh && skew < 135);
    if (mode === 'raw') {
      for (const [i,name] of cameras.entries()) renderer.draw($(`raw-${name}-frame`).getBoundingClientRect(), origin, 2, i === 1);
    } else {
      renderer.draw($('map-frame').getBoundingClientRect(), origin, 0, false);
      if (mode === 'parking') {
        const selected = direction === 'rear' ? rear : front;
        const frame = $('preview-frame'), isRear = direction === 'rear';
        const view = previewGeometry(profile.preview, isRear, mirror, frame.clientWidth/frame.clientHeight);
        renderer.draw(frame.getBoundingClientRect(), origin, 1, isRear, view);
        const key = JSON.stringify([profile[direction], direction, mirror, frame.clientWidth, frame.clientHeight, selected.fresh]);
        if (guideKey !== key) {
          guideKey = key;
          drawGuides(profile[direction], profile.map, isRear, view, config.width, config.height, selected.fresh);
        }
      }
    }
    canvas.dataset.draws = ++drawCount;
  }

  function badge(id, on, text) {
    const el = $(id);
    el.hidden = !on;
    el.textContent = on ? text : '';
  }
  function statusLabel(id, s) {
    const el = $(id);
    el.textContent = s.fresh ? `${s.fps.toFixed(1)} FPS` : '等待画面';
    el.dataset.state = s.fresh ? 'live' : 'offline';
    el.dataset.frames = s.frames;
  }
  function tick() {
    if (lost || document.hidden) return;
    const now = performance.now();
    for (const name of cameras) streams[name].connect(now);
    const front = streams.front.status(now), rear = streams.rear.status(now);
    const key = `${front.fresh}/${rear.fresh}`;
    if (signature !== key) { signature = key; requestDraw(); }
    const current = direction === 'rear' ? rear : front;
    statusLabel('preview-state', current);
    statusLabel('raw-front-state', front);
    statusLabel('raw-rear-state', rear);
    $('map-state').textContent = `前 ${front.fresh ? front.fps.toFixed(1) : '—'} · 后 ${rear.fresh ? rear.fps.toFixed(1) : '—'} FPS`;
    $('map-state').dataset.state = front.fresh && rear.fresh ? 'live' : 'offline';
    badge('preview-loss', !current.fresh, '无实时画面');
    badge('raw-front-loss', !front.fresh, '前摄无实时画面');
    badge('raw-rear-loss', !rear.fresh, '后摄无实时画面');
  }
  function sync() {
    board.dataset.mode = mode;
    $('preview-card').hidden = mode !== 'parking';
    $('map-card').hidden = mode === 'raw';
    for (const name of cameras) $(`raw-${name}-card`).hidden = mode !== 'raw';
    for (const b of $('modes').querySelectorAll('button')) b.setAttribute('aria-pressed', b.dataset.mode === mode);
    $('preview-label').textContent = direction === 'rear' ? '后方 · 去畸变' : '前方 · 去畸变';
    $('switch-dir').textContent = direction === 'rear' ? '切到前视' : '切到后视';
    $('mirror').hidden = direction !== 'rear';
    $('mirror').setAttribute('aria-pressed', mirror);
    tick();
    requestDraw();
  }
  $('modes').onclick = e => { if (e.target.dataset.mode) { mode = e.target.dataset.mode; sync(); } };
  $('switch-dir').onclick = () => { direction = direction === 'rear' ? 'front' : 'rear'; sync(); };
  $('mirror').onclick = () => { mirror = !mirror; sync(); };
  function syncPayload(value) {
    return {revision:value.revision, ...Object.fromEntries(cameras.map(name => [
      name, Object.fromEntries(fields.map(([key]) => [key, value[name][key]]))
    ]))};
  }
  const draft = () => JSON.stringify(syncPayload(profile));
  let synced = config.vr_profile ? JSON.stringify(syncPayload(config.vr_profile)) : '';
  const updateSyncNote = () => {
    $('sync-state').textContent = draft() === synced
      ? '已同步 · 下次启动 VR 时生效'
      : '当前调参未同步到 VR';
  };
  $('sync-vr').onclick = async () => {
    const button = $('sync-vr'), body = draft();
    button.disabled = true;
    $('sync-state').textContent = '正在同步…';
    try {
      const response = await fetch('/api/vr-profile', {
        method:'POST', headers:{'Content-Type':'application/json'}, body,
        signal:AbortSignal.timeout(7000),
      });
      const data = await response.json();
      if (!response.ok || !data.profile) throw new Error(data.error || '保存未成功');
      synced = JSON.stringify(syncPayload(data.profile));
      updateSyncNote();
    } catch (error) {
      $('sync-state').textContent = `同步未确认：${error.message}。请重试。`;
    } finally { button.disabled = false; }
  };
  updateSyncNote();
  function buildFields() {
    const name = $('cam-cal').value, cam = profile[name], base = defaults[name];
    $('fields').replaceChildren(...fields.map(([key,label,min,max,step]) => {
      const root = document.createElement('label'), value = document.createElement('output');
      const input = document.createElement('input');
      const offset = key === 'yaw' ? base.yaw : 0;
      input.type = 'range';
      input.min = min; input.max = max; input.step = step; input.value = cam[key]-offset;
      value.value = Number(input.value);
      input.oninput = () => {
        cam[key] = Number(input.value) + offset;
        value.value = input.value;
        try { localStorage.setItem(STORE, JSON.stringify(profile)); } catch {}
        updateSyncNote();
        requestDraw();
      };
      root.append(label, value, input);
      return root;
    }));
  }
  $('toggle-cal').onclick = () => {
    $('calibration').hidden = !$('calibration').hidden;
    $('toggle-cal').setAttribute('aria-expanded', !$('calibration').hidden);
    if (!$('calibration').hidden) buildFields();
  };
  $('cam-cal').onchange = buildFields;
  $('reset').onclick = () => {
    profile = structuredClone(defaults);
    try { localStorage.setItem(STORE, JSON.stringify(profile)); } catch {}
    updateSyncNote();
    buildFields();
    requestDraw();
  };
  const stop = () => { for (const name of cameras) streams[name].stop(); };
  document.addEventListener('visibilitychange', () => { if (document.hidden) stop(); else sync(); });
  window.addEventListener('pagehide', stop);
  window.addEventListener('pageshow', sync);
  new ResizeObserver(requestDraw).observe(board);
  canvas.addEventListener('webglcontextlost', e => {
    e.preventDefault(); lost = true; stop(); showError('图形上下文已中断，恢复后会重载。');
  });
  canvas.addEventListener('webglcontextrestored', () => location.reload());
  setInterval(tick, 230);
  sync();
}
function showError(message) {
  $('error').hidden = false;
  $('error').textContent = message;
}
fetch('/api/config', {cache:'no-store'}).then(r => {
  if (!r.ok) throw new Error(`配置请求失败 (${r.status})`);
  return r.json();
}).then(boot).catch(error => {
  console.error(error);
  showError(`画面无法启动：${error.message}`);
});
