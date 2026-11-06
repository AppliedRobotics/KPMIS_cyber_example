// Робототехнический контроллер «Кибернетический» (Ardu8)
// Страница пульта управления платформой 4WD — отдаётся ESP32 по адресу http://192.168.4.1
#pragma once

const char PAGE_HTML[] PROGMEM = R"rawliteral(<!doctype html>
<html lang="ru">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1, user-scalable=no">
<title>Пульт платформы 4WD</title>
<style>
:root{--bg:#0E1014;--card:#171A20;--line:#2A2F38;--text:#ECEAE5;--dim:#A7ABB3;--blue:#5AAEE8;--orange:#F59A4A;--violet:#B197FC;--ok:#4CC38A;--bad:#F2605A;--warn:#F2C94C}
*{box-sizing:border-box}
html,body{margin:0;background:var(--bg);color:var(--text);font-family:system-ui,-apple-system,"Segoe UI",Roboto,sans-serif;-webkit-tap-highlight-color:transparent}
header{display:flex;flex-wrap:wrap;justify-content:space-between;align-items:center;gap:8px 16px;padding:14px 16px;border-bottom:1px solid var(--line)}
h1{margin:0;font-size:20px;line-height:26px}
h1 small{display:block;font-size:13px;font-weight:400;color:var(--dim)}
h2{margin:0 0 12px;font-size:13px;letter-spacing:1.2px;text-transform:uppercase;color:var(--dim)}
.pill{display:inline-flex;align-items:center;gap:8px;padding:6px 12px;border:1px solid var(--line);border-radius:999px;font-size:14px}
.dot{width:10px;height:10px;border-radius:5px;background:#555;flex:none}
.dot.ok{background:var(--ok)}.dot.bad{background:var(--bad)}.dot.warn{background:var(--warn)}
main{display:grid;grid-template-columns:repeat(auto-fit,minmax(300px,1fr));gap:16px;padding:16px;max-width:1200px;margin:0 auto}
.card{background:var(--card);border:1px solid var(--line);border-radius:14px;padding:16px}
.pad{display:grid;grid-template-columns:repeat(3,1fr);gap:10px;max-width:340px;margin:0 auto}
.pad button,.rot button,.btn{appearance:none;border:1px solid var(--line);background:#20242C;color:var(--text);border-radius:12px;font-size:28px;min-height:72px;touch-action:none;user-select:none;-webkit-user-select:none;cursor:pointer}
.pad button.on,.rot button.on{background:var(--blue);color:#04121F;border-color:var(--blue)}
.pad .stop{background:#3A1C1C;border-color:#6B2E2E;font-size:18px;font-weight:600}
.rot{display:grid;grid-template-columns:1fr 1fr;gap:10px;max-width:340px;margin:10px auto 0}
.rot button{font-size:18px;min-height:56px}
.row{display:flex;align-items:center;gap:12px;margin-top:14px;font-size:15px}
.row input[type=range]{flex:1;accent-color:var(--blue);height:28px}
.btn{font-size:16px;min-height:44px;padding:0 16px}
.hint{color:var(--dim);font-size:13px;margin:12px 0 0}
.alert{margin-top:12px;padding:10px 12px;border-radius:10px;background:#3A1C1C;border:1px solid #6B2E2E;font-size:14px;display:none}
.mods{display:flex;flex-direction:column;gap:10px}
.mod{display:flex;align-items:center;gap:10px;font-size:15px}
.mod .v{margin-left:auto;color:var(--dim);font-size:14px;text-align:right}
.lines{display:grid;grid-template-columns:repeat(4,1fr);gap:8px}
.cell{border:1px solid var(--line);border-radius:10px;padding:8px 4px;text-align:center;font-size:12px;color:var(--dim)}
.cell .sq{height:34px;border-radius:6px;margin-bottom:6px;background:#F1EFEA;border:1px solid #888}
.cell.black .sq{background:#111;border-color:#666}
.dist{display:flex;align-items:baseline;gap:8px;margin-top:16px}
.dist b{font-size:32px;font-variant-numeric:tabular-nums}
.bar{height:10px;border-radius:5px;background:#252A33;overflow:hidden;margin-top:6px}
.bar i{display:block;height:100%;width:0;background:var(--ok);transition:width .15s}
.chips{display:flex;flex-wrap:wrap;gap:8px;margin-top:16px}
.chip{display:inline-flex;align-items:center;gap:8px;padding:6px 10px;border:1px solid var(--line);border-radius:8px;font-size:14px}
.chip.on{border-color:var(--warn);color:var(--warn)}
.wheels{display:grid;grid-template-columns:1fr 1fr;gap:10px}
.wh{border:1px solid var(--line);border-radius:10px;padding:10px}
.wh .n{font-size:14px;font-weight:600}
.wh .m{font-size:13px;color:var(--dim);margin-top:6px;font-variant-numeric:tabular-nums}
.pbar{position:relative;height:10px;border-radius:5px;background:#252A33;margin-top:8px;overflow:hidden}
.pbar i{position:absolute;top:0;bottom:0;left:50%;width:0;background:var(--blue)}
.pbar::after{content:"";position:absolute;left:50%;top:0;bottom:0;width:1px;background:#555}
.imu h3{margin:14px 0 6px;font-size:14px;font-weight:600}
.imu h3 span{font-weight:400;color:var(--dim);font-size:13px}
.axes{display:grid;grid-template-columns:20px 1fr 76px;gap:6px 10px;align-items:center;font-size:13px}
.axes .val{text-align:right;font-variant-numeric:tabular-nums}
.axes .pbar{margin-top:0}
.axes .pbar i{background:var(--violet)}
.ang{display:grid;grid-template-columns:repeat(3,1fr);gap:8px;margin-top:14px}
.ang div{border:1px solid var(--line);border-radius:10px;padding:8px;text-align:center;font-size:12px;color:var(--dim)}
.ang b{display:block;font-size:20px;color:var(--text);font-variant-numeric:tabular-nums}
</style>
</head>
<body>
<header>
<h1>Платформа 4WD<small>Робототехнический контроллер «Кибернетический»</small></h1>
<span class="pill"><span class="dot" id="connDot"></span><span id="connTxt">Подключение…</span></span>
</header>
<main>

<section class="card">
<h2>Управление</h2>
<div class="pad">
<button data-vx="1" data-vy="1" aria-label="Вперёд-влево">↖</button>
<button data-vx="1" data-vy="0" aria-label="Вперёд">↑</button>
<button data-vx="1" data-vy="-1" aria-label="Вперёд-вправо">↗</button>
<button data-vx="0" data-vy="1" aria-label="Влево">←</button>
<button class="stop" data-vx="0" data-vy="0" aria-label="Стоп">СТОП</button>
<button data-vx="0" data-vy="-1" aria-label="Вправо">→</button>
<button data-vx="-1" data-vy="1" aria-label="Назад-влево">↙</button>
<button data-vx="-1" data-vy="0" aria-label="Назад">↓</button>
<button data-vx="-1" data-vy="-1" aria-label="Назад-вправо">↘</button>
</div>
<div class="rot">
<button data-w="1">⟲ Поворот</button>
<button data-w="-1">Поворот ⟳</button>
</div>
<div class="row"><label for="spd">Скорость</label><input type="range" id="spd" min="20" max="100" step="5" value="50"><b id="spdV">50%</b></div>
<div class="row"><input type="checkbox" id="guard" checked><label for="guard">Не ехать вперёд, если препятствие ближе 15 см</label></div>
<div class="row"><button class="btn" id="beep">Сигнал</button></div>
<div class="alert" id="alert"></div>
<p class="hint">Удерживайте кнопку, чтобы ехать. Клавиатура: W A S D — движение, Q E — поворот, пробел — стоп.</p>
</section>

<section class="card">
<h2>Колёса</h2>
<div class="wheels">
<div class="wh"><div class="n">Переднее левое</div><div class="pbar"><i id="p0"></i></div><div class="m" id="m0">—</div></div>
<div class="wh"><div class="n">Переднее правое</div><div class="pbar"><i id="p1"></i></div><div class="m" id="m1">—</div></div>
<div class="wh"><div class="n">Заднее левое</div><div class="pbar"><i id="p2"></i></div><div class="m" id="m2">—</div></div>
<div class="wh"><div class="n">Заднее правое</div><div class="pbar"><i id="p3"></i></div><div class="m" id="m3">—</div></div>
</div>
<p class="hint">Полоса — мощность (вправо — вперёд). Скорость в тиках энкодера за 100 мс.</p>
</section>

<section class="card">
<h2>Модули</h2>
<div class="mods">
<div class="mod"><span class="dot" id="dArd"></span>Arduino · датчики<span class="v" id="vArd">—</span></div>
<div class="mod"><span class="dot" id="dImu"></span>IMU ICM-20948 · на плате<span class="v" id="vImu">—</span></div>
<div class="mod"><span class="dot" id="d72"></span>Модуль моторов ID 72 · левый борт<span class="v" id="v72">—</span></div>
<div class="mod"><span class="dot" id="d71"></span>Модуль моторов ID 71 · правый борт<span class="v" id="v71">—</span></div>
<div class="mod"><span class="dot ok"></span>Wi-Fi ESP32<span class="v" id="vCl">—</span></div>
<div class="mod"><span class="dot" id="dUp"></span>ESP32 работает<span class="v" id="vUp">—</span></div>
<div class="mod"><span class="dot ok"></span>Последний запуск<span class="v" id="vRst">—</span></div>
<div class="mod"><span class="dot ok"></span>Запусков всего<span class="v" id="vBoots">—</span></div>
</div>
</section>

<section class="card">
<h2>Датчики</h2>
<div class="lines">
<div class="cell" id="l0"><div class="sq"></div>Л внеш.</div>
<div class="cell" id="l1"><div class="sq"></div>Л внутр.</div>
<div class="cell" id="l2"><div class="sq"></div>П внутр.</div>
<div class="cell" id="l3"><div class="sq"></div>П внеш.</div>
</div>
<div class="dist"><b id="dist">—</b><span>см · ультразвуковой дальномер</span></div>
<div class="bar"><i id="distBar"></i></div>
<div class="chips">
<span class="chip" id="endL"><span class="dot"></span>Концевой левый</span>
<span class="chip" id="endR"><span class="dot"></span>Концевой правый</span>
</div>
<div class="chips" id="abtn">
<span class="chip"><span class="dot"></span>Кн. 1</span>
<span class="chip"><span class="dot"></span>Кн. 2</span>
<span class="chip"><span class="dot"></span>Кн. 3 · стоп</span>
<span class="chip"><span class="dot"></span>Кн. 4</span>
</div>
</section>


<section class="card imu">
<h2>IMU ICM-20948</h2>
<h3>Акселерометр <span>· g</span></h3>
<div class="axes" id="accAx"></div>
<h3>Гироскоп <span>· °/с</span></h3>
<div class="axes" id="gyrAx"></div>
<h3>Магнитометр <span>· мкТл</span></h3>
<div class="axes" id="magAx"></div>
<div class="ang">
<div><b id="roll">—</b>Крен, °</div>
<div><b id="pitch">—</b>Тангаж, °</div>
<div><b id="head">—</b>Курс, °</div>
</div>
<p class="hint">Оси — как у датчика на плате. Крен и тангаж — по акселерометру, курс — по магнитометру без калибровки и учёта наклона.</p>
</section>

</main>
<script>
const $ = id => document.getElementById(id);
let padCmd = {vx: 0, vy: 0, w: 0};
let lastUp = null, restarts = 0;
const keys = new Set();

document.querySelectorAll('.pad button, .rot button').forEach(b => {
  const release = () => { padCmd = {vx: 0, vy: 0, w: 0}; b.classList.remove('on'); };
  b.addEventListener('pointerdown', e => {
    e.preventDefault();
    b.setPointerCapture(e.pointerId);
    padCmd = {vx: +(b.dataset.vx || 0), vy: +(b.dataset.vy || 0), w: +(b.dataset.w || 0)};
    if (!b.classList.contains('stop')) b.classList.add('on');
  });
  ['pointerup', 'pointercancel', 'lostpointercapture'].forEach(ev => b.addEventListener(ev, release));
  b.addEventListener('contextmenu', e => e.preventDefault());
});

const KEYMAP = ['w', 'a', 's', 'd', 'q', 'e', ' ', 'ц', 'ф', 'ы', 'в', 'й', 'у'];
const RU = {'ц': 'w', 'ф': 'a', 'ы': 's', 'в': 'd', 'й': 'q', 'у': 'e'};
addEventListener('keydown', e => { const k = e.key.toLowerCase(); if (KEYMAP.includes(k)) { keys.add(RU[k] || k); e.preventDefault(); } });
addEventListener('keyup', e => { const k = e.key.toLowerCase(); keys.delete(RU[k] || k); });
addEventListener('blur', () => keys.clear());

$('spd').addEventListener('input', () => $('spdV').textContent = $('spd').value + '%');
$('beep').addEventListener('click', () => fetch('/beep').catch(() => {}));

function current() {
  if (keys.size) {
    if (keys.has(' ')) return {vx: 0, vy: 0, w: 0};
    return {
      vx: (keys.has('w') ? 1 : 0) - (keys.has('s') ? 1 : 0),
      vy: (keys.has('a') ? 1 : 0) - (keys.has('d') ? 1 : 0),
      w:  (keys.has('q') ? 1 : 0) - (keys.has('e') ? 1 : 0)
    };
  }
  return padCmd;
}

// Строки X / Y / Z для блоков IMU
const AX = {};
['accAx', 'gyrAx', 'magAx'].forEach(id => {
  AX[id] = ['X', 'Y', 'Z'].map(n => {
    const l = document.createElement('span'); l.textContent = n;
    const p = document.createElement('div'); p.className = 'pbar';
    const i = document.createElement('i'); p.appendChild(i);
    const v = document.createElement('span'); v.className = 'val'; v.textContent = '—';
    $(id).append(l, p, v);
    return {i, v};
  });
});

function axes(id, vals, scale, range, digits, ok) {
  AX[id].forEach((a, k) => {
    const x = vals[k] / scale;
    a.v.textContent = ok ? x.toFixed(digits) : '—';
    const w = ok ? Math.min(50, Math.abs(x) / range * 50) : 0;
    a.i.style.width = w + '%';
    a.i.style.left = x >= 0 ? '50%' : (50 - w) + '%';
  });
}

function renderImu(s) {
  const lvl = s.ard ? s.imu : 0;
  $('dImu').className = 'dot ' + (lvl === 2 ? 'ok' : lvl === 1 ? 'warn' : 'bad');
  $('vImu').textContent = lvl === 2 ? 'на связи' : lvl === 1 ? 'без магнитометра' : 'не найден';
  axes('accAx', s.acc, 1000, 2, 2, lvl > 0);
  axes('gyrAx', s.gyr, 10, 250, 1, lvl > 0);
  axes('magAx', s.mag, 10, 100, 1, lvl === 2);
  if (lvl > 0) {
    const [ax, ay, az] = s.acc;
    $('roll').textContent = Math.round(Math.atan2(ay, az) * 180 / Math.PI);
    $('pitch').textContent = Math.round(Math.atan2(-ax, Math.hypot(ay, az)) * 180 / Math.PI);
  } else { $('roll').textContent = '—'; $('pitch').textContent = '—'; }
  if (lvl === 2) {
    let h = Math.atan2(s.mag[1], s.mag[0]) * 180 / Math.PI;
    if (h < 0) h += 360;
    $('head').textContent = Math.round(h);
  } else $('head').textContent = '—';
}

function setConn(ok) {
  $('connDot').className = 'dot ' + (ok ? 'ok' : 'bad');
  $('connTxt').textContent = ok ? 'Платформа на связи' : 'Нет связи с платформой';
}

function modRow(dot, val, ok, okText, badText) {
  $(dot).className = 'dot ' + (ok ? 'ok' : 'bad');
  $(val).textContent = ok ? okText : badText;
}

function render(s) {
  modRow('dArd', 'vArd', s.ard, 'на связи', 'нет данных — DIP2, скетч датчиков');
  modRow('d72', 'v72', s.m72, 'на связи', 'не отвечает — DIP3, питание');
  modRow('d71', 'v71', s.m71, 'на связи', 'не отвечает — DIP3, питание');
  $('vCl').textContent = 'клиентов: ' + s.cl;
  const up = s.up, mm = Math.floor(up / 60), ss = up % 60;
  $('vUp').textContent = (mm ? mm + ' мин ' : '') + ss + ' с';
  if (lastUp !== null && up < lastUp) restarts++;   // время сбросилось — ESP32 перезапустился
  lastUp = up;
  $('dUp').className = 'dot ' + (restarts ? 'warn' : 'ok');
  if (restarts) $('vUp').textContent += ' · перезапусков за сеанс: ' + restarts;
  $('vRst').textContent = s.rst;
  $('vBoots').textContent = s.boots;

  for (let i = 0; i < 4; i++) $('l' + i).classList.toggle('black', s.ard && s.line[i] === 1);

  const d = s.ard ? s.dist : -1;
  $('dist').textContent = d < 0 ? '—' : d;
  const bar = $('distBar');
  bar.style.width = d < 0 ? '0' : Math.min(100, d / 2) + '%';
  bar.style.background = d < 15 ? 'var(--bad)' : d < 40 ? 'var(--warn)' : 'var(--ok)';

  [['endL', s.endL], ['endR', s.endR]].forEach(([id, v]) => {
    const on = s.ard && v === 1;
    $(id).classList.toggle('on', on);
    $(id).querySelector('.dot').className = 'dot' + (on ? ' warn' : '');
  });
  $('abtn').querySelectorAll('.chip').forEach((c, i) => {
    const on = s.ard && (s.abtn >> i) & 1;
    c.classList.toggle('on', !!on);
    c.querySelector('.dot').className = 'dot' + (on ? ' warn' : '');
  });

  for (let i = 0; i < 4; i++) {
    const p = s.pw[i], bar = $('p' + i);
    bar.style.width = Math.abs(p) / 2 + '%';
    bar.style.left = p >= 0 ? '50%' : (50 - Math.abs(p) / 2) + '%';
    $('m' + i).textContent = 'мощн. ' + p + '% · скор. ' + s.spd[i] + ' · энк. ' + s.enc[i];
  }

  renderImu(s);

  const a = $('alert');
  const msg = s.estop ? 'Аварийный стоп: нажата кнопка 3 на Arduino.'
            : s.blk ? 'Препятствие ближе 15 см — движение вперёд заблокировано.' : '';
  a.textContent = msg;
  a.style.display = msg ? 'block' : 'none';
}

async function tick() {
  const c = current(), sp = +$('spd').value;
  const q = '/cmd?vx=' + c.vx * sp + '&vy=' + c.vy * sp + '&w=' + c.w * sp + '&g=' + ($('guard').checked ? 1 : 0);
  const ac = new AbortController();
  const to = setTimeout(() => ac.abort(), 800);
  try {
    const r = await fetch(q, {signal: ac.signal, cache: 'no-store'});
    render(await r.json());
    setConn(true);
  } catch (e) {
    setConn(false);
  }
  clearTimeout(to);
  setTimeout(tick, 100);
}
tick();
</script>
</body>
</html>
)rawliteral";
