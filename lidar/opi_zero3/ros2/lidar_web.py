#!/usr/bin/env python3
"""Веб-страница лидара для демонстрации без установки программ.

Откройте в любом браузере (МОС ОС, Windows, планшет), подключившись к Wi-Fi платы:
    http://10.42.0.1:8080

На странице: карта SLAM (/map), текущий скан (/scan), положение и направление
лидара, линия до ближайшего препятствия и расстояния по 8 секторам.
Без SLAM (только драйвер лидара) показывается один скан вокруг лидара.

Запуск: вместе с lidar_slam (slam_launch.py) или отдельно:
    python3 ~/ydlidar_ros2_ws/lidar_web.py
Порт можно поменять: python3 lidar_web.py 8081
Zero 3, 16.11.2026.
"""
import base64
import json
import math
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

PORT = next((int(a) for a in sys.argv[1:] if a.isdigit()), 8080)
# поворот лидара относительно робота, рад (--yaw; у RPLIDAR C1 = pi)
YAW = float(sys.argv[sys.argv.index('--yaw') + 1]) if '--yaw' in sys.argv else 0.0
SECTORS = 8

# ---------------------------------------------------------------- общие данные
lock = threading.Lock()
state = {
    'scan': None,      # {'amin', 'ainc', 'r': [...], 't'}
    'pose': None,      # {'x', 'y', 'yaw', 'frame'} — лидар в системе карты
    'map': None,       # {'w','h','res','ox','oy','png'} — base64 кадров карты
    'map_ver': 0,
    'nearest': None,   # [расстояние_м, угол_град]
    'sectors': [0.0] * SECTORS,
    'off': YAW,
}


def analyse(amin, ainc, ranges, rmin, rmax):
    """Ближайшая точка и минимумы по секторам (сектор 0 — вперёд ±22,5°)."""
    best = [0.0] * SECTORS
    near = None
    for i, r in enumerate(ranges):
        if not (math.isfinite(r) and rmin <= r <= rmax):
            continue
        a = amin + i * ainc + YAW            # угол в системе робота
        if near is None or r < near[0]:
            near = [r, math.degrees(a) % 360]
        k = int(((math.degrees(a) + 22.5) % 360) // 45)
        if best[k] == 0.0 or r < best[k]:
            best[k] = r
    return near, best


def grid_to_b64(w, h, data):
    """Карта -> base64 байтов: 0 = стена, 255 = свободно, 128 = неизвестно."""
    try:
        import numpy as np
        g = np.asarray(data, dtype=np.int16)        # -1 неизвестно, 0..100 занятость
        out = np.full(g.shape, 128, dtype=np.uint8)
        out[g == 0] = 255
        occ = g > 0
        out[occ] = (255 - g[occ] * 255 // 100).astype(np.uint8)
        b = out.tobytes()
    except ImportError:
        b = bytes(128 if v < 0 else (255 - v * 255 // 100) for v in data)
    return base64.b64encode(b).decode('ascii')


# ---------------------------------------------------------------- веб-сервер
PAGE = r"""<!DOCTYPE html>
<html lang="ru"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Лидар — карта и расстояния</title>
<style>
 body{margin:0;font-family:system-ui,"Segoe UI",Arial,sans-serif;background:#16181d;color:#e8e8e8}
 header{padding:8px 14px;background:#22252c;display:flex;gap:18px;align-items:center;flex-wrap:wrap}
 header b{font-size:18px} .st{font-size:14px;color:#aaa}
 .ok{color:#6c6}.bad{color:#e66}
 main{display:flex;gap:10px;padding:10px;height:calc(100vh - 62px);box-sizing:border-box}
 #wrap{flex:1;position:relative;background:#808080;border-radius:6px;overflow:hidden}
 canvas{display:block;width:100%;height:100%}
 aside{width:290px;display:flex;flex-direction:column;gap:10px}
 .box{background:#22252c;border-radius:6px;padding:10px}
 .big{font-size:42px;font-weight:700;color:#ffd84a;text-align:center}
 table{width:100%;border-collapse:collapse;font-size:15px}
 td{padding:3px 4px} td.v{text-align:right;font-weight:600;font-variant-numeric:tabular-nums}
 .bar{height:8px;border-radius:4px;background:#444;margin-top:2px}
 .bar i{display:block;height:100%;border-radius:4px}
 button{background:#343843;color:#eee;border:0;border-radius:4px;padding:6px 10px;cursor:pointer;font-size:14px}
 button.on{background:#5b6cff}
 .hint{font-size:12px;color:#999}
 @media (max-width:800px){main{flex-direction:column;height:auto}#wrap{height:60vh}aside{width:auto}}
</style></head><body>
<header><b>Лидар · Orange Pi Zero 3</b>
 <span class="st" id="st">подключение…</span>
 <span class="st" id="hz"></span>
 <span style="flex:1"></span>
 <button id="bFollow" class="on">Следить за лидаром</button>
 <button id="bFit">Вся карта</button>
</header>
<main>
 <div id="wrap"><canvas id="cv"></canvas></div>
 <aside>
  <div class="box">До ближайшего препятствия
   <div class="big" id="near">—</div>
   <div class="hint" id="nearDir"></div></div>
  <div class="box">Расстояния по направлениям
   <table id="tab"></table></div>
  <div class="box hint">Белое — свободно, чёрное — стены, серое — неизвестно.
   Красные точки — текущий скан. Синяя стрелка — лидар, остриё — «вперёд».
   Колесо мыши — масштаб, перетаскивание — сдвиг.</div>
 </aside>
</main>
<script>
const NAMES=["вперёд","вперёд-влево","влево","назад-влево","назад","назад-вправо","вправо","вперёд-вправо"];
const cv=document.getElementById('cv'),ctx=cv.getContext('2d');
let D=null,MAP=null,mapImg=null,mapVer=-1,follow=true,scale=100,cx=0,cy=0,last=0,frames=[];
const tab=document.getElementById('tab');
tab.innerHTML=NAMES.map((n,i)=>`<tr><td>${n}<div class="bar"><i id="b${i}"></i></div></td><td class="v" id="s${i}">—</td></tr>`).join('');
function col(r){return r<=0?'#555':r<0.3?'#e44':r<0.6?'#f90':r<1.0?'#ec3':'#4c4'}
function resize(){const r=cv.getBoundingClientRect();cv.width=r.width*devicePixelRatio;cv.height=r.height*devicePixelRatio;draw()}
window.addEventListener('resize',resize);
document.getElementById('bFollow').onclick=e=>{follow=!follow;e.target.classList.toggle('on',follow);draw()};
document.getElementById('bFit').onclick=()=>{follow=false;document.getElementById('bFollow').classList.remove('on');fit();draw()};
cv.addEventListener('wheel',e=>{e.preventDefault();scale*=e.deltaY<0?1.15:1/1.15;scale=Math.max(5,Math.min(600,scale));draw()},{passive:false});
let drag=null;
cv.addEventListener('pointerdown',e=>{drag=[e.clientX,e.clientY];cv.setPointerCapture(e.pointerId)});
cv.addEventListener('pointerup',()=>drag=null);
cv.addEventListener('pointermove',e=>{if(!drag)return;const k=devicePixelRatio/scale;
 cx-=(e.clientX-drag[0])*k;cy+=(e.clientY-drag[1])*k;drag=[e.clientX,e.clientY];
 if(follow){follow=false;document.getElementById('bFollow').classList.remove('on')}draw()});
function fit(){if(!MAP)return;const w=MAP.w*MAP.res,h=MAP.h*MAP.res;cx=MAP.ox+w/2;cy=MAP.oy+h/2;
 scale=0.9*Math.min(cv.width/w,cv.height/h)}
function W2S(x,y){return[cv.width/2+(x-cx)*scale,cv.height/2-(y-cy)*scale]}
function buildMap(m){const raw=atob(m.data),c=document.createElement('canvas');c.width=m.w;c.height=m.h;
 const g=c.getContext('2d'),im=g.createImageData(m.w,m.h);
 for(let y=0;y<m.h;y++)for(let x=0;x<m.w;x++){const v=raw.charCodeAt(y*m.w+x),p=((m.h-1-y)*m.w+x)*4;
  im.data[p]=im.data[p+1]=im.data[p+2]=v;im.data[p+3]=255}
 g.putImageData(im,0,0);return c}
function draw(){
 const P=D&&D.pose?D.pose:{x:0,y:0,yaw:0};
 if(follow){cx=P.x;cy=P.y}
 ctx.fillStyle='#808080';ctx.fillRect(0,0,cv.width,cv.height);
 if(mapImg&&MAP&&D&&D.pose&&D.pose.frame==='map'){const[x0,y0]=W2S(MAP.ox,MAP.oy+MAP.h*MAP.res);
  ctx.imageSmoothingEnabled=false;ctx.drawImage(mapImg,x0,y0,MAP.w*MAP.res*scale,MAP.h*MAP.res*scale)}
 // сетка 1 м
 ctx.strokeStyle='rgba(60,120,255,.35)';ctx.lineWidth=1;const step=1,x1=cx-cv.width/2/scale,x2=cx+cv.width/2/scale,y1=cy-cv.height/2/scale,y2=cy+cv.height/2/scale;
 ctx.beginPath();for(let x=Math.floor(x1);x<=x2;x+=step){const[a]=W2S(x,0);ctx.moveTo(a,0);ctx.lineTo(a,cv.height)}
 for(let y=Math.floor(y1);y<=y2;y+=step){const[,b]=W2S(0,y);ctx.moveTo(0,b);ctx.lineTo(cv.width,b)}ctx.stroke();
 if(!D||!D.scan)return;
 const S=D.scan,c=Math.cos(P.yaw),s=Math.sin(P.yaw),dot=Math.max(2,2.5*devicePixelRatio);
 ctx.fillStyle='#e0303a';
 for(let i=0;i<S.r.length;i++){const r=S.r[i];if(!(r>0))continue;const a=S.amin+i*S.ainc,lx=r*Math.cos(a),ly=r*Math.sin(a);
  const[px,py]=W2S(P.x+c*lx-s*ly,P.y+s*lx+c*ly);ctx.fillRect(px-dot/2,py-dot/2,dot,dot)}
 const[rx,ry]=W2S(P.x,P.y);
 if(D.nearest){const r=D.nearest[0],a=D.nearest[1]*Math.PI/180-(D.off||0),lx=r*Math.cos(a),ly=r*Math.sin(a);
  const[nx,ny]=W2S(P.x+c*lx-s*ly,P.y+s*lx+c*ly);
  ctx.strokeStyle='#ffd84a';ctx.lineWidth=2*devicePixelRatio;ctx.beginPath();ctx.moveTo(rx,ry);ctx.lineTo(nx,ny);ctx.stroke();
  ctx.fillStyle='#ffd84a';ctx.beginPath();ctx.arc(nx,ny,5*devicePixelRatio,0,7);ctx.fill();
  ctx.font=`bold ${16*devicePixelRatio}px system-ui`;ctx.fillStyle='#000';ctx.fillText(r.toFixed(2)+' м',nx+9*devicePixelRatio,ny-7*devicePixelRatio)}
 // лидар: стрелка по оси «вперёд»
 const L=14*devicePixelRatio;ctx.save();ctx.translate(rx,ry);ctx.rotate(-(P.yaw-(D.off||0)));
 ctx.fillStyle='#2f6bff';ctx.strokeStyle='#fff';ctx.lineWidth=1.5*devicePixelRatio;
 ctx.beginPath();ctx.moveTo(L*1.4,0);ctx.lineTo(-L*0.8,L*0.8);ctx.lineTo(-L*0.4,0);ctx.lineTo(-L*0.8,-L*0.8);ctx.closePath();ctx.fill();ctx.stroke();ctx.restore();
}
function panel(){
 if(!D)return;
 const n=D.nearest;document.getElementById('near').textContent=n?n[0].toFixed(2)+' м':'—';
 document.getElementById('near').style.color=n?col(n[0]):'#ffd84a';
 if(n){const k=Math.floor(((n[1]+22.5)%360)/45);document.getElementById('nearDir').textContent='направление: '+NAMES[k]+' ('+n[1].toFixed(0)+'°)'}
 D.sectors.forEach((r,i)=>{document.getElementById('s'+i).textContent=r>0?r.toFixed(2)+' м':'—';
  const b=document.getElementById('b'+i);b.style.width=(r>0?Math.min(100,r/3*100):0)+'%';b.style.background=col(r)});
}
async function tick(){
 try{const r=await fetch('/data',{cache:'no-store'});D=await r.json();
  const st=document.getElementById('st');
  if(!D.scan){st.textContent='нет данных лидара — запущен ли драйвер?';st.className='st bad'}
  else{st.textContent=D.pose&&D.pose.frame==='map'?'карта строится (SLAM)':'только скан (SLAM не запущен)';st.className='st ok'}
  if(D.scan&&D.scan.t!==last){last=D.scan.t;frames.push(performance.now());while(frames.length>11)frames.shift();
   if(frames.length>1)document.getElementById('hz').textContent=((frames.length-1)*1000/(frames.at(-1)-frames[0])).toFixed(1)+' скан/с'}
  if(D.map_ver!==mapVer&&D.map_ver>0){const m=await (await fetch('/map',{cache:'no-store'})).json();
   const first=!MAP;MAP=m;mapImg=buildMap(m);mapVer=D.map_ver;if(first&&!follow)fit()}
  draw();panel();
 }catch(e){const st=document.getElementById('st');st.textContent='нет связи с платой (10.42.0.1)';st.className='st bad'}
 setTimeout(tick,150);
}
resize();tick();
</script></body></html>
"""


class Handler(BaseHTTPRequestHandler):
    def _send(self, code, body, ctype):
        b = body.encode('utf-8') if isinstance(body, str) else body
        self.send_response(code)
        self.send_header('Content-Type', ctype)
        self.send_header('Content-Length', str(len(b)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(b)

    def do_GET(self):
        path = self.path.split('?')[0]
        if path in ('/', '/index.html'):
            self._send(200, PAGE, 'text/html; charset=utf-8')
        elif path == '/data':
            with lock:
                d = {k: state[k] for k in ('scan', 'pose', 'map_ver', 'nearest', 'sectors', 'off')}
                body = json.dumps(d, separators=(',', ':'))
            self._send(200, body, 'application/json')
        elif path == '/map':
            with lock:
                m = state['map']
            self._send(200, json.dumps(m or {}), 'application/json')
        else:
            self._send(404, 'not found', 'text/plain')

    def log_message(self, *args):   # не засорять терминал
        pass


def start_server():
    srv = ThreadingHTTPServer(('0.0.0.0', PORT), Handler)
    srv.daemon_threads = True
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    print(f'lidar_web: откройте http://10.42.0.1:{PORT}', flush=True)
    return srv


# ---------------------------------------------------------------- ROS 2
def ros_main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
    from rclpy.time import Time
    from sensor_msgs.msg import LaserScan
    from nav_msgs.msg import OccupancyGrid
    from tf2_ros import Buffer, TransformListener

    class LidarWeb(Node):
        def __init__(self):
            super().__init__('lidar_web')
            self.tf = Buffer()
            self.tfl = TransformListener(self.tf, self)
            self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)
            mq = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                            reliability=ReliabilityPolicy.RELIABLE)
            self.create_subscription(OccupancyGrid, 'map', self.on_map, mq)

        def pose_of(self, frame):
            """Положение лидара в системе карты (или None, если карты ещё нет)."""
            try:
                t = self.tf.lookup_transform('map', frame, Time())
            except Exception:
                return None
            q = t.transform.rotation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            return {'x': t.transform.translation.x, 'y': t.transform.translation.y,
                    'yaw': yaw, 'frame': 'map'}

        def on_scan(self, s):
            near, best = analyse(s.angle_min, s.angle_increment, s.ranges, s.range_min, s.range_max)
            r = [round(v, 3) if math.isfinite(v) and s.range_min <= v <= s.range_max else 0
                 for v in s.ranges]
            pose = self.pose_of(s.header.frame_id) or {'x': 0.0, 'y': 0.0, 'yaw': 0.0,
                                                        'frame': s.header.frame_id}
            with lock:
                state['scan'] = {'amin': s.angle_min, 'ainc': s.angle_increment, 'r': r,
                                 't': time.time()}
                state['pose'] = pose
                state['nearest'] = near
                state['sectors'] = [round(v, 3) for v in best]

        def on_map(self, m):
            info = m.info
            data = grid_to_b64(info.width, info.height, m.data)
            with lock:
                state['map'] = {'w': info.width, 'h': info.height, 'res': info.resolution,
                                'ox': info.origin.position.x, 'oy': info.origin.position.y,
                                'data': data}
                state['map_ver'] += 1

    rclpy.init()
    node = LidarWeb()
    start_server()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    ros_main()
