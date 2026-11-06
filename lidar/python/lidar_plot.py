"""Графическое отображение скана лидара (круговая диаграмма)."""
import sys, math, time, threading, collections
import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from ydlidar import scans

RMAX = float(sys.argv[1]) if len(sys.argv) > 1 else 3.0   # масштаб, м

ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
ser.reset_input_buffer()
state = {'pts': [], 'times': collections.deque(maxlen=20)}

def reader():                       # чтение лидара в отдельном потоке
    for pts in scans(ser):
        state['pts'] = pts
        state['times'].append(time.time())
threading.Thread(target=reader, daemon=True).start()

fig = plt.figure(figsize=(8, 8))
ax = fig.add_subplot(projection='polar')
ax.set_theta_zero_location('N')     # 0° — вверх
ax.set_theta_direction(-1)          # углы по часовой стрелке
ax.set_rlim(0, RMAX)
sc = ax.scatter([], [], s=5, c='tab:red')
title = ax.set_title('')

def update(_):
    pts = state['pts']
    if pts:
        sc.set_offsets([(math.radians(a), d / 1000) for a, d in pts])
        t = state['times']
        hz = (len(t) - 1) / (t[-1] - t[0]) if len(t) > 1 and t[-1] > t[0] else 0
        title.set_text(f'YDLIDAR X3 Pro: {len(pts)} точек, {hz:.1f} об/с')
    return sc, title

ani = FuncAnimation(fig, update, interval=100, cache_frame_data=False)
plt.show()
ser.close()
