"""Текстовый вывод скана: ближайшее препятствие и минимумы по 8 секторам."""
import time
import serial
from ydlidar import scans

ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
ser.reset_input_buffer()
print('Ctrl+C — остановить')
t0 = None
try:
    for i, pts in enumerate(scans(ser)):
        t = time.time()
        if i == 0 or not pts:
            t0 = t; continue
        hz = 1 / (t - t0); t0 = t
        ang, d = min(pts, key=lambda p: p[1])
        sect = [min([p[1] for p in pts if int(p[0] // 45) == k] or [0]) for k in range(8)]
        print(f'{len(pts):4d} точек {hz:4.1f} Гц | ближайшее {d/1000:4.2f} м на {ang:5.1f}° | '
              'секторы по 45° (м): ' + ' '.join(f'{x/1000:4.2f}' for x in sect))
except KeyboardInterrupt:
    pass
finally:
    ser.close()
