"""Разбор потока данных лидара YDLIDAR X3 Pro.

Версия 2 (16.11.2026, проверена на записи с Orange Pi Zero 3):
- поправка угла для треугольного лидара (как в YDLidar SDK), параметр correct;
- чтение порта без задержки (сколько байт пришло), поэтому кадры выдаются
  равномерно и частота в lidar_scan.py больше не скачет 9 <-> 18 Гц.

Данные: один пакет начала оборота (младший бит CT = 1) на каждый оборот,
около 340 измерений на оборот (шаг ~1,06°), ~14,5 оборота в секунду.
При открытии порта мотор перезапускается (сигнал DTR), первые 0,3 с
оборот ещё разгоняется.
"""
import math
import struct


def packets(ser):
    """Выделяет из потока пакеты и проверяет контрольную сумму."""
    buf = bytearray()
    while True:
        buf += ser.read(ser.in_waiting or 1)   # не ждём 512 байт — нет рывков
        while True:
            i = buf.find(b'\xaa\x55')
            if i < 0:
                del buf[:-1]; break
            if len(buf) < i + 10:
                del buf[:i]; break
            ct, lsn = buf[i + 2], buf[i + 3]
            n = 10 + 2 * lsn
            if len(buf) < i + n:
                del buf[:i]; break
            pkt = bytes(buf[i:i + n])
            fsa, lsa, cs = struct.unpack_from('<HHH', pkt, 4)
            samples = struct.unpack_from('<%dH' % lsn, pkt, 10)
            chk = 0x55AA ^ (ct | lsn << 8) ^ fsa ^ lsa
            for s in samples:
                chk ^= s
            if chk != cs:            # повреждённый пакет — пропускаем
                del buf[:i + 2]; continue
            del buf[:i + n]
            yield ct, fsa, lsa, samples


def angle_correction(d):
    """Поправка угла (градусы) для треугольного лидара на расстоянии d мм."""
    return math.degrees(math.atan(21.8 * (155.3 - d) / (155.3 * d)))


def scans(ser, min_mm=100, correct=True):
    """Возвращает списки точек (угол в градусах, расстояние в мм) за каждый оборот.

    correct=True — учитывать поправку угла (от -5° на 0,4 м до -8° вдали).
    """
    pts = []
    for ct, fsa, lsa, samples in packets(ser):
        if ct & 1:                   # пакет начала нового оборота
            if pts:
                yield pts
            pts = []
        a1, a2 = (fsa >> 1) / 64, (lsa >> 1) / 64
        n = len(samples)
        diff = (a2 - a1) % 360
        for k, s in enumerate(samples):
            d = s / 4                # миллиметры
            if d < min_mm:           # ближе 0,1 м лидар не измеряет (0 — нет отражения)
                continue
            ang = a1 + (diff * k / (n - 1) if n > 1 else 0)
            if correct:
                ang += angle_correction(d)
            pts.append((ang % 360, d))
