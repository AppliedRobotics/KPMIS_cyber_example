"""Разбор потока данных лидара YDLIDAR X3 Pro."""
import struct

def packets(ser):
    """Выделяет из потока пакеты и проверяет контрольную сумму."""
    buf = bytearray()
    while True:
        buf += ser.read(512)
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

def scans(ser, min_mm=100):
    """Возвращает списки точек (угол в градусах, расстояние в мм) за каждый оборот."""
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
            ang = (a1 + (diff * k / (n - 1) if n > 1 else 0)) % 360
            d = s / 4                # миллиметры
            if d >= min_mm:          # ближе 0,1 м лидар не измеряет
                pts.append((ang, d))
