"""
lidar_ld06.py
-------------
Driver para LDROBOT LD06 / LD19 (UART 230400 8N1, sin comandos: transmite solo).

Paquete de 47 bytes:
  0x54 | 0x2C | velocidad(2) | ángulo_inicial(2, 0.01°) | 12 x [dist_mm(2) intens(1)]
  | ángulo_final(2, 0.01°) | timestamp(2) | CRC8
El LIDAR mide ángulos en sentido HORARIO con 0° al frente: config.LIDAR_ANGLE_SIGN
los convierte al marco del robot (antihorario).
"""
import math
import threading
import time

import numpy as np

import config as cfg

PKT_LEN = 47
POINTS = 12


def _make_table():
    table = []
    for i in range(256):
        crc = i
        for _ in range(8):
            crc = ((crc << 1) ^ 0x4D) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
        table.append(crc)
    return table


_CRC = _make_table()


def crc8(data: bytes) -> int:
    crc = 0
    for b in data:
        crc = _CRC[(crc ^ b) & 0xFF]
    return crc


class LD06Parser:
    """Alimenta con bytes (feed) y devuelve escaneos completos (360°)."""

    def __init__(self, min_intensity=5):
        self.buf = bytearray()
        self.points = []          # (ángulo_deg_horario, dist_m, intensidad)
        self.last_start = None
        self.min_intensity = min_intensity
        self.bad_crc = 0

    def feed(self, data: bytes):
        scans = []
        self.buf.extend(data)
        while True:
            i = self.buf.find(b"\x54\x2c")
            if i < 0:
                self.buf = self.buf[-1:]
                break
            if i > 0:
                del self.buf[:i]
            if len(self.buf) < PKT_LEN:
                break
            pkt = bytes(self.buf[:PKT_LEN])
            if crc8(pkt[:-1]) != pkt[-1]:
                self.bad_crc += 1
                del self.buf[:2]          # falso encabezado: seguir buscando
                continue
            del self.buf[:PKT_LEN]
            scan = self._handle_packet(pkt)
            if scan is not None:
                scans.append(scan)
        return scans

    def _handle_packet(self, pkt):
        start = int.from_bytes(pkt[4:6], "little") / 100.0
        end = int.from_bytes(pkt[42:44], "little") / 100.0
        if end < start:
            end += 360.0
        step = (end - start) / (POINTS - 1)
        finished = None
        if self.last_start is not None and start < self.last_start - 180.0 and self.points:
            finished = self._build_scan()
            self.points = []
        self.last_start = start
        for k in range(POINTS):
            o = 6 + 3 * k
            dist = int.from_bytes(pkt[o:o + 2], "little") / 1000.0
            inten = pkt[o + 2]
            if dist <= 0 or inten < self.min_intensity:
                continue
            self.points.append(((start + step * k) % 360.0, dist, inten))
        return finished

    def _build_scan(self):
        arr = np.array(self.points, dtype=np.float32)
        ang = np.deg2rad(cfg.LIDAR_ANGLE_SIGN * arr[:, 0] + cfg.LIDAR_ANGLE_OFFSET_DEG)
        ang = (ang + math.pi) % (2 * math.pi) - math.pi
        return ang, arr[:, 1]


class LD06Reader:
    """Hilo que lee el puerto serie y deja el último escaneo en .latest()."""

    def __init__(self, port=None, baud=None):
        import serial
        self.ser = serial.Serial(port or cfg.LIDAR_PORT, baud or cfg.LIDAR_BAUD, timeout=0.1)
        self.parser = LD06Parser()
        self._scan = None
        self._stamp = 0.0
        self._lock = threading.Lock()
        self._run = True
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        while self._run:
            try:
                data = self.ser.read(512)
            except Exception:
                time.sleep(0.1)
                continue
            for scan in self.parser.feed(data):
                with self._lock:
                    self._scan, self._stamp = scan, time.time()

    def latest(self, consume=True):
        with self._lock:
            scan = self._scan
            if consume:
                self._scan = None
            return scan

    def close(self):
        self._run = False
        self.ser.close()
