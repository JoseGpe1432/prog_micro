"""
esp32_link.py
-------------
Protocolo ASCII con el ESP32, una línea por mensaje:  $<payload>*<XOR en hex>\n

Pi -> ESP32
  $D,<v_mm_s>,<w_mrad_s>*HH   conducir (velocidad lineal y angular)
  $S*HH   parar          $K*HH  recalibrar IR de precipicio (robot sobre el piso)
  $Z*HH   poner yaw/encoders en cero    $G*HH  recalibrar giroscopio (robot quieto)
  $C*HH   limpiar fallas (si ya no aplican)
  $W*HH   pedir datos de la IMU detectada -> $W,<dirección I2C>,<WHO_AM_I>,<ok>*HH

ESP32 -> Pi
  $T,<ms>,<batt_mV>,<yaw_mdeg>,<encL>,<encR>,<cliff_mask>,<faults>*HH   (20 Hz)
  $R,<c0>,<c1>,<c2>,<c3>*HH   lecturas crudas de los IR (solo si DEBUG_CLIFF_RAW=1)

El ESP32 detiene los motores solo si no recibe comandos válidos por 400 ms.
"""
import threading
import time
from dataclasses import dataclass

FAULT_CLIFF = 1
FAULT_BATT_LOW = 2
FAULT_BATT_CRIT = 4
FAULT_WATCHDOG = 8
FAULT_IMU = 16


@dataclass
class Telemetry:
    ms: int = 0
    batt_mv: int = 0
    yaw_mdeg: int = 0
    enc_l: int = 0
    enc_r: int = 0
    cliff_mask: int = 0
    faults: int = 0


def checksum(payload: str) -> int:
    x = 0
    for ch in payload.encode():
        x ^= ch
    return x


def encode(payload: str) -> bytes:
    return f"${payload}*{checksum(payload):02X}\n".encode()


def drive_cmd(v_m_s: float, w_rad_s: float) -> bytes:
    return encode(f"D,{int(round(v_m_s * 1000))},{int(round(w_rad_s * 1000))}")


def parse_line(line: str):
    """Devuelve (tipo, [campos]) o None si el checksum/formato es inválido."""
    line = line.strip()
    if not line.startswith("$") or "*" not in line:
        return None
    body, _, chk = line[1:].rpartition("*")
    try:
        if int(chk, 16) != checksum(body):
            return None
    except ValueError:
        return None
    parts = body.split(",")
    return parts[0], parts[1:]


def parse_telemetry(line: str):
    r = parse_line(line)
    if not r or r[0] != "T" or len(r[1]) != 7:
        return None
    try:
        f = [int(v) for v in r[1]]
    except ValueError:
        return None
    return Telemetry(*f)


class ESP32Link:
    """Hilo lector + envío de comandos. Uso:
        link = ESP32Link("/dev/ttyUSB0"); link.send_drive(0.1, 0.0); link.telemetry"""

    def __init__(self, port, baud=115200):
        import serial
        self.ser = serial.Serial(port, baud, timeout=0.05)
        self.telemetry = None
        self.last_rx = 0.0
        self.raw_cliff = None
        self.imu_info = None
        self._run = True
        self._lock = threading.Lock()
        self._th = threading.Thread(target=self._reader, daemon=True)
        self._th.start()

    def _reader(self):
        buf = b""
        while self._run:
            try:
                buf += self.ser.read(256)
            except Exception:
                time.sleep(0.1)
                continue
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                text = line.decode(errors="ignore")
                t = parse_telemetry(text)
                if t:
                    with self._lock:
                        self.telemetry = t
                        self.last_rx = time.time()
                else:
                    r = parse_line(text)
                    if r and r[0] == "R":
                        self.raw_cliff = r[1]
                    elif r and r[0] == "W":
                        self.imu_info = r[1]

    def get_telemetry(self):
        with self._lock:
            return self.telemetry

    def _send(self, data: bytes):
        self.ser.write(data)

    def send_drive(self, v, w):
        self._send(drive_cmd(v, w))

    def stop(self):
        self._send(encode("S"))

    def calibrate_cliff(self):
        self._send(encode("K"))

    def zero(self):
        self._send(encode("Z"))

    def calibrate_gyro(self):
        self._send(encode("G"))

    def request_imu_info(self):
        self._send(encode("W"))

    def clear_faults(self):
        self._send(encode("C"))

    def close(self):
        try:
            self.stop()
        finally:
            self._run = False
            self.ser.close()
