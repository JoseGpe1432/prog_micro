"""
pose.py
-------
Estimación de pose (x, y, theta) con:
  * rumbo (theta): giroscopio de la IMU integrado en el ESP32
  * distancia: encoders si USE_ENCODERS, o en caso contrario v_comandada * dt
    (menos preciso: el error crece con el tiempo; ver README)
Hay un gancho correct() para añadir luego corrección por LIDAR (ICP/SLAM).
"""
import math

import config as cfg


class PoseEstimator:
    def __init__(self):
        self.x = self.y = self.th = 0.0
        self._yaw0 = None
        self._enc_prev = None
        self._th_offset = 0.0     # correcciones por LIDAR acumuladas

    def as_tuple(self):
        return (self.x, self.y, self.th)

    def correct(self, x, y, th):
        """Aplica una corrección externa (p. ej. scan matching). El rumbo
        corregido se conserva aunque el giroscopio siga integrando."""
        self._th_offset += math.atan2(math.sin(th - self.th), math.cos(th - self.th))
        self.x, self.y, self.th = x, y, th

    def update(self, telem, v_applied, dt):
        yaw = math.radians(telem.yaw_mdeg / 1000.0) * cfg.GYRO_YAW_SIGN
        if self._yaw0 is None:
            self._yaw0 = yaw
            self._enc_prev = (telem.enc_l, telem.enc_r)
        th_new = yaw - self._yaw0 + self._th_offset
        dth = math.atan2(math.sin(th_new - self.th), math.cos(th_new - self.th))

        if cfg.USE_ENCODERS:
            dl = (telem.enc_l - self._enc_prev[0]) / cfg.TICKS_PER_M
            dr = (telem.enc_r - self._enc_prev[1]) / cfg.TICKS_PER_M
            ds = 0.5 * (dl + dr)
        else:
            ds = v_applied * dt * cfg.SPEED_SCALE
        self._enc_prev = (telem.enc_l, telem.enc_r)

        mid = self.th + 0.5 * dth
        self.x += ds * math.cos(mid)
        self.y += ds * math.sin(mid)
        self.th = th_new
