"""
scan_match.py
-------------
Corrección de pose por LIDAR (correlative scan matching ligero).

El giroscopio deriva (unos grados por minuto) y la odometría acumula error.
Cada ~0.3 s se compara el escaneo actual contra el mapa construido y se
prueba un pequeño conjunto de ajustes (±3° de rumbo, ±2 cm en x/y). Se aplica
el que mejor alinea los impactos del LIDAR con las paredes ya mapeadas, solo si
mejora claramente sobre "no corregir". Es el mismo principio que usan los SLAM
2D (Hector, Cartographer) en versión mínima.
"""
import cv2
import numpy as np

import config as cfg


class ScanMatcher:
    def __init__(self, grid):
        self.grid = grid
        self.thetas = np.deg2rad(np.arange(-3.0, 3.01, 1.0))
        self.offs = (-0.02, -0.01, 0.0, 0.01, 0.02)

    def match(self, pose, angles, dists):
        """Devuelve (dx, dy, dth) a sumar a la pose, o None si no hay mejora clara."""
        g = self.grid
        if int((g.lo > 1.2).sum()) < 60:
            return None                                   # mapa aún sin paredes
        ok = (dists < cfg.LIDAR_MAP_RANGE) & (dists > cfg.LIDAR_MIN_RANGE)
        a, d = angles[ok], dists[ok]
        if len(d) < 80:
            return None
        field = cv2.GaussianBlur(np.clip(g.lo, 0.0, None), (5, 5), 1.0)
        x, y, th = pose
        n, o, res = g.n, g.origin, g.res

        def score(t, dx, dy):
            # muestreo BILINEAL del campo: objetivo continuo (sin "saltos" por discretizar)
            fc = (x + dx + np.cos(a + t) * d) / res + o
            fr = o - (y + dy + np.sin(a + t) * d) / res
            c0 = np.floor(fc).astype(np.int32)
            r0 = np.floor(fr).astype(np.int32)
            m = (r0 >= 0) & (r0 < n - 1) & (c0 >= 0) & (c0 < n - 1)
            c0, r0, fc, fr = c0[m], r0[m], fc[m], fr[m]
            wc, wr = fc - c0, fr - r0
            v = (field[r0, c0] * (1 - wc) * (1 - wr) + field[r0, c0 + 1] * wc * (1 - wr)
                 + field[r0 + 1, c0] * (1 - wc) * wr + field[r0 + 1, c0 + 1] * wc * wr)
            return float(v.sum())

        base = score(th, 0.0, 0.0)
        best, best_s = (0.0, 0.0, 0.0), base
        for dth in self.thetas:
            for dx in self.offs:
                for dy in self.offs:
                    s = score(th + dth, dx, dy)
                    s -= 0.15 * (abs(np.rad2deg(dth)) + abs(dx) / 0.01 + abs(dy) / 0.01)
                    if s > best_s:
                        best_s, best = s, (dx, dy, float(dth))
        if best_s > base * 1.03:
            return best
        return None
