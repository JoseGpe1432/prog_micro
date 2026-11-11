"""
navigator.py
------------
Convierte una ruta de celdas (de BFS/DFS/UCS) en comandos (v, w), con una
capa reactiva: si el LIDAR ve un obstáculo muy cerca delante, frena sin
esperar a replanificar.
"""
import math

import numpy as np

import config as cfg


def simplify(path):
    """Quita celdas intermedias colineales."""
    if len(path) < 3:
        return list(path)
    out = [path[0]]
    for i in range(1, len(path) - 1):
        d1 = (path[i][0] - out[-1][0], path[i][1] - out[-1][1])
        d2 = (path[i + 1][0] - path[i][0], path[i + 1][1] - path[i][1])
        # misma dirección unitaria => colineal
        if (d1[0] * d2[1] - d1[1] * d2[0]) != 0 or (d1[0] * d2[0] + d1[1] * d2[1]) < 0:
            out.append(path[i])
    out.append(path[-1])
    return out


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def front_clearance(angles, dists, half_angle_deg=35.0):
    """Distancia libre (m) entre el borde del robot y el obstáculo más cercano
    en el cono frontal. 9.9 si no hay lectura."""
    if angles is None or len(dists) == 0:
        return 9.9
    m = np.abs(angles) < math.radians(half_angle_deg)
    m &= dists > cfg.LIDAR_MIN_RANGE
    if not m.any():
        return 9.9
    return float(dists[m].min() - cfg.ROBOT_RADIUS)


class PathFollower:
    def __init__(self):
        self.waypoints = []      # [(x, y)] en metros
        self.idx = 0

    def set_path(self, world_points):
        self.waypoints = list(world_points)
        self.idx = 1 if len(self.waypoints) > 1 else 0   # el 0 es donde estamos

    def clear(self):
        self.waypoints, self.idx = [], 0

    @property
    def done(self):
        return self.idx >= len(self.waypoints)

    def command(self, pose, front_clear):
        """Devuelve (v, w, bloqueado)."""
        x, y, th = pose
        while self.idx < len(self.waypoints):
            tx, ty = self.waypoints[self.idx]
            if math.hypot(tx - x, ty - y) < cfg.WAYPOINT_TOL:
                self.idx += 1
            else:
                break
        if self.done:
            return 0.0, 0.0, False

        tx, ty = self.waypoints[self.idx]
        err = wrap(math.atan2(ty - y, tx - x) - th)
        w = max(-cfg.W_MAX, min(cfg.W_MAX, 2.2 * err))

        if abs(err) > math.radians(40):
            return 0.0, w, False              # girar en sitio (el robot es circular)

        v = cfg.V_CLEAN * max(0.3, math.cos(err))   # arco suave si el error es moderado
        dist = math.hypot(tx - x, ty - y)
        v = min(v, 0.4 + 1.5 * dist)          # no pasarse del waypoint
        if front_clear < cfg.FRONT_STOP_DIST:
            return 0.0, 0.0, True
        if front_clear < cfg.FRONT_SLOW_DIST:
            span = cfg.FRONT_SLOW_DIST - cfg.FRONT_STOP_DIST
            v *= max(0.25, (front_clear - cfg.FRONT_STOP_DIST) / span)
        return v, w, False
