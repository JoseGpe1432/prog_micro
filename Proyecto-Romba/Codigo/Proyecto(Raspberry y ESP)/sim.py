"""
sim.py
------
Simulador para probar el cerebro (brain.py) sin hardware:
  * mundo 2D con paredes/muebles (el LIDAR los ve) y una zona de "escalón"
    (el LIDAR NO la ve; solo los IR de precipicio, como en la vida real)
  * robot diferencial con la MISMA lógica de seguridad del firmware
    (bloqueo de avance con precipicio, corte por batería crítica)
  * LIDAR con ruido, cámara que ve basura al frente, gyro con deriva, batería
"""
import math
import random

import cv2
import numpy as np

import config as cfg
from esp32_link import Telemetry, FAULT_CLIFF, FAULT_BATT_LOW, FAULT_BATT_CRIT
from brain import Brain, DONE, STOPPED_LOWBATT
from detection import Detection, DEBRIS, TOO_BIG


class World:
    def __init__(self, size=3.6, res=0.02, boxes=(), drop=None):
        self.res, self.size = res, size
        self.n = int(size / res)
        self.occ = np.zeros((self.n, self.n), dtype=bool)
        self.occ[:2, :] = self.occ[-2:, :] = True
        self.occ[:, :2] = self.occ[:, -2:] = True
        for x0, y0, x1, y1 in boxes:
            self.occ[int(y0 / res):int(y1 / res), int(x0 / res):int(x1 / res)] = True
        self.drop = drop                                   # (x0, y0, x1, y1) o None

    def is_occ(self, x, y):
        ix, iy = int(x / self.res), int(y / self.res)
        if not (0 <= ix < self.n and 0 <= iy < self.n):
            return True
        return bool(self.occ[iy, ix])

    def in_drop(self, x, y):
        if not self.drop:
            return False
        x0, y0, x1, y1 = self.drop
        return x0 <= x <= x1 and y0 <= y <= y1

    def raycast(self, x, y, angles, maxr):
        s = np.arange(self.res, maxr, self.res, dtype=np.float32)
        xs = x + np.cos(angles)[:, None] * s[None, :]
        ys = y + np.sin(angles)[:, None] * s[None, :]
        ix = np.clip((xs / self.res).astype(np.int32), 0, self.n - 1)
        iy = np.clip((ys / self.res).astype(np.int32), 0, self.n - 1)
        hit = self.occ[iy, ix]
        has = hit.any(axis=1)
        first = hit.argmax(axis=1)
        return np.where(has, s[first], 0.0), has      # 0.0 = sin lectura (como el LD06)


class FakeRobot:
    def __init__(self, world, start=(0.55, 0.55, 0.0), seed=1, batt_start=15800,
                 drain_mv_per_s=0.0, gyro_bias_dps=0.02, dust_points=(), detect_prob=0.8,
                 hazards=(), hazard_detect=True):
        self.w = world
        self.x, self.y, self.th = start
        self.rng = random.Random(seed)
        self.np_rng = np.random.default_rng(seed)
        self.v = self.wz = 0.0
        self.batt = float(batt_start)
        self.drain = drain_mv_per_s
        self.t = 0.0
        self.yaw_deg = 0.0
        self.bias = gyro_bias_dps
        self.enc_l = self.enc_r = 0.0
        self.collisions = 0
        self.falls = 0
        self._in_contact = False
        self.crit_t = 0.0
        self.crit_latched = False
        self.dust = [list(p) for p in dust_points]
        self.detect_prob = detect_prob
        self.hazards = [tuple(h) for h in hazards]    # (x, y, tamaño m): objetos en el piso, invisibles al LIDAR
        self.hazard_detect = hazard_detect
        self.ran_over = set()                          # hazards que el robot pisó con su cabezal
        self.trail = []
        self.home_world = (start[0], start[1])

    # ---- física ----
    def _collides(self, x, y):
        R = cfg.ROBOT_RADIUS
        if self.w.is_occ(x, y):
            return True
        for k in range(12):
            a = 2 * math.pi * k / 12
            if self.w.is_occ(x + R * math.cos(a), y + R * math.sin(a)):
                return True
        return False

    def cliff_active(self):
        R = cfg.ROBOT_RADIUS - 0.02
        for da in (-0.5, 0.0, 0.5):
            if self.w.in_drop(self.x + R * math.cos(self.th + da), self.y + R * math.sin(self.th + da)):
                return True
        return False

    def step(self, dt, v_cmd, w_cmd):
        """Avanza dt segundos aplicando las reglas de seguridad del firmware."""
        sub = 5
        h = dt / sub
        for _ in range(sub):
            v, wz = v_cmd, w_cmd
            if self.cliff_active():                      # el ESP32 bloquea el avance
                if v > 0:
                    v = 0.0
                if v >= 0:
                    wz = 0.0
            if self.batt < cfg.BATT_CRIT_MV:
                self.crit_t += h
                if self.crit_t > 2.0:
                    self.crit_latched = True
            else:
                self.crit_t = 0.0
            if self.crit_latched:
                v = wz = 0.0

            self.th += wz * h
            nx = self.x + v * math.cos(self.th) * h
            ny = self.y + v * math.sin(self.th) * h
            if self._collides(nx, ny):
                if not self._in_contact:
                    self.collisions += 1
                self._in_contact = True
                v = 0.0
            else:
                self._in_contact = False
                self.x, self.y = nx, ny
            if self.w.in_drop(self.x, self.y):
                self.falls += 1

            self.yaw_deg += math.degrees(wz * h) + (self.bias + self.np_rng.normal(0, 0.02)) * h
            d = v * h
            self.enc_l += (d - wz * cfg.WHEEL_BASE / 2 * h) * cfg.TICKS_PER_M
            self.enc_r += (d + wz * cfg.WHEEL_BASE / 2 * h) * cfg.TICKS_PER_M
            self.batt -= self.drain * h * (1.0 if v != 0 else 0.3)
            self.t += h
            for i, (hx, hy, hs) in enumerate(self.hazards):
                if math.hypot(hx - self.x, hy - self.y) < 0.12:
                    self.ran_over.add(i)
            # quitar basura aspirada
            self.dust = [p for p in self.dust if math.hypot(p[0] - self.x, p[1] - self.y) > 0.09]
        self.trail.append((self.x, self.y))

    # ---- sensores ----
    def telemetry(self):
        faults = 0
        if self.cliff_active():
            faults |= FAULT_CLIFF
        if self.batt < cfg.BATT_LOW_MV:
            faults |= FAULT_BATT_LOW
        if self.crit_latched:
            faults |= FAULT_BATT_CRIT
        return Telemetry(ms=int(self.t * 1000), batt_mv=int(self.batt),
                         yaw_mdeg=int(self.yaw_deg * 1000), enc_l=int(self.enc_l),
                         enc_r=int(self.enc_r), cliff_mask=1 if faults & FAULT_CLIFF else 0,
                         faults=faults)

    def lidar(self):
        # el brain recibe ángulos con LIDAR_ANGLE_SIGN ya aplicado (marco del robot, antihorario)
        ang = np.deg2rad(np.arange(-180, 180, 1.0))
        d, has = self.w.raycast(self.x, self.y, ang + self.th, 4.0)
        d = d + self.np_rng.normal(0, 0.008, d.shape)
        keep = has & (d > cfg.LIDAR_MIN_RANGE)
        return ang[keep].astype(np.float32), d[keep].astype(np.float32)

    def camera(self):
        """Lo que reportaría la visión: posición y TAMAÑO REAL de cada objeto en el campo de visión."""
        def visible(px, py):
            dx, dy = px - self.x, py - self.y
            fwd = dx * math.cos(self.th) + dy * math.sin(self.th)
            left = -dx * math.sin(self.th) + dy * math.cos(self.th)
            ok = 0.20 < fwd < 0.70 and abs(math.atan2(left, fwd)) < math.radians(28)
            return ok, fwd, left

        out = []
        for px, py in self.dust:
            ok, fwd, left = visible(px, py)
            if ok and self.rng.random() < self.detect_prob:
                out.append(Detection(fwd + self.rng.gauss(0, 0.02), left + self.rng.gauss(0, 0.02),
                                     0.012, 0.008, DEBRIS))
        if self.hazard_detect:
            for hx, hy, hs in self.hazards:
                ok, fwd, left = visible(hx, hy)
                if ok and self.rng.random() < self.detect_prob:
                    size = hs * (1.0 + self.rng.gauss(0, 0.1))
                    out.append(Detection(fwd + self.rng.gauss(0, 0.03), left + self.rng.gauss(0, 0.03),
                                         size, size * 0.9, TOO_BIG))
        return out


def coverage_percent(world, trail, brush=0.14):
    """% del suelo alcanzable (a >=10 cm de paredes) que quedó a <=brush m de la trayectoria."""
    free = (~world.occ).astype(np.uint8)
    clearance = cv2.distanceTransform(free, cv2.DIST_L2, 3) * world.res
    coverable = (clearance >= 0.10) & (~world.occ)
    painted = np.zeros(world.occ.shape, np.uint8)
    for x, y in trail[::2]:
        cv2.circle(painted, (int(x / world.res), int(y / world.res)), int(brush / world.res), 1, -1)
    got = (painted > 0) & coverable
    return 100.0 * got.sum() / max(1, coverable.sum())


def run_simulation(world, seconds=600, quiet=True, **robot_kw):
    robot = FakeRobot(world, **robot_kw)
    log_lines = []
    brain = Brain(log=(lambda m: log_lines.append(m)))
    dt = 1.0 / cfg.TICK_HZ
    v = w = 0.0
    t_end = None
    for _ in range(int(seconds / dt)):
        robot.step(dt, v, w)
        v, w = brain.tick(dt, robot.telemetry(), robot.lidar(), robot.camera())
        if brain.state in (DONE, STOPPED_LOWBATT):
            t_end = t_end or robot.t
            if robot.t - t_end > 1.0:
                break
    home_dist = math.hypot(robot.x - robot.home_world[0], robot.y - robot.home_world[1])
    return {
        "state": brain.state, "sim_time": round(robot.t, 1),
        "coverage": round(coverage_percent(world, robot.trail), 1),
        "collisions": robot.collisions, "falls": robot.falls,
        "home_dist": round(home_dist, 2), "batt_mv": int(robot.batt),
        "dust_left": len(robot.dust), "dust_cleaned": brain.dust_cleaned,
        "ran_over": len(robot.ran_over), "hazards_marked": brain.hazards_marked,
        "cliff_events": brain.cliff_events, "note": brain.note, "log": log_lines,
    }


def default_world(with_drop=False):
    return World(size=3.6, boxes=[(1.6, 1.1, 2.1, 1.6), (0.5, 2.4, 1.0, 2.9)],
                 drop=(2.7, 2.4, 3.3, 3.3) if with_drop else None)
