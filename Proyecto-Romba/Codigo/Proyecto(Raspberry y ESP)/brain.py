"""
brain.py
--------
Máquina de estados del robot. No toca hardware: recibe sensores y devuelve
(v, w). Así el mismo código corre en el robot real (run_robot.py) y en el
simulador (sim.py).

Estados
  INIT          espera a que el giroscopio se calibre y llegue el primer escaneo
  CLEANING      va a la celda sin limpiar más cercana         (BFS, o DFS si se configura)
  GOTO_DUST     va a la basura detectada por la cámara         (UCS: menor costo)
  RETURN_HOME   batería baja o misión terminada: vuelve a casa (UCS: ruta lejos de paredes)
  BACKUP        retrocede tras detectar precipicio u obstáculo bloqueante
  DONE          llegó a casa y se detuvo
  STOPPED_LOWBATT  el ESP32 reportó batería crítica: motores apagados

Prioridades de seguridad (de mayor a menor):
  1. batería crítica   2. precipicio (IR)   3. obstáculo cercano (LIDAR)   4. batería baja
"""
import math

import numpy as np

import config as cfg
from bfs import bfs
from dfs import dfs
from ucs import ucs
from esp32_link import FAULT_CLIFF, FAULT_BATT_CRIT
from grid_map import OccupancyGrid, PlanningGrid
from navigator import PathFollower, simplify, front_clearance
from pose import PoseEstimator
from detection import Detection, DEBRIS, AVOID_KINDS
from scan_match import ScanMatcher

INIT, CLEANING, GOTO_DUST, RETURN_HOME, BACKUP, DONE, STOPPED_LOWBATT = (
    "INIT", "CLEANING", "GOTO_DUST", "RETURN_HOME", "BACKUP", "DONE", "STOPPED_LOWBATT")


class Brain:
    def __init__(self, log=None):
        self.grid = OccupancyGrid()
        self.pose = PoseEstimator()
        self.follower = PathFollower()
        self.matcher = ScanMatcher(self.grid)
        self._n_ticks = 0
        self.match_applied = 0
        n = self.grid.n
        self.covered = np.zeros((n, n), dtype=bool)
        self.trail = np.zeros((n, n), dtype=bool)   # celdas por donde pasó el centro del robot
        k = int(math.ceil(cfg.COVER_RADIUS / cfg.MAP_RES))
        self._cover_offsets = [(dr, dc) for dr in range(-k, k + 1) for dc in range(-k, k + 1)
                               if dr * dr + dc * dc <= k * k]

        self.state = INIT
        self.prev_state = CLEANING
        self.t = 0.0
        self.state_t = 0.0
        self.v_cmd = 0.0
        self.w_cmd = 0.0
        self.batt = None
        self.front_clear = 9.9

        self.full_path = []        # celdas completas (para validar)
        self.path_state = None
        self.last_plan_t = -99.0
        self.blocked_t = 0.0
        self.fail_plans = 0

        self.dust = []             # [{"x","y","n"}]  basura aspirable
        self.hazards = []          # [{"x","y","n","r","marked"}]  objetos a rodear (muy grandes / cables)
        self.hazards_marked = 0
        self.dust_cleaned = 0
        self.cliff_events = 0
        self.note = ""
        self.log = log or (lambda msg: None)

    # ------------------------------------------------------------------
    def _set_state(self, s):
        if s != self.state:
            self.log(f"[{self.t:6.1f}s] {self.state} -> {s}")
            self.state, self.state_t = s, 0.0

    def _out(self, v, w):
        self.v_cmd, self.w_cmd = v, w
        return v, w

    # ------------------------------------------------------------------
    def tick(self, dt, telem, scan, detections=()):
        """telem: Telemetry; scan: (angulos_rad, dists_m) o None;
        detections: [(x_adelante, y_izquierda)] de basura vista, en metros."""
        self.t += dt
        self.state_t += dt

        v_applied = self.v_cmd
        if (telem.faults & FAULT_CLIFF and v_applied > 0) or (telem.faults & FAULT_BATT_CRIT):
            v_applied = 0.0                     # el ESP32 lo bloqueó
        self.pose.update(telem, v_applied, dt)
        pose = self.pose.as_tuple()

        mv = telem.batt_mv
        self.batt = mv if self.batt is None else self.batt + cfg.BATT_FILTER * (mv - self.batt)

        self._n_ticks += 1
        if scan is not None:
            ang, d = scan
            if (cfg.SCAN_MATCH and self.state != INIT and self._n_ticks % 3 == 0
                    and abs(self.w_cmd) < 0.3):
                fix = self.matcher.match(pose, ang, d)
                if fix:
                    self.pose.correct(pose[0] + fix[0], pose[1] + fix[1], pose[2] + fix[2])
                    pose = self.pose.as_tuple()
                    self.match_applied += 1
            self.grid.integrate_scan(pose, ang, d)
            self.front_clear = front_clearance(ang, d)
        self._mark_covered(pose)
        self._update_dust(pose, detections)

        # ---------- 1. seguridad ----------
        if telem.faults & FAULT_BATT_CRIT:
            self._set_state(STOPPED_LOWBATT)
        if self.state in (STOPPED_LOWBATT, DONE):
            return self._out(0.0, 0.0)

        if telem.faults & FAULT_CLIFF and self.state != BACKUP:
            self._start_backup(pose, cliff=True)

        # ---------- 2. estados simples ----------
        if self.state == INIT:
            if self.state_t >= cfg.INIT_WAIT_S and scan is not None:
                self._set_state(CLEANING)
            return self._out(0.0, 0.0)

        if self.state == BACKUP:
            clear = not (telem.faults & FAULT_CLIFF)
            if (self.state_t >= cfg.BACKUP_S and clear) or self.state_t > 3.0:
                self._set_state(self.prev_state)
                self.follower.clear()
                self.last_plan_t = -99.0
            else:
                return self._out(cfg.BACKUP_V, 0.0)

        # ---------- 3. batería baja -> volver a casa ----------
        if self.batt < cfg.BATT_LOW_MV and self.state in (CLEANING, GOTO_DUST):
            self.note = f"bateria baja ({self.batt:.0f} mV): regresando"
            self._set_state(RETURN_HOME)
            self.follower.clear()

        if self.state == RETURN_HOME and math.hypot(pose[0], pose[1]) < cfg.HOME_TOL:
            self.note = "llego a casa"
            self._set_state(DONE)
            return self._out(0.0, 0.0)

        # ---------- 4. planear y seguir ----------
        self._maybe_plan(pose)
        v, w, blocked = self.follower.command(pose, self.front_clear)
        if blocked:
            self.blocked_t += dt
            if self.blocked_t > 1.0:                       # algo se cruzó: replanear
                self.blocked_t = 0.0
                self.follower.clear()
                self.last_plan_t = -99.0
                self._start_backup(pose, cliff=False)
        else:
            self.blocked_t = 0.0
        return self._out(v, w)

    # ------------------------------------------------------------------
    def _start_backup(self, pose, cliff):
        if cliff:
            x, y, th = pose
            d = cfg.ROBOT_RADIUS + 0.10
            self.grid.mark_obstacle(x + math.cos(th) * d, y + math.sin(th) * d, 0.18)
            self.cliff_events += 1
            self.log(f"[{self.t:6.1f}s] PRECIPICIO: se marca obstaculo virtual")
        if self.state != BACKUP:
            self.prev_state = self.state if self.state not in (INIT, BACKUP) else CLEANING
        self._set_state(BACKUP)
        self.follower.clear()

    def _mark_covered(self, pose):
        r0, c0 = self.grid.world_to_cell(pose[0], pose[1])
        n = self.grid.n
        self.trail[max(0, r0 - 1):r0 + 2, max(0, c0 - 1):c0 + 2] = True
        for dr, dc in self._cover_offsets:
            r, c = r0 + dr, c0 + dc
            if 0 <= r < n and 0 <= c < n:
                self.covered[r, c] = True

    # ------------------------- basura ------------------------------
    def _update_dust(self, pose, detections):
        """Procesa lo que ve la cámara. Cada detección trae posición y TAMAÑO REAL:
          DEBRIS  -> basura que cabe: se agrega a la lista de objetivos (UCS)
          TOO_BIG / CABLE -> objeto que NO hay que aspirar: se marca como obstáculo permanente
        Se piden varios avistamientos para confirmar (la cámara puede equivocarse una vez)."""
        x, y, th = pose
        kept = []
        for d in self.dust:
            if math.hypot(d["x"] - x, d["y"] - y) <= cfg.DUST_CLEAN_DIST:
                if d["n"] >= cfg.DUST_CONFIRM:
                    self.dust_cleaned += 1
            else:
                kept.append(d)
        self.dust = kept

        for det in detections:
            if not isinstance(det, Detection):                 # compatibilidad: (x, y) = basura
                det = Detection(det[0], det[1], 0.01, 0.01, DEBRIS)
            wx = x + math.cos(th) * det.x - math.sin(th) * det.y
            wy = y + math.sin(th) * det.x + math.cos(th) * det.y
            if det.kind == DEBRIS:
                self._merge(self.dust, wx, wy, cfg.DUST_MERGE_DIST)
            elif det.kind in AVOID_KINDS:
                h = self._merge(self.hazards, wx, wy, cfg.DUST_MERGE_DIST,
                                r=max(0.04, det.length / 2.0 + cfg.HAZARD_MARGIN))
                h["r"] = max(h["r"], det.length / 2.0 + cfg.HAZARD_MARGIN)
                if h["n"] >= cfg.HAZARD_CONFIRM and not h["marked"]:
                    self.grid.mark_obstacle(h["x"], h["y"], h["r"])
                    h["marked"] = True
                    self.hazards_marked += 1
                    self.follower.clear()
                    self.last_plan_t = -99.0
                    self.log(f"[{self.t:6.1f}s] OBJETO A EVITAR ({det.kind}, {det.length*100:.0f} cm) "
                             f"en ({h['x']:.2f}, {h['y']:.2f})")

    @staticmethod
    def _merge(items, wx, wy, radius, **extra):
        for it in items:
            if math.hypot(it["x"] - wx, it["y"] - wy) < radius:
                it["x"] = (it["x"] * it["n"] + wx) / (it["n"] + 1)
                it["y"] = (it["y"] * it["n"] + wy) / (it["n"] + 1)
                it["n"] += 1
                return it
        it = {"x": wx, "y": wy, "n": 1}
        if extra:
            it.update(extra, marked=False)
        items.append(it)
        return it

    def _dust_goals(self, pg):
        goals = set()
        for d in self.dust:
            if d["n"] < cfg.DUST_CONFIRM:
                continue
            r0, c0 = self.grid.world_to_cell(d["x"], d["y"])
            best = None
            for dr in (0, -1, 1):
                for dc in (0, -1, 1):
                    if pg.is_passable((r0 + dr, c0 + dc)):
                        best = (r0 + dr, c0 + dc)
                        break
                if best:
                    break
            if best:
                goals.add(best)
        return goals

    # ------------------------- planeación --------------------------
    def _maybe_plan(self, pose):
        need = self.follower.done
        if not need and (self.t - self.last_plan_t) < cfg.PLAN_PERIOD_S:
            return
        self.last_plan_t = self.t
        rc = self.grid.world_to_cell(pose[0], pose[1])
        pg = PlanningGrid(self.grid, rc, extra_passable=self.trail)
        self._rc = rc

        # cambios de modo por basura
        if self.state == CLEANING and self._dust_goals(pg):
            self._set_state(GOTO_DUST)
        elif self.state == GOTO_DUST and not self._dust_goals(pg):
            self._set_state(CLEANING)

        if not need and self._path_still_valid(pg, rc):
            return

        route = self._plan(pg, rc)
        if route:
            self.fail_plans = 0
            self.full_path = route
            self.path_state = self.state
            pts = [self.grid.cell_to_world(c) for c in simplify(route)]
            self.follower.set_path(pts)
        else:
            self.follower.clear()
            self.full_path = []
            if self.state in (CLEANING, GOTO_DUST, RETURN_HOME):
                self.fail_plans += 1
                if self.fail_plans >= 6:
                    self.note = "sin ruta posible: detenido por seguridad"
                    self._set_state(DONE)

    def _path_still_valid(self, pg, rc):
        if not self.full_path or self.path_state != self.state:
            return False
        i0 = min(range(len(self.full_path)),
                 key=lambda i: max(abs(self.full_path[i][0] - rc[0]), abs(self.full_path[i][1] - rc[1])))
        return all(pg.is_passable(c) for c in self.full_path[i0 + 1:])

    def _plan(self, pg, rc):
        if self.state == GOTO_DUST:
            goals = self._dust_goals(pg)
            if goals:
                route, _ = ucs(pg, rc, goals)            # UCS: basura de menor costo
                if route:
                    return route
                self.dust = [d for d in self.dust if d["n"] < cfg.DUST_CONFIRM]  # inalcanzable
            self._set_state(CLEANING)

        if self.state == CLEANING:
            lane = self._lane_route(pg, rc)
            if lane:
                return lane
            goals = {tuple(p) for p in np.argwhere(pg.passable_np & ~self.covered).tolist()}
            if goals:
                if cfg.EXPLORE_PLANNER == "DFS":
                    route = dfs(pg, rc, goals, orden=cfg.DFS_ORDER)
                else:
                    route = bfs(pg, rc, goals)           # BFS: la más cercana por nº de pasos
                if route:
                    return route
            self.note = "limpieza completa: regresando"
            self._set_state(RETURN_HOME)

        if self.state == RETURN_HOME:
            home = self.grid.world_to_cell(0.0, 0.0)
            pg._pass[home[0]][home[1]] = True
            route, _ = ucs(pg, rc, home)                  # UCS: ruta de menor costo a casa
            if route is None:
                # respaldo: inflado reducido (la capa reactiva del LIDAR sigue frenando)
                pg2 = PlanningGrid(self.grid, rc, inflate=max(1, cfg.INFLATE_CELLS - 2),
                                   extra_passable=self.trail)
                pg2._pass[home[0]][home[1]] = True
                route, _ = ucs(pg2, rc, home)
                if route:
                    self.log(f"[{self.t:6.1f}s] ruta a casa con inflado reducido")
            return route
        return None

    def _lane_route(self, pg, rc):
        """Avanzar en línea recta (carril) mientras haya suelo sin limpiar delante.
        Reduce giros y paradas; BFS/DFS solo decide el SIGUIENTE carril."""
        th = self.pose.th
        # dirección cardinal más cercana al rumbo actual (la cuadrícula es de 4 vecinos)
        cands = {(-1, 0): math.pi / 2, (1, 0): -math.pi / 2, (0, 1): 0.0, (0, -1): math.pi}
        dr, dc = min(cands, key=lambda k: abs(math.atan2(math.sin(th - cands[k]), math.cos(th - cands[k]))))
        r, c = rc
        route, last_unc, n_unc = [rc], 0, 0
        while True:
            r, c = r + dr, c + dc
            if not pg.is_passable((r, c)):
                break
            route.append((r, c))
            if not self.covered[r, c]:
                last_unc = len(route) - 1
                n_unc += 1
        if n_unc >= cfg.LANE_MIN_CELLS:
            return route[:last_unc + 1]
        return None

    # ------------------------------------------------------------------
    def status(self):
        x, y, th = self.pose.as_tuple()
        return {"state": self.state, "x": round(x, 2), "y": round(y, 2),
                "th_deg": round(math.degrees(th), 1),
                "batt_mv": None if self.batt is None else round(self.batt),
                "covered_cells": int(self.covered.sum()), "dust": len(self.dust),
                "dust_cleaned": self.dust_cleaned, "hazards": self.hazards_marked, "cliff_events": self.cliff_events,
                "note": self.note}
