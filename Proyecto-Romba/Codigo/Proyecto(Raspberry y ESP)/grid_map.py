"""
grid_map.py
-----------
OccupancyGrid : mapa de ocupación (log-odds) construido con el LIDAR.
PlanningGrid  : "foto" del mapa lista para planear. Expone la misma
                interfaz que el Environment de las prácticas
                (neighbors, neighbors_ordered, cost) para reutilizar
                SIN CAMBIOS bfs.py, dfs.py y ucs.py.

Convención de celdas: (fila, columna). El mundo es (x adelante, y izquierda)
con el origen donde arrancó el robot (centro del mapa).
   col = round(x/res) + origen        fila = origen - round(y/res)
"""
import math
import numpy as np

import config as cfg

L_OCC = 0.85
L_FREE = -0.40
L_MIN, L_MAX = -4.0, 4.0
OCC_T = 1.2      # log-odds por encima: ocupado (≈2 impactos)
FREE_T = -0.7    # log-odds por debajo: libre (≈2 rayos)

DIRECCIONES = {"ARRIBA": (-1, 0), "ABAJO": (1, 0), "IZQUIERDA": (0, -1), "DERECHA": (0, 1)}


def dilate(mask, n):
    """Dilatación cuadrada de n celdas (para inflar obstáculos)."""
    out = mask.copy()
    for _ in range(n):
        m = out.copy()
        m[1:, :] |= out[:-1, :]
        m[:-1, :] |= out[1:, :]
        m[:, 1:] |= out[:, :-1]
        m[:, :-1] |= out[:, 1:]
        m[1:, 1:] |= out[:-1, :-1]
        m[:-1, :-1] |= out[1:, 1:]
        m[1:, :-1] |= out[:-1, 1:]
        m[:-1, 1:] |= out[1:, :-1]
        out = m
    return out


class OccupancyGrid:
    def __init__(self):
        self.res = cfg.MAP_RES
        self.n = int(round(cfg.MAP_SIZE_M / self.res))
        self.origin = self.n // 2
        self.lo = np.zeros((self.n, self.n), dtype=np.float32)
        # Precipicios: permanentes. El LIDAR 've a través' de un escalón y lo marcaría
        # como libre, borrando la marca; por eso van en una máscara aparte.
        self.cliff = np.zeros((self.n, self.n), dtype=bool)

    # -------- conversiones --------
    def world_to_cell(self, x, y):
        return (self.origin - int(round(y / self.res)), int(round(x / self.res)) + self.origin)

    def cell_to_world(self, cell):
        r, c = cell
        return ((c - self.origin) * self.res, (self.origin - r) * self.res)

    def in_bounds(self, r, c):
        return 0 <= r < self.n and 0 <= c < self.n

    # -------- integración del LIDAR --------
    def integrate_scan(self, pose, angles, dists):
        """angles: rad en el marco del robot (antihorario); dists: m.
        Marca libre a lo largo de cada rayo y ocupada la celda del impacto."""
        x, y, th = pose
        lx = x + math.cos(th) * cfg.LIDAR_OFFSET[0] - math.sin(th) * cfg.LIDAR_OFFSET[1]
        ly = y + math.sin(th) * cfg.LIDAR_OFFSET[0] + math.cos(th) * cfg.LIDAR_OFFSET[1]

        ok = (dists >= cfg.LIDAR_MIN_RANGE)
        angles, dists = angles[ok], dists[ok]
        if len(dists) == 0:
            return
        a = angles + th
        maxr = cfg.LIDAR_MAP_RANGE
        d = np.minimum(dists, maxr)
        hit = dists < maxr

        s = np.arange(0.0, maxr, self.res * 0.5, dtype=np.float32)
        ca, sa = np.cos(a)[:, None], np.sin(a)[:, None]
        xs = lx + ca * s[None, :]
        ys = ly + sa * s[None, :]
        free_mask = s[None, :] < (d[:, None] - self.res * 0.6)
        cols = np.rint(xs / self.res).astype(np.int32) + self.origin
        rows = self.origin - np.rint(ys / self.res).astype(np.int32)
        inb = (rows >= 0) & (rows < self.n) & (cols >= 0) & (cols < self.n)
        m = free_mask & inb
        flat_free = np.unique(rows[m] * self.n + cols[m])
        lo = self.lo.reshape(-1)
        lo[flat_free] += L_FREE

        hx = lx + np.cos(a[hit]) * d[hit]
        hy = ly + np.sin(a[hit]) * d[hit]
        hc = np.rint(hx / self.res).astype(np.int32) + self.origin
        hr = self.origin - np.rint(hy / self.res).astype(np.int32)
        hin = (hr >= 0) & (hr < self.n) & (hc >= 0) & (hc < self.n)
        flat_hit = np.unique(hr[hin] * self.n + hc[hin])
        lo[flat_hit] += L_OCC
        np.clip(self.lo, L_MIN, L_MAX, out=self.lo)

    def mark_obstacle(self, x, y, radius):
        """Obstáculo virtual (p. ej. precipicio detectado por los IR)."""
        r0, c0 = self.world_to_cell(x, y)
        k = int(math.ceil(radius / self.res))
        for dr in range(-k, k + 1):
            for dc in range(-k, k + 1):
                if dr * dr + dc * dc <= k * k and self.in_bounds(r0 + dr, c0 + dc):
                    self.lo[r0 + dr, c0 + dc] = L_MAX
                    self.cliff[r0 + dr, c0 + dc] = True


class PlanningGrid:
    """Interfaz tipo Environment para bfs/dfs/ucs."""

    def __init__(self, occ: OccupancyGrid, robot_cell, inflate=None, extra_passable=None):
        lo = occ.lo
        occupied = (lo > OCC_T) | occ.cliff
        free = (lo < FREE_T) & ~occ.cliff
        inflated = dilate(occupied, cfg.INFLATE_CELLS if inflate is None else inflate)
        passable = free & ~inflated
        if extra_passable is not None:
            # 'migajas de pan': por donde el robot ya pasó se puede volver a pasar
            passable |= (extra_passable & ~occupied)
        r0, c0 = robot_cell
        passable[max(0, r0 - 1):r0 + 2, max(0, c0 - 1):c0 + 2] = True   # el robot está aquí

        # distancia (en celdas, Chebyshev) a obstáculos reales, hasta WALL_COST_RANGE
        dist = np.full(occupied.shape, cfg.WALL_COST_RANGE, dtype=np.int16)
        layer = occupied.copy()
        reached = occupied.copy()
        dist[occupied] = 0
        for k in range(1, cfg.WALL_COST_RANGE):
            layer = dilate(layer, 1)
            new = layer & ~reached
            dist[new] = k
            reached |= layer
        costs = 1 + cfg.WALL_COST_GAIN * np.clip(cfg.WALL_COST_RANGE - dist, 0, None)

        self.rows = self.cols = occ.n
        self.passable_np = passable
        self._pass = passable.tolist()
        self._cost = costs.astype(np.int32).tolist()

    def in_bounds(self, pos):
        r, c = pos
        return 0 <= r < self.rows and 0 <= c < self.cols

    def is_passable(self, pos):
        return self.in_bounds(pos) and self._pass[pos[0]][pos[1]]

    def neighbors_ordered(self, pos, orden):
        r, c = pos
        out = []
        for nombre in orden:
            dr, dc = DIRECCIONES[nombre]
            p = (r + dr, c + dc)
            if self.in_bounds(p) and self._pass[p[0]][p[1]]:
                out.append(p)
        return out

    def neighbors(self, pos):
        return self.neighbors_ordered(pos, ("ARRIBA", "ABAJO", "IZQUIERDA", "DERECHA"))

    def cost(self, pos):
        return self._cost[pos[0]][pos[1]]
