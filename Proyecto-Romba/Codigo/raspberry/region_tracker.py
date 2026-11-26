"""V6.3: asociación temporal de cajas residuales. SOLO diagnóstico.

No es un detector de obstáculos ni certifica espacio libre.
"""
import time
import math
import numpy as np
from collections import deque


class RegionTracker:
    def __init__(self, min_hits=3, max_misses=5, max_center_px=45, max_tracks=16):
        self.min_hits = min_hits
        self.max_misses = max_misses
        self.max_center_px = max_center_px
        self.max_tracks = max_tracks
        self.tracks = {}
        self.next_id = 1
        self.last_time = None
        self.state = 'INICIALIZANDO'
        self.lost_ids = []
        self.intervals = deque(maxlen=30)
        self.processing_ms = 0.0

    @staticmethod
    def _center(box):
        x, y, w, h = box
        return (x + w / 2.0, y + h / 2.0)

    @staticmethod
    def _iou(a, b):
        x0 = max(a[0], b[0]); y0 = max(a[1], b[1])
        x1 = min(a[0] + a[2], b[0] + b[2]); y1 = min(a[1] + a[3], b[1] + b[3])
        intersection = max(0, x1 - x0) * max(0, y1 - y0)
        union = a[2]*a[3] + b[2]*b[3] - intersection
        return intersection / union if union > 0 else 0.0

    def reset(self, state='SEGUIMIENTO_PERDIDO'):
        self.lost_ids = sorted(self.tracks)
        self.tracks.clear()
        self.state = state
        self.last_time = None
        self.intervals.clear()

    def update(self, boxes, valid=True, timestamp=None, global_shift_px=(0, 0)):
        start = time.perf_counter()
        now = time.monotonic() if timestamp is None else timestamp
        if not valid or (self.last_time is not None and (now < self.last_time or now - self.last_time > 1.5)):
            self.reset('SEGUIMIENTO_PERDIDO')
            if not valid:
                return self.status(now)
        dt = now - self.last_time if self.last_time is not None else 0.15
        if dt > 0: self.intervals.append(dt)
        dt = max(0.01, dt)
        self.last_time = now
        self.lost_ids = []
        candidates = [list(map(int, b)) for b in boxes if len(b) == 4 and b[2] > 0 and b[3] > 0][:self.max_tracks]
        # Predicción aproximada del desplazamiento global de escena entre fotogramas.
        dx, dy = (float(global_shift_px[0]), float(global_shift_px[1]))
        edges = []
        for tid, t in self.tracks.items():
            b = t['box']
            F = np.eye(4); F[0,2] = F[1,3] = dt
            t['x'] = F @ t['x'] + np.array([dx,dy,0,0])
            t['P'] = F @ t['P'] @ F.T + np.diag([9,9,100,100])*dt
            predicted = [t['x'][0]-b[2]/2, t['x'][1]-b[3]/2, b[2], b[3]]
            t['box'] = [int(round(v)) for v in predicted]
            cx, cy = self._center(predicted)
            for j, candidate in enumerate(candidates):
                px, py = self._center(candidate)
                distance = math.hypot(cx - px, cy - py)
                iou = self._iou(predicted, candidate)
                if iou >= 0.08 or distance <= self.max_center_px:
                    # Preferir solapamiento; distancia sólo como alternativa.
                    edges.append((2 * iou - distance / (self.max_center_px * 4), tid, j))
        edges.sort(reverse=True)
        assigned_tracks = set(); assigned_boxes = set()
        for _, tid, j in edges:
            if tid in assigned_tracks or j in assigned_boxes:
                continue
            t = self.tracks[tid]
            H = np.eye(4)[:2]; R = np.eye(2)*16
            K = t['P'] @ H.T @ np.linalg.inv(H @ t['P'] @ H.T + R)
            t['x'] += K @ (np.array(self._center(candidates[j])) - H @ t['x'])
            A = np.eye(4)-K@H
            t['P'] = A@t['P']@A.T + K@R@K.T
            t['box'] = candidates[j]
            t['hits'] += 1
            t['misses'] = 0
            t['last_seen'] = now
            assigned_tracks.add(tid); assigned_boxes.add(j)
        for tid in list(self.tracks):
            if tid not in assigned_tracks:
                self.tracks[tid]['misses'] += 1
                if self.tracks[tid]['misses'] > self.max_misses or now-self.tracks[tid]['last_seen'] > 1.5:
                    self.lost_ids.append(tid)
                    del self.tracks[tid]
        for j, b in enumerate(candidates):
            if j not in assigned_boxes and len(self.tracks) < self.max_tracks:
                tid = self.next_id; self.next_id += 1
                self.tracks[tid] = {'box': b, 'hits': 1, 'misses': 0, 'first_seen': now, 'last_seen': now, 'x': np.array([*self._center(b),0.,0.]), 'P': np.diag([25.,25.,400.,400.])}
        self.state = 'SEGUIMIENTO_ACTIVO' if self.tracks else 'SIN_CANDIDATOS'
        self.processing_ms = (time.perf_counter()-start)*1000
        return self.status(now)

    def status(self, timestamp=None):
        now = time.monotonic() if timestamp is None else timestamp
        items = []
        for tid, t in sorted(self.tracks.items()):
            persistent = t['hits'] >= self.min_hits and t['misses'] == 0 and now-t['last_seen'] <= 0.6
            items.append({'id': tid, 'box': t['box'], 'hits': t['hits'],
                          'misses': t['misses'], 'age_s': round(max(0, now - t['first_seen']), 2),
                          'state': ('SEGUIMIENTO_PERDIDO' if t['misses'] or now-t['last_seen'] > 0.6 else
                                    'CANDIDATO_PERSISTENTE' if persistent else 'CANDIDATO_NUEVO'),
                          'persistent': persistent})
        return {'state': self.state, 'tracks': items, 'lost_ids': list(self.lost_ids),
                'update_hz': round(len(self.intervals)/sum(self.intervals),2) if self.intervals else 0,
                'tracker_ms': round(self.processing_ms,2),
                'persistent_count': sum(t['persistent'] for t in items),
                'movement_authorized': False, 'obstacle_confirmed': False,
                'free_space_verified': False, 'map_updated': False,
                'note': 'Persistencia de residuos visuales; NO demuestra obstáculo ni espacio libre.'}
