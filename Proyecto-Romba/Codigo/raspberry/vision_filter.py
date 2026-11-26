"""Confirmación temporal de candidatos visuales. NO mide profundidad ni garantiza paso libre."""
from collections import deque
import time

class VisualFilter:
    def __init__(self, history=5, min_hits=3, lateral_m=0.20, near_m=0.60):
        self.history=deque(maxlen=history)
        self.min_hits=min_hits
        self.lateral_m=lateral_m
        self.near_m=near_m
        self.last_update=0.0
        self.confirmed=[]

    def update(self, detections, avoid_kinds):
        self.last_update=time.monotonic()
        candidates=[d for d in detections if d.kind in avoid_kinds
                    and 0.05 < d.x < self.near_m
                    and abs(d.y) < self.lateral_m + min(d.width,0.20)*0.5]
        self.history.append(candidates)
        confirmed=[]
        for d in candidates:
            hits=sum(any(e.kind==d.kind and abs(e.x-d.x)<0.09
                         and abs(e.y-d.y)<0.09 for e in frame)
                     for frame in self.history)
            if hits>=self.min_hits:
                confirmed.append(d)
        self.confirmed=confirmed
        return confirmed

    def status(self):
        age=time.monotonic()-self.last_update if self.last_update else 1e9
        return {'confirmed':len(self.confirmed),
                'fresh':age<0.65,
                'frames':len(self.history),
                'corridor_half_width_m':self.lateral_m,
                'range_limit_m':self.near_m,
                'warning':'Clasificación por contraste, no detección fiable de espacio libre'}
