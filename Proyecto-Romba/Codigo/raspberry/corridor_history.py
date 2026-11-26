"""Experimental visual history for a static, supervised test arena.
Never invent observations or import synthetic map cells. Unknown/stale cells stop.
"""
import math
import time
import numpy as np
import config as cfg
from vision import GroundProjector

class CorridorHistory:
    TTL = 30.0
    def __init__(self):
        self.seen = {}
        self.result = {'complete':False,'reason':'SIN OBSERVACIONES'}
        self.updated = None

    def invalidate(self):
        self.seen.clear()
        self.updated = None
        self.result = {'complete':False,'reason':'POSE O IMAGEN NO VALIDADA'}

    def update(self, grid, mask, pose, now=None):
        now = time.monotonic() if now is None else now
        if mask is None or not all(math.isfinite(v) for v in pose):
            self.invalidate(); return
        h,w = mask.shape
        proj = GroundProjector(w,h)
        a,b,t = pose; co,si = math.cos(t),math.sin(t)
        # Project each world cell's centre and four inset corners. Every sample
        # must be inside the segmentation ROI and apparently unoccluded floor.
        rr,cc = np.indices((grid.n,grid.n)); rr=rr.ravel(); cc=cc.ravel()
        wx=(cc-grid.origin)*grid.res; wy=(grid.origin-rr)*grid.res
        good=np.ones(len(rr),bool); bad=np.zeros(len(rr),bool)
        occluded=np.maximum.accumulate((mask>0)[::-1],axis=0)[::-1]
        d=grid.res*.49
        for dx,dy in ((0,0),(-d,-d),(-d,d),(d,-d),(d,d)):
            x=co*(wx+dx-a)+si*(wy+dy-b); y=-si*(wx+dx-a)+co*(wy+dy-b)
            u,v,ok=proj.floor_to_pixel(x,y)
            visible=ok & (x>cfg.CAM_X) & (x<.8) & (u>=2)&(u<w-2)&(v>=int(h*.45)+2)&(v<int(h*.96)-2)
            ui=np.clip(np.rint(u),0,w-1).astype(int); vi=np.clip(np.rint(v),0,h-1).astype(int)
            hit=visible & occluded[vi,ui]
            good &= visible & ~hit; bad |= hit
        self.seen={k:v for k,v in self.seen.items() if 0<=now-v<=self.TTL}
        for r,c in zip(rr[bad],cc[bad]): self.seen.pop((int(r),int(c)),None)
        current={(int(r),int(c)) for r,c in zip(rr[good],cc[good]) if not grid.cliff[r,c] and grid.lo[r,c]<=1.2}
        for k in current: self.seen[k]=now
        # Full swept front rectangle, including immediate blind area, plus
        # side margin. Conservative superset of a forward circular footprint.
        needed=set()
        for x in np.linspace(cfg.ROBOT_RADIUS, cfg.ROBOT_RADIUS+.35, 16):
            for y in np.linspace(-cfg.ROBOT_RADIUS-.05,cfg.ROBOT_RADIUS+.05,21):
                needed.add(grid.world_to_cell(a+co*x-si*y,b+si*x+co*y))
        valid={k for k in needed if grid.in_bounds(*k) and not grid.cliff[k] and grid.lo[k]<=1.2}
        present=valid & current
        previous=(valid & self.seen.keys())-present
        unknown=needed-present-previous
        self.updated=now
        self.result={'complete':bool(needed and not unknown and present),'reason':'COBERTURA VISUAL HISTORICA EXPERIMENTAL' if needed and not unknown and present else 'CORREDOR CON CELDAS SIN OBSERVACION', 'required_cells':len(needed),'current_cells':len(present),'historical_cells':len(previous),'unknown_cells':len(unknown),'history_ttl_s':self.TTL,'free_space_verified':False}

    def status(self, now=None):
        now=time.monotonic() if now is None else now
        result=dict(self.result)
        age=None if self.updated is None else now-self.updated
        result['age_s']=age
        if age is None or not 0<=age<=.4:
            result.update(complete=False,reason='HISTORIAL SIN ACTUALIZACION RECIENTE')
        return result
