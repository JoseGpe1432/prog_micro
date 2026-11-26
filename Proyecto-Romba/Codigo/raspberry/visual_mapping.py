"""Mapa de apariencia experimental. Nunca autoriza movimiento."""
import math
import numpy as np
from vision import GroundProjector
from grid_map import OccupancyGrid

class VisualMapping:
    def __init__(self):
        self.frames=0
        self.last_free_cells=0
        self.last_occupied_cells=0
        self.state='ESPERANDO_CALIBRACION'
        self.last_pose=None
        self.pending_free=set()
        self.pending_occupied=set()
    def update(self, grid, mask, pose, tracks):
        if mask is None: return
        if not all(math.isfinite(v) for v in pose): return
        if self.last_pose is not None:
            a=self.last_pose
            if math.hypot(pose[0]-a[0],pose[1]-a[1])<.015 and abs(math.atan2(math.sin(pose[2]-a[2]),math.cos(pose[2]-a[2])))<.035:
                # Freeze the map until a distinct estimated pose is available.
                self.state="MAPA_ESTABLE_EN_REPOSO"; return
        h,w=mask.shape; proj=GroundProjector(w,h)
        yy,xx=np.mgrid[int(h*.48):int(h*.94):4, 8:w-8:4]
        x,y,ok=proj.pixels_to_floor(xx.ravel(),yy.ravel())
        valid=ok & (x>.18)&(x<.8)&(np.abs(y)<.65)
        # Exclude entire columns above the nearest non-floor region, avoiding inferred floor behind objects.
        import cv2
        horizontal=cv2.dilate((mask>0).astype(np.uint8),np.ones((1,5),np.uint8))
        horizontal[int(h*.96):]=0
        occluded=np.maximum.accumulate(horizontal[::-1],axis=0)[::-1]
        valid &= occluded[yy.ravel(),xx.ravel()]==0
        def cells(x,y):
            a,b,t=pose; c,s=math.cos(t),math.sin(t)
            return {grid.world_to_cell(a+c*i-s*j,b+s*i+c*j) for i,j in zip(x,y)}
        free=cells(x[valid],y[valid]); occupied=set()
        for t in tracks:
            if not t.get('persistent') or t.get('misses'): continue
            u,v,bw,bh=t['box']; bottom=v+bh-1
            if bw>w*.45 or u<=2 or u+bw>=w-2 or v<=int(h*.45)+2 or bottom>=int(h*.95): continue # contact may be outside image
            for px in range(u,u+bw,4):
                p=proj.pixel_to_floor(px,bottom)
                if p and .18<p[0]<.8: occupied |= cells([p[0]],[p[1]])
        # Suelo aparente desde primera vista; ocupación requiere dos vistas.
        for r,c in (free-occupied):
            if grid.in_bounds(r,c) and grid.lo[r,c]<=1.2: grid.lo[r,c]=max(-4.,grid.lo[r,c]-.8)
        for r,c in occupied&self.pending_occupied:
            if grid.in_bounds(r,c): grid.lo[r,c]=min(4.,grid.lo[r,c]+1.3)
        self.pending_free=free-occupied
        self.pending_occupied=occupied
        self.last_free_cells=len(free-occupied); self.last_occupied_cells=len(occupied)
        self.frames+=1; self.last_pose=pose; self.state='MAPA_EXPERIMENTAL_ACTIVO'
    def status(self):
        return {'state':self.state,'frames':self.frames,'last_free_cells':self.last_free_cells,'last_occupied_cells':self.last_occupied_cells,'movement_authorized':False,'warning':'Proyección sobre suelo plano; escala y obstáculos aproximados, no espacio libre verificado.'}
