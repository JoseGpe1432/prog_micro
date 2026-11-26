"""Proyección conservadora de regiones no suelo; estimaciones, no sensor de distancia."""
import numpy as np
import config as cfg
from vision import GroundProjector

def assess(mask):
    h,w=mask.shape
    yy,xx=np.mgrid[int(h*.45):int(h*.96):2,0:w:2]
    x,y,ok=GroundProjector(w,h).pixels_to_floor(xx,yy)
    bad=mask[yy,xx]>0
    corridor=ok & (x>0) & (abs(y)<=cfg.ROBOT_RADIUS+.05)
    near=corridor & (x<=cfg.ROBOT_RADIUS+.25)
    mid=corridor & (x<=cfg.ROBOT_RADIUS+.35)
    def ratio(m): return float(np.mean(bad[m])) if m.any() else None
    hits=x[corridor & bad]
    # Cobertura métrica, no cantidad de píxeles: comprueba toda la anchura.
    gx,gy=np.meshgrid(np.linspace(.18,.35,18)+cfg.ROBOT_RADIUS,
                      np.linspace(-cfg.ROBOT_RADIUS-.05,cfg.ROBOT_RADIUS+.05,45))
    u,v,valid=GroundProjector(w,h).floor_to_pixel(gx,gy)
    visible=valid & (u>=0)&(u<w)&(v>=int(h*.45))&(v<int(h*.96))
    fraction=float(visible.mean())
    center=GroundProjector(w,h).pixel_to_floor(w/2,int(h*.96)-1)
    blind=None if center is None else max(0.,float(center[0]-cfg.ROBOT_RADIUS))
    return {'near_ratio':ratio(near),'mid_ratio':ratio(mid),'nearest_m':float(hits.min()-cfg.ROBOT_RADIUS) if hits.size else None,'near_blocked':bool(np.any(bad & near)), 'coverage':int(near.sum()), 'metric_coverage':round(fraction,3), 'corridor_visible':bool(visible.all()), 'blind_front_m_est':blind, 'estimated_only':True}



def annotate(frame, overlay):
    """Orange barrier contact estimates for diagnosis only, not motor permission."""
    import cv2
    h,w=frame.shape[:2]
    proj=GroundProjector(w,h)
    for dist in (.25,.40,.60):
        x=cfg.ROBOT_RADIUS+dist
        u,v,ok=proj.floor_to_pixel(np.array([x,x]),np.array([-cfg.ROBOT_RADIUS-.05,cfg.ROBOT_RADIUS+.05]))
        if ok.all() and np.isfinite(u).all() and np.isfinite(v).all():
            a,b=tuple(np.round([u[0],v[0]]).astype(int)),tuple(np.round([u[1],v[1]]).astype(int))
            cv2.line(overlay,a,b,(255,255,0),2)
            if 0<=a[1]<h: cv2.putText(overlay,f'{dist*100:.0f} cm est.',(max(0,min(w-100,a[0])),max(12,a[1]-4)),0,.45,(255,255,0),1)
    hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
    mask=cv2.inRange(hsv,np.array([4,100,70]),np.array([25,255,255]))
    mask=cv2.morphologyEx(mask,cv2.MORPH_OPEN,np.ones((5,5),np.uint8))
    contours,_=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    out=[]
    for c in sorted(contours,key=cv2.contourArea,reverse=True)[:12]:
        if cv2.contourArea(c)<100: continue
        x,y,bw,bh=cv2.boundingRect(c)
        pts=c.reshape(-1,2); bottom=pts[pts[:,1]>=pts[:,1].max()-2]
        u,v=np.median(bottom,axis=0)
        pos=proj.pixel_to_floor(u,v) if y+bh<h-2 else None
        distance=None if pos is None or pos[0]<=0 else round(pos[0]-cfg.ROBOT_RADIUS,3)
        out.append({'box':[x,y,bw,bh],'front_m_est':distance,'diagnostic_only':True})
        cv2.rectangle(overlay,(x,y),(x+bw,y+bh),(0,165,255),2)
        cv2.putText(overlay,'NARANJA '+('?' if distance is None else f'{distance:.2f} m est.'),(x,max(14,y-3)),0,.45,(0,165,255),1)
    return out
