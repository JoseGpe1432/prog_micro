"""Odometría visual experimental del suelo, NO SLAM completo.
Requiere cámara fija, suelo plano, calibración CAM_HEIGHT/CAM_PITCH_DEG/CAM_HFOV_DEG.
No detecta obstáculos ni permite conducción segura por sí sola.
"""
import math
import time
import cv2
import numpy as np
from vision import GroundProjector

class VisualOdometry:
    def __init__(self):
        self.prev_gray = None
        self.prev_pts = None
        self.prev_yaw = None
        self.x = self.y = 0.0
        self.th = 0.0
        self.features = 0
        self.inliers = 0
        self.quality = 'INICIALIZANDO'
        self.last_update = None
        self.path = [[0.0, 0.0]]
        self.detector = cv2.GFTTDetector_create(maxCorners=350, qualityLevel=0.004, minDistance=6, blockSize=5)
        self.proj = None
        self.motion_state = "INDETERMINADO"
        self.last_flow_px = 0.0
        self.cal_start = time.monotonic()
        self.noise = []
        self.yaw_noise = []
        self.flow_threshold = 1.0
        self.yaw_bias = 0.0
        self.yaw_threshold = math.radians(1.0)
        self.prev_time = None
        self.last_observation = None
        self.moving = False
        self.motion_votes = 0
        self.still_votes = 0
        self.calibrated = False
        self.phase_response = 0.0
        self.phase_flow = None
        self.cal_reason = 'ESPERANDO IMAGEN'
        self.cal_retries = 0

    def update(self, frame, yaw_rad=None, motion_state=None):
        if frame is None:
            return self.status()
        now = time.monotonic()
        dt = now-self.prev_time if self.prev_time is not None else 0.0
        self.prev_time = now
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        h,w = gray.shape
        if self.proj is None or (self.proj.w,self.proj.h)!=(w,h):
            self.proj = GroundProjector(w,h)
            self.prev_gray = self.prev_pts = None
        mask = np.zeros_like(gray)
        mask[int(h*.45):int(h*.94), int(w*.08):int(w*.92)] = 255
        yy,xx=np.indices(gray.shape)
        gx,gy,gok=self.proj.pixels_to_floor(xx,yy)
        mask[~gok | (gx<0.20) | (gx>1.5) | (np.abs(gy)>0.8)] = 0
        if self.prev_gray is None :
            self.prev_pts = self.detector.detect(gray, mask)
            self.prev_pts = np.array([p.pt for p in self.prev_pts], dtype=np.float32).reshape(-1,1,2) if self.prev_pts else None
            self.prev_gray = gray
            self.prev_yaw = yaw_rad
            self.features = 0 if self.prev_pts is None else len(self.prev_pts)
            self.quality = 'BUSCANDO PUNTOS' if self.calibrated else 'CALIBRANDO_REPOSO'
            return self.status()
        a = b = np.empty((0,2),np.float32)
        if self.prev_pts is not None and len(self.prev_pts):
            curr, st, err = cv2.calcOpticalFlowPyrLK(self.prev_gray, gray, self.prev_pts, None,
                                                     winSize=(21,21), maxLevel=2,
                                                     criteria=(cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_COUNT,30,.01))
            if curr is None or st is None:
                curr = self.prev_pts.copy(); st = np.zeros((len(curr),1),np.uint8); err = None
            good = st.reshape(-1).astype(bool)
            a = self.prev_pts.reshape(-1,2)[good]
            b = curr.reshape(-1,2)[good]
            if err is not None:
                e=err.reshape(-1)[good]; a=a[e<30];b=b[e<30]
            # Rechazar correspondencias inconsistentes mediante flujo inverso.
            if len(b):
                back, bst, _ = cv2.calcOpticalFlowPyrLK(gray, self.prev_gray, b.reshape(-1,1,2), None, winSize=(21,21), maxLevel=2, criteria=(cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_COUNT,30,.01))
                if back is not None and bst is not None:
                    keep = (bst.ravel()==1) & (np.linalg.norm(back.reshape(-1,2)-a,axis=1)<1.0)
                    a,b=a[keep],b[keep]
                else: a,b=a[:0],b[:0]
        self.features=len(b)
        self.inliers=0
        self.motion_state = motion_state or 'INDETERMINADO'
        self.last_flow_px = float(np.median(np.linalg.norm(b-a,axis=1))) if len(b) else 0.0
        # Medir ruido con el robot físicamente quieto tras inicio/reinicio.
        raw_yaw = None if yaw_rad is None or self.prev_yaw is None else math.atan2(math.sin(yaw_rad-self.prev_yaw), math.cos(yaw_rad-self.prev_yaw))
        valid_sample = len(b)>=25 and raw_yaw is not None and 0.01<dt<1.0
        # Correlación de fase exclusivamente para confirmar reposo, nunca para integrar traslación.
        roi = np.s_[int(h*.48):int(h*.90),int(w*.12):int(w*.88)]
        g0 = self.prev_gray[roi].astype(np.float32)
        g1 = gray[roi].astype(np.float32)
        self.phase_flow = None; self.phase_response = 0.0
        if min(g0.shape)>=16 and min(g0.std(),g1.std())>=3:
            window = cv2.createHanningWindow((g0.shape[1],g0.shape[0]),cv2.CV_32F)
            shift,response = cv2.phaseCorrelate(g0,g1,window)
            if np.isfinite(shift).all() and np.isfinite(response):
                self.phase_flow = math.hypot(*shift); self.phase_response = float(response)
        phase_still = self.phase_flow is not None and self.phase_response>=.65 and self.phase_flow<=.6
        lk_still = len(b)>=25 and self.last_flow_px<=.7
        # Si LK informa movimiento grande pero fase estable, exigir también similitud directa.
        direct_still = float(np.mean(np.abs(self.prev_gray[roi].astype(np.float32)-gray[roi].astype(np.float32))))<=3.0
        rest_visual = (lk_still and (self.phase_flow is None or self.phase_response<.65 or self.phase_flow<=.8)) or (phase_still and direct_still)
        yaw_valid = raw_yaw is not None and .01<dt<1.0
        if not self.calibrated:
            if yaw_valid and rest_visual and abs(raw_yaw/dt)<math.radians(5):
                self.noise.append(min(self.last_flow_px,.7) if lk_still else self.phase_flow)
                self.yaw_noise.append(raw_yaw/dt)
                self.cal_reason='MUESTRA VALIDA'
            else:
                self.noise.clear(); self.yaw_noise.clear()
                self.cal_reason='SIN IMU O INTERVALO INVALIDO' if not yaw_valid else 'MOVIMIENTO O IMAGEN NO COHERENTE'
            if now-self.cal_start>=5 and len(self.noise)>=8:
                self.flow_threshold=float(np.clip(np.percentile(self.noise,95)*1.8+.5,.5,2.0))
                self.yaw_bias=float(np.median(self.yaw_noise))
                self.yaw_threshold=max(math.radians(1),float(np.percentile(np.abs(np.array(self.yaw_noise)-self.yaw_bias),95))*2)
                self.calibrated=True
            elif now-self.cal_start>=15:
                self.cal_start=now; self.cal_retries+=1; self.noise.clear(); self.yaw_noise.clear()
            self.quality='CALIBRANDO_REPOSO'
            self.prev_gray=gray; self.prev_pts=self._points(gray,mask); self.prev_yaw=yaw_rad
            return self.status()
        corrected_yaw = raw_yaw-self.yaw_bias*dt if yaw_valid else 0.0
        # Exigir evidencia visual de suelo; yaw solo no autoriza integrar.
        strong = valid_sample and not rest_visual and (self.last_flow_px>=self.flow_threshold or (self.last_flow_px>=1.2 and abs(corrected_yaw)/dt>=self.yaw_threshold))
        weak = valid_sample and not rest_visual and (self.last_flow_px>=self.flow_threshold*.75 or (self.last_flow_px>=1.2 and abs(corrected_yaw)/dt>=self.yaw_threshold*.75))
        self.motion_votes=self.motion_votes+1 if strong else 0
        still = yaw_valid and rest_visual and abs(corrected_yaw)/dt < self.yaw_threshold
        self.still_votes=self.still_votes+1 if still else 0
        if self.motion_votes>=2: self.moving=True
        if self.still_votes>=3: self.moving=False
        if not (self.moving and weak):
            self.prev_gray=gray; self.prev_pts=self._points(gray,mask); self.prev_yaw=yaw_rad
            self.inliers=0
            self.quality='REPOSO_CONFIRMADO' if still and self.still_votes>=3 else 'MOVIMIENTO_NO_VALIDADO'
            if self.quality=='REPOSO_CONFIRMADO': self.last_observation=now
            return self.status()
        if len(b)<18:
            self.prev_pts=self._points(gray,mask);self.prev_gray=gray;self.prev_yaw=yaw_rad;self.quality='POCOS PUNTOS'
            return self.status()
        ax,ay,ak=self.proj.pixels_to_floor(a[:,0],a[:,1])
        bx,by,bk=self.proj.pixels_to_floor(b[:,0],b[:,1])
        valid=ak&bk&(ax>0.04)&(bx>0.04)&(ax<2.5)&(bx<2.5)
        if valid.sum()<15:
            self.prev_pts=self._points(gray,mask);self.prev_gray=gray;self.prev_yaw=yaw_rad;self.quality='SUELO NO VISIBLE'
            return self.status()
        pa=np.stack([ax[valid],ay[valid]],axis=1)
        pb=np.stack([bx[valid],by[valid]],axis=1)
        # Si la IMU está disponible, compensa la rotación entre fotogramas.
        dth=0.0
        if yaw_rad is not None and self.prev_yaw is not None:
            dth=corrected_yaw if abs(corrected_yaw)/dt>=self.yaw_threshold*.75 else 0.0
        c,s=math.cos(dth),math.sin(dth)
        rot=np.array([[c,-s],[s,c]])
        deltas=pa-pb@rot.T
        med=np.median(deltas,axis=0)
        residual=np.linalg.norm(deltas-med,axis=1)
        inliers=residual<0.045
        self.inliers=int(inliers.sum())
        if self.inliers>=12:
            dx,dy=np.median(deltas[inliers],axis=0)
            # Rechaza saltos físicamente implausibles entre fotogramas.
            if np.linalg.norm([dx,dy])<0.07:
                heading=self.th+0.5*dth
                self.x+=float(dx*math.cos(heading)-dy*math.sin(heading))
                self.y+=float(dx*math.sin(heading)+dy*math.cos(heading))
                self.th+=dth
                self.last_update=time.monotonic()
                self.last_observation=now
                self.quality='SEGUIMIENTO' if self.inliers>=25 else 'DEBIL'
                if not self.path or math.hypot(self.x-self.path[-1][0],self.y-self.path[-1][1])>.01:
                    self.path.append([round(self.x,3),round(self.y,3)])
                    self.path=self.path[-1000:]
            else:self.quality='SALTO RECHAZADO'
        else:self.quality='MOVIMIENTO INCIERTO'
        self.prev_gray=gray
        self.prev_pts=self._points(gray,mask)
        self.prev_yaw=yaw_rad
        return self.status()

    def _points(self, gray, mask):
        pts=self.detector.detect(gray,mask)
        return np.array([p.pt for p in pts],dtype=np.float32).reshape(-1,1,2) if pts else None

    def status(self):
        return {'quality':self.quality,'motion_state':self.motion_state,'flow_px':round(self.last_flow_px,3),'features':self.features,'inliers':self.inliers,
                'x':round(self.x,3),'y':round(self.y,3),
                'heading_deg':round(math.degrees(self.th),1),
                'calibrated_rest':self.calibrated,'calibration_samples':len(self.noise),'calibration_reason':self.cal_reason,'calibration_retries':self.cal_retries,'phase_response':round(self.phase_response,3),'phase_flow_px':None if self.phase_flow is None else round(self.phase_flow,3),'flow_threshold_px':round(self.flow_threshold,3),'yaw_bias_deg_s':round(math.degrees(self.yaw_bias),3),'yaw_threshold_deg_s':round(math.degrees(self.yaw_threshold),3),'motion_votes':self.motion_votes,'still_votes':self.still_votes,
                'observation_recent':self.last_observation is not None and time.monotonic()-self.last_observation<0.6,
                'recent':self.last_update is not None and time.monotonic()-self.last_update<2.0,
                'path':list(self.path)}
