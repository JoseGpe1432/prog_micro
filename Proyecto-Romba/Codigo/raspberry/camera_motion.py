"""V6: diagnóstico de movimiento global entre fotogramas. NO detecta espacio libre."""
import time
import cv2
import numpy as np

class CameraMotion:
    def __init__(self):
        self.previous = None
        self.previous_time = None
        self.last = self._empty('SIN_DATOS')
        self.motion_streak = 0
        self.still_streak = 0
        self.reference_invalidated = False

    @staticmethod
    def _empty(state):
        return dict(state=state, tracked_points=0, inlier_ratio=0.0,
                    median_displacement_px=0.0, rotation_deg=0.0,
                    dx_px=0.0, dy_px=0.0, frame_age_s=None,
                    reference_invalidated=False, movement_authorized=False,
                    obstacle_detected=False, free_space_verified=False,
                    note='Flujo óptico global: no mide profundidad, obstáculos ni desniveles.')

    def reset_reference(self):
        # No declarar inmovilidad al calibrar: solo reiniciar el bloqueo de referencia.
        self.reference_invalidated = False
        self.motion_streak = 0
        self.still_streak = 0

    def update(self, frame, timestamp=None):
        now = time.monotonic() if timestamp is None else timestamp
        if frame is None:
            self.last = self._empty('SIN_CAMARA')
            return self.status(now)
        gray = cv2.cvtColor(cv2.resize(frame, (320, 240)), cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5,5), 0)
        if self.previous is None or self.previous.shape != gray.shape or (self.previous_time is not None and now-self.previous_time > 1.5):
            self.previous, self.previous_time = gray, now
            self.last = self._empty('INICIALIZANDO')
            return self.status(now)
        self._old_gray = self.previous
        pts = cv2.goodFeaturesToTrack(self.previous, maxCorners=250, qualityLevel=.015, minDistance=8, blockSize=7)
        self.previous, old_time = gray, self.previous_time
        self.previous_time = now
        if pts is None or len(pts)<25:
            self.last = self._empty('SEGUIMIENTO_INSUFICIENTE')
            self.still_streak = 0
            return self.status(now)
        # pts pertenecen al fotograma anterior: guardado antes de sobrescribir.
        # Se utiliza la copia old_gray conservada abajo mediante el campo temporal.
        old_gray = self._old_gray
        nxt, st, _ = cv2.calcOpticalFlowPyrLK(old_gray, gray, pts, None, winSize=(21,21), maxLevel=3)
        if nxt is None or st is None:
            self.last = self._empty('SEGUIMIENTO_INSUFICIENTE')
            return self.status(now)
        p0, p1 = pts[st.ravel()==1].reshape(-1,2), nxt[st.ravel()==1].reshape(-1,2)
        if len(p0)<20:
            self.last = self._empty('SEGUIMIENTO_INSUFICIENTE')
            return self.status(now)
        mat, mask = cv2.estimateAffinePartial2D(p0,p1,method=cv2.RANSAC,ransacReprojThreshold=2.5)
        if mat is None or mask is None:
            self.last = self._empty('SEGUIMIENTO_INSUFICIENTE')
            return self.status(now)
        inliers = mask.ravel().astype(bool)
        ratio = float(inliers.mean())
        shifts = p1[inliers]-p0[inliers]
        displacement = float(np.median(np.linalg.norm(shifts,axis=1))) if len(shifts) else 0.0
        rotation = float(np.degrees(np.arctan2(mat[1,0],mat[0,0])))
        # Umbrales conservadores en imagen reducida 320x240; ajustar con pruebas reales.
        detected = (displacement > 2.0 or abs(rotation)>0.8) and ratio>=.55 and len(shifts)>=18
        still = displacement < .6 and abs(rotation)<.25 and ratio>=.65 and len(shifts)>=25
        self.motion_streak = self.motion_streak+1 if detected else 0
        self.still_streak = self.still_streak+1 if still else 0
        if self.motion_streak>=2:
            self.reference_invalidated=True
        state = ('MOVIMIENTO_DETECTADO' if self.motion_streak>=2 else
                 'APARENTEMENTE_ESTABLE' if self.still_streak>=4 else
                 'INDETERMINADO')
        self.last = dict(state=state,tracked_points=int(len(p0)),inlier_ratio=round(ratio,3),
                         median_displacement_px=round(displacement,2),rotation_deg=round(rotation,2),
                         dx_px=round(float(np.median(shifts[:,0])),2) if len(shifts) else 0.0,
                         dy_px=round(float(np.median(shifts[:,1])),2) if len(shifts) else 0.0,
                         frame_age_s=0.0,reference_invalidated=self.reference_invalidated,
                         movement_authorized=False,obstacle_detected=False,free_space_verified=False,
                         note='Movimiento aparente de escena, no velocidad física ni distancia segura.')
        return self.status(now)

    def status(self, now=None):
        d=dict(self.last)
        d['reference_invalidated']=self.reference_invalidated
        if self.previous_time is not None:
            d['frame_age_s']=round(max(0.0,(time.monotonic() if now is None else now)-self.previous_time),2)
            if d['frame_age_s']>1.0: d['state']='SIN_DATOS_RECIENTES'
        return d
