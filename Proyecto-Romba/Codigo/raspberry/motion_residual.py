"""V6.1: residuos fotométricos tras alineación entre fotogramas.
Solo diagnóstico. No estima distancia ni ocupación segura.
"""
import time
import cv2
import numpy as np

class MotionResidual:
    def __init__(self):
        self.previous = None
        self.previous_time = None
        self.last = self._empty('INICIALIZANDO')
        self.overlay = None
        self.history = {k: [] for k in ('izquierda','centro','derecha')}

    @staticmethod
    def _empty(state):
        return {'state':state,'tracked_points':0,'inlier_ratio':0.0,
                'overlap_ratio':0.0,'median_error_px':None,
                'global_shift_px':[0.0,0.0],'residual_ratio':0.0,
                'zones':{k:{'residual_ratio':0.0,'persistent':False} for k in ('izquierda','centro','derecha')},
                'candidate_boxes':[], 'frame_age_s':None,
                'movement_authorized':False,'obstacle_confirmed':False,'free_space_verified':False,
                'note':'Residuo visual, NO obstáculo ni ruta segura.'}

    def update(self, frame, timestamp=None):
        now = time.monotonic() if timestamp is None else timestamp
        if frame is None:
            self.last = self._empty('SIN_CAMARA'); self.overlay = None
            return self.status(now)
        small=cv2.resize(frame,(320,240),interpolation=cv2.INTER_AREA)
        gray=cv2.GaussianBlur(cv2.cvtColor(small,cv2.COLOR_BGR2GRAY),(5,5),0)
        if self.previous is None or self.previous.shape!=gray.shape or (self.previous_time is not None and now-self.previous_time>1.5):
            self.previous,self.previous_time=gray,now
            self.last=self._empty('INICIALIZANDO'); self.overlay=small.copy()
            self._clear_history()
            return self.status(now)
        old=self.previous
        self.previous,self.previous_time=gray,now
        pts=cv2.goodFeaturesToTrack(old,maxCorners=300,qualityLevel=.015,minDistance=7,blockSize=7)
        if pts is None or len(pts)<35:
            return self._fail('SEGUIMIENTO_INSUFICIENTE',small,now)
        nxt,st,_=cv2.calcOpticalFlowPyrLK(old,gray,pts,None,winSize=(21,21),maxLevel=3)
        if nxt is None or st is None:
            return self._fail('SEGUIMIENTO_INSUFICIENTE',small,now)
        p0=pts[st.ravel()==1].reshape(-1,2)
        p1=nxt[st.ravel()==1].reshape(-1,2)
        if len(p0)<30:
            return self._fail('SEGUIMIENTO_INSUFICIENTE',small,now)
        matrix,inlier=cv2.estimateAffinePartial2D(p0,p1,method=cv2.RANSAC,ransacReprojThreshold=2.5,maxIters=1500,confidence=.99)
        if matrix is None or inlier is None:
            return self._fail('ALINEACION_FALLIDA',small,now)
        good=inlier.ravel().astype(bool)
        ratio=float(np.mean(good))
        predicted=cv2.transform(p0.reshape(-1,1,2),matrix).reshape(-1,2)
        error=float(np.median(np.linalg.norm(predicted[good]-p1[good],axis=1))) if np.any(good) else 999
        # Alineación global sólo razonable con consenso suficiente.
        if ratio<.65 or good.sum()<30 or error>2.5:
            return self._fail('ALINEACION_NO_CONFIABLE',small,now)
        h,w=gray.shape
        warped=cv2.warpAffine(old,matrix,(w,h),flags=cv2.INTER_LINEAR,borderMode=cv2.BORDER_CONSTANT,borderValue=0)
        valid=cv2.warpAffine(np.full_like(old,255),matrix,(w,h),flags=cv2.INTER_NEAREST,borderMode=cv2.BORDER_CONSTANT)>0
        # Excluir bordes de warp y zona alta, donde perspectiva/paralaje domina.
        valid[:int(h*.35),:]=False
        valid=cv2.erode(valid.astype(np.uint8),np.ones((5,5),np.uint8),iterations=1).astype(bool)
        overlap=float(valid.sum())/max(1,(h-int(h*.35))*w)
        if overlap<.60:
            return self._fail('POCA_SUPERPOSICION',small,now)
        # Compensación de brillo uniforme en región compartida.
        brightness=float(np.median(gray[valid].astype(np.float32)-warped[valid].astype(np.float32)))
        aligned=np.clip(warped.astype(np.float32)+brightness,0,255).astype(np.uint8)
        diff=cv2.absdiff(gray,aligned)
        # Diferencias locales persistentes, NO clasificación de objetos.
        mask=np.uint8((diff>28)&valid)*255
        mask=cv2.morphologyEx(mask,cv2.MORPH_OPEN,np.ones((3,3),np.uint8))
        mask=cv2.morphologyEx(mask,cv2.MORPH_CLOSE,np.ones((5,5),np.uint8))
        bounds={'izquierda':(0,int(w/3)),'centro':(int(w/3),int(2*w/3)),'derecha':(int(2*w/3),w)}
        zones={}
        for key,(x0,x1) in bounds.items():
            area=valid[:,x0:x1]
            value=float(np.count_nonzero(mask[:,x0:x1]))/max(1,int(np.count_nonzero(area)))
            self.history[key].append(value)
            self.history[key]=self.history[key][-5:]
            zones[key]={'residual_ratio':round(value,4),'persistent':len(self.history[key])>=5 and sum(v>.10 for v in self.history[key])>=3}
        contours,_=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        boxes=[]
        for c in contours:
            x,y,bw,bh=cv2.boundingRect(c)
            if cv2.contourArea(c)>=120 and bw*bh>=200:
                boxes.append([int(x),int(y),int(bw),int(bh)])
        boxes=sorted(boxes,key=lambda b:b[2]*b[3],reverse=True)[:12]
        annotated=small.copy()
        tint=annotated.copy(); tint[mask>0]=(0,0,255)
        annotated=cv2.addWeighted(annotated,.7,tint,.3,0)
        for x,y,bw,bh in boxes: cv2.rectangle(annotated,(x,y),(x+bw,y+bh),(0,200,255),1)
        for x in (w//3,2*w//3):cv2.line(annotated,(x,int(h*.35)),(x,h-1),(255,255,0),1)
        self.overlay=annotated
        self.last={'state':'RESIDUOS_CALCULADOS','tracked_points':int(len(p0)),
                   'inlier_ratio':round(ratio,3),'median_error_px':round(error,2),
                   'overlap_ratio':round(overlap,3),
                   'global_shift_px':[round(float(matrix[0,2]),2),round(float(matrix[1,2]),2)],
                   'residual_ratio':round(float(np.count_nonzero(mask))/max(1,int(np.count_nonzero(valid))),4),
                   'zones':zones,'candidate_boxes':boxes,'frame_age_s':0.0,
                   'movement_authorized':False,'obstacle_confirmed':False,'free_space_verified':False,
                   'note':'Diferencias residuales tras alineación: pueden ser paralaje, luz, sombras u objetos. No son distancias ni zonas libres.'}
        return self.status(now)

    def _clear_history(self):
        for k in self.history:self.history[k]=[]

    def _fail(self,state,frame,now):
        self._clear_history();self.overlay=frame.copy()
        self.last=self._empty(state)
        return self.status(now)

    def status(self,now=None):
        d=dict(self.last)
        if self.previous_time is not None:
            d['frame_age_s']=round(max(0,(time.monotonic() if now is None else now)-self.previous_time),2)
            if d['frame_age_s']>1:d['state']='SIN_DATOS_RECIENTES'
        return d

    def annotated_jpeg(self):
        if self.overlay is None:return None
        ok,enc=cv2.imencode('.jpg',self.overlay,[cv2.IMWRITE_JPEG_QUALITY,75])
        return enc.tobytes() if ok else None
