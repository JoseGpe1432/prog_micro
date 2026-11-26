"""V6.4: modelo de apariencia del suelo, exclusivamente diagnóstico."""
import cv2
import numpy as np
import time
from region_tracker import RegionTracker
from proximity import assess, annotate

class FloorSegmentation:
    def __init__(self):
        self.model = None
        self.ambiguous_count = 0
        self.history = []
        self.tracker = RegionTracker()
        self.mask = None
        self.overlay = None
        self.updated = None
        self.result = {'state':'SIN_CALIBRAR'}

    @staticmethod
    def features(frame):
        smooth = cv2.GaussianBlur(frame, (5,5), 0)
        lab = cv2.cvtColor(smooth, cv2.COLOR_BGR2LAB).astype(np.float32)
        hsv = cv2.cvtColor(smooth, cv2.COLOR_BGR2HSV).astype(np.float32)
        return np.stack([lab[:,:,1], lab[:,:,2], lab[:,:,0], hsv[:,:,1]], axis=-1)

    def calibrate(self, frame):
        if frame is None or min(frame.shape[:2]) < 80:
            return False, 'Sin imagen válida'
        h,w = frame.shape[:2]
        f = self.features(frame)[int(h*.72):int(h*.93), int(w*.25):int(w*.75)]
        med = np.median(f.reshape(-1,4), axis=0)
        mad = np.median(np.abs(f.reshape(-1,4)-med), axis=0)
        self.model = (med, np.maximum(mad*1.4826, [7,7,18,22]))
        self.history=[]
        self.tracker.reset(); self.mask = None; self.overlay = None; self.updated = None
        self.result = {'state':'CALIBRADO_ESPERANDO_IMAGEN'}
        return True, 'Modelo de suelo calibrado; solo diagnóstico'

    def clear(self):
        self.__init__()

    def update(self, frame, now):
        if self.model is None: return
        h,w = frame.shape[:2]; y0=int(h*.45); y1=int(h*.96)
        med, scale = self.model
        z = np.abs(self.features(frame)-med)/scale
        mask = ((np.maximum(z[:,:,0],z[:,:,1])>3) | (z[:,:,2]>3) | (z[:,:,3]>3)).astype(np.uint8)*255
        mask[:y0]=0; mask[y1:]=0
        mask=cv2.morphologyEx(mask,cv2.MORPH_OPEN,np.ones((5,5),np.uint8))
        mask=cv2.morphologyEx(mask,cv2.MORPH_CLOSE,np.ones((7,7),np.uint8))
        self.ambiguous_count=0
        # Mayoría temporal solo para escena aparentemente quieta.
        if getattr(self, 'stationary', False):
            self.history.append(mask.copy()); self.history=self.history[-3:]
            if len(self.history)==3:
                mask=(np.count_nonzero(self.history,axis=0)>=2).astype(np.uint8)*255
        else: self.history=[]
        n, labels, stats, _ = cv2.connectedComponentsWithStats(mask)
        clean=np.zeros_like(mask); boxes=[]; seam_candidates=[]
        for i in range(1,n):
            x,y,bw,bh,area=map(int,stats[i])
            if area < max(70,h*w*.0005) or min(bw,bh)<6: continue
            clean[labels==i]=255
            # Una línea delgada acromática puede ser junta o cable; solo etiquetar.
            elong=max(bw,bh)/max(1,min(bw,bh))
            if elong>=5 and min(bw,bh)<=max(8,int(w*.02)):
                seam_candidates.append([x,y,bw,bh])
            # Regiones extensas o pegadas al borde siguen siendo desconocidas,
            # pero no se proyectan como objetos individuales.
            if bw>w*.45 or x<=2 or x+bw>=w-2 or y<=y0+2 or y+bh>=y1-2:
                self.ambiguous_count+=1; continue
            boxes.append([x,y,bw,bh])
        boxes.sort(key=lambda b:b[2]*b[3],reverse=True)
        tracks=self.tracker.update(boxes,timestamp=now)
        zones={}
        for name,a,b in [('izquierda',0,w//3),('centro',w//3,2*w//3),('derecha',2*w//3,w)]:
            ratio=float(np.mean(clean[y0:y1,a:b]>0))
            zones[name]={'non_floor_ratio':round(ratio,4)}
        order=sorted(['derecha','izquierda'],key=lambda k:zones[k]['non_floor_ratio'])
        overlay=frame.copy(); roi=overlay[y0:y1]; m=clean[y0:y1]>0
        tint=np.zeros_like(roi); tint[:]=(0,140,0); tint[m]=(0,0,255)
        overlay[y0:y1]=cv2.addWeighted(roi,.65,tint,.35,0)
        for t in tracks['tracks']:
            if t['misses']: continue
            x,y,bw,bh=t['box']; cv2.rectangle(overlay,(x,y),(x+bw,y+bh),(0,0,255),2)
            cv2.putText(overlay,'ID %s'%t['id'],(max(0,x),max(12,y-4)),0,.45,(255,255,255),1)
        for x in (w//3,2*w//3): cv2.line(overlay,(x,y0),(x,y1),(255,255,0),1)
        orange = annotate(frame, overlay)
        self.mask=clean; self.overlay=overlay; self.updated=now
        self.result={'state':'SEGMENTACION_ACTIVA','zones':zones,'candidate_boxes':boxes[:16], 'ambiguous_regions':self.ambiguous_count, 'inspection_order':order,'evaluation_suggested':'EVALUAR_'+order[0].upper(),'tracking':tracks,'proximity':assess(clean),'orange_barriers':orange,'possible_seams':seam_candidates}

    def status(self):
        age=None if self.updated is None else max(0,time.monotonic()-self.updated)
        result=dict(self.result)
        if age is not None and age>1:
            result={'state':'SIN_DATOS_RECIENTES'}
        return dict(result,calibrated=self.model is not None,frame_age_s=age, movement_authorized=False,free_space_verified=False,map_updated=False,warning='Apariencia del suelo, no distancia ni seguridad; no controla motores.')
