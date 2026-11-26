"""Evasión experimental supervisada: no certifica el espacio lateral.
El operador comprueba físicamente todo el espacio de giro antes y durante el permiso.
"""
import math
from camera_forward import evaluate, command

class CameraAvoid:
    def __init__(self):
        self.reset()
    def reset(self):
        self.phase='FORWARD'; self.until=0.; self.sign=0; self.attempts=0
    def status(self):
        return dict(phase=self.phase, direction={1:'izquierda',-1:'derecha'}.get(self.sign),attempts=self.attempts,side_clearance_verified=False)
    def halt(self,msg):
        self.phase='HALT'
        return 0.,0.,msg
    def step(self,seg,contrast,fresh,now):
        r=evaluate(seg,contrast,fresh)
        if self.phase=='HALT': return 0.,0.,'PARADA: SUELTA Y REVISA'
        # No girar ante errores, distancia incierta, o proximidad extrema.
        allowed={'REGION CERCANA','OBSTACULO A 25 CM O MENOS (EST.)'}
        if any(x not in allowed for x in r['reasons']):
            return self.halt('PARADA: '+' / '.join(r['reasons']))
        d=r['nearest_m']
        if r['blocked'] and (d is None or d<=.15):
            return self.halt('PARADA: DISTANCIA INCIERTA O MENOR/IGUAL A 15 CM')
        if self.phase=='FORWARD':
            if not r['blocked']: return command(seg,contrast,fresh)
            self.phase='BRAKE'; self.until=now+.45
            return 0.,0.,'EVASION: FRENANDO ANTES DE GIRAR'
        if self.phase=='TURN' and now>=self.until:
            self.phase='SETTLE'; self.until=now+.45
        if self.phase in ('BRAKE','SETTLE'):
            if now<self.until: return 0.,0.,'EVASION: PAUSA Y REEVALUACION'
            if not r['blocked']:
                self.reset(); return command(seg,contrast,fresh)
            zones=seg.get('zones',{})
            choices=[]
            for name,sign in [('izquierda',1),('derecha',-1)]:
                ratio=zones.get(name,{}).get('non_floor_ratio')
                if not isinstance(ratio,(int,float)) or not math.isfinite(ratio) or not 0<=ratio<=.05: continue
                if any(math.hypot(c.x,c.y)<=.42 and c.y*sign>.05 for c in contrast): continue
                choices.append((ratio,sign))
            if not choices or self.attempts>=8: return self.halt('PARADA: SIN ALTERNATIVA VISUAL O LIMITE DE GIROS')
            if self.sign:
                choices=[c for c in choices if c[1]==self.sign]
                if not choices: return self.halt('PARADA: LADO ELEGIDO YA NO DESPEJADO')
            self.sign=min(choices)[1]; self.attempts+=1
            self.phase='TURN'; self.until=now+.8
        if self.phase=='TURN':
            name='izquierda' if self.sign==1 else 'derecha'
            ratio=seg.get('zones',{}).get(name,{}).get('non_floor_ratio')
            if not isinstance(ratio,(int,float)) or not math.isfinite(ratio) or not 0<=ratio<=.05 or any(math.hypot(c.x,c.y)<=.42 and c.y*self.sign>.05 for c in contrast):
                return self.halt('PARADA: CANDIDATO EN LADO DEL GIRO')
            return 0.,self.sign*.18,'EVASION: GIRO '+name.upper()+' SUPERVISADO'
        return self.halt('PARADA: ESTADO INVALIDO')
