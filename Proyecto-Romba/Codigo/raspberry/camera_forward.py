"""Avance recto experimental por apariencia y proyección al suelo.
No certifica espacio libre; requiere recinto controlado y permiso mantenido.
"""
import math
import config as cfg

STOP_M = .25
SLOW_M = .40

def evaluate(seg, contrast=(), fresh=True):
    p = seg.get('proximity') or {}
    age = seg.get('frame_age_s')
    reasons = []
    distances = []
    if not seg.get('calibrated') or seg.get('state') != 'SEGMENTACION_ACTIVA' or not isinstance(age,(int,float)) or not math.isfinite(age) or not 0 <= age <= .4:
        reasons.append('SEGMENTACION SIN DATOS')
    if not fresh:
        reasons.append('CONTRASTE SIN DATOS')
    if not p.get('coverage') or p.get('near_ratio') is None:
        reasons.append('SIN MUESTRAS VISIBLES A 25 CM')
    def add(value):
        if isinstance(value,(int,float)) and math.isfinite(value):
            distances.append(value)
        else:
            reasons.append('DISTANCIA INCIERTA')
    if p.get('nearest_m') is not None:
        add(p['nearest_m'])
    if p.get('near_blocked'):
        reasons.append('REGION CERCANA')
    for d in contrast:
        if not math.isfinite(d.x) or not math.isfinite(d.y):
            reasons.append('CONTRASTE INCIERTO')
        elif abs(d.y) <= cfg.ROBOT_RADIUS + .05:
            add(d.x - cfg.ROBOT_RADIUS)
    for d in seg.get('orange_barriers', []):
        add(d.get('front_m_est'))
    if seg.get('ambiguous_regions') and p.get('nearest_m') is None:
        reasons.append('REGION AMBIGUA')
    nearest = min(distances) if distances else None
    if nearest is not None and nearest <= STOP_M + 1e-6:
        reasons.append('OBSTACULO A 25 CM O MENOS (EST.)')
    return {'blocked':bool(reasons),'reasons':reasons,'nearest_m':nearest,
            'stop_m':STOP_M,'blind_front_m_est':p.get('blind_front_m_est'),
            'metric_coverage':p.get('metric_coverage'),'free_space_verified':False}

def command(seg, contrast=(), fresh=True):
    result = evaluate(seg,contrast,fresh)
    if result['blocked']:
        return 0.,0.,'PARADA: '+ ' / '.join(result['reasons'])
    d=result['nearest_m']
    if d is not None and d <= SLOW_M:
        return .015,0.,'AVANCE LENTO: DISTANCIA ESTIMADA'
    return .03,0.,'AVANCE RECTO POR CAMARA (SUPERVISADO)'
