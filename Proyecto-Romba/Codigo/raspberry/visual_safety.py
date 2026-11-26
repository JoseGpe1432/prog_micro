"""Single conservative decision from current visual evidence; not a range sensor."""
import math
import config as cfg

def evaluate(seg, contrast, fresh, history=None):
    p=seg.get('proximity') or {}
    age=seg.get('frame_age_s')
    reasons=[]
    if seg.get('state')!='SEGMENTACION_ACTIVA' or age is None or age>.4: reasons.append('SEGMENTACION SIN DATOS')
    if not fresh: reasons.append('CONTRASTE SIN DATOS')
    if not ((history or {}).get('complete',False) if history is not None else p.get('corridor_visible',False)): reasons.append('CORREDOR SIN OBSERVACIONES SUFICIENTES')
    if p.get('near_blocked'): reasons.append('REGION CERCANA SEGMENTACION')
    if any(not math.isfinite(d.x) or d.x<=cfg.ROBOT_RADIUS+.18 for d in contrast): reasons.append('CONTRASTE CERCANO')
    if seg.get('ambiguous_regions') and p.get('nearest_m') is None: reasons.append('REGION AMBIGUA')
    orange=seg.get('orange_barriers',[])
    if any(isinstance(d.get('front_m_est'),(int,float)) and d['front_m_est']<=.18 for d in orange): reasons.append('NARANJA CERCANO ESTIMADO')
    return {'blocked':bool(reasons),'reasons':reasons,'metric_coverage':p.get('metric_coverage'),'blind_front_m_est':p.get('blind_front_m_est'),'turn_authorized':False,'free_space_verified':False}
