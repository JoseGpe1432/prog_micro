"""Exploración reactiva experimental con permiso renovable del operador."""
import math

def command(seg, quality, pose, history=None):
    if quality not in ('SEGUIMIENTO','REPOSO_CONFIRMADO'):
        return 0.,0.,'ODOMETRIA NO VALIDADA'
    if seg.get('state')!='SEGMENTACION_ACTIVA' or seg.get('frame_age_s',99)>.4:
        return 0.,0.,'SEGMENTACION NO DISPONIBLE'
    if max(abs(pose[0]),abs(pose[1]))>1.15:
        return 0.,0.,'LIMITE ESTIMADO DEL MAPA'
    zones=seg.get('zones',{})
    if not all(k in zones for k in ('centro','izquierda','derecha')):
        return 0.,0.,'ZONAS NO DISPONIBLES'
    c,l,r=(zones[k]['non_floor_ratio'] for k in ('centro','izquierda','derecha'))
    p=seg.get('proximity')
    if p is not None:
        historical = bool((history or {}).get('complete',False))
        if (history is not None and not historical) or (not historical and (not p.get('coverage') or p.get('near_ratio') is None or not p.get('corridor_visible',False))):
            return 0.,0.,'CORREDOR NO VISIBLE: revisar zona ciega y anchura'
        if p['near_blocked']:
            return 0.,0.,'PARADA: REGION CERCANA'
        if seg.get('ambiguous_regions') and p.get('nearest_m') is None:
            return 0.,0.,'ENTORNO INCIERTO'
        if historical and (not p.get('corridor_visible',False) or p.get('near_ratio') is None):
            return .015,0.,'AVANCE LENTO: HISTORIAL VISUAL EXPERIMENTAL'
        if p.get('nearest_m') is not None or c>=.025:
            return .015,0.,'AVANCE LENTO: REGION LEJANA ESTIMADA'
        return .03,0.,'AVANCE EXPERIMENTAL'
    if c<.025 and not seg.get('candidate_boxes') and not seg.get('ambiguous_regions') and max(l,r)<.05:
        return .03,0.,'AVANCE EXPERIMENTAL'
    if seg.get('ambiguous_regions') or min(l,r)>.025:
        return 0.,0.,'ENTORNO INCIERTO'
    return 0., .18 if l<r else -.18,'GIRO EXPERIMENTAL'


def startup_command(seg, quality, pose, safety, history):
    """Short operator-confirmed straight test, never marks unknown cells free."""
    if quality not in ('REPOSO_CONFIRMADO','SEGUIMIENTO'):
        return 0.,0.,'ARRANQUE: ODOMETRIA NO VALIDADA'
    if max(abs(pose[0]),abs(pose[1]))>1.15:
        return 0.,0.,'ARRANQUE: LIMITE DEL MAPA'
    reasons=safety.get('reasons',[])
    if any(r!='CORREDOR SIN OBSERVACIONES SUFICIENTES' for r in reasons):
        return 0.,0.,'ARRANQUE: '+ ' / '.join(reasons)
    p=seg.get('proximity') or {}
    if (not seg.get('calibrated') or seg.get('state')!='SEGMENTACION_ACTIVA'
        or not p.get('coverage') or p.get('near_ratio') is None or p.get('near_blocked')
        or seg.get('ambiguous_regions') or seg.get('candidate_boxes')):
        return 0.,0.,'ARRANQUE: OBSERVACION INCIERTA'
    if not history.get('current_cells',0):
        return 0.,0.,'ARRANQUE: SIN OBSERVACION ACTUAL'
    return .015,0.,'ARRANQUE MANUAL SUPERVISADO: ZONA CIEGA NO VERIFICADA'
