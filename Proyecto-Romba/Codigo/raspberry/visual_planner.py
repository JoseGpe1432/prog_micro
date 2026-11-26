"""Interfaz de inspección visual para los cinco planificadores.

SOLO DIAGNÓSTICO: una referencia de cámara fija no es un mapa de espacio libre.
Las prioridades NO cambian celdas de ocupación, rutas ni órdenes de motores.
"""
import math
import time

NAMES = ('izquierda', 'centro', 'derecha')
PLANNERS = ('BFS', 'DFS', 'UCS', 'GREEDY', 'ASTAR')


def evaluate(reference, planner='BFS', goal_bearing=None, age_seconds=None):
    """Devuelve zonas observadas y prioridad de inspección, nunca permiso de movimiento.

    goal_bearing: radianes relativos al frente (+izquierda), si existe una
    orientación de meta calibrada y fiable. Sin ella, GREEDY/ASTAR usan
    únicamente prioridades visuales, explícitamente NO heurística de meta.
    """
    result = {
        'planner': planner,
        'zones': {k: 'DESCONOCIDO' for k in NAMES},
        'evaluation_suggested': 'DETENER_Y_REEVALUAR',
        'inspection_order': [],
        'reason': 'No existe referencia visual válida',
        'route_authorized': False,
        'movement_authorized': False,
        'map_updated_from_camera': False,
        'reference_only_stationary': True,
        'goal_guidance_used': False,
    }
    if planner not in PLANNERS:
        result['reason'] = 'Algoritmo no reconocido'
        return result
    if not reference or not reference.get('calibrated'):
        return result
    if age_seconds is not None and age_seconds > 1.0:
        result['reason'] = 'Observación visual desactualizada'
        return result
    if int(reference.get('frames', 0)) < 5:
        result['reason'] = 'Esperando confirmación temporal (5 muestras)'
        return result
    zones = reference.get('zones', {})
    if not all(k in zones and isinstance(zones[k], dict) for k in NAMES):
        result['reason'] = 'Faltan mediciones de zonas'
        return result
    confirmed = {k: bool(zones[k].get('confirmed_change', False)) for k in NAMES}
    ratios = {k: float(zones[k].get('changed_ratio', 1.0)) for k in NAMES}
    for k in NAMES:
        result['zones'][k] = 'CAMBIO_CONFIRMADO' if confirmed[k] else 'SIN_CAMBIO_CONFIRMADO'
    # No priorizar el lado con cambio confirmado. No implica que el otro sea seguro.
    candidates = [k for k in ('izquierda', 'derecha') if not confirmed[k]]
    if not candidates:
        result['reason'] = 'Ambos laterales presentan cambios confirmados'
        return result
    if not any(confirmed.values()):
        result['reason'] = 'No hay cambios confirmados; sin evidencia de espacio libre'
        return result
    if planner == 'BFS':
        order = sorted(candidates, key=lambda k: ('izquierda', 'derecha').index(k))
        method = 'Orden fijo de inspección para BFS (no modifica su búsqueda)'
    elif planner == 'DFS':
        order = sorted(candidates, key=lambda k: ('derecha', 'izquierda').index(k))
        method = 'Orden fijo de inspección para DFS (no modifica su búsqueda)'
    else:
        # Preferencia de inspección, no coste de arista ni probabilidad de seguridad.
        if goal_bearing is not None and math.isfinite(goal_bearing) and planner in ('GREEDY', 'ASTAR'):
            preferred = 'izquierda' if goal_bearing > 0 else 'derecha'
            goal_penalty = {k: (0.0 if k == preferred else 0.2) for k in candidates}
            result['goal_guidance_used'] = True
        else:
            goal_penalty = {k: 0.0 for k in candidates}
        if planner == 'GREEDY':
            order = sorted(candidates, key=lambda k: (goal_penalty[k], ratios[k], k))
        elif planner == 'ASTAR':
            order = sorted(candidates, key=lambda k: (ratios[k] + goal_penalty[k], k))
        else:  # UCS
            order = sorted(candidates, key=lambda k: (ratios[k], k))
        method = 'Prioridad de inspección por cambio visual; no es coste/ruta de ' + planner
    result['inspection_order'] = order
    result['evaluation_suggested'] = 'EVALUAR_' + order[0].upper()
    result['reason'] = method + '. Solo observar, sin activar motores.'
    return result
