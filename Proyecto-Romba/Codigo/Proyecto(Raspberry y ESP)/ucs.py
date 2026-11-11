"""
ucs.py
------
Uniform Cost Search (búsqueda de costo uniforme) con cola de prioridad.
No depende de Pygame: solo necesita un objeto 'environment' que ofrezca
neighbors(pos) y cost(pos) (el costo de ENTRAR a esa celda).

    ruta, costo_total = ucs(mapa, inicio, objetivo)

'objetivo' puede ser una celda (fila, col) o un conjunto de celdas
(termina en la de menor costo acumulado).
"""

import heapq
import itertools


def ucs(environment, start, goal):
    goal_set = set(goal) if isinstance(goal, (set, frozenset)) else {goal}

    contador = itertools.count()          # desempate estable en el heap
    frontera = [(0, next(contador), start)]
    mejor_costo = {start: 0}
    padre = {start: None}
    expandidos = set()

    while frontera:
        g, _, nodo = heapq.heappop(frontera)
        if nodo in expandidos or g > mejor_costo.get(nodo, float("inf")):
            continue                      # entrada desactualizada
        expandidos.add(nodo)

        if nodo in goal_set:
            return _reconstruir_ruta(padre, nodo), g

        for vecino in environment.neighbors(nodo):
            if vecino in expandidos:
                continue
            nuevo_costo = g + environment.cost(vecino)
            if vecino not in mejor_costo or nuevo_costo < mejor_costo[vecino]:
                mejor_costo[vecino] = nuevo_costo
                padre[vecino] = nodo
                heapq.heappush(frontera, (nuevo_costo, next(contador), vecino))

    return None, None


def _reconstruir_ruta(padre, nodo):
    ruta = []
    while nodo is not None:
        ruta.append(nodo)
        nodo = padre[nodo]
    ruta.reverse()
    return ruta
