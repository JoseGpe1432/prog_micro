"""
bfs.py
------
Breadth-First Search (búsqueda en anchura) para el robot aspirador
(sección 6 de la práctica). No depende de Pygame: solo necesita un
objeto 'environment' que ofrezca neighbors(pos) -> lista de vecinos
válidos. Esto permite reutilizarlo tal cual en el software de control
del robot real (sección 14).

Se ofrecen dos formas de uso:

1) bfs(environment, start, goal) -> lista de posiciones o None
   Función "de una sola llamada":
       ruta = bfs(mapa, inicio, objetivo)

2) BFSSearch: la misma búsqueda pero PASO A PASO (step()), para poder
   animarla en pantalla y mostrar cómo crece la frontera por niveles
   (secciones 9 y 10).
"""

from collections import deque

import search_status as st


def bfs(environment, start, goal):
    """BFS clásico, de una sola llamada.

    'goal' puede ser una sola celda (fila, col) o un conjunto de celdas
    (usado en el reto de la sección 19: terminar al encontrar
    CUALQUIER celda con basura). Devuelve la ruta completa (lista de
    posiciones, inicio y fin incluidos) o None si no existe solución.
    """
    goal_set = set(goal) if isinstance(goal, (set, frozenset)) else {goal}

    if start in goal_set:
        return [start]

    frontera = deque([start])          # cola FIFO
    padre = {start: None}              # para reconstruir la ruta

    while frontera:                    # 2. Mientras la cola no esté vacía
        nodo = frontera.popleft()      #    extraer el primer nodo
        for vecino in environment.neighbors(nodo):   # generar vecinos válidos
            if vecino not in padre:                  # no visitado
                padre[vecino] = nodo                  # guardar padre
                if vecino in goal_set:                # es el objetivo
                    return _reconstruir_ruta(padre, vecino)
                frontera.append(vecino)               # insertar al final

    return None  # 4. la cola quedó vacía: no existe solución


def _reconstruir_ruta(padre, nodo):
    """Recorre el diccionario de padres desde el objetivo hasta el
    estado inicial y luego invierte el resultado (sección 7, Etapa 7)."""
    ruta = []
    while nodo is not None:
        ruta.append(nodo)
        nodo = padre[nodo]
    ruta.reverse()
    return ruta


class BFSSearch:
    """Versión paso a paso del mismo algoritmo, para visualizar en
    pantalla cómo se expande la frontera nivel por nivel."""

    kind = "BFS"

    # Se mantienen como alias de search_status para no romper código
    # que ya use BFSSearch.ENCONTRADO, etc.
    BUSCANDO = st.BUSCANDO
    ENCONTRADO = st.ENCONTRADO
    SIN_SOLUCION = st.SIN_SOLUCION

    def __init__(self, environment, start, goal):
        self.env = environment
        self.start = start

        # 'goal' puede ser una celda única o un conjunto (reto sección 19)
        self.multi_goal = isinstance(goal, (set, frozenset))
        self.goal_set = set(goal) if self.multi_goal else {goal}

        self.frontera = deque([start])   # 1. estado inicial en la cola
        self.en_frontera = {start}       # para dibujar FRONTERA
        self.visitados = set()           # nodos ya expandidos -> VISITADO
        self.padre = {start: None}
        self.profundidad = {start: 0}

        self.nodo_actual = None
        self.status = st.BUSCANDO
        self.ruta = None
        self.goal_reached = None
        self.nodos_explorados = 0

    def step(self):
        """Ejecuta UNA expansión de BFS (una iteración del ciclo while
        del pseudocódigo de la sección 6)."""
        if self.status != st.BUSCANDO:
            return

        if not self.frontera:
            self.status = st.SIN_SOLUCION
            return

        nodo = self.frontera.popleft()
        self.en_frontera.discard(nodo)
        self.nodo_actual = nodo
        self.visitados.add(nodo)
        self.nodos_explorados += 1

        if nodo in self.goal_set:
            self.status = st.ENCONTRADO
            self.goal_reached = nodo
            self.ruta = _reconstruir_ruta(self.padre, nodo)
            return

        for vecino in self.env.neighbors(nodo):
            if vecino not in self.padre:            # aún no descubierto
                self.padre[vecino] = nodo
                self.profundidad[vecino] = self.profundidad[nodo] + 1
                self.frontera.append(vecino)
                self.en_frontera.add(vecino)

    @property
    def profundidad_actual(self):
        if self.nodo_actual is None:
            return 0
        return self.profundidad.get(self.nodo_actual, 0)
