"""
dfs.py
------
Depth-First Search (búsqueda en profundidad) iterativa con pila LIFO,
para el robot aspirador (sección 6 de la práctica 2.1B). No depende de
Pygame: solo necesita un objeto 'environment' que ofrezca
neighbors_ordered(pos, orden) -> lista de vecinos válidos en el orden
indicado. Esto permite reutilizarlo, junto con bfs.py, en el software
de control del robot real, intercambiando la estrategia de búsqueda
sin rehacer el resto del sistema (sección 1).

Se ofrecen dos formas de uso, igual que en bfs.py:

1) dfs(environment, start, goal) -> lista de posiciones o None
       ruta = dfs(mapa, inicio, objetivo)

2) DFSSearch: la misma búsqueda pero PASO A PASO (step()), para poder
   animar en pantalla la profundización y el retroceso (secciones 10
   y 11).
"""

import search_status as st

ORDEN_POR_DEFECTO = ("ARRIBA", "DERECHA", "ABAJO", "IZQUIERDA")


def dfs(environment, start, goal, orden=ORDEN_POR_DEFECTO):
    """DFS iterativo de una sola llamada, con pila LIFO explícita
    (sección 6). 'goal' puede ser una sola celda (fila, col) o un
    conjunto de celdas (reto de la sección 21: detenerse al encontrar
    CUALQUIER celda con basura). Devuelve la ruta completa (lista de
    posiciones, inicio y fin incluidos) o None si no existe solución.
    """
    goal_set = set(goal) if isinstance(goal, (set, frozenset)) else {goal}

    pila = [start]                 # 1. estado inicial en la pila
    visitados = set()
    padre = {start: None}

    while pila:                            # 2. mientras la pila no esté vacía
        nodo = pila.pop()                  #    extraer el elemento en la cima
        if nodo in visitados:              #    si ya fue visitado
            continue                       #    continuar (se descarta)
        visitados.add(nodo)                #    marcar el nodo como visitado

        if nodo in goal_set:               #    si el nodo es el objetivo
            return _reconstruir_ruta(padre, nodo)   # 3. reconstruir la ruta

        vecinos = environment.neighbors_ordered(nodo, orden)
        for vecino in reversed(vecinos):   # se invierte para respetar 'orden'
            if vecino not in visitados:
                if vecino not in padre:    #    guardar su padre si no tiene
                    padre[vecino] = nodo
                pila.append(vecino)        #    insertar el vecino en la pila

    return None  # 4. la pila quedó vacía: no existe solución


def _reconstruir_ruta(padre, nodo):
    """Recorre el diccionario de padres desde el objetivo hasta el
    estado inicial y luego invierte el resultado (Etapa 8)."""
    ruta = []
    while nodo is not None:
        ruta.append(nodo)
        nodo = padre[nodo]
    ruta.reverse()
    return ruta


class DFSSearch:
    """Versión paso a paso del mismo algoritmo, para animar en pantalla
    la profundización y el retroceso (secciones 10 y 11).

    Cada llamada a step() ejecuta EXACTAMENTE una iteración del ciclo
    'while' del pseudocódigo de la sección 6 -- incluidas las
    iteraciones que solo descartan un nodo ya visitado -- para que la
    animación refleje fielmente el algoritmo tal como se especifica.
    """

    kind = "DFS"

    BUSCANDO = st.BUSCANDO
    ENCONTRADO = st.ENCONTRADO
    SIN_SOLUCION = st.SIN_SOLUCION

    def __init__(self, environment, start, goal, orden=ORDEN_POR_DEFECTO):
        self.env = environment
        self.start = start
        self.orden = orden

        self.multi_goal = isinstance(goal, (set, frozenset))
        self.goal_set = set(goal) if self.multi_goal else {goal}

        self.pila = [start]              # 1. estado inicial en la pila
        self.visitados = set()
        self.padre = {start: None}
        self.profundidad = {start: 0}

        self.nodo_actual = None
        self._nodo_anterior = None       # último nodo realmente procesado
        self.status = st.BUSCANDO
        self.ruta = None
        self.goal_reached = None

        self.nodos_visitados = 0
        self.profundidad_maxima = 0
        self.retrocesos = 0
        self.retroceso_cells = set()     # celdas RETROCESO (callejones sin salida)
        self.rama_activa = [start]       # camino actual desde el inicio

    def step(self):
        """Una sola iteración del ciclo while (sección 6)."""
        if self.status != st.BUSCANDO:
            return

        if not self.pila:
            self.status = st.SIN_SOLUCION
            return

        nodo = self.pila.pop()                 # extraer el elemento en la cima
        if nodo in self.visitados:              # si ya fue visitado
            return                              # continuar (se descarta este paso)

        self.visitados.add(nodo)                # marcar el nodo como visitado
        self.nodos_visitados += 1

        profundidad_nodo = self.profundidad.get(nodo, 0)
        self.profundidad_maxima = max(self.profundidad_maxima, profundidad_nodo)

        # Detección de retroceso: si la profundidad del nuevo nodo NO es
        # mayor que la del último nodo procesado, es porque esa rama se
        # agotó (callejón sin salida o todo ya visitado) y DFS retomó
        # una alternativa pendiente más arriba en la pila.
        if self._nodo_anterior is not None:
            prof_anterior = self.profundidad.get(self._nodo_anterior, 0)
            if profundidad_nodo <= prof_anterior:
                self.retrocesos += 1
                self.retroceso_cells.add(self._nodo_anterior)

        self.nodo_actual = nodo
        self._nodo_anterior = nodo
        self.rama_activa = _reconstruir_ruta(self.padre, nodo)

        if nodo in self.goal_set:               # si el nodo es el objetivo
            self.status = st.ENCONTRADO
            self.goal_reached = nodo
            self.ruta = _reconstruir_ruta(self.padre, nodo)
            return

        vecinos = self.env.neighbors_ordered(nodo, self.orden)
        for vecino in reversed(vecinos):
            if vecino not in self.visitados:
                if vecino not in self.padre:
                    self.padre[vecino] = nodo
                    self.profundidad[vecino] = profundidad_nodo + 1
                self.pila.append(vecino)

    @property
    def profundidad_actual(self):
        if self.nodo_actual is None:
            return 0
        return self.profundidad.get(self.nodo_actual, 0)
