"""
search_status.py
-----------------
Estados compartidos por los distintos planificadores de búsqueda
(BFS, DFS y, en el futuro, UCS/A*). Tenerlos en un solo lugar evita
que main.py y ui.py tengan que saber de qué clase concreta es
'search' para comparar su estado: cualquier planificador que use
estas constantes es intercambiable.
"""

BUSCANDO = "BUSCANDO"
ENCONTRADO = "OBJETIVO ENCONTRADO"
SIN_SOLUCION = "NO EXISTE RUTA"
