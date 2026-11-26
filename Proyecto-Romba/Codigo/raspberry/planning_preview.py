"""Planificador de vista previa. Nunca envía órdenes a motores.
Mapa REAL: solo celdas observadas libres; DEMO: escenario sintético explícito.
"""
import numpy as np
import config as cfg
from grid_map import OccupancyGrid, PlanningGrid
from bfs import bfs
from dfs import dfs
from ucs import ucs
from informed_search import greedy, astar

ALGORITHMS = ('BFS','DFS','UCS','GREEDY','ASTAR')

class Preview:
    def __init__(self):
        self.grid=OccupancyGrid()
        self.demo=False
        self.goal=None
        self.planner='BFS'
        self.route=[]
        self.message='Selecciona una meta en el mapa. Sin ruta sobre zonas desconocidas.'
        self.last_detections=[]

    def set_demo(self, enabled):
        self.demo=bool(enabled)
        self.goal=None
        self.route=[]
        self.grid=OccupancyGrid()
        if self.demo:
            n=self.grid.n
            self.grid.lo[:,:]=-3.0
            # Dos barreras con puertas; demuestra rutas diferentes.
            self.grid.lo[n//3, 5:n-5]=4.0
            self.grid.lo[n//3, n//2-3:n//2+4]=-3.0
            self.grid.lo[2*n//3, 5:n-5]=4.0
            self.grid.lo[2*n//3, n//2+9:n//2+16]=-3.0
        self.message='Escenario sintético (no representa tu habitación)' if self.demo else 'Mapa real: zonas no exploradas bloqueadas'

    def scan(self, pose, scan):
        if not self.demo and scan is not None:
            self.grid.integrate_scan(pose,*scan)

    def set_goal(self, r, c):
        if not self.grid.in_bounds(r,c):
            return False,'Meta fuera del mapa'
        self.goal=(r,c)
        return True,'Meta seleccionada'

    def plan(self, pose, planner=None):
        if planner is not None:
            if planner not in ALGORITHMS:
                return False,'Algoritmo desconocido'
            self.planner=planner
        if self.goal is None:
            self.route=[]
            return False,'Selecciona una meta haciendo clic en el mapa'
        start=self.grid.world_to_cell(pose[0],pose[1])
        if not self.grid.in_bounds(*start):
            self.route=[]
            return False,'Posición estimada fuera del mapa'
        pg=PlanningGrid(self.grid,start)
        if not pg.is_passable(self.goal):
            self.route=[]
            return False,'Meta desconocida u ocupada: no hay ruta experimental'
        funcs={'BFS':lambda:bfs(pg,start,self.goal),
               'DFS':lambda:dfs(pg,start,self.goal,orden=cfg.DFS_ORDER),
               'UCS':lambda:ucs(pg,start,{self.goal})[0],
               'GREEDY':lambda:greedy(pg,start,self.goal),
               'ASTAR':lambda:astar(pg,start,self.goal)}
        route=funcs[self.planner]()
        self.route=[list(p) for p in (route or [])]
        self.message=(f'{self.planner}: ruta de {len(self.route)} celdas (SOLO VISTA PREVIA EXPERIMENTAL)' if route
                      else f'{self.planner}: sin ruta sobre espacio conocido')
        return bool(route),self.message

    def snapshot(self,pose):
        lo=self.grid.lo
        cells=np.where(lo>1.2,2,np.where(lo<-.7,1,0)).astype('uint8').tolist()
        rc=self.grid.world_to_cell(pose[0],pose[1])
        return {'n':self.grid.n,'cells':cells,'robot':list(rc) if self.grid.in_bounds(*rc) else None,
                'goal':list(self.goal) if self.goal else None,'route':self.route,
                'demo':self.demo,'message':self.message,'planner':self.planner}
