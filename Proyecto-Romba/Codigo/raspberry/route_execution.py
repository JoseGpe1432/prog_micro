"""Route execution in a synthetic world; physical commands are suggestions only."""
import math
import numpy as np
import config as cfg
from planning_preview import Preview, ALGORITHMS
from grid_map import dilate

def wrap(a): return math.atan2(math.sin(a),math.cos(a))

def desired_twist(pose, target):
    dx,dy=target[0]-pose[0],target[1]-pose[1]
    d=math.hypot(dx,dy)
    if d<.008: return 0.,0.
    err=wrap(math.atan2(dy,dx)-pose[2])
    if abs(err)>.035: return 0.,max(-.7,min(.7,3*err))
    return min(.08,2*d),max(-.15,min(.15,2*err))

def simulate(algorithm, goal=None):
    if algorithm not in ALGORITHMS: raise ValueError('Algoritmo inválido')
    p=Preview(); p.set_demo(True)
    g=p.grid
    # Puertas de 65 cm y margen exterior explícito en este escenario sintético.
    n=g.n; g.lo[n//3,n//2-6:n//2+7]=-3
    g.lo[2*n//3,n//2+4:n//2+17]=-3
    g.lo[0,:]=g.lo[-1,:]=4; g.lo[:,0]=g.lo[:,-1]=4
    start=(g.origin,g.origin)
    goal=tuple(goal) if goal is not None else (g.origin-19,g.origin+14)
    if not g.in_bounds(*goal): raise ValueError('Meta fuera del mapa')
    p.set_goal(*goal)
    ok,msg=p.plan((0,0,0),algorithm)
    result={'algorithm':algorithm,'mode':'SIMULACION SIN MOTORES','route':p.route,'cells':p.snapshot((0,0,0))['cells'],'goal':list(goal),'trajectory':[],'state':'SIN_RUTA','steps':0}
    if not ok:return result
    blocked=dilate((g.lo>=-.7)|g.cliff,cfg.INFLATE_CELLS)
    k=cfg.INFLATE_CELLS; blocked[:k,:]=blocked[-k:,:]=True; blocked[:,:k]=blocked[:,-k:]=True
    pose=[0.,0.,0.]; trail=[pose.copy()]; dt=.1; steps=0
    for cell in p.route[1:]:
        target=g.cell_to_world(cell)
        for _ in range(400):
            v,w=desired_twist(pose,target)
            if not(v or w):break
            theta=wrap(pose[2]+w*dt)
            nxt=[pose[0]+v*dt*math.cos(pose[2]+w*dt/2),pose[1]+v*dt*math.sin(pose[2]+w*dt/2),theta]
            rc=g.world_to_cell(*nxt[:2])
            if not g.in_bounds(*rc) or blocked[rc]:
                result.update(state='PARADA_HUELLA_NO_LIBRE',trajectory=trail,steps=steps);return result
            pose=nxt;steps+=1
            if steps%5==0:trail.append(pose.copy())
            if steps>=40000:
                result.update(state='LIMITE_SIMULACION',trajectory=trail,steps=steps);return result
        else:
            result.update(state='META_LOCAL_NO_ALCANZADA',trajectory=trail,steps=steps);return result
    trail.append(pose.copy());result.update(state='META_ALCANZADA',trajectory=trail,steps=steps);return result
