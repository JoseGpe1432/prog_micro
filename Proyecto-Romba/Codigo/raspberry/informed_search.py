"""Greedy best-first y A* sobre PlanningGrid de 4 vecinos."""
import heapq
import itertools

def _h(p, goals):
    return min(abs(p[0]-g[0])+abs(p[1]-g[1]) for g in goals)

def _search(env, start, goal, greedy_mode=False):
    goals = set(goal) if isinstance(goal, (set, frozenset)) else {goal}
    if not goals: return None
    count = itertools.count()
    heap = [(_h(start, goals), next(count), start)]
    gscore = {start: 0.0}
    parent = {start: None}
    seen = set()
    while heap:
        _, _, cur = heapq.heappop(heap)
        if cur in seen: continue
        seen.add(cur)
        if cur in goals:
            out = []
            while cur is not None:
                out.append(cur); cur = parent[cur]
            return out[::-1]
        for nb in env.neighbors(cur):
            if nb in seen: continue
            new = gscore[cur] + env.cost(nb)
            if nb not in gscore or new < gscore[nb]:
                gscore[nb] = new
                parent[nb] = cur
                priority = _h(nb, goals) if greedy_mode else new + _h(nb, goals)
                heapq.heappush(heap, (priority, next(count), nb))
    return None

def greedy(env, start, goals):
    return _search(env, start, goals, True)

def astar(env, start, goals):
    return _search(env, start, goals, False)
