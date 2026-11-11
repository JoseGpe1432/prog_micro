"""
test_all.py -- pruebas automáticas (no requieren hardware).
Ejecuta:  python3 test_all.py
"""
import math
import random
import sys
import time

import cv2
import numpy as np

import config as cfg
from esp32_link import encode, parse_line, parse_telemetry, drive_cmd, checksum
from lidar_ld06 import LD06Parser, crc8
from vision import FloorObjectDetector, GroundProjector, classify
from synth_camera import render
from detection import DEBRIS, TOO_BIG, CABLE, IGNORE
from calibrate_camera import solve
from grid_map import OccupancyGrid, PlanningGrid
from bfs import bfs
from dfs import dfs
from ucs import ucs
from sim import run_simulation, default_world, World

RESULTS = []


def check(name, ok, detail=""):
    ok = bool(ok)
    RESULTS.append(ok)
    print(("  OK   " if ok else "  FALLA"), name, detail)


# ---------------------------------------------------------------------
print("== Protocolo ESP32 ==")
msg = drive_cmd(0.2, -0.5).decode()
check("encode/parse ida y vuelta", parse_line(msg) == ("D", ["200", "-500"]), msg.strip())
t = parse_telemetry(encode("T,1234,14400,90000,500,-20,1,3").decode())
check("telemetria valida", t is not None and t.batt_mv == 14400 and t.yaw_mdeg == 90000 and t.faults == 3)
bad = encode("T,1,2,3,4,5,6,7").decode().replace("T,1,2", "T,9,2")
check("checksum corrupto se rechaza", parse_telemetry(bad) is None)
check("linea basura se ignora", parse_line("hola mundo") is None and parse_line("$T,1*ZZ") is None)

# ---------------------------------------------------------------------
print("== LIDAR LD06/LD19 ==")


def build_packet(start_deg, end_deg, dists_mm, inten=100):
    p = bytearray([0x54, 0x2C]) + (3600).to_bytes(2, "little") + int(start_deg * 100).to_bytes(2, "little")
    for d in dists_mm:
        p += int(d).to_bytes(2, "little") + bytes([inten])
    p += int(end_deg * 100).to_bytes(2, "little") + (0).to_bytes(2, "little")
    p.append(crc8(bytes(p)))
    return bytes(p)


check("tabla CRC8 (poly 0x4D)", crc8(b"\x01") == 0x4D)
stream = bytearray(b"\x00\x13\x54")                    # basura inicial
for k in range(31):                                    # 31 paquetes de ~12° = 372°
    a0 = (k * 12.0) % 360
    stream += build_packet(a0, (a0 + 11.0) % 360, [1000 + 10 * k] * 12)
stream[100] ^= 0xFF                                    # corrompe un byte -> un paquete se descarta
parser = LD06Parser()
scans = []
for i in range(0, len(stream), 50):                    # llegada fragmentada
    scans += parser.feed(bytes(stream[i:i + 50]))
check("ensambla un escaneo completo", len(scans) >= 1)
check("detecta CRC malo y resincroniza", parser.bad_crc >= 1)
if scans:
    ang, dist = scans[0]
    check("distancias en metros", 0.9 < dist.min() and dist.max() < 1.4, f"{dist.min():.2f}-{dist.max():.2f}")
    check("angulos en [-pi, pi]", ang.min() >= -math.pi - 1e-3 and ang.max() <= math.pi + 1e-3)

# ---------------------------------------------------------------------
print("== Vision por TAMAÑO REAL (sin entrenamiento) ==")
check("clasifica: 8 mm = basura", classify(0.008, 0.008) == DEBRIS)
check("clasifica: 3.4 cm = basura (limite)", classify(0.034, 0.03) == DEBRIS)
check("clasifica: 5.5 cm = demasiado grande", classify(0.055, 0.055) == TOO_BIG)
check("clasifica: 25 cm x 5 mm = cable", classify(0.25, 0.006) == CABLE)
check("clasifica: cabello 8 cm x 1.5 mm = basura", classify(0.08, 0.0015) == DEBRIS)
check("clasifica: 1 mm = ruido", classify(0.001, 0.001) == IGNORE)
check("clasifica: 60 cm = sombra/alfombra, se ignora", classify(0.6, 0.4) == IGNORE)

scene = [("miga oscura 8mm", "ellipse", 0.40, 0.06, 0.008, 0.008, 0, (40, 50, 60), DEBRIS),
         ("pelusa clara 14x7mm", "ellipse", 0.50, -0.10, 0.014, 0.007, 30, (215, 215, 210), DEBRIS),
         ("moneda 24mm", "ellipse", 0.34, -0.17, 0.024, 0.024, 0, (170, 175, 180), DEBRIS),
         ("bloque 55mm", "rect", 0.56, 0.14, 0.055, 0.055, 15, (40, 40, 200), TOO_BIG),
         ("cable 25cm x 5mm", "rect", 0.62, -0.22, 0.250, 0.005, 20, (30, 30, 30), CABLE)]
img = render([(k, x, y, l, w, a_, col) for (_, k, x, y, l, w, a_, col, _) in scene], seed=3)
det = FloorObjectDetector().detect(img, with_ignored=True)
for name, k, x, y, l, w, a_, col, expected in scene:
    best = min(det, key=lambda d: math.hypot(d.x - x, d.y - y), default=None)
    near = best is not None and math.hypot(best.x - x, best.y - y) < 0.03
    check(f"{name}: detectado y clasificado {expected}", near and best.kind == expected,
          f"medido {best.length*1000:.0f} mm (real {l*1000:.0f}), error pos {math.hypot(best.x-x, best.y-y)*1000:.1f} mm" if best else "")
    if near and expected != CABLE:
        check(f"{name}: tamaño medido dentro de ±30 %", abs(best.length - l) / l < 0.30 or abs(best.length - l) < 0.003)
check("sin falsos positivos en la escena", len(det) <= len(scene) + 1, f"{len(det)} detecciones")
blank = FloorObjectDetector().detect(render([], seed=5))
check("piso vacio: sin detecciones", len(blank) == 0, f"{len(blank)}")

gp = GroundProjector(640, 480)
p = gp.pixel_to_floor(320, 240)
a_c = math.radians(cfg.CAM_PITCH_DEG)
check("proyeccion: centro de imagen", abs(p[0] - (cfg.CAM_X + cfg.CAM_HEIGHT / math.tan(a_c))) < 1e-6 and abs(p[1]) < 1e-9, f"x={p[0]:.3f} m")
pl, pr = gp.pixel_to_floor(200, 300), gp.pixel_to_floor(440, 300)
check("proyeccion: izquierda de imagen = +y", pl[1] > 0 > pr[1] and abs(pl[1] + pr[1]) < 1e-9)
u_, v_, ok_ = gp.floor_to_pixel(np.array([p[0]]), np.array([p[1]]))
check("proyeccion directa/inversa consistentes", abs(u_[0] - 320) < 1e-6 and abs(v_[0] - 240) < 1e-6)

# calibración con cinta métrica: recupera inclinación y FOV conocidos
def _pix(x, y, pitch, hfov, w=640, h=480):
    a = math.radians(pitch); fx = (w / 2) / math.tan(math.radians(hfov) / 2)
    dx = x - cfg.CAM_X; zc = dx * math.cos(a) + cfg.CAM_HEIGHT * math.sin(a); yc = -dx * math.sin(a) + cfg.CAM_HEIGHT * math.cos(a)
    return w / 2 + fx * (-y) / zc, h / 2 + fx * yc / zc
rng_ = np.random.default_rng(1)
marks = [(*(np.array(_pix(x, y, 47.3, 63.5)) + rng_.normal(0, 2, 2)), x, y)
         for x, y in [(0.30, 0), (0.45, 0), (0.55, 0.18), (0.60, -0.15), (0.70, 0)]]
pitch_e, hfov_e, rms_e = solve(marks, cfg.CAM_HEIGHT, cfg.CAM_X, 640, 480)
check("calibrate_camera recupera inclinacion y FOV (ruido 2 px)", abs(pitch_e - 47.3) < 1.0 and abs(hfov_e - 63.5) < 2.0,
      f"{pitch_e:.1f}° / {hfov_e:.1f}°")

# ---------------------------------------------------------------------
print("== Planificadores BFS / DFS / UCS sobre el mapa del LIDAR ==")
world = default_world()
from sim import FakeRobot
r0 = FakeRobot(world)
occ = OccupancyGrid()
for _ in range(5):
    ang, d = r0.lidar()
    occ.integrate_scan((0, 0, 0), ang, d)
rc = occ.world_to_cell(0, 0)
pg = PlanningGrid(occ, rc)
goal = (rc[0] - 30, rc[1] + 8)
check("meta alcanzable", pg.is_passable(goal))
pb = bfs(pg, rc, goal)
pu, cu = ucs(pg, rc, goal)
pd = dfs(pg, rc, goal, orden=cfg.DFS_ORDER)
check("BFS/DFS/UCS encuentran ruta", pb and pu and pd)
cost_of = lambda path: sum(pg.cost(c) for c in path[1:])
check("UCS: costo <= costo de la ruta BFS", cu <= cost_of(pb), f"UCS={cu} BFS={cost_of(pb)}")
check("BFS: menos o igual pasos que UCS", len(pb) <= len(pu), f"BFS={len(pb)-1} UCS={len(pu)-1}")
check("todas las rutas solo por celdas libres", all(pg.is_passable(c) for c in pb + pu + pd))
wall_cells = [(r, c) for r in range(pg.rows) for c in range(pg.cols) if not pg.passable_np[r, c]]
check("meta en pared: no existe ruta", bfs(pg, rc, wall_cells[0]) is None)

# ---------------------------------------------------------------------
print("== Simulacion completa (cerebro + mapa + planificadores) ==")


def report(r):
    return (f"estado={r['state']} t={r['sim_time']}s cobertura={r['coverage']}% colisiones={r['collisions']} "
            f"caidas={r['falls']} casa={r['home_dist']}m bateria={r['batt_mv']}mV")


def run(name, **kw):
    t0 = time.time()
    r = run_simulation(kw.pop("world", default_world()), seconds=kw.pop("seconds", 900), **kw)
    print(f"  [{name}] {report(r)}  ({time.time()-t0:.0f}s reales)")
    return r


r = run("A: limpieza normal")
check("A: termina y se detiene en casa", r["state"] == "DONE" and r["home_dist"] < 0.3)
check("A: sin colisiones", r["collisions"] == 0)
check("A: cobertura >= 85%", r["coverage"] >= 85.0, f"{r['coverage']}%")

r = run("B: bateria baja a mitad", batt_start=13500, drain_mv_per_s=3.0)
check("B: regresa a casa por bateria baja", "bateria baja" in r["note"] or "llego" in r["note"])
check("B: se detiene en casa ANTES de bateria critica",
      r["state"] == "DONE" and r["home_dist"] < 0.3 and r["batt_mv"] > cfg.BATT_CRIT_MV, f"{r['batt_mv']}mV")
check("B: sin colisiones", r["collisions"] == 0)

random.seed(5)
dust = []
w_ = default_world()
while len(dust) < 8:
    x, y = random.uniform(0.5, 3.1), random.uniform(0.5, 3.1)
    if not w_.is_occ(x, y) and not any(w_.is_occ(x + dx, y + dy) for dx in (-.2, .2) for dy in (-.2, .2)):
        dust.append((x, y))
r = run("C: basura con camara", dust_points=dust)
check("C: entra en modo GOTO_DUST", any("GOTO_DUST" in l for l in r["log"]))
check("C: casi toda la basura aspirada", r["dust_left"] <= 1, f"quedan {r['dust_left']}/8")
check("C: sin colisiones", r["collisions"] == 0)

r = run("D: escalon invisible al LIDAR", world=default_world(with_drop=True))
check("D: detecta el precipicio con IR y no insiste", 1 <= r["cliff_events"] <= 20, f"{r['cliff_events']} eventos")
check("D: nunca se cae", r["falls"] == 0)
check("D: termina en casa sin colisiones", r["state"] == "DONE" and r["collisions"] == 0 and r["home_dist"] < 0.4,
      f"casa a {r['home_dist']}m  nota={r['note']}")

cfg.USE_ENCODERS = True
r = run("E: con encoders + deriva de gyro 0.05 deg/s", gyro_bias_dps=0.05)
cfg.USE_ENCODERS = False
check("E: sin colisiones y regresa a casa", r["collisions"] == 0 and r["state"] == "DONE" and r["home_dist"] < 0.4,
      f"casa a {r['home_dist']}m")

cfg.EXPLORE_PLANNER = "DFS"
r = run("F: exploracion con DFS")
cfg.EXPLORE_PLANNER = "BFS"
check("F: DFS es seguro (0 colisiones) y avanza", r["collisions"] == 0 and r["coverage"] >= 40.0,
      f"{r['coverage']}% en {r['sim_time']}s (BFS: ~94% en ~300s: DFS es mas lento por diseño)")

r = run("G: IMU mala (0.15 deg/s) sin encoders", gyro_bias_dps=0.15)
check("G: IMU mala: <=1 roce y regresa cerca de casa",
      r["collisions"] <= 1 and r["state"] == "DONE" and r["home_dist"] < 0.5, f"casa a {r['home_dist']}m")

cfg.BATT_LOW_MV = 11000          # simula que la estimacion de bateria falla: el robot NO vuelve a casa
r = run("H: bateria critica lejos de casa (el ESP32 corta)", batt_start=12500, drain_mv_per_s=4.0)
cfg.BATT_LOW_MV = 13000
check("H: motores cortados por bateria critica, lejos de casa", r["state"] == "STOPPED_LOWBATT" and r["home_dist"] > 0.5,
      f"casa a {r['home_dist']}m")
check("H: sin colisiones", r["collisions"] == 0)

hz = [(1.0, 0.9, 0.06), (2.4, 0.7, 0.07), (1.2, 2.0, 0.05)]
r0 = run("I0: objetos grandes SIN deteccion (control)", hazards=hz, hazard_detect=False)
check("I0 (control): sin deteccion el robot SI pisa los objetos", r0["ran_over"] >= 2, f"{r0['ran_over']}/3")
r = run("I: objetos grandes CON deteccion por tamaño", hazards=hz)
check("I: rodea los 3 objetos (0 pisados)", r["ran_over"] == 0 and r["hazards_marked"] >= 3, f"pisados {r['ran_over']}/3, marcados {r['hazards_marked']}")
check("I: sigue limpiando y vuelve a casa", r["state"] == "DONE" and r["home_dist"] < 0.4 and r["coverage"] >= 80.0,
      f"{r['coverage']}% casa a {r['home_dist']}m")

print()
print(f"RESUMEN: {sum(RESULTS)}/{len(RESULTS)} pruebas OK")
sys.exit(0 if all(RESULTS) else 1)
