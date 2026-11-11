"""
test_pty_integration.py
-----------------------
Ejecuta run_robot.py REAL (puertos serie, hilos, parser LD06, protocolo) contra
emuladores del ESP32 y del LIDAR conectados por pseudo-terminales, en tiempo real.
No prueba hardware; prueba TODO el software de la Pi de punta a punta.
"""
import math
import os
import select
import signal
import sys
import threading
import time

import numpy as np

import config as cfg
from esp32_link import encode, parse_line
from lidar_ld06 import crc8
from sim import FakeRobot, default_world

RUN_SECONDS = float(sys.argv[1]) if len(sys.argv) > 1 else 25.0

esp_master, esp_slave = os.openpty()
lid_master, lid_slave = os.openpty()
cfg.ESP32_PORT = os.ttyname(esp_slave)
cfg.LIDAR_PORT = os.ttyname(lid_slave)

world = default_world()
robot = FakeRobot(world)
state = {"v": 0.0, "w": 0.0, "last_cmd": 0.0, "cmds": 0, "stops": 0, "bad": 0, "run": True}
stop_seen_after_exit = threading.Event()


def esp_emulator():
    buf = b""
    t_prev = time.time()
    while state["run"]:
        r, _, _ = select.select([esp_master], [], [], 0.02)
        if r:
            buf += os.read(esp_master, 512)
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                p = parse_line(line.decode(errors="ignore"))
                if not p:
                    state["bad"] += 1
                    continue
                kind, f = p
                if kind == "D":
                    state["v"], state["w"] = int(f[0]) / 1000.0, int(f[1]) / 1000.0
                    state["last_cmd"] = time.time()
                    state["cmds"] += 1
                elif kind == "S":
                    state["v"] = state["w"] = 0.0
                    state["last_cmd"] = time.time()
                    state["stops"] += 1
        now = time.time()
        if now - t_prev >= 0.05:
            dt, t_prev = now - t_prev, now
            v, w = state["v"], state["w"]
            if now - state["last_cmd"] > 0.4:       # watchdog del firmware
                v = w = 0.0
            robot.step(dt, v, w)
            t = robot.telemetry()
            payload = f"T,{t.ms},{t.batt_mv},{t.yaw_mdeg},{t.enc_l},{t.enc_r},{t.cliff_mask},{t.faults}"
            try:
                os.write(esp_master, encode(payload))
            except OSError:
                break


def build_packet(start_deg, dists_mm):
    p = bytearray([0x54, 0x2C]) + (3600).to_bytes(2, "little") + int(start_deg * 100).to_bytes(2, "little")
    for d in dists_mm:
        p += int(d).to_bytes(2, "little") + bytes([120])
    p += int(((start_deg + 11) % 360) * 100).to_bytes(2, "little") + (0).to_bytes(2, "little")
    p.append(crc8(bytes(p)))
    return bytes(p)


def lidar_emulator():
    while state["run"]:
        ang, d = robot.lidar()
        by_deg = {int(round(math.degrees(a))): dd for a, dd in zip(ang, d)}
        full = []
        for i in range(360):                          # i = ángulo del LIDAR (horario)
            a = (-i + 180) % 360 - 180                # equivalente antihorario en el marco del robot
            full.append(by_deg.get(a, 0) * 1000.0)
        data = b"".join(build_packet(k * 12, full[k * 12:(k + 1) * 12]) for k in range(30))
        try:
            os.write(lid_master, data)
        except OSError:
            break
        time.sleep(0.1)


def stopper():
    time.sleep(RUN_SECONDS)
    os.kill(os.getpid(), signal.SIGINT)               # equivale a Ctrl+C


threading.Thread(target=esp_emulator, daemon=True).start()
threading.Thread(target=lidar_emulator, daemon=True).start()
threading.Thread(target=stopper, daemon=True).start()

sys.argv = ["run_robot.py", "--no-camera", "--log", "/tmp/robot_log.csv"]
import run_robot

t0 = time.time()
x0, y0 = robot.x, robot.y
try:
    run_robot.main()
except SystemExit as e:
    print("SystemExit:", e)
time.sleep(0.6)
state["run"] = False

moved = math.hypot(robot.x - x0, robot.y - y0)
ok = True


def check(name, cond, extra=""):
    global ok
    ok &= bool(cond)
    print(("  OK   " if cond else "  FALLA"), name, extra)


print(f"\nResultado tras {time.time()-t0:.0f}s de tiempo real:")
check("la Pi mando comandos D al ESP32", state["cmds"] > 100, f"{state['cmds']} comandos")
check("ningun comando corrupto llego al ESP32", state["bad"] == 0, f"{state['bad']} invalidos")
check("el robot se movio (cerebro->ESP32->motores)", moved > 0.5, f"{moved:.2f} m")
check("sin colisiones", robot.collisions == 0)
check("al salir (Ctrl+C) la Pi mando STOP", state["stops"] >= 1, f"{state['stops']} STOP")
check("el robot quedo detenido", state["v"] == 0.0 and state["w"] == 0.0)
log_lines = open("/tmp/robot_log.csv").read().strip().split("\n")
check("registro CSV de bateria/pose generado", len(log_lines) > 20 and log_lines[0].startswith("t_s,estado"), f"{len(log_lines)-1} filas")
sys.exit(0 if ok else 1)
