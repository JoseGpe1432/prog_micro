#!/usr/bin/env python3
"""
run_robot.py -- ejecuta el robot REAL.

    python3 run_robot.py --no-motors        # PRIMERA prueba: sensores+mapa+cerebro, sin mover ruedas
    python3 run_robot.py                    # robot completo
    python3 run_robot.py --planner DFS      # exploración en profundidad (más lenta)

Fail-safes de este programa (además de los del firmware):
  * telemetría del ESP32 vieja (>0.5 s)  -> manda STOP
  * sin escaneos del LIDAR (>1.5 s)      -> manda STOP
  * Ctrl+C / error                        -> manda STOP y cierra puertos
"""
import argparse
import threading
import time

import cv2

import config as cfg
from brain import Brain, DONE, STOPPED_LOWBATT
from esp32_link import ESP32Link
from lidar_ld06 import LD06Reader
from vision import FloorObjectDetector


class Camera(threading.Thread):
    """Lee la webcam en un hilo y conserva solo el último cuadro (baja latencia)."""

    def __init__(self, index, size):
        super().__init__(daemon=True)
        self.cap = cv2.VideoCapture(index)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, size[0])
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, size[1])
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.frame, self._run = None, True
        self._lock = threading.Lock()
        self.start()

    def run(self):
        while self._run:
            ok, f = self.cap.read()
            if ok:
                with self._lock:
                    self.frame = f
            else:
                time.sleep(0.05)

    def latest(self):
        with self._lock:
            f, self.frame = self.frame, None
            return f

    def close(self):
        self._run = False
        self.cap.release()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--no-motors", action="store_true", help="no manda movimiento (pruebas de banco)")
    ap.add_argument("--no-camera", action="store_true")
    ap.add_argument("--log", metavar="ARCHIVO.csv", help="registra tiempo, estado, voltaje y pose (para ajustar umbrales de batería)")
    ap.add_argument("--planner", choices=["BFS", "DFS"], default=cfg.EXPLORE_PLANNER)
    args = ap.parse_args()
    cfg.EXPLORE_PLANNER = args.planner

    link = ESP32Link(cfg.ESP32_PORT, cfg.ESP32_BAUD)
    lidar = LD06Reader()
    cam = None if args.no_camera else Camera(cfg.CAMERA_INDEX, cfg.CAMERA_SIZE)
    detector = FloorObjectDetector()
    logf = open(args.log, "w") if args.log else None
    if logf:
        logf.write("t_s,estado,batt_mV,x,y,theta_deg,fallas\n")
    t_start = time.time()

    print("Esperando telemetría del ESP32 (¡robot QUIETO y sobre el piso!)...")
    t0 = time.time()
    while link.get_telemetry() is None:
        if time.time() - t0 > 6:
            raise SystemExit("No llega telemetría: revisa el puerto/cable del ESP32 (config.ESP32_PORT).")
        time.sleep(0.1)
    link.zero()

    brain = Brain(log=print)
    dt = 1.0 / cfg.TICK_HZ
    next_t = time.time()
    last_scan_t = time.time()
    last_print = 0.0
    warned = False
    n = 0
    done_since = None
    try:
        while True:
            now = time.time()
            telem = link.get_telemetry()
            scan = lidar.latest()
            if scan is not None:
                last_scan_t = now

            if telem is None or now - link.last_rx > 0.5 or now - last_scan_t > 1.5:
                link.stop()
                if not warned:
                    print("⚠ sensores sin datos (ESP32 o LIDAR): robot detenido")
                    warned = True
                time.sleep(0.1)
                continue
            warned = False

            detections = []
            if cam is not None and n % 3 == 0:
                frame = cam.latest()
                if frame is not None:
                    detections = detector.detect(frame)   # posición y tamaño REAL de cada objeto

            v, w = brain.tick(dt, telem, scan, detections)
            if args.no_motors:
                v = w = 0.0
            link.send_drive(v, w)
            n += 1

            if logf and n % 5 == 0:
                st = brain.status()
                logf.write(f"{now - t_start:.1f},{st['state']},{telem.batt_mv},{st['x']},{st['y']},{st['th_deg']},{telem.faults}\n")
                logf.flush()
            if now - last_print > 2.0:
                last_print = now
                print(brain.status())
            if brain.state in (DONE, STOPPED_LOWBATT):
                link.stop()
                done_since = done_since or now
                if now - done_since > 3.0:
                    print("Fin:", brain.status())
                    break

            next_t += dt
            time.sleep(max(0.0, next_t - time.time()))
            if next_t < time.time() - 1.0:       # nos atrasamos mucho: resincronizar
                next_t = time.time()
    except KeyboardInterrupt:
        print("\nInterrumpido por el usuario")
    finally:
        try:
            link.stop()
        finally:
            link.close()
            lidar.close()
            if logf:
                logf.close()
            if cam:
                cam.close()


if __name__ == "__main__":
    main()
