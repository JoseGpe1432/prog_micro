#!/usr/bin/env python3
"""Raspberry Pi + ESP32 HC-SR04. Inicio PAUSADO; panel http://127.0.0.1:5000."""
import argparse
import threading
import time
import math
import cv2
import numpy as np
import config as cfg
from brain import Brain, DONE, STOPPED_LOWBATT
from esp32_link import ESP32Link, FAULT_BATT_CRIT
from vision import FloorObjectDetector
from dashboard import make_app

PLANNERS = {'BFS', 'DFS', 'UCS', 'GREEDY', 'ASTAR'}

class Camera(threading.Thread):
    def __init__(self, index, size):
        super().__init__(daemon=True)
        self.cap = cv2.VideoCapture(index)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, size[0]); self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, size[1])
        self._lock = threading.Lock(); self._frame = None; self._running = True
        self.start()
    def run(self):
        while self._running:
            ok, frame = self.cap.read()
            if ok:
                with self._lock: self._frame = frame
            else: time.sleep(.1)
    def latest(self):
        with self._lock:
            return None if self._frame is None else self._frame.copy()
    def close(self):
        self._running = False
        self.cap.release()

class Controller:
    def __init__(self, no_motors=False, no_camera=False):
        self.lock = threading.RLock()
        self.mode = 'PAUSED'
        self.planner = 'BFS'
        self.brain = Brain(log=print)
        self.no_motors = no_motors
        self.link = None
        self.cam = None if no_camera else Camera(cfg.CAMERA_INDEX, cfg.CAMERA_SIZE)
        self.detector = FloorObjectDetector()
        self.last_ultra = None
        self.last_error = ''
        self.last_detection_count = 0
        self.last_scan_time = 0.0
        self.last_cam_time = 0.0
        self.last_jpeg = None
        self._running = True
        self.worker = threading.Thread(target=self.loop, daemon=True)
        self.worker.start()
    def start(self, planner):
        if planner not in PLANNERS: return False, 'Algoritmo desconocido'
        with self.lock:
            if self.mode == 'STOPPED': return False, 'Parada enclavada: reinicia el programa'
            if self.link is None: return False, 'ESP32 desconectada'
            if time.monotonic()-self.link.last_ultrasonic_rx > cfg.ULTRA_MAX_AGE_S: return False, 'HC-SR04 sin lectura reciente'
            if time.time()-self.link.last_rx > .5: return False, 'Telemetría ESP32 ausente'
            u,_=self.link.get_ultrasonic()
            if not u or not u['valid'] or u['blocked']: return False, 'Frente bloqueado o distancia no válida'
            t=self.link.get_telemetry()
            if not t or t.faults & FAULT_BATT_CRIT: return False, 'Batería crítica o sin telemetría'
            if self.mode != 'RUNNING':
                # Reinicia la misión en pausa para no reanudar una ruta antigua.
                self.brain = Brain(log=print)
                self.link.zero()
            self.planner=planner; cfg.EXPLORE_PLANNER=planner
            self.mode='RUNNING'
        return True, 'Iniciado'
    def pause(self):
        with self.lock:
            if self.mode != 'STOPPED': self.mode='PAUSED'
            if self.link: self.link.stop()
    def stop(self):
        with self.lock:
            self.mode='STOPPED'
            if self.link: self.link.stop()
    def status(self):
        with self.lock:
            st=self.brain.status()
            u=self.last_ultra
            return {'mode':self.mode,'planner':self.planner,'esp32':self.link is not None,
                    'ultrasonic':u,'camera':self.cam is not None and self.cam.latest() is not None,
                    'detections_last_frame':self.last_detection_count,'no_motors':self.no_motors,
                    'error':self.last_error,'navigation':st}
    def map_snapshot(self):
        with self.lock:
            grid=self.brain.grid
            lo=grid.lo
            cells=np.where(lo>1.2,2,np.where(lo<-.7,1,0)).astype(np.uint8).tolist()
            rc=grid.world_to_cell(self.brain.pose.x,self.brain.pose.y)
            return {'n':grid.n,'cells':cells,'robot':list(rc) if grid.in_bounds(*rc) else None}
    def camera_jpeg(self):
        with self.lock: return self.last_jpeg
    def loop(self):
        last=time.monotonic(); n=0
        try:
            self.link=ESP32Link(cfg.ESP32_PORT,cfg.ESP32_BAUD)
            self.link.stop()
            while self._running:
                now=time.monotonic(); dt=min(.2,max(.01,now-last)); last=now
                telem=self.link.get_telemetry()
                u,ut=self.link.get_ultrasonic()
                fresh=(u is not None and now-ut < cfg.ULTRA_MAX_AGE_S)
                serial_ok=(telem is not None and time.time()-self.link.last_rx < .5)
                # No data = no motion. Never use an invalid echo as 'free space'.
                scan=None
                if fresh and u['valid'] and u['mm']>0:
                    dist=u['mm']/1000.0
                    if cfg.ULTRA_MIN_M <= dist <= cfg.ULTRA_MAX_M:
                        scan=(np.array([0.0],dtype=np.float32),np.array([dist],dtype=np.float32))
                frame=self.cam.latest() if self.cam else None
                detections=[]
                if frame is not None:
                    if n%3==0:
                        try: detections=self.detector.detect(frame)
                        except Exception as exc: self.last_error='Visión: '+str(exc)
                    if n%2==0:
                        ok,jpg=cv2.imencode('.jpg',frame,[cv2.IMWRITE_JPEG_QUALITY,70])
                        if ok:
                            with self.lock: self.last_jpeg=jpg.tobytes()
                with self.lock:
                    self.last_ultra=u if fresh else None
                    self.last_detection_count=len(detections)
                    safe=serial_ok and scan is not None and not (telem.faults & FAULT_BATT_CRIT)
                    if self.mode=='RUNNING' and not safe:
                        self.mode='PAUSED'; self.last_error='Parada preventiva: sensor, batería o enlace'
                    if self.mode=='RUNNING':
                        cfg.EXPLORE_PLANNER=self.planner
                        v,w=self.brain.tick(dt,telem,scan,detections)
                        if self.brain.state in (DONE,STOPPED_LOWBATT):
                            self.mode='PAUSED'; v=w=0.0
                        if self.no_motors: v=w=0.0
                        self.link.send_drive(v,w)
                    else:
                        self.link.stop()
                n+=1
                time.sleep(max(0,1/cfg.TICK_HZ-(time.monotonic()-now)))
        except Exception as exc:
            with self.lock: self.mode='STOPPED'; self.last_error=str(exc)
            print('Error del robot:',exc)
        finally:
            if self.link:
                try: self.link.stop(); self.link.close()
                except Exception: pass
    def close(self):
        self.stop(); self._running=False
        self.worker.join(timeout=1.5)
        if self.cam: self.cam.close()

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--no-motors',action='store_true',help='Prueba de interfaz sin mover ruedas')
    ap.add_argument('--no-camera',action='store_true')
    ap.add_argument('--host',default='127.0.0.1',help='127.0.0.1 por defecto; 0.0.0.0 solo en LAN confiable')
    ap.add_argument('--port',type=int,default=5000)
    args=ap.parse_args()
    robot=Controller(args.no_motors,args.no_camera)
    try:
        print('Panel: http://127.0.0.1:%s  | INICIA PAUSADO' % args.port)
        make_app(robot).run(host=args.host,port=args.port,threaded=True,use_reloader=False)
    finally: robot.close()
if __name__=='__main__': main()
