#!/usr/bin/env python3
"""Raspberry Pi + ESP32, prueba de motores con hombre-presente, sin HC-SR04."""
import argparse
import threading
import time
import cv2
import numpy as np
import config as cfg
from brain import Brain, DONE, STOPPED_LOWBATT
from esp32_link import ESP32Link, FAULT_BATT_CRIT, FAULT_IMU
from vision import FloorObjectDetector
from dashboard import make_app
from supervised_auto import command as auto_command, startup_command
from camera_forward import command as camera_command, evaluate as camera_evaluate
from camera_avoid import CameraAvoid
from visual_safety import evaluate as visual_safety
from visual_odometry import VisualOdometry
from vision_filter import VisualFilter
from floor_reference import FloorReference
from visual_planner import evaluate as evaluate_visual_planner
from camera_motion import CameraMotion
from motion_residual import MotionResidual
from region_tracker import RegionTracker
from floor_segmentation import FloorSegmentation
from visual_mapping import VisualMapping
from corridor_history import CorridorHistory
from planning_preview import Preview
from detection import AVOID_KINDS, DEBRIS
import math

PLANNERS = {'BFS', 'DFS', 'UCS', 'GREEDY', 'ASTAR'}


class Camera(threading.Thread):
    """Captura y codifica JPEG sin depender del ciclo de navegación."""
    def __init__(self, index, size):
        super().__init__(daemon=True)
        self._lock = threading.Lock()
        self._frame = None
        self._frame_at = 0.0
        self._jpeg = None
        self._running = True
        self.error = ''
        self.cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
        if self.cap.isOpened():
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, size[0])
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, size[1])
            print('Cámara abierta en índice', index, flush=True)
        else:
            self.error = f'No se pudo abrir la cámara {index}'
            print('ERROR:', self.error, flush=True)
        self.start()

    def run(self):
        try:
            while self._running:
                if not self.cap.isOpened():
                    time.sleep(0.25)
                    continue
                ok, frame = self.cap.read()
                if not ok:
                    self.error = 'No se pudo leer un fotograma de la cámara'
                    time.sleep(0.1)
                    continue
                ok_jpeg, jpg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 75])
                with self._lock:
                    self._frame = frame
                    self._frame_at = time.monotonic()
                    if ok_jpeg:
                        self._jpeg = jpg.tobytes()
                        self.error = ''
        except Exception as exc:
            self.error = f'Error de cámara: {exc}'
            print(self.error, flush=True)
        finally:
            self.cap.release()

    def latest(self):
        with self._lock:
            return None if self._frame is None else self._frame.copy()

    def latest_with_timestamp(self):
        with self._lock:
            return (None if self._frame is None else self._frame.copy(), self._frame_at)

    def frame_age(self):
        with self._lock:
            return time.monotonic() - self._frame_at if self._frame_at else float("inf")

    def latest_jpeg(self):
        with self._lock:
            return self._jpeg

    def close(self):
        self._running = False
        if self.is_alive():
            self.join(timeout=2)


class Controller:
    def __init__(self, no_motors=False, no_camera=False, supervised_motors=False):
        self.lock = threading.RLock()
        self.mode = 'PAUSED'
        self.planner = 'BFS'
        self.brain = Brain(log=print)
        self.no_motors = no_motors
        self.supervised_motors = supervised_motors
        self.motor_deadline = 0.0
        self.manual_deadline = 0.0
        self.startup_until = 0.0
        self.startup_held = False
        self.camera_stop_latched = False
        self.camera_avoid = CameraAvoid()
        self.startup_active = False
        self.manual_v = 0.0
        self.manual_w = 0.0
        self.last_frame_id = 0
        self.last_camera_frame_at = 0.0
        self.motion_state = "DETENIDO"
        self.last_vision_obstacle = 0.0
        self.visual_test_override = False
        self.last_debris = 0.0
        self.turn_until = 0.0
        self.turn_sign = 1
        self.link = None
        self.cam = None if no_camera else Camera(cfg.CAMERA_INDEX, cfg.CAMERA_SIZE)
        self.detector = FloorObjectDetector(roi_top=0.35, min_pixels=18)
        self.visual_filter = VisualFilter()
        self.floor_reference = FloorReference()
        self.camera_motion = CameraMotion()
        self.motion_residual = MotionResidual()
        self.region_tracker = RegionTracker()
        self.floor_segmentation = FloorSegmentation()
        self.visual = VisualOdometry()
        self.preview = Preview()
        self.mapping = VisualMapping()
        self.corridor_history = CorridorHistory()
        self.preview_running = False
        self.last_detections = []
        self.last_detections_raw = []
        self.visual_status = self.visual.status()
        self.last_visual_frame = 0.0
        self.last_visual_capture = 0.0
        self.last_ultra = None
        self.last_error = ''
        self.last_detection_count = 0
        self.last_scan_time = 0.0
        self.last_cam_time = 0.0
        self._running = True
        self.worker = threading.Thread(target=self.loop, daemon=True)
        self.worker.start()

    def start(self, planner):
        """El inicio de planificación NO activa motores automáticamente."""
        if planner not in PLANNERS:
            return False, 'Algoritmo desconocido'
        with self.lock:
            if self.mode == 'STOPPED':
                return False, 'Parada enclavada: reinicia el programa'
            self.planner = planner
            self.preview_running = True
            _, msg = self.preview.plan(self._visual_pose(), planner)
            self.mode = 'PREVIEW'
            self.manual_deadline = 0.0
            return True, 'Planificación activa (sin conducción autónoma): ' + msg

    def _hardware_safe(self):
        if self.link is None or time.time() - self.link.last_rx > 0.35:
            return False
        telem = self.link.get_telemetry()
        if telem is None or (telem.faults & (FAULT_IMU | FAULT_BATT_CRIT)):
            return False
        if self.cam is None or self.cam.latest() is None:
            return False
        return self.cam.frame_age() < 0.40

    def set_visual_test_override(self, enabled):
        """Excepción SOLO para prueba manual, nunca para navegación autónoma."""
        with self.lock:
            self.manual_deadline = 0.0
            self.manual_v = self.manual_w = 0.0
            if self.link:
                self.link.stop()
            self.visual_test_override = bool(enabled)
            return True, ("PRUEBA MANUAL: detección visual solo informativa; "
                          "mantén pulsado Adelante y vigila físicamente el robot"
                          if enabled else "Bloqueo visual de avance restaurado")

    def _visual_obstacle_candidates(self):
        # Son candidatos, NO obstáculos confirmados: el detector actual
        # clasifica diferencias de color/textura como objetos.
        return list(self.visual_filter.confirmed)

    def manual_drive(self, direction):
        """Permiso renovable de 220 ms. Sin renovación, parada. No es autonomía."""
        commands = {'forward': (0.025,0.0), 'left': (0.0,0.25),
                    'right': (0.0,-0.25), 'back': (-0.020,0.0)}
        with self.lock:
            if direction == 'release':
                self.camera_avoid.reset()
                self.camera_stop_latched = False
                self.startup_held = self.startup_active = False
                self.startup_until = 0.0
                self.manual_deadline = 0.0
                if self.mode in ('AUTO','MANUAL'): self.mode='PAUSED'
                self.manual_v = self.manual_w = 0.0
                if self.link: self.link.stop()
                return True, 'Motores detenidos'
            if direction in ('auto','startup'):
                if direction == 'auto' and self.camera_stop_latched:
                    return False, 'Parada enclavada: suelta el boton y revisa el espacio'
                if self.mode == 'STOPPED' or self.preview.demo or self.no_motors or not self.supervised_motors or self.visual_test_override or not self._hardware_safe() or not self.floor_segmentation.status().get('calibrated') or (direction == 'startup' and not self.visual_status.get('calibrated_rest')):
                    return False, 'Autonomía bloqueada: revisa calibración, sensores y modo'
                if direction == 'startup':
                    if not self.startup_held:
                        self.startup_held = True
                        self.startup_until = time.monotonic()+2.0
                    if time.monotonic() >= self.startup_until:
                        return False, 'Arranque terminado: suelta el botón y revisa el espacio'
                    self.startup_active = True
                else:
                    self.startup_active = False
                self.mode='AUTO'
                self.manual_deadline=time.monotonic()+.22
                return True, 'Exploración reactiva supervisada'
            if direction not in commands:
                return False, 'Dirección inválida'
            if self.mode == 'STOPPED' or self.preview.demo:
                return False, 'Parada enclavada o escenario DEMO activo'
            if not self.supervised_motors or self.no_motors:
                return False, 'Ejecuta con --supervised-motors para pruebas de ruedas'
            if not self._hardware_safe():
                return False, 'ESP32, IMU o cámara sin datos recientes'
            # Candidatos visuales: bloqueo conservador de avance; NO miden distancia.
            if direction == 'forward' and (self._visual_obstacle_candidates() or not self.visual_filter.status()['fresh']) and not self.visual_test_override:
                self.manual_deadline = 0.0
                if self.link: self.link.stop()
                return False, 'Posible obstáculo visual: avance bloqueado'
            self.startup_active = False
            self.camera_avoid.reset()
            self.manual_v,self.manual_w=commands[direction]
            if direction == 'forward' and self.visual_test_override:
                self.manual_v = min(self.manual_v, 0.015)
            self.manual_deadline=time.monotonic()+0.22
            self.mode='MANUAL'
            self.motion_state='PRUEBA SUPERVISADA: '+direction.upper()
            return True, self.motion_state

    def set_planner(self, planner):
        if planner not in PLANNERS:
            return False, 'Algoritmo desconocido'
        with self.lock:
            self.planner = planner
            ok, msg = self.preview.plan(self._visual_pose(), planner)
            return True, msg

    def set_goal(self, row, col):
        with self.lock:
            ok, msg = self.preview.set_goal(row, col)
            if ok:
                _, msg = self.preview.plan(self._visual_pose(), self.planner)
            return ok, msg

    def set_demo(self, enabled):
        with self.lock:
            self.preview.set_demo(enabled)
            self.mapping = VisualMapping()
            self.corridor_history = CorridorHistory()
            return True, self.preview.message

    def _visual_pose(self):
        v = self.visual_status
        return (v['x'], v['y'], math.radians(v['heading_deg']))

    def pause(self):
        with self.lock:
            self.camera_stop_latched = True
            if self.mode != 'STOPPED':
                self.mode = 'PAUSED'
                self.preview_running = False
            self.motion_state = 'DETENIDO'
            self.manual_deadline = 0.0
            if self.link:
                self.link.stop()

    def stop(self):
        with self.lock:
            self.mode = 'STOPPED'
            self.preview_running = False
            self.motion_state = 'DETENIDO'
            self.manual_deadline = 0.0
            if self.link:
                self.link.stop()

    def _visual_planning_status(self):
        ref = self.floor_reference.status()
        updated = ref.get('updated_monotonic')
        age = max(0.0, time.monotonic() - updated) if updated is not None else None
        # La referencia solo es válida si la cámara está fija y el robot no se mueve.
        # Incluso con motores detenidos, no es posible certificar cámara inmóvil.
        motion = self.camera_motion.status()
        if motion['reference_invalidated'] or motion['state'] in ('MOVIMIENTO_DETECTADO','SIN_DATOS_RECIENTES','SEGUIMIENTO_INSUFICIENTE','INDETERMINADO','INICIALIZANDO','SIN_DATOS'):
            ref = {'calibrated': False}
        result = evaluate_visual_planner(ref, self.planner, age_seconds=age)
        result['motion_state_visual'] = motion['state']
        result['reference_invalidated_by_motion'] = motion['reference_invalidated']
        result['floor_appearance_diagnostic'] = self.floor_segmentation.status()
        result['map_planner'] = self.preview.planner
        result['map_route_cells'] = len(self.preview.route)
        result['map_message'] = self.preview.message
        result['camera_stationarity_verified'] = False
        result['note'] = 'Referencia estática independiente del mapa experimental V6.5; no ejecuta rutas.'
        return result

    def status(self):
        with self.lock:
            return {
                'mode': self.mode,
                'planner': self.planner,
                'esp32': self.link is not None,
                'ultrasonic': 'DESHABILITADO',
                'camera': self.cam is not None and self.cam.latest_jpeg() is not None,
                'camera_error': self.cam.error if self.cam else 'Cámara deshabilitada',
                'detections_last_frame': self.last_detection_count,
                'no_motors': self.no_motors,
                'supervised_motors': self.supervised_motors,
                'motion_state': self.motion_state,
                'corridor_history': self.corridor_history.status(),
                'camera_avoid': self.camera_avoid.status(),
                'camera_forward': camera_evaluate(self.floor_segmentation.status(),self._visual_obstacle_candidates(),self.visual_filter.status()['fresh']),
                'visual_safety': visual_safety(self.floor_segmentation.status(),self._visual_obstacle_candidates(),self.visual_filter.status()['fresh'], self.corridor_history.status()),
                'visual_test_override': self.visual_test_override,
                'visual_obstacle_candidates': len(self._visual_obstacle_candidates()),
                'visual_filter': self.visual_filter.status(),
                'contrast_near_candidates': [vars(d).copy() for d in self._visual_obstacle_candidates() if d.x <= cfg.ROBOT_RADIUS+.18],
                'floor_reference': self.floor_reference.status(),
                'camera_motion': self.camera_motion.status(),
                'motion_residual': self.motion_residual.status(),
                'region_tracking': self.region_tracker.status(),
                'floor_segmentation': self.floor_segmentation.status(),
                'visual_mapping': self.mapping.status(),
                'visual_planning': self._visual_planning_status(), 
                'seconds_remaining': round(max(0, self.manual_deadline-time.monotonic()),2) if self.mode=='MANUAL' else 0,
                'error': self.last_error,
                'navigation': self.brain.status(),
                'preview': {'active':self.preview_running, 'planner':self.planner,
                            'message':self.preview.message, 'demo':self.preview.demo,
                            'route_cells':len(self.preview.route)},
                'detections':list(self.last_detections),
                'visual_odometry': {k:v for k,v in self.visual_status.items() if k != 'path'},
            }

    def map_snapshot(self):
        with self.lock:
            return dict(self.preview.snapshot(self._visual_pose()), mapping=self.mapping.status())

    def reset_mapping(self):
        with self.lock:
            if self.mode not in ("PAUSED", "PREVIEW"): return False, "Detén primero los motores"
            self.preview.set_demo(False)
            self.mapping = VisualMapping()
            self.corridor_history = CorridorHistory()
            self.visual = VisualOdometry()
            self.visual_status = self.visual.status()
            return True, "Mapa y odometría reiniciados"

    def visual_path(self):
        with self.lock:
            return {'trajectory': list(self.visual_status.get('path', [])),
                    'quality': self.visual_status.get('quality', 'SIN DATOS'),
                    'units': 'metros estimados (requiere calibracion)'}

    def calibrate_floor(self):
        with self.lock:
            if self.mode in ('MANUAL','AUTO'):
                return False,'Detén primero los motores'
            frame=self.cam.latest() if self.cam else None
            ok, msg = self.floor_reference.calibrate(frame)
            if ok: self.camera_motion.reset_reference()
            return ok, msg

    def clear_floor(self):
        with self.lock:
            self.floor_reference.clear()
            self.camera_motion.reference_invalidated = True
            return True,'Referencia eliminada'

    def camera_jpeg(self):
        return None if self.cam is None else self.cam.latest_jpeg()

    def annotated_camera_jpeg(self):
        frame=self.cam.latest() if self.cam else None
        if frame is None:return None
        with self.lock:
            annotated=self.floor_reference.annotate(frame)
        ok,jpg=cv2.imencode('.jpg',annotated,[cv2.IMWRITE_JPEG_QUALITY,75])
        return jpg.tobytes() if ok else None

    def calibrate_segmentation(self):
        with self.lock:
            if self.mode not in ("PAUSED", "PREVIEW") or self.cam is None or self.cam.frame_age() >= 0.4:
                return False, "Requiere robot detenido y cámara reciente"
            ok,msg = self.floor_segmentation.calibrate(self.cam.latest())
            if ok:
                self.preview.set_demo(False)
                self.mapping = VisualMapping()
                self.corridor_history = CorridorHistory()
                self.visual = VisualOdometry()
                self.visual_status = self.visual.status()
            return ok,msg

    def segmentation_jpeg(self):
        with self.lock:
            frame = self.floor_segmentation.overlay
            if frame is None: return None
            ok, jpg = cv2.imencode(".jpg", frame)
            return jpg.tobytes() if ok else None

    def motion_residual_jpeg(self):
        with self.lock:
            return self.motion_residual.annotated_jpeg()

    def loop(self):
        last = time.monotonic()
        n = 0
        try:
            self.link = ESP32Link(cfg.ESP32_PORT, cfg.ESP32_BAUD)
            self.link.stop()
            while self._running:
                now = time.monotonic()
                dt = min(0.2, max(0.01, now - last))
                last = now
                telem = self.link.get_telemetry()
                u, fresh = None, False
                serial_ok = telem is not None and time.time() - self.link.last_rx < 0.5
                scan = None
                if fresh and u['valid'] and u['mm'] > 0:
                    dist = u['mm'] / 1000.0
                    if cfg.ULTRA_MIN_M <= dist <= cfg.ULTRA_MAX_M:
                        scan = (np.array([0.0], dtype=np.float32),
                                np.array([dist], dtype=np.float32))
                frame, capture_at = self.cam.latest_with_timestamp() if self.cam else (None, 0.0)
                # Observación visual: NO modifica brain.pose ni los comandos de motor.
                if frame is not None and capture_at > self.last_visual_capture and self.cam.frame_age()<.4 and now - self.last_visual_frame >= 0.15:
                    self.last_visual_capture = capture_at
                    yaw = None
                    if serial_ok and not (telem.faults & FAULT_IMU):
                        yaw = math.radians(telem.yaw_mdeg / 1000.0) * cfg.GYRO_YAW_SIGN
                    try:
                        self.camera_motion.update(frame, now)
                        with self.lock:
                            self.floor_segmentation.stationary = self.camera_motion.status(now)["state"] == "APARENTEMENTE_ESTABLE"
                            self.floor_segmentation.update(frame, now)
                        residual_status = self.motion_residual.update(frame, now)
                        self.region_tracker.update(
                            residual_status.get('candidate_boxes', []),
                            valid=residual_status.get('state') == 'RESIDUOS_CALCULADOS',
                            timestamp=now,
                            global_shift_px=residual_status.get('global_shift_px', (0, 0)))
                        if self.motion_residual.overlay is not None:
                            for track in self.region_tracker.status(now)['tracks']:
                                if track['misses']:
                                    continue
                                x, y, w, h = track['box']
                                color = (0, 255, 255) if track['persistent'] else (255, 180, 0)
                                cv2.rectangle(self.motion_residual.overlay, (x, y), (x+w, y+h), color, 2)
                                cv2.putText(self.motion_residual.overlay,
                                            'ID %s %s' % (track['id'], 'P' if track['persistent'] else 'N'),
                                            (x, max(12, y-5)), cv2.FONT_HERSHEY_SIMPLEX,
                                            0.42, color, 1, cv2.LINE_AA)
                        with self.lock:
                            visual_status = self.visual.update(frame, yaw, self.camera_motion.status(now)["state"])
                            self.visual_status = visual_status
                            if not self.preview.demo and yaw is not None and self.cam.frame_age()<.4 and visual_status["quality"] in ("SEGUIMIENTO", "REPOSO_CONFIRMADO") and self.floor_segmentation.mask is not None and self.floor_segmentation.status().get("state")=="SEGMENTACION_ACTIVA":
                                if visual_status.get("observation_recent") and self.floor_segmentation.status().get("frame_age_s",99)<.4:
                                    self.corridor_history.update(self.preview.grid, self.floor_segmentation.mask, self._visual_pose(), capture_at)
                                else:
                                    self.corridor_history.invalidate()
                                if self.mapping.frames == 0 or visual_status["quality"] in ("SEGUIMIENTO", "REPOSO_CONFIRMADO"):
                                    self.mapping.update(self.preview.grid, self.floor_segmentation.mask, self._visual_pose(), self.floor_segmentation.status().get("tracking",{}).get("tracks",[]))
                                if self.preview.goal is not None:
                                    self.preview.plan(self._visual_pose(), self.planner)
                            else:
                                self.corridor_history.invalidate()
                                self.mapping.state = "ESPERANDO: " + visual_status["quality"] + " / " + self.floor_segmentation.status().get("state", "SIN DATOS") + (" / SIN IMU" if yaw is None else "")
                    except Exception as exc:
                        with self.lock:
                            self.corridor_history.invalidate()
                            self.last_error = 'Odometría visual: ' + str(exc)
                            self.visual_status = dict(self.visual_status, quality='ERROR_VISUAL', observation_recent=False)
                    self.last_visual_frame = now
                detections = []
                if frame is not None and n % 3 == 0:
                    try:
                        detections = self.detector.detect(frame)
                    except Exception as exc:
                        with self.lock:
                            self.last_error = 'Visión: ' + str(exc)
                with self.lock:
                    self.last_ultra = u if fresh else None
                    if frame is not None:
                        self.last_cam_time = now
                    if scan is not None:
                        self.last_scan_time = now
                    if frame is not None and n % 3 == 0:
                        self.last_detection_count = len(detections)
                        self.last_detections_raw = detections
                        self.visual_filter.update(detections, AVOID_KINDS)
                        if not self.camera_motion.reference_invalidated and self.camera_motion.status()['state'] == 'APARENTEMENTE_ESTABLE':
                            self.floor_reference.analyze(frame)
                        self.last_detections = [{'kind':d.kind,'x':round(d.x,3),
                            'y':round(d.y,3),'length':round(d.length,3)} for d in detections[:30]]
                    # Actualizar mapa real solo con ultrasonido válido y orientación visual disponible.
                    if scan is not None and serial_ok and self.visual_status.get('recent'):
                        self.preview.scan(self._visual_pose(), scan)
                    if self.preview_running and self.preview.goal is not None and n % 10 == 0:
                        self.preview.plan(self._visual_pose(), self.planner)
                    if self.mode == 'AUTO':
                        history = self.corridor_history.status()
                        v,w,msg = self.camera_avoid.step(self.floor_segmentation.status(), self._visual_obstacle_candidates(), self.visual_filter.status()['fresh'], time.monotonic()) if not self.startup_active else (0.,0.,'ARRANQUE')
                        safety = visual_safety(self.floor_segmentation.status(),self._visual_obstacle_candidates(),self.visual_filter.status()['fresh'], history)
                        reason = None
                        if self.startup_active:
                            v,w,msg = startup_command(self.floor_segmentation.status(), self.visual_status['quality'], self._visual_pose(), safety, history)
                            if camera_evaluate(self.floor_segmentation.status(), self._visual_obstacle_candidates(), self.visual_filter.status()['fresh'])['blocked']:
                                v=w=0.; msg='ARRANQUE: PARADA POR CAMARA'
                            if time.monotonic() >= self.startup_until:
                                reason='ARRANQUE TERMINADO: SUELTA EL BOTON'
                        if not self._hardware_safe(): reason='PARADA: ESP32 / IMU / CAMARA'
                        elif time.monotonic() >= self.manual_deadline: reason='PARADA: PERMISO CADUCADO'
                        elif self.preview.demo or self.visual_test_override: reason='PARADA: MODO INCOMPATIBLE'
                        elif self.startup_active and (v or w) and any(r!='CORREDOR SIN OBSERVACIONES SUFICIENTES' for r in safety['reasons']): reason='PARADA: '+ ' / '.join(safety['reasons'])
                        elif w and (self.startup_active or self.camera_avoid.phase!='TURN'): reason='PARADA: GIRO NO AUTORIZADO'
                        elif self.startup_active and not self.visual_status.get('observation_recent'): reason='PARADA: ODOMETRIA SIN DATOS RECIENTES'
                        if reason:
                            v=w=0.; msg=reason
                            if reason != 'PARADA: ODOMETRIA SIN DATOS RECIENTES': self.mode='PAUSED'
                        if not self.startup_active and (reason or (not (v or w) and self.camera_avoid.phase not in ('BRAKE','SETTLE'))):
                            self.camera_stop_latched = True
                            self.mode = 'PAUSED'
                        if self.startup_active and not (v or w):
                            self.startup_until=0.0
                            self.mode='PAUSED'
                        self.motion_state=msg
                        if v or w: self.link.send_drive(v,w)
                        else: self.link.stop()
                    elif self.mode == 'MANUAL':
                        safe = self._hardware_safe() and now < self.manual_deadline
                        visual_block = self.manual_v > 0 and (bool(self._visual_obstacle_candidates()) or not self.visual_filter.status()['fresh']) and not self.visual_test_override
                        if not safe or visual_block:
                            self.mode = 'PAUSED'
                            self.manual_deadline = 0.0
                            self.motion_state = 'PARADA: CONTROL/SENSOR/DETECCIÓN'
                            self.link.stop()
                        else:
                            self.link.send_drive(self.manual_v, self.manual_w)
                    else:
                        self.link.stop()
                n += 1
                time.sleep(max(0, 1 / cfg.TICK_HZ - (time.monotonic() - now)))
        except Exception as exc:
            with self.lock:
                self.mode = 'STOPPED'
                self.last_error = str(exc)
            print('Error del robot:', exc, flush=True)
        finally:
            if self.link:
                try:
                    self.link.stop()
                    self.link.close()
                except Exception as exc:
                    print('Error cerrando enlace ESP32:', exc, flush=True)
                finally:
                    with self.lock:
                        self.link = None

    def close(self):
        self.stop()
        self._running = False
        self.worker.join(timeout=1.5)
        if self.cam:
            self.cam.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--no-motors', action='store_true', help='Prueba de interfaz sin mover ruedas')
    ap.add_argument('--no-camera', action='store_true')
    ap.add_argument('--supervised-motors', action='store_true', help='Habilita pruebas de motores con control mantenido (sin HC-SR04)')
    ap.add_argument('--host', default='127.0.0.1',
                    help='127.0.0.1 por defecto; 0.0.0.0 solo en LAN confiable')
    ap.add_argument('--port', type=int, default=5000)
    args = ap.parse_args()
    # Los motores requieren autorización explícita y no pueden ejecutarse con --no-motors.
    robot = Controller(no_motors=(not args.supervised_motors) or args.no_motors,
                       no_camera=args.no_camera, supervised_motors=args.supervised_motors)
    try:
        print(f'Panel: http://127.0.0.1:{args.port} | INICIA PAUSADO', flush=True)
        make_app(robot).run(host=args.host, port=args.port,
                            threaded=True, use_reloader=False)
    finally:
        robot.close()


if __name__ == '__main__':
    main()

