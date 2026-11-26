"""V4: comparación de escena fija y evaluación por zonas; SOLO diagnóstico.
No estima profundidad ni certifica zonas transitables. No controla motores.
"""
from collections import deque
import time
import cv2
import numpy as np

class FloorReference:
    def __init__(self):
        self.reference = None
        self.shape = None
        self.last = None
        self.last_mask = None
        self.updated = 0.0
        self.history = deque(maxlen=5)
        self.warning = ('Comparación de cámara FIJA. Izquierda/derecha son hipótesis visuales, '
                        'NO rutas seguras. No usar para activar motores autónomos.')

    def calibrate(self, frame):
        if frame is None:
            return False, 'No hay imagen'
        self.reference = cv2.cvtColor(cv2.GaussianBlur(frame, (7, 7), 0), cv2.COLOR_BGR2LAB)
        self.shape = frame.shape[:2]
        self.last = None
        self.last_mask = None
        self.history.clear()
        self.updated = time.monotonic()
        return True, 'Referencia capturada. Cámara fija; diagnóstico solamente'

    def clear(self):
        self.reference = None
        self.last = None
        self.last_mask = None
        self.history.clear()

    @staticmethod
    def _regions(w, h):
        # La parte superior suele contener paredes y mobiliario distante.
        # Regiones iguales, con franja inferior recortada para evitar el propio robot.
        y0, y1 = int(h * .40), int(h * .94)
        return {
            'izquierda': (int(w * .08), int(w * .36), y0, y1),
            'centro': (int(w * .36), int(w * .64), y0, y1),
            'derecha': (int(w * .64), int(w * .92), y0, y1),
        }

    def analyze(self, frame, update_history=True):
        if frame is None or self.reference is None or frame.shape[:2] != self.shape:
            return None
        h, w = frame.shape[:2]
        lab = cv2.cvtColor(cv2.GaussianBlur(frame, (7, 7), 0), cv2.COLOR_BGR2LAB)
        # Compensar cambios globales leves de brillo, no cambios de geometría.
        delta = lab.astype(np.float32) - self.reference.astype(np.float32)
        brightness_shift = float(np.clip(np.median(delta[:, :, 0]), -15, 15))
        delta[:, :, 0] -= brightness_shift
        score = np.sqrt(.20 * delta[:, :, 0] ** 2 + delta[:, :, 1] ** 2 + delta[:, :, 2] ** 2)
        mask = np.uint8(score > 30) * 255
        mask[:int(h * .40), :] = 0
        mask[int(h * .94):, :] = 0
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
        regions = self._regions(w, h)
        ratios = {}
        for name, (x0, x1, y0, y1) in regions.items():
            area = max(1, (x1 - x0) * (y1 - y0))
            ratios[name] = float(np.count_nonzero(mask[y0:y1, x0:x1])) / area
        if update_history:
            self.history.append(ratios)
        # Sin historial suficiente, no proponer una dirección.
        confirmed = {}
        for name in regions:
            votes = sum(item[name] >= .09 for item in self.history)
            confirmed[name] = len(self.history) >= 5 and votes >= 3
        decision = 'ESPERAR_CONFIRMACION'
        if len(self.history) >= 5:
            if confirmed['centro']:
                if not confirmed['izquierda'] and confirmed['derecha']:
                    decision = 'EVALUAR_IZQUIERDA'
                elif not confirmed['derecha'] and confirmed['izquierda']:
                    decision = 'EVALUAR_DERECHA'
                elif not confirmed['izquierda'] and not confirmed['derecha']:
                    # Elegir menor diferencia solo si la separación es notable.
                    difference = ratios['izquierda'] - ratios['derecha']
                    decision = ('EVALUAR_DERECHA' if difference > .12 else
                                'EVALUAR_IZQUIERDA' if difference < -.12 else
                                'DETENER_Y_REEVALUAR')
                else:
                    decision = 'DETENER_Y_REEVALUAR'
            elif confirmed['izquierda'] and not confirmed['derecha']:
                decision = 'EVALUAR_DERECHA'
            elif confirmed['derecha'] and not confirmed['izquierda']:
                decision = 'EVALUAR_IZQUIERDA'
            elif any(confirmed.values()):
                decision = 'DETENER_Y_REEVALUAR'
            else:
                decision = 'SIN_CAMBIO_CONFIRMADO'
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        boxes = []
        for contour in contours:
            if cv2.contourArea(contour) < max(150, h * w * .001):
                continue
            boxes.append(tuple(map(int, cv2.boundingRect(contour))))
        self.last_mask = mask
        self.last = {
            'changed_ratio': round(ratios['centro'], 4),
            'change_detected': confirmed['centro'],
            'boxes': boxes[:20],
            'zones': {name: {'changed_ratio': round(ratios[name], 4),
                             'confirmed_change': confirmed[name]} for name in regions},
            'frames': len(self.history),
            'updated_monotonic': time.monotonic(),
            'decision': decision,
            'valid_for_motion': False,
            'warning': self.warning,
        }
        return mask

    def status(self):
        return {'calibrated': self.reference is not None,
                **(self.last or {'changed_ratio': None, 'change_detected': False,
                                 'boxes': [], 'zones': {}, 'frames': 0,
                                 'decision': 'SIN_REFERENCIA',
                                 'valid_for_motion': False, 'warning': self.warning})}

    def annotate(self, frame):
        image = frame.copy()
        if self.reference is None:
            cv2.putText(image, 'SIN REFERENCIA: captura suelo despejado', (10, 28),
                        cv2.FONT_HERSHEY_SIMPLEX, .55, (0, 210, 255), 2)
            return image
        # No registrar otro voto por cada petición HTTP de vídeo.
        mask = self.analyze(frame, update_history=False)
        h, w = image.shape[:2]
        if mask is not None:
            overlay = image.copy()
            overlay[mask > 0] = (0, 0, 255)
            image = cv2.addWeighted(image, .75, overlay, .25, 0)
            for name, (x0, x1, y0, y1) in self._regions(w, h).items():
                zone = self.last['zones'][name]
                color = (0, 0, 255) if zone['confirmed_change'] else (255, 220, 0)
                cv2.rectangle(image, (x0, y0), (x1, y1), color, 2)
                cv2.putText(image, f"{name[:3].upper()} {zone['changed_ratio']:.0%}",
                            (x0 + 3, y0 + 21), cv2.FONT_HERSHEY_SIMPLEX, .48, color, 2)
            cv2.putText(image, self.last['decision'], (10, 28),
                        cv2.FONT_HERSHEY_SIMPLEX, .54, (0, 255, 255), 2)
        return image
