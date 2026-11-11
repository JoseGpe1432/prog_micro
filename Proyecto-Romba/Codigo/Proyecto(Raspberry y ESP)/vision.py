"""
vision.py
---------
Detección de basura por TAMAÑO REAL, sin entrenar con imágenes.

Idea: la cámara está a una altura y ángulo conocidos, así que cada píxel del piso
corresponde a una posición física (modelo pinhole + plano del piso). Entonces:

  1. SEGMENTAR  lo que se diferencia del piso (color Lab vs. un fondo local por mediana)
  2. PROYECTAR  el contorno de cada objeto al plano del piso  ->  metros
  3. MEDIR      largo y ancho reales (rectángulo mínimo en el piso)
  4. DECIDIR    por tamaño/forma:  DEBRIS (cabe) | TOO_BIG (rodear) | CABLE (rodear) | IGNORE

Limitaciones físicas (no se pueden evitar con una sola cámara):
  * Se asume que el objeto está APOYADO en el piso. Un objeto alto parece más grande/lejano.
  * La resolución cae con la distancia (config.DETECT_MAX_RANGE): lejos el tamaño es aproximado.
  * Requiere contraste con el piso; pisos con mucho dibujo producen falsos positivos
    (el cerebro pide varios avistamientos antes de actuar).
  * La precisión depende de calibrar altura/ángulo/FOV (calibrate_camera.py).
"""
import math

import cv2
import numpy as np

import config as cfg
from detection import Detection, DEBRIS, TOO_BIG, CABLE, IGNORE


class GroundProjector:
    """Modelo pinhole: píxel (u, v) <-> punto del piso (x adelante, y izquierda) en metros,
    respecto al centro del robot."""

    def __init__(self, width=None, height=None):
        w, h = cfg.CAMERA_SIZE
        self.w, self.h = width or w, height or h
        self.fx = (self.w / 2.0) / math.tan(math.radians(cfg.CAM_HFOV_DEG) / 2.0)
        self.fy = self.fx
        self.cx, self.cy = self.w / 2.0, self.h / 2.0
        a = math.radians(cfg.CAM_PITCH_DEG)
        self.sa, self.ca = math.sin(a), math.cos(a)

    def pixel_to_floor(self, u, v):
        x, y, ok = self.pixels_to_floor(np.array([u], np.float64), np.array([v], np.float64))
        return (float(x[0]), float(y[0])) if ok[0] else None

    def pixels_to_floor(self, us, vs):
        """Versión vectorizada. Devuelve (x, y, válido)."""
        xn = (us - self.cx) / self.fx
        yn = (vs - self.cy) / self.fy
        denom = self.sa + self.ca * yn              # componente hacia abajo del rayo
        ok = denom > 1e-6
        t = np.where(ok, cfg.CAM_HEIGHT / np.where(ok, denom, 1.0), 0.0)
        x = cfg.CAM_X + t * (self.ca - self.sa * yn)
        y = -t * xn
        return x, y, ok

    def floor_to_pixel(self, x, y):
        """Inversa (para pruebas y para dibujar): punto del piso -> píxel."""
        dx = x - cfg.CAM_X
        zc = dx * self.ca + cfg.CAM_HEIGHT * self.sa
        yc = -dx * self.sa + cfg.CAM_HEIGHT * self.ca
        xc = -y
        ok = zc > 1e-6
        zc = np.where(ok, zc, 1.0)
        return self.cx + self.fx * xc / zc, self.cy + self.fy * yc / zc, ok


def classify(length, width):
    """Decide qué hacer con un objeto según su tamaño físico (m). Sin aprendizaje."""
    if length > cfg.MAX_OBJECT_DIM or length < cfg.PICKUP_MIN_DIM:
        return IGNORE
    if width < cfg.CABLE_MIN_WIDTH:                       # hilo, cabello, pelusa alargada
        return DEBRIS if length <= cfg.HAIR_MAX_LEN else CABLE
    if length >= cfg.CABLE_MIN_LENGTH and length / max(width, 1e-3) >= cfg.CABLE_ASPECT:
        return CABLE                                      # cable/cordón grueso: enreda cepillos
    return DEBRIS if length <= cfg.PICKUP_MAX_DIM else TOO_BIG


class FloorObjectDetector:
    def __init__(self, roi_top=0.05, min_pixels=8):
        self.roi_top = roi_top
        self.min_pixels = min_pixels
        self._proj = None

    def _projector(self, w, h):
        if self._proj is None or (self._proj.w, self._proj.h) != (w, h):
            self._proj = GroundProjector(w, h)
        return self._proj

    def segment(self, bgr):
        """Máscara de lo que difiere del piso. Devuelve (máscara, y0 del ROI)."""
        h = bgr.shape[0]
        y0 = int(h * self.roi_top)                       # ignora la parte alta (horizonte)
        roi = bgr[y0:, :]
        lab = cv2.cvtColor(cv2.GaussianBlur(roi, (3, 3), 0), cv2.COLOR_BGR2LAB)
        sw, sh = max(8, roi.shape[1] // 8), max(8, roi.shape[0] // 8)
        small = cv2.resize(lab, (sw, sh), interpolation=cv2.INTER_AREA)
        k = max(5, (sw // 4) | 1)                        # kernel ~25% del ancho: robusto a objetos grandes
        bg_small = cv2.medianBlur(small, k)
        bg = cv2.resize(bg_small, (roi.shape[1], roi.shape[0]), interpolation=cv2.INTER_LINEAR)
        diff = lab.astype(np.float32) - bg.astype(np.float32)
        dev = np.sqrt((diff ** 2).sum(axis=2))
        med = float(np.median(dev))
        mad = float(np.median(np.abs(dev - med))) * 1.4826
        thr = max(cfg.VISION_MIN_DEVIATION, med + 6.0 * mad)
        mask = (dev > thr).astype(np.uint8) * 255
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        return mask, y0

    def detect(self, bgr, with_ignored=False):
        """Devuelve la lista de Detection (posición y tamaño reales en metros)."""
        proj = self._projector(bgr.shape[1], bgr.shape[0])
        mask, y0 = self.segment(bgr)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        out = []
        for c in contours:
            if cv2.contourArea(c) < self.min_pixels and len(c) < self.min_pixels:
                continue
            pts = c.reshape(-1, 2).astype(np.float64)
            us, vs = pts[:, 0], pts[:, 1] + y0
            x, y, ok = proj.pixels_to_floor(us, vs)
            if ok.mean() < 0.9:
                continue
            x, y = x[ok], y[ok]
            (cx, cy), (rw, rh), _ = cv2.minAreaRect(np.stack([x, y], axis=1).astype(np.float32))
            # los puntos del contorno son centros de píxel del borde: sumar ~1 píxel de tamaño
            px = proj.pixels_to_floor(np.array([us.mean(), us.mean() + 1]), np.array([vs.mean(), vs.mean() + 1]))
            pix = float(math.hypot(px[0][1] - px[0][0], px[1][1] - px[1][0])) / math.sqrt(2.0)
            length, width = max(rw, rh) + pix, min(rw, rh) + pix
            if not (0.05 < cx < cfg.DETECT_MAX_RANGE):
                continue
            kind = classify(length, width)
            if kind == IGNORE and not with_ignored:
                continue
            out.append(Detection(float(cx), float(cy), float(length), float(width), kind))
        return out
