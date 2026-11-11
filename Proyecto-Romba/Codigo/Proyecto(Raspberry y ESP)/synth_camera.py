"""
synth_camera.py
---------------
Genera imágenes SINTÉTICAS de una cámara apuntando a un piso con objetos de tamaño físico
conocido, usando la misma geometría (altura/inclinación/FOV) que vision.GroundProjector.
Sirve para probar que la medición de tamaños y la clasificación son coherentes. NO
reproduce el aspecto de un piso real (texturas, brillos, sombras): validar con fotos reales.
"""
import cv2
import numpy as np

from vision import GroundProjector

S = 1000          # píxeles por metro del plano del piso
X0, X1, Y0, Y1 = 0.0, 1.4, -0.7, 0.7


def floor_texture(seed=0, base=(105, 128, 150), grain=6.0):
    rng = np.random.default_rng(seed)
    h, w = int((Y1 - Y0) * S), int((X1 - X0) * S)
    noise = cv2.GaussianBlur(rng.normal(0, 1, (h, w)).astype(np.float32), (0, 0), sigmaX=2, sigmaY=40)
    noise = noise / noise.std() * grain
    tex = np.dstack([np.full((h, w), c, np.float32) + noise * k for c, k in zip(base, (0.8, 1.0, 1.1))])
    return tex


def draw_object(tex, kind, x, y, length, width, angle_deg, color):
    """Dibuja un objeto de length x width METROS centrado en (x, y) del piso."""
    c = (int((x - X0) * S), int((y - Y0) * S))
    axes = (max(1, int(length * S / 2)), max(1, int(width * S / 2)))
    if kind == "ellipse":
        cv2.ellipse(tex, c, axes, angle_deg, 0, 360, color, -1, cv2.LINE_AA)
    else:
        box = cv2.boxPoints(((c[0], c[1]), (length * S, width * S), angle_deg)).astype(np.int32)
        cv2.fillPoly(tex, [box], color, cv2.LINE_AA)


def render(objects, size=(640, 480), seed=0, noise=2.5, grain=6.0):
    """objects: [(kind, x, y, length, width, angle_deg, (b,g,r)), ...] -> imagen BGR uint8"""
    tex = floor_texture(seed, grain=grain)
    for o in objects:
        draw_object(tex, *o)
    w, h = size
    proj = GroundProjector(w, h)
    uu, vv = np.meshgrid(np.arange(w, dtype=np.float64), np.arange(h, dtype=np.float64))
    x, y, ok = proj.pixels_to_floor(uu, vv)
    mx = ((x - X0) * S).astype(np.float32)
    my = ((y - Y0) * S).astype(np.float32)
    img = cv2.remap(tex, mx, my, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=(150, 150, 150))
    img[~ok] = (150, 150, 150)
    # viñeteo suave + desenfoque + ruido de sensor
    yy, xx = np.mgrid[0:h, 0:w]
    vig = 1.0 - 0.12 * (((xx - w / 2) / (w / 2)) ** 2 + ((yy - h / 2) / (h / 2)) ** 2)
    img = img * vig[..., None]
    img = cv2.GaussianBlur(img, (0, 0), 0.7)
    rng = np.random.default_rng(seed + 1)
    img = img + rng.normal(0, noise, img.shape)
    return np.clip(img, 0, 255).astype(np.uint8)
