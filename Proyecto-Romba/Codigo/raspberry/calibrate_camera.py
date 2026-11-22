#!/usr/bin/env python3
"""
calibrate_camera.py -- calibra INCLINACIÓN y CAMPO DE VISIÓN de la cámara con una cinta métrica.
(La altura y CAM_X se miden directamente con regla y se escriben en config.py.)

Procedimiento:
  1. Mide con regla: altura de la lente al piso (CAM_HEIGHT) y distancia de la lente al centro
     del robot, hacia adelante (CAM_X). Escríbelas en config.py.
  2. Pon 3-5 marcas pequeñas (cinta de color) en el piso, DELANTE del robot, a distancias
     conocidas medidas desde el CENTRO del robot (p. ej. 0.30, 0.45, 0.60 m al frente, algunas
     a la izquierda/derecha). Mide también cuánto se desplazan lateralmente (izquierda = +).
  3. Toma una foto:        python3 calibrate_camera.py --capture frame.jpg
  4. Abre frame.jpg, anota el píxel (u,v) de cada marca (cualquier visor/GIMP) y ejecuta:
        python3 calibrate_camera.py --image frame.jpg --mark 320,400:0.30 --mark 330,300:0.50 \\
                                    --mark 150,320:0.50,0.20 --mark 330,230:0.70
     formato de cada marca:  u,v:adelante[,izquierda]   (metros)
  5. Copia a config.py los valores CAM_PITCH_DEG y CAM_HFOV_DEG que imprime.
  6. Verifica el tamaño con un objeto conocido (moneda):
        python3 calibrate_camera.py --image frame.jpg --measure 300,350 340,352 [mismas marcas]
"""
import argparse
import math

import numpy as np

import config as cfg


def _project(alpha_deg, hfov_deg, uv, width, hpx, height, cam_x):
    a = math.radians(alpha_deg)
    fx = (width / 2.0) / math.tan(math.radians(hfov_deg) / 2.0)
    xn = (uv[:, 0] - width / 2.0) / fx
    yn = (uv[:, 1] - hpx / 2.0) / fx
    denom = math.sin(a) + math.cos(a) * yn
    if np.any(denom <= 1e-6):
        return None
    t = height / denom
    return np.stack([cam_x + t * (math.cos(a) - math.sin(a) * yn), -t * xn], axis=1)


def solve(marks, height, cam_x, width, hpx):
    """marks: [(u, v, x_adelante, y_izquierda)] -> (pitch_deg, hfov_deg, error_rms_m)"""
    uv = np.array([[m[0], m[1]] for m in marks], dtype=np.float64)
    truth = np.array([[m[2], m[3]] for m in marks], dtype=np.float64)

    def cost(al, hf):
        p = _project(al, hf, uv, width, hpx, height, cam_x)
        return np.inf if p is None else float(((p - truth) ** 2).sum())

    best = (np.inf, 45.0, 60.0)
    for al in np.arange(10.0, 85.0, 0.5):
        for hf in np.arange(35.0, 110.0, 1.0):
            c = cost(al, hf)
            if c < best[0]:
                best = (c, al, hf)
    for step_a, step_h in ((0.1, 0.2), (0.02, 0.05)):           # refinamiento local
        c0, a0, h0 = best
        for al in np.arange(a0 - 10 * step_a, a0 + 10 * step_a + 1e-9, step_a):
            for hf in np.arange(h0 - 10 * step_h, h0 + 10 * step_h + 1e-9, step_h):
                c = cost(al, hf)
                if c < best[0]:
                    best = (c, al, hf)
    return best[1], best[2], math.sqrt(best[0] / len(marks))


def parse_mark(s):
    pix, _, dist = s.partition(":")
    u, v = (float(t) for t in pix.split(","))
    d = [float(t) for t in dist.split(",")]
    return (u, v, d[0], d[1] if len(d) > 1 else 0.0)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--capture", metavar="ARCHIVO.jpg")
    ap.add_argument("--image")
    ap.add_argument("--mark", action="append", default=[])
    ap.add_argument("--measure", nargs=2, metavar=("U1,V1", "U2,V2"))
    a = ap.parse_args()

    if a.capture:
        import cv2
        cap = cv2.VideoCapture(cfg.CAMERA_INDEX)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, cfg.CAMERA_SIZE[0])
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cfg.CAMERA_SIZE[1])
        for _ in range(10):
            cap.read()
        ok, f = cap.read()
        cap.release()
        if not ok:
            raise SystemExit("No se pudo leer la cámara")
        cv2.imwrite(a.capture, f)
        print(f"Foto guardada en {a.capture} ({f.shape[1]}x{f.shape[0]})")
        return

    if len(a.mark) < 2:
        raise SystemExit("Necesitas al menos 2 marcas (mejor 4-5) con --mark u,v:adelante[,izquierda]")
    w, h = cfg.CAMERA_SIZE
    if a.image:
        import cv2
        img = cv2.imread(a.image)
        h, w = img.shape[:2]
    marks = [parse_mark(m) for m in a.mark]
    pitch, hfov, rms = solve(marks, cfg.CAM_HEIGHT, cfg.CAM_X, w, h)
    print(f"Con CAM_HEIGHT={cfg.CAM_HEIGHT} m y CAM_X={cfg.CAM_X} m (imagen {w}x{h}):")
    print(f"   CAM_PITCH_DEG = {pitch:.2f}")
    print(f"   CAM_HFOV_DEG  = {hfov:.2f}")
    print(f"   error medio de ajuste = {rms*1000:.1f} mm  "
          f"({'bien' if rms < 0.015 else 'revisa las marcas, la altura o CAM_X'})")
    if a.measure:
        p = [tuple(float(t) for t in s.split(",")) for s in a.measure]
        pts = _project(pitch, hfov, np.array(p), w, h, cfg.CAM_HEIGHT, cfg.CAM_X)
        d = float(np.hypot(*(pts[0] - pts[1])))
        print(f"Distancia real entre los dos puntos: {d*1000:.1f} mm")


if __name__ == "__main__":
    main()
