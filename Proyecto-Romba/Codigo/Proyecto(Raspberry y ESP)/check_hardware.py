#!/usr/bin/env python3
"""
check_hardware.py -- puesta en marcha, un componente a la vez.

    python3 check_hardware.py imu       # ¿qué chip es el "HW-123"? (dirección I2C y WHO_AM_I)
    python3 check_hardware.py esp32     # telemetría: batería, yaw, IR de precipicio, fallas
    python3 check_hardware.py encoders  # calibra pulsos por metro (empujando el robot 1 m)
    python3 check_hardware.py motors    # prueba cada sentido de giro (RUEDAS EN EL AIRE)
    python3 check_hardware.py lidar     # distancias en 8 sectores
    python3 check_hardware.py camera    # detecta objetos, mide su tamaño real y los clasifica
"""
import math
import sys
import time

import numpy as np

import config as cfg

WHO = {0x68: "MPU6050 (o compatible)", 0x69: "MPU6050 con AD0=1", 0x70: "MPU6500",
       0x71: "MPU9250", 0x73: "MPU9255", 0x98: "ICM-20689?", 0x12: "ICM-20602"}


def do_imu():
    from esp32_link import ESP32Link
    link = ESP32Link(cfg.ESP32_PORT, cfg.ESP32_BAUD)
    try:
        for _ in range(10):
            link.request_imu_info()
            time.sleep(0.3)
            if link.imu_info:
                break
        if not link.imu_info:
            raise SystemExit("El ESP32 no respondió ($W). ¿Firmware cargado y puerto correcto?")
        addr, who, ok = (int(v) for v in link.imu_info)
        print(f"Dirección I2C: 0x{addr:02X}   WHO_AM_I: 0x{who:02X} -> {WHO.get(who, 'DESCONOCIDO')}")
        print("Giroscopio leído correctamente:", "SÍ" if ok else "NO (revisa SDA/SCL/alimentación)")
        if who not in WHO:
            print("⚠ Chip no reconocido: el firmware asume registros tipo MPU6050. Dime el número grabado en el chip.")
        print("Ahora gira el robot a mano ANTIHORARIO: el yaw debe SUBIR (si baja, GYRO_YAW_SIGN=-1 en firmware y config).")
        t0 = time.time()
        while time.time() - t0 < 12:
            t = link.get_telemetry()
            if t:
                print(f"\ryaw = {t.yaw_mdeg/1000:8.1f}°", end="", flush=True)
            time.sleep(0.1)
        print()
    finally:
        link.close()


def do_esp32():
    from esp32_link import ESP32Link
    link = ESP32Link(cfg.ESP32_PORT, cfg.ESP32_BAUD)
    print("Compara el voltaje con un multímetro (BATT_CAL). Tapa/destapa cada IR. Ctrl+C para salir.")
    try:
        while True:
            t = link.get_telemetry()
            if t:
                print(f"bat={t.batt_mv/1000:6.2f}V  yaw={t.yaw_mdeg/1000:8.1f}°  encL={t.enc_l:7d} encR={t.enc_r:7d}"
                      f"  cliff={t.cliff_mask:04b}  fallas={t.faults:05b}")
            else:
                print("(sin telemetría)")
            time.sleep(0.3)
    except KeyboardInterrupt:
        pass
    finally:
        link.close()


def do_encoders():
    from esp32_link import ESP32Link
    link = ESP32Link(cfg.ESP32_PORT, cfg.ESP32_BAUD)
    try:
        input("Pon el robot en el piso con una cinta métrica de 1.00 m al lado. Enter para marcar el INICIO...")
        link.zero()
        time.sleep(0.5)
        input("EMPUJA el robot EN LÍNEA RECTA hacia ADELANTE exactamente 1.00 m, sin patinar. Enter al terminar...")
        time.sleep(0.3)
        t = link.get_telemetry()
        if not t:
            raise SystemExit("Sin telemetría")
        l, r = t.enc_l, t.enc_r
        print(f"Pulsos: izquierda={l}  derecha={r}")
        if l <= 0 or r <= 0:
            print("⚠ Un contador salió <= 0 al avanzar: invierte ENC_L_SIGN / ENC_R_SIGN en el firmware"
                  " (o revisa A/B). Si salen 0: cableado/pines.")
        avg = 0.5 * (abs(l) + abs(r))
        circ = math.pi * cfg.WHEEL_DIAMETER
        print(f"TICKS_PER_M ≈ {avg:.0f}  ->  pulsos por vuelta de rueda ENC_TICKS_PER_REV ≈ {avg*circ:.1f}")
        if abs(l - r) > 0.05 * max(1, avg):
            print("⚠ Las dos ruedas difieren >5 %: ¿empujaste recto? ¿diámetro distinto? ¿un canal con ruido?")
        print("Pon ese valor en ENC_TICKS_PER_REV (firmware y config.py). Luego prueba USE_ENCODERS=True.")
        print("Además: si el valor ≈ la mitad/doble de lo esperado, tu encoder es de un canal o cuenta flancos distintos.")
    finally:
        link.close()


def do_motors():
    from esp32_link import ESP32Link
    input("⚠ RUEDAS EN EL AIRE. Enter para continuar...")
    link = ESP32Link(cfg.ESP32_PORT, cfg.ESP32_BAUD)
    tests = [("adelante (ambas ruedas hacia adelante)", 0.10, 0.0),
             ("atrás", -0.10, 0.0),
             ("girar a la IZQUIERDA (antihorario): rueda der adelante, izq atrás", 0.0, 1.0),
             ("girar a la DERECHA (horario)", 0.0, -1.0)]
    try:
        for name, v, w in tests:
            print("->", name)
            t0 = time.time()
            while time.time() - t0 < 1.5:
                link.send_drive(v, w)
                time.sleep(0.1)
            link.stop()
            time.sleep(0.7)
            t = link.get_telemetry()
            if t:
                print(f"   yaw={t.yaw_mdeg/1000:.1f}°  encL={t.enc_l} encR={t.enc_r}")
        print("Si 'izquierda' NO subió el yaw: GYRO_YAW_SIGN. Si una rueda gira al revés: MOTOR_*_SIGN en el firmware.")
    finally:
        link.close()


def do_lidar():
    from lidar_ld06 import LD06Reader
    lid = LD06Reader()
    print("Sectores (m): frente, izq-frente, izq, izq-atrás, atrás, der-atrás, der, der-frente. Ctrl+C para salir.")
    try:
        while True:
            s = lid.latest()
            if s is None:
                time.sleep(0.05)
                continue
            ang, d = s
            row = []
            for k in range(8):
                c = math.radians(45 * k)
                diff = np.abs(np.arctan2(np.sin(ang - c), np.cos(ang - c)))
                m = diff < math.radians(22.5)
                row.append(f"{d[m].min():5.2f}" if m.any() else "  -- ")
            print(f"puntos={len(d):4d}  " + "  ".join(row))
            time.sleep(0.3)
    except KeyboardInterrupt:
        pass
    finally:
        lid.close()


def do_camera():
    import cv2
    from vision import FloorObjectDetector
    cap = cv2.VideoCapture(cfg.CAMERA_INDEX)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, cfg.CAMERA_SIZE[0])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cfg.CAMERA_SIZE[1])
    for _ in range(10):
        cap.read()
    ok, frame = cap.read()
    cap.release()
    if not ok:
        raise SystemExit("No se pudo leer la cámara (config.CAMERA_INDEX).")
    from vision import GroundProjector
    gp = GroundProjector(frame.shape[1], frame.shape[0])
    det = FloorObjectDetector().detect(frame, with_ignored=True)
    colors = {"DEBRIS": (0, 200, 0), "TOO_BIG": (0, 0, 255), "CABLE": (0, 165, 255), "IGNORE": (160, 160, 160)}
    print(f"{len(det)} objetos. Compara los tamaños con una regla (si difieren, ejecuta calibrate_camera.py):")
    for d in det:
        # dibuja: reproyecta el centro del objeto al píxel
        u, v, ok2 = gp.floor_to_pixel(np.array([d.x]), np.array([d.y]))
        if ok2[0]:
            cv2.circle(frame, (int(u[0]), int(v[0])), 16, colors[d.kind], 2)
            cv2.putText(frame, f"{d.length*100:.1f}cm {d.kind}", (int(u[0]) + 18, int(v[0])),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, colors[d.kind], 1)
        print(f"   {d.kind:8s} a {d.x:.2f} m al frente, {d.y:+.2f} m lateral: {d.length*100:5.1f} x {d.width*100:4.1f} cm")
    cv2.imwrite("frame.jpg", frame)
    print("Imagen anotada guardada en frame.jpg  (verde=aspirar, rojo/naranja=rodear, gris=ignorado)")


if __name__ == "__main__":
    modes = {"imu": do_imu, "esp32": do_esp32, "encoders": do_encoders, "motors": do_motors,
             "lidar": do_lidar, "camera": do_camera}
    if len(sys.argv) != 2 or sys.argv[1] not in modes:
        raise SystemExit(__doc__)
    modes[sys.argv[1]]()
