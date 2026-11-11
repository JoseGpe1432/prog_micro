"""
config.py
---------
Parámetros del robot (lado Raspberry Pi). AJÚSTALOS a tu hardware real:
las marcadas con [MEDIR] deben calibrarse con tu robot.
"""
import math

# ---------------- Puertos ----------------
# Mejor usar rutas estables: /dev/serial/by-id/usb-...
ESP32_PORT = "/dev/ttyUSB0"
ESP32_BAUD = 115200
LIDAR_PORT = "/dev/ttyUSB1"
LIDAR_BAUD = 230400          # LD06 / LD19
CAMERA_INDEX = 0
CAMERA_SIZE = (640, 480)

# ---------------- Geometría ----------------
ROBOT_RADIUS = 0.17          # m: radio del cuerpo   [MEDIR: diámetro/2 de tu chasis]
WHEEL_BASE = 0.214           # m entre ruedas (medido)
WHEEL_DIAMETER = 0.075       # m (medido)
LIDAR_OFFSET = (0.0, 0.0)    # (x adelante, y izquierda) del LIDAR respecto al centro
LIDAR_ANGLE_SIGN = -1        # LD06/LD19 miden en sentido horario -> -1 los pasa a antihorario
LIDAR_ANGLE_OFFSET_DEG = 0.0 # giro de montaje del LIDAR (0 = su "frente" mira al frente del robot)
LIDAR_MIN_RANGE = 0.05
LIDAR_MAP_RANGE = 3.5        # alcance usado para mapear

# ---------------- Odometría ----------------
USE_ENCODERS = False         # pasa a True DESPUÉS de calibrar TICKS_PER_M (check_hardware.py encoders)
ENC_TICKS_PER_REV = 508.8    # [MEDIR] pulsos por vuelta de RUEDA (valor provisional, NO verificado)
TICKS_PER_M = ENC_TICKS_PER_REV / (math.pi * WHEEL_DIAMETER)
SPEED_SCALE = 1.0            # sin encoders: v_real = v_cmd * SPEED_SCALE  [MEDIR]
SCAN_MATCH = True            # corrige deriva con el LIDAR (muy recomendado)
GYRO_YAW_SIGN = 1            # +1 si el yaw sube al girar antihorario (visto desde arriba)

# ---------------- Movimiento ----------------
V_CLEAN = 0.20               # m/s
W_MAX = 1.2                  # rad/s
FRONT_SLOW_DIST = 0.25       # m libres delante donde empieza a frenar
FRONT_STOP_DIST = 0.04       # m libres delante donde se detiene

# ---------------- Mapa / planeación ----------------
MAP_SIZE_M = 8.0             # el robot arranca en el centro del mapa
MAP_RES = 0.05               # m por celda
INFLATE_CELLS = 4            # >= ROBOT_RADIUS / MAP_RES (+margen): 4 x 5 cm = 20 cm
WALL_COST_RANGE = 11         # celdas: cerca de paredes el costo UCS sube
WALL_COST_GAIN = 2
COVER_RADIUS = 0.12          # m alrededor del robot que se marcan como "limpios"
EXPLORE_PLANNER = "BFS"      # "BFS" (celda sin limpiar más cercana) | "DFS" (exploración en profundidad)
DFS_ORDER = ("ARRIBA", "DERECHA", "ABAJO", "IZQUIERDA")
LANE_MIN_CELLS = 3           # celdas sin limpiar al frente para seguir recto
PLAN_PERIOD_S = 1.0          # cada cuánto se revalida/replanifica la ruta
WAYPOINT_TOL = 0.06          # m

# ---------------- Basura (cámara) ----------------
DUST_CONFIRM = 2             # avistamientos para confirmar un objetivo
DUST_MERGE_DIST = 0.15       # m para fusionar avistamientos
DUST_CLEAN_DIST = 0.10       # m: se considera aspirado si el robot pasa tan cerca
CAM_HEIGHT = 0.30            # m sobre el piso (~30 cm)  [MEDIR]
CAM_PITCH_DEG = 45.0         # inclinación hacia abajo [MEDIR; calibrate_camera.py lo ajusta]
CAM_X = 0.10                 # m adelante del centro del robot [MEDIR]
CAM_HFOV_DEG = 60.0          # campo de visión horizontal (ficha de la webcam; calibrate_camera.py)
DETECT_MAX_RANGE = 0.80      # m: más lejos el tamaño medido es poco fiable

# Clasificación por TAMAÑO REAL (sin entrenamiento). Ajusta a lo que tu aspirador puede tragar.
PICKUP_MAX_DIM = 0.035       # m: dimensión mayor máxima que se intenta aspirar
PICKUP_MIN_DIM = 0.006       # m: más pequeño se considera ruido
CABLE_MIN_LENGTH = 0.10      # m: largo y delgado => cable (evitar)
CABLE_MIN_WIDTH = 0.004      # m: más delgado que esto (un cabello) sí se aspira
CABLE_ASPECT = 5.0
HAIR_MAX_LEN = 0.15          # m: más delgado que CABLE_MIN_WIDTH y hasta este largo = cabello/hilo (se aspira)
VISION_MIN_DEVIATION = 12.0  # diferencia mínima de color (Lab) respecto al piso para considerar un objeto
MAX_OBJECT_DIM = 0.40        # m: más grande = sombra/alfombra/mueble (lo ve el LIDAR), se ignora
HAZARD_MARGIN = 0.02         # m extra alrededor de un objeto a evitar
HAZARD_CONFIRM = 2           # avistamientos para confirmar un objeto a evitar

# ---------------- Batería ----------------
# Pack Ni-MH 14.4 V (12 celdas). Su curva de descarga es MUY plana: el voltaje es un mal
# indicador de carga. Valores de partida: ajústalos con un registro real (run_robot.py --log).
# Deben coincidir con el firmware.
BATT_LOW_MV = 13000          # regresar a casa
BATT_CRIT_MV = 12000         # el ESP32 corta motores por su cuenta (1.0 V/celda)
BATT_FILTER = 0.15           # filtro exponencial por tick

# ---------------- Tiempos ----------------
TICK_HZ = 10.0
INIT_WAIT_S = 2.5            # espera inicial (calibración del giroscopio)
BACKUP_S = 0.8
BACKUP_V = -0.10
HOME_TOL = 0.20              # m
