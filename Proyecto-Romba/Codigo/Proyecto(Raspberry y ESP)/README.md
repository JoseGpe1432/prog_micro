# Robot aspirador casero — Raspberry Pi 5 + ESP32-S3-N16R8

Software para tu Roomba hecho a mano. Reutiliza **BFS, DFS y UCS** de las prácticas como planificadores de ruta.

```
                    USB (115200, protocolo con checksum)
 ┌─────────────────────────┐ ◄────────────────────────────► ┌───────────────────────────┐
 │ RASPBERRY PI 5 (cerebro)│     $D,v,w*HH  ───────────►    │ ESP32-S3 (tiempo real)    │
 │  mapa LIDAR + corrección│     $T,...*HH  ◄───────────    │  2 × BTS7960 (PWM)        │
 │  BFS / DFS / UCS        │       telemetría 20 Hz         │  IMU HW-123 (yaw)         │
 │  visión por TAMAÑO REAL │                                │  4 IR de precipicio       │
 │  máquina de estados     │                                │  encoders · voltaje       │
 └──┬───────────────┬──────┘                                │  SEGURIDAD AUTÓNOMA       │
  LIDAR LD19     webcam USB                                 └───────────────────────────┘
```

**Regla de diseño:** lo crítico para la seguridad (precipicio, batería crítica, watchdog, límite de 12 V al motor)
vive en el **ESP32**, independiente de la Pi. Si la Pi se cuelga, el robot se detiene solo a los 400 ms.

## ⚠️ Qué está probado y qué NO

| Parte | Estado |
|---|---|
| Cerebro, mapa, BFS/DFS/UCS, batería, precipicio, objetos a rodear, basura | ✅ **57/57 pruebas** en simulación (`python3 test_all.py`) |
| Mapa de pines del ESP32-S3-N16R8 | ⚠️ elegido evitando los GPIO reservados según la documentación de Espressif; **verifícalo contra el pinout de TU placa** (hay clones con variaciones) |
| Protocolo, parser LD06/LD19, medición de tamaños, calibración de cámara | ✅ con datos sintéticos físicamente consistentes |
| `run_robot.py` completo sobre puertos serie (emuladores por pty) | ✅ en tiempo real (`test_pty_integration.py`) |
| Firmware ESP32 | ⚠️ **solo sintaxis C++ verificada** (cabeceras Arduino simuladas, 4 combinaciones de opciones). Nunca compilado con el core real ni probado en hardware |
| Visión sobre **tu piso real** | ⚠️ solo escenas sintéticas (sin brillos, sombras ni texturas reales). Hay que validarla |
| Todo valor marcado `[MEDIR]` | ⚠️ estimaciones: calíbralos |

Un simulador no sustituye al hardware: espera ajustar cosas en las primeras pruebas.

## Tu hardware y cómo se usa

| Componente | Configuración / nota |
|---|---|
| **LIDAR** LDROBOT **LD19** (o LD06) | UART 230400 vía adaptador USB-UART (CP2102). Montar arriba, al centro, sin obstrucciones |
| **2 × BTS7960** | Cada uno usa **RPWM** (avance), **LPWM** (retroceso) y **R_EN+L_EN unidos**. Nunca se activan RPWM y LPWM a la vez |
| **Motores 12 V con pack 14.4 V** | El pack cargado llega a ~16 V. El firmware **limita el voltaje efectivo a 12 V** (`MOTOR_NOM_MV`): PWM = fracción × 12 V / V_batería |
| **Pack Ni-MH 14.4 V 3000 mAh** | Divisor **100 kΩ / 15 kΩ** + 100 nF al ADC (cambió respecto a lo que te dije antes: 17.5 V → 2.3 V, zona lineal del ADC y margen) |
| **IMU "HW-123"** | El número de módulo no identifica el chip. El firmware autodetecta la familia MPU (6050/6500/9250). Verifica con `check_hardware.py imu` |
| **Encoders** | Rueda Ø 75 mm, separación 214 mm. Faltan: pulsos por vuelta y si son de 1 o 2 canales → `check_hardware.py encoders` |
| **Raspberry Pi 5** | Alimentar con un buck 5 V de ≥ 5 A desde el pack, con fusible en el pack |
| **Webcam a ~30 cm** | Detecta **por tamaño real**, sin entrenar (ver abajo) |
| **Sin base de carga** | Al terminar o con batería baja, vuelve al punto de arranque y se detiene; **se carga a mano** |

## Conexiones

👉 **Todo el detalle (esquemas, pines, qué resistencia va dónde y lista de componentes) está en [`CONEXIONES.md`](CONEXIONES.md).**

Resumen para tu **ESP32-S3-N16R8** (en ese módulo los GPIO 26–37 están ocupados por la flash/PSRAM, 19–20 son USB, 43–44 UART0 y 0/3/45/46 son de arranque):

| Función | GPIO |
|---|---|
| BTS7960 izq: RPWM / LPWM / EN | 38 / 39 / 40 |
| BTS7960 der: RPWM / LPWM / EN | 41 / 42 / 47 |
| IMU SDA / SCL | 8 / 9 |
| Encoders izq A,B — der A,B | 10,11 — 12,13 |
| Batería (ADC) | 1 |
| IR de precipicio ×4 (ADC) | 2, 4, 5, 6 |
| Emisores IR (opcional, con transistor) | 7 |

Pi ↔ ESP32: **solo el cable USB** al puerto USB-C rotulado "UART" (en Arduino IDE: *USB CDC On Boot: Disabled*; aparece como `/dev/ttyUSB0`).
En el ESP32 clásico usa `#define ROOMBA_CLASSIC_ESP32`. El ESP32 **no debe imprimir depuración** por `Serial` salvo líneas que no empiecen con `$`.

## ⚠️ IR de precipicio del Roomba: verifica ANTES de conectar

Me pasaste una descripción del arnés. **No la puedo verificar** y contiene riesgos para el ESP32, así que trátala como hipótesis:

1. **Pull-up a 3.3 V, NUNCA a 5 V.** Los pines ADC del ESP32 soportan 3.3 V como máximo; un pull-up a 5 V puede dañarlos.
2. **Emisores:** la descripción dice que son 4 LED en serie "a 5 V con 220 Ω". Compruébalo: con el multímetro en *prueba de diodos*
   mide la caída total de la cadena (un LED infrarrojo ≈ 1.1–1.4 V, así que 4 en serie ≈ 4.5–5.5 V).
   Si la suma ya es ≈ 5 V, **con 5 V casi no circula corriente**. Calcula la resistencia con `R = (V_fuente − ΣVf) / 15 mA`
   y elige una fuente mayor que `ΣVf` (p. ej. 9 V) o alimenta cada LED por separado. Mídelo y dime los valores.
3. **Receptores:** fototransistores con GND común y colector individual. Cada colector → nodo con **10 kΩ a 3.3 V** → pin ADC.
4. **Luz ambiente:** el sol/lámparas saturan el sensor. Mejora opcional: conmuta los emisores con un transistor desde `CLIFF_EMIT_PIN`
   (ESP32 → 1 kΩ → base de un 2N2222, o MOSFET lógico) y el firmware resta la lectura con el emisor apagado.
5. Calibra: pon `DEBUG_CLIFF_RAW 1`, observa `$R,...` (llega a `link.raw_cliff`), y ajusta `CLIFF_DELTA_RATIO` / `CLIFF_MIN_DELTA`.
   El firmware calibra el piso al encender: **enciende siempre con el robot sobre el piso**.

## Visión por tamaño real (sin entrenar con imágenes)

La cámara está a altura y ángulo conocidos, así que cada píxel del piso es una posición física. Para cada objeto distinto del piso:
se proyecta su contorno al plano del piso → se miden **largo y ancho en metros** → se decide por tamaño y forma:

| Tamaño medido (config.py) | Clase | Acción del robot |
|---|---|---|
| 6 mm – 3.5 cm (`PICKUP_MIN/MAX_DIM`) | **DEBRIS** | Va a aspirarlo (UCS) |
| < 4 mm de ancho y ≤ 15 cm de largo | DEBRIS (cabello/hilo) | Va a aspirarlo |
| > 3.5 cm | **TOO_BIG** | Lo marca como obstáculo y lo rodea |
| largo ≥ 10 cm, ancho ≥ 4 mm, aspecto ≥ 5 | **CABLE** | Lo rodea (enreda cepillos) |
| < 6 mm o > 40 cm | IGNORE | Ruido, sombra, alfombra, muebles (el LIDAR ve los muebles) |

En simulación, con 3 objetos de 5–7 cm en el piso: **sin** esta visión el robot pisa los 3; **con** ella, 0 de 3 (cobertura 94 % → 88 %).

**Calibración (imprescindible):** mide `CAM_HEIGHT` y `CAM_X` con regla; pon marcas a distancias conocidas y ejecuta
`calibrate_camera.py` (ver su encabezado) para obtener `CAM_PITCH_DEG` y `CAM_HFOV_DEG`.
Sensibilidad medida en simulación: **10 % de error en la altura ⇒ ±10–20 % en el tamaño y 3–6 cm en la posición**;
3° de error en la inclinación ⇒ ±10–30 % en el tamaño.

**Límites físicos (no se evitan con una sola cámara):** se asume el objeto *apoyado en el piso* (uno alto parece más grande y lejano);
lejos de ~0.8 m la resolución cae; hace falta contraste con el piso; los pisos con mucho dibujo darán falsos positivos
(por eso se exigen 2 avistamientos). Bloquea la exposición/balance automático de la webcam si puedes.

## Instalación

**Raspberry Pi 5** (Raspberry Pi OS):
```bash
sudo apt install python3-pip python3-opencv
pip3 install --break-system-packages numpy pyserial   # opencv ya viene con python3-opencv
sudo usermod -aG dialout $USER                        # reinicia sesión
ls /dev/serial/by-id/                                 # usa estas rutas en pi/config.py
```
**ESP32:** Arduino IDE → core "esp32 by Espressif" (2.x o 3.x) → abre `esp32_firmware/roomba_esp32/roomba_esp32.ino`
→ placa **"ESP32S3 Dev Module"**, Flash 16 MB, PSRAM "OPI PSRAM", *USB CDC On Boot: Disabled* → sube con el puerto "UART". **Al encender, el robot debe estar quieto y sobre el piso** (calibra giroscopio y IR).

## Puesta en marcha (en este orden; con las ruedas en el aire hasta el paso 5)

1. `check_hardware.py imu` — identifica el chip de la IMU. Girar a mano **antihorario** debe SUBIR el yaw (si no: `GYRO_YAW_SIGN`).
   Si el WHO_AM_I sale "DESCONOCIDO", dime el número grabado en el chip.
2. `check_hardware.py esp32` — compara el voltaje con un multímetro y ajusta `BATT_CAL`. Tapa/destapa cada IR y mira `cliff`.
3. `check_hardware.py motors` — comprueba sentidos (`MOTOR_*_SIGN`). Mide velocidad real de rueda a 12 V efectivos → `V_MAX_MM_S`;
   y el mínimo que la mueve → `MIN_DUTY`. Comprueba que los BTS7960 no se calientan.
4. `check_hardware.py encoders` — empuja el robot 1.00 m. Corrige signos (`ENC_*_SIGN`) y copia `ENC_TICKS_PER_REV` al firmware y a `config.py`;
   luego `USE_ENCODERS = True`. Si quieres control de velocidad en lazo cerrado, activa `ENC_CLOSED_LOOP 1` **después** de esto.
5. `check_hardware.py lidar` — el sector "frente" baja al poner la mano delante. Si el mapa sale espejado: `LIDAR_ANGLE_SIGN`; girado: `LIDAR_ANGLE_OFFSET_DEG`.
6. Cámara: `calibrate_camera.py` y luego `check_hardware.py camera` (compara con una regla los tamaños que reporta).
7. `run_robot.py --no-motors` — empuja el robot a mano: mapa y cerebro funcionan sin que se mueva solo.
8. Primera limpieza **supervisada**, cuarto despejado, sin escaleras, registrando la batería: `run_robot.py --log bateria.csv`.

## Batería Ni-MH: umbrales

Predeterminados: aviso **13.0 V** (regresa a casa), corte **12.0 V** (1.0 V/celda). **Son un punto de partida:** la curva de descarga del Ni-MH es
muy plana, así que el voltaje es un mal indicador de carga y además cae bajo carga. Haz una descarga real con `--log bateria.csv`,
revisa la curva (voltaje vs. tiempo) y ajusta `BATT_LOW_MV` en **ambos** archivos (`.ino` y `config.py`). No descargues por debajo de ~1.0 V/celda.
Como no hay base de carga, cuando termina queda detenido en su punto de partida.

## Cómo se usan los tres algoritmos

| Situación | Algoritmo | Por qué |
|---|---|---|
| Siguiente zona sin limpiar | **BFS** (multi-meta) | La más cercana en nº de pasos. Un "carril recto" evita zigzag |
| Ir a la basura detectada | **UCS** | Menor costo; el costo sube cerca de paredes → rutas holgadas |
| Volver a casa | **UCS** | Ruta de menor costo, alejada de obstáculos |
| Exploración alternativa | **DFS** (`EXPLORE_PLANNER="DFS"`) | Seguro, pero **mucho más lento** (53 % en 900 s vs 94 % en 300 s con BFS) |

## Seguridad implementada

1. **Batería crítica** (ESP32): motores cortados y etapa de potencia deshabilitada tras 2 s bajo `BATT_CRIT_MV`.
2. **Precipicio** (ESP32): con un IR activo solo se permite retroceder; la Pi marca un obstáculo permanente.
3. **Obstáculo cercano** (Pi): frena con el cono frontal del LIDAR sin replanificar.
4. **Batería baja** (Pi): vuelve a casa por UCS.
5. **Watchdog** (ESP32) y **sensores muertos** (Pi): paro automático.
6. **Voltaje al motor ≤ 12 V** (ESP32), aunque el pack esté a 16 V.

## Límites conocidos

* **Deriva:** la posición se corrige con LIDAR (`scan_match.py`), pero **no hay cierre de bucles**; recorridos largos o cuartos muy simétricos acumulan error.
* **Acoplamiento a cargador:** no existe; vuelve al punto de arranque.
* **Visión:** ver "Límites físicos". No hay modelo de aprendizaje; si una detección falla en tu piso, guarda la foto y la revisamos.
* No controla bumpers ni IR de pared. Los cepillos/succión (siempre encendidos) van fuera de este sistema: **alimentación y fusible propios**.
* Aún no usa la corriente de los BTS7960 (pines IS) para detectar atascos; sería una buena mejora.

## Información que todavía necesito

1. **Encoders:** ¿pulsos por vuelta de rueda, 1 o 2 canales, voltaje de alimentación? (o simplemente corre `check_hardware.py encoders`).
2. **IMU:** resultado de `check_hardware.py imu` (WHO_AM_I) o el número del chip.
3. **IR del Roomba:** caída total de la cadena de emisores en prueba de diodos y cuántos hilos tiene el conector.
4. **Chasis:** diámetro real del cuerpo (para `ROBOT_RADIUS`) y altura total con el LIDAR (¿pasa bajo muebles?).
5. **Cámara:** ángulo de inclinación aproximado y modelo/FOV.
6. **BTS7960:** ¿qué corriente de bloqueo tienen tus motores? (para fusible y futura detección de atascos).

## Archivos

```
esp32_firmware/roomba_esp32/roomba_esp32.ino   firmware
pi/run_robot.py   robot real        pi/check_hardware.py   puesta en marcha (imu/esp32/encoders/motors/lidar/camera)
pi/brain.py       máquina de estados   pi/config.py          TODOS los parámetros
pi/vision.py      detección por tamaño real   pi/calibrate_camera.py   calibración con cinta métrica
pi/grid_map.py    mapa + adaptador de planificación   pi/scan_match.py  corrección de deriva
pi/bfs.py dfs.py ucs.py   planificadores (de las prácticas)
pi/lidar_ld06.py esp32_link.py pose.py navigator.py detection.py
CONEXIONES.md   esquemas de cableado, resistencias y verificaciones previas
pi/sim.py synth_camera.py test_all.py test_pty_integration.py   simulador y pruebas
```
