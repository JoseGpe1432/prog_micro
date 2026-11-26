# Raspberry Pi 5: HC-SR04 + menú de navegación + cámara

Se modificó el proyecto original y se conservan sus módulos de visión, mapas y algoritmos.

## Instalación

```bash
sudo apt update
sudo apt install -y python3-venv python3-opencv python3-numpy
cd ~/robot_aspirador
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
pip install pyserial flask
ls /dev/serial/by-id/ /dev/ttyACM* /dev/ttyUSB* 2>/dev/null
```

En `config.py` ajusta `ESP32_PORT` al puerto real. Conecta la ESP32-S3 por USB a la Pi.
La ESP32 debe tener cargado **roomba_esp32_hcsr04.ino**, no el firmware de prueba de ruedas.
El ESP32 debe emitir `$T` y `$U` con XOR. No se requiere LIDAR.

## Uso

```bash
python3 run_robot.py --no-motors
```

Abre Chromium de la Raspberry Pi en `http://127.0.0.1:5000`.
Comprueba cámara, telemetría, distancia y mapa. Solo después de calibrar, prueba:

```bash
python3 run_robot.py
```

Inicia siempre PAUSADO. Elige BFS, DFS, UCS, Greedy o A* y pulsa Iniciar.
Pausar envía STOP y permite volver a iniciar con misión nueva.
DETENER enclava la parada hasta reiniciar el proceso.
Cierra el programa con Ctrl+C y corta alimentación de motores al terminar.
Para abrir desde otro dispositivo en una red privada: `python3 run_robot.py --host 0.0.0.0` y visita `http://IP_DE_LA_PI:5000`. **El panel no tiene contraseña: no lo expongas a Internet.**

## Seguridad y límites

- El HC-SR04 debe tener divisor de tensión en ECHO (5 V a ~3.3 V). TRIG GPIO16, ECHO GPIO17, GND común.
- El robot necesita barreras físicas en el perímetro; el HC-SR04 NO detecta líneas de cinta ni precipicios.
- El mapa de 3 x 3 m asume **arranque cerca del centro del escenario** y orientación arbitraria. No hay localización global ni SLAM robusto; el mapa deriva.
- El sensor ultrasónico tiene un cono ancho, no un rayo láser; el mapa será aproximado. En caso de lectura inválida, bloqueo frontal o telemetría perdida, la Pi se pausa.
- La protección de batería depende de un divisor real y calibrado conectado a la ESP32. Verifica antes de habilitar motores.
- `USE_ENCODERS` está en False hasta calibrar pulsos/revolución; con False la posición se estima por velocidad comandada y acumula error. Después de calibrar, cambiar a True.
- La detección por cámara requiere calibración con `calibrate_camera.py` y luz estable.
- El selector afecta la búsqueda de celdas conocidas no cubiertas. La exploración de áreas desconocidas se intenta con giros de reconocimiento; NO garantiza cobertura total ni evitar colisiones laterales.
- El botón de parada es **software** y depende de la conexión; instala un interruptor físico de corte de alimentación para emergencias.
- La Pi y el ESP32 no son sistemas de seguridad certificados. Prueba con ruedas levantadas y supervisión.
