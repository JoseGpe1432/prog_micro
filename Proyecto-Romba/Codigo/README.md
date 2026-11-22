# Robot aspirador — archivos organizados

Esta entrega separa el software de **Raspberry Pi 5** del firmware de **ESP32-S3**. Se quitaron de esta entrega los duplicados y las prácticas anteriores de agente/entorno/BFS; **los originales no se modificaron**.

## Raspberry Pi

Copiar el contenido de `raspberry/` a `~/robot_aspirador/` (manteniendo los archivos juntos: los imports originales son planos).

- `run_robot.py`: punto de entrada del robot físico.
- `config.py`: puertos y parámetros **pendientes de calibrar**.
- `brain.py`, `navigator.py`, `pose.py`, `grid_map.py`, `scan_match.py`: misión, navegación y mapa.
- `bfs.py`, `dfs.py`, `ucs.py`, `search_status.py`: algoritmos y estados originales del proyecto físico.
- `vision.py`, `detection.py`, `calibrate_camera.py`: detección geométrica y calibración.
- `esp32_link.py`, `lidar_ld06.py`: comunicación y LIDAR.
- `check_hardware.py`: diagnóstico.
- `test_all.py`, `test_pty_integration.py`, `sim.py`, `synth_camera.py`: pruebas y simuladores, **no se requieren para iniciar el robot**, pero se conservan para reproducibilidad.

### Instalación

```bash
cd ~/robot_aspirador
sudo apt update
sudo apt install -y python3-venv python3-opencv python3-numpy
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
pip install pyserial
python3 test_all.py
python3 check_hardware.py esp32
```

Ajusta `ESP32_PORT` y `LIDAR_PORT` en `config.py` según `/dev/serial/by-id/` o `/dev/ttyUSB*`. Con motores sin alimentar, prueba primero la telemetría. Para ejecutar sin movimiento: `python3 run_robot.py --no-motors`. Esta ejecución aún requiere telemetría y LIDAR.

## ESP32

Abrir `esp32/roomba_esp32/roomba_esp32.ino` con Arduino IDE y cargarlo a la placa. El firmware aún no se ha validado en hardware real.

## Limitaciones importantes

- El proyecto original contiene BFS, DFS y UCS, **pero aún no incluye Greedy ni A***; no cumple toda la rúbrica de IA.
- La visión es por tamaño, sin YOLO; confirmar con el docente la contradicción entre la opción de un detector y la rúbrica que evalúa ambos.
- `config.py` conserva el mapa original de 8 m, no se cambió silenciosamente a 3 m; hay que adaptar origen, límites y pruebas.
- El control de aspiración todavía no está implementado en el firmware.
- No conectar los seis cables de cada rueda sin identificar físicamente motor, encoder y sensor de rueda levantada.
- La alimentación de Raspberry Pi y ESP32 será por powerbank; motores por batería separada, con tierra común y protecciones.
- Los archivos de pruebas permanecen en la misma carpeta por compatibilidad con imports; esto no es duplicación.

Documentación original en `documentacion/`.
