# Roomba V6 — Fase 3: diagnóstico de movimiento de cámara

Parche incremental para instalar **sobre V5**. No modifica ESP32, motores ni mapas.

## Instalación
1. Detén `run_robot.py`, haz respaldo de `~/robot_aspirador_organizado/raspberry`.
2. Extrae todos los archivos de este ZIP dentro de esa carpeta, sobrescribiendo los existentes.
3. `source .venv/bin/activate`
4. `python -m unittest test_visual_planner test_camera_motion -v`
5. `python run_robot.py --no-motors`
6. Abre `http://127.0.0.1:5000` y consulta `camera_motion` en el panel o en `/api/status`.

## Prueba
- Apoya la cámara sin moverla durante varios segundos: `APARENTEMENTE_ESTABLE` si hay textura suficiente.
- Mueve la cámara lentamente con la mano: `MOVIMIENTO_DETECTADO` y `reference_invalidated: true`.
- Detén la cámara: puede regresar a `APARENTEMENTE_ESTABLE`, **pero** la referencia seguirá invalidada hasta recapturarla con la cámara fija.
- Con mochila, mano o personas moviéndose delante de cámara fija puede detectarse movimiento de escena. No es una medición de movimiento propio: interpretar como **hipótesis**.
- Si aparece `SEGUIMIENTO_INSUFICIENTE`, mejora iluminación/textura. No infiere camino libre.

## Límites
- El flujo óptico estima desplazamientos en **píxeles**, no metros ni distancias a obstáculos.
- No separa con certeza objetos móviles de movimiento de cámara, no mide profundidad, no detecta desniveles.
- No permite conducción autónoma ni modifica mapa o algoritmos de búsqueda.
- La referencia V4/V5 es **estática** y se suspende cuando se detecta movimiento. En la primera captura y durante seguimiento insuficiente, la planificación se mantiene en DESCONOCIDO.
- Para autonomía real se requiere percepción de profundidad/obstáculos y detección de desniveles con hardware adecuado.
