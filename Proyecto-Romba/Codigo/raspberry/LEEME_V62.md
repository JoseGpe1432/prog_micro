# Roomba V6.2 — seguimiento temporal de regiones residuales

Parche incremental **sobre V6.1**. No es un proyecto completo: conserva el resto de los archivos existentes de tu carpeta `raspberry`.

## Instalación
1. Detén `run_robot.py`.
2. Haz una copia de seguridad de `~/robot_aspirador_organizado/raspberry`.
3. Descomprime los archivos `.py` de este ZIP dentro de `~/robot_aspirador_organizado/raspberry`, sobrescribiendo `run_robot.py` y `dashboard.py`.
4. Ejecuta:

```bash
cd ~/robot_aspirador_organizado/raspberry
source .venv/bin/activate
python -m unittest test_visual_planner test_camera_motion test_motion_residual test_region_tracker -v
python run_robot.py --no-motors
```

5. Abre `http://127.0.0.1:5000` y busca `V6.2 · Seguimiento temporal de regiones`.
6. Consulta JSON:

```bash
curl -s http://127.0.0.1:5000/api/status | python -c "import json,sys; d=json.load(sys.stdin); print(json.dumps({'motion_residual':d.get('motion_residual'),'region_tracking':d.get('region_tracking')},indent=2,ensure_ascii=False))"
```

## Qué hace
- Usa las cajas `candidate_boxes` producidas por V6.1 después de la alineación de fotogramas.
- Asocia cajas consecutivas con desplazamiento global estimado, solapamiento y distancia entre centros.
- Asigna IDs, registra observaciones consecutivas y marca `CANDIDATO_PERSISTENTE` desde la tercera coincidencia; tolera hasta dos observaciones perdidas antes de retirar un ID.
- Si la alineación falla, reinicia el seguimiento. Un hueco de más de 1.5 s también reinicia los IDs.
- Muestra `ID ... N` (nuevo) o `ID ... P` (persistente) sobre `/video/residual`.

## Pruebas sugeridas (sin motores)
1. Cámara inmóvil y suelo despejado: esperar pocas regiones, posiblemente ninguna.
2. Desplazar lentamente la Roomba a mano: observar `global_shift_px`, IDs y su estabilidad.
3. Colocar una mochila delante y mover la Roomba suavemente: comprobar si alguna región se mantiene 3 observaciones o más.
4. Retirar mochila y repetir. La ausencia de residuo no demuestra espacio libre.

**Límites:** un patrón de baldosas, una sombra o el paralaje también pueden producir regiones persistentes. El seguimiento no reconoce objetos, no mide distancia y no actualiza el mapa. `movement_authorized`, `obstacle_confirmed`, `free_space_verified` y `map_updated` son siempre `false`. No se han añadido órdenes autónomas a motores ni al firmware ESP32. Evita escaleras, bordes y personas; mantén `--no-motors`.
