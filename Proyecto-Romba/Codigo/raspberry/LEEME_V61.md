# Roomba V6.1 — Compensación de movimiento (diagnóstico)

Parche incremental **sobre V6**, no sustituye el proyecto completo. No cambia ESP32 ni firmware, ni activa motores, ni modifica el mapa ni BFS/DFS/UCS/Greedy/A*.

1. Detener `run_robot.py` y respaldar `~/robot_aspirador_organizado/raspberry`.
2. Extraer los `.py` del ZIP dentro de esa carpeta, sobrescribiendo `run_robot.py` y `dashboard.py`.
3. `source .venv/bin/activate`
4. `python -m unittest test_visual_planner test_camera_motion test_motion_residual -v`
5. `python run_robot.py --no-motors`
6. Panel `http://127.0.0.1:5000` → V6.1 y `/api/status` → `motion_residual`.

## Pruebas
- Cámara quieta: `residual_ratio` bajo.
- Girar manualmente lentamente a derecha e izquierda: comparar `global_shift_px`, `inlier_ratio`, `residual_ratio` y `zones`.
- Mover un objeto sin mover cámara: observar residuo, sin interpretarlo como distancia.
- Mover cámara cerca de objetos: el paralaje puede producir residuo fuerte aun sin obstáculos nuevos.

**Limitaciones**: modelo afín 2D aproximado. La perspectiva, el paralaje, la iluminación y el desenfoque generan falsos residuos. Un residuo bajo NO certifica camino libre. `movement_authorized`, `obstacle_confirmed` y `free_space_verified` permanecen `false`. No habilitar navegación autónoma. Mantener `--no-motors` y entorno plano sin desniveles.
