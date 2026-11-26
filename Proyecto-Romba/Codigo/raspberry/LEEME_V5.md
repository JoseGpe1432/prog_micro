# Roomba V5 — Evaluación visual integrada con cinco planificadores

Esta actualización se aplica a la carpeta `raspberry` de la versión V4. No cambia el firmware ESP32.

## Archivos modificados
- `floor_reference.py`: ahora la decisión considera cambios laterales incluso cuando el centro no cambia.
- `visual_planner.py`: nuevo adaptador que produce estados por zona y orden de inspección para BFS, DFS, UCS, GREEDY y ASTAR.
- `run_robot.py`: publica `visual_planning` en `/api/status` junto a la ruta existente del planificador.
- `dashboard.py`: muestra la evaluación y el orden de inspección.
- `test_visual_planner.py`: pruebas de la lógica de selección.

## Instalación
1. Detener `run_robot.py` y hacer copia de seguridad de la carpeta `raspberry`.
2. Extraer este ZIP dentro de `~/robot_aspirador_organizado/raspberry` y sobrescribir los archivos indicados.
3. `source .venv/bin/activate`
4. `python -m unittest test_visual_planner -v`
5. `python run_robot.py --no-motors`
6. Abrir `http://127.0.0.1:5000`, capturar la referencia con la cámara fija y probar mochila izquierda/derecha.
7. Consultar `curl -s http://127.0.0.1:5000/api/status | python -c "import json,sys; print(json.dumps(json.load(sys.stdin)['visual_planning'],indent=2,ensure_ascii=False))"`

## Importante
- **No mueve motores ni modifica el mapa de ocupación.** La selección es una *prioridad de inspección*, no una ruta segura.
- Los cinco algoritmos de rutas siguen funcionando sobre el mapa real o DEMO como antes. La integración NO modifica sus costes, aristas ni heurísticas.
- Sin cámara fija, la comparación con referencia deja de ser válida. La inmovilidad de cámara NO se verifica automáticamente.
- `SIN_CAMBIO_CONFIRMADO` **no significa libre**. Los candidatos de contraste siguen siendo experimentales.
- En mapa real, sin datos fiables de distancia, las celdas permanecen desconocidas y no se inventan rutas.
- Para autonomía móvil se necesitan detección robusta de obstáculos, desniveles y localización calibrada, además de corte físico de motores.
