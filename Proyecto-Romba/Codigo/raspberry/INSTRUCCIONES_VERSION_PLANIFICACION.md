# Roomba: navegación y detección — versión de pruebas seguras

## Qué funciona
- Cámara MJPEG y odometría visual experimental.
- Visualización de detecciones candidatas por tipo (DEBRIS, TOO_BIG, CABLE); NO son identificaciones semánticas confiables.
- Mapa de ocupación actualizado con mediciones HC-SR04 válidas y pose visual estimada; NO es un mapa validado para conducción autónoma.
- Selección en vivo de BFS, DFS, UCS, GREEDY y ASTAR. Cada selección recalcula la ruta.
- Clic en mapa para fijar una meta; el planificador utiliza únicamente celdas conocidas libres.
- Modo DEMO con mapa sintético para verificar algoritmos aunque el ultrasonido no entregue medidas válidas.
- Iniciar, Pausar y Detener la vista previa desde la web.

## Seguridad
**No mueve las ruedas.** El firmware puede manejar motores, pero esta versión de `run_robot.py` fuerza `no_motors=True` y nunca ejecuta comandos de avance en PREVIEW. No retirar esta protección sin resolver primero la detección de precipicios, batería real, fallos del HC-SR04 y localización de escala fiable. La cámara monocular no sustituye sensores anticaída ni detección de obstáculos fiable.

## Instalación
1. Respaldar la carpeta original.
2. Extraer el ZIP sobre una carpeta nueva, por ejemplo `~/roomba_preview/raspberry`.
3. `cd ~/roomba_preview/raspberry`
4. Activar un entorno virtual existente con las dependencias de `requirements.txt`, o crear uno e instalar dependencias.
5. Ejecutar `python run_robot.py --no-motors`.
6. Abrir `http://127.0.0.1:5000` en la Raspberry.

## Prueba de algoritmos
1. Marcar `Escenario DEMO (sintético)`.
2. Hacer clic en una celda libre del mapa.
3. Elegir BFS y pulsar `Iniciar planificación`.
4. Cambiar a DFS, UCS, GREEDY y ASTAR y observar el trazado.
5. Desmarcar DEMO para usar el mapa observado real. Si no hay celdas libres conectadas, mostrará `sin ruta`, sin inventar un recorrido.

## Notas técnicas
- `visual_odometry.x/y` todavía requiere calibración de escala y compensación precisa de rotación.
- `navigation` original se conserva pero no se ejecuta para mover motores.
- El valor `batt_mv=12000` en el firmware anterior es nominal, no una medición.
- La Raspberry Pi ha mostrado advertencia de bajo voltaje: corregir alimentación antes de pruebas adicionales.
