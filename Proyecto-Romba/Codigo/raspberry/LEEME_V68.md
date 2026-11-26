# V6.8 — Historial visual experimental para recinto estático supervisado

Base: V6.7. Conserva geometría, planificadores, simulación y ESP32. No cambia firmware.

El corredor frontal se evalúa con celdas del suelo aparente observadas en la imagen actual o durante los 30 segundos anteriores. No se importa el mapa DEMO ni se marca la zona ciega libre por defecto. Se proyecta el centro y las cuatro esquinas interiores de cada celda; se descartan celdas fuera de imagen, ocluidas, con indicios de ocupación o desnivel registrado. La pérdida de pose válida borra el historial. Cada evaluación caduca a los 400 ms. El historial es experimental: ni la cámara ni la odometría garantizan ausencia de obstáculos. Mantener supervisión física, recinto plano y vacío de personas/mascotas, parada física disponible y alimentación correcta. No se permiten giros automáticos sin cobertura lateral.

Con motores desconectados, ejecutar:

    source .venv/bin/activate
    python run_robot.py --no-motors

Calibrar suelo despejado y esperar REPOSO_CONFIRMADO. Desde una posición inicial detrás de la zona que se pretende recorrer, desplazar manualmente el robot (motores desconectados), observando el suelo desde distintos puntos y manteniendo la cámara fija en su soporte. No reiniciar mapa entre estas observaciones. Consultar /api/status -> corridor_history. complete=true indica que el muestreo del corredor tiene evidencia visual reciente; unknown_cells debe ser cero. Una única imagen no basta para observar el suelo que queda fuera de la cámara. Si no se consigue cobertura o la odometría falla, no se autoriza avance. El arranque en modo supervisado reinicia el historial: hay que repetir el procedimiento con alimentación de motores físicamente cortada antes de habilitarla. No trasladar el robot sin que el seguimiento pueda estimar el movimiento.

    python run_robot.py --supervised-motors

Solo tras validar todo lo anterior, mantener pulsado Explorar automáticamente. El controlador usa 0.015 m/s solicitados si depende del historial para compensar cobertura parcial. Al soltar se mantiene la parada existente. Las distancias y velocidades reales no están garantizadas sin sensores adicionales. El historial no resuelve por sí mismo el giro físico, la odometría deficiente o una zona inicial nunca observada. Los cinco algoritmos siguen disponibles para planificación/simulación, no conducción física por rutas.

Pruebas realizadas: python -m unittest test_v68 test_v665 test_supervised_auto test_proximity test_visual_mapping test_phase_rest test_odometry_stationary (15 pruebas correctas); compileall correcto. No probados Flask ni hardware en este entorno.
