# Visión V3 — diagnóstico con referencia estática

Esta versión fue elaborada desde raspberry(1).7z. No modifica firmware ESP32.

## Instalación
Detén run_robot.py. Haz copia de seguridad de los archivos originales.
Copia run_robot.py, dashboard.py, vision.py, vision_filter.py y floor_reference.py
al directorio raspberry. Mantén los demás archivos existentes.

Ejecuta `python run_robot.py --no-motors` desde tu entorno virtual.

## Uso
1. Fija la cámara; apunta al suelo despejado y presiona Capturar referencia.
2. Coloca la mochila SIN mover la cámara ni cambiar la iluminación.
3. Observa /video/diagnostic y floor_reference.changed_ratio en /api/status.
4. Retira la mochila; el porcentaje debe disminuir.

El rojo señala cambios de imagen, NO confirma obstáculos físicos ni espacio
libre. Si la cámara o el robot se mueve, borra y captura nueva referencia.
La referencia se pierde al reiniciar. No se usa para autorizar movimiento.
La navegación autónoma continúa deshabilitada; los controles supervisados
mantienen sus protecciones y su filtro visual original.
