# V6.5: mapa visual experimental
Conserva firmware y módulos V6.4. No activa autonomía. Desconecta alimentación de motores.
1. Copia los archivos del ZIP sobre raspberry (conserva .venv). Haz copia de seguridad antes.
2. source .venv/bin/activate; python run_robot.py --no-motors --host 0.0.0.0
3. Calibra segmentación con suelo despejado. Esto reinicia mapa y odometría.
4. Mueve el robot a mano lentamente, con IMU conectada y suelo con textura. El mapa se actualiza con seguimiento visual de calidad SEGUIMIENTO y telemetría reciente sin fallo IMU. Si se pierde orientación, reinicia y repite.
5. Selecciona algoritmo y meta clara. Ruta experimental, no ejecutar físicamente. Guarda JSON desde panel.
Mapa 3x3 m, resolución 5 cm, origen central. La proyección depende de CAM_HEIGHT, CAM_PITCH_DEG, CAM_X, CAM_HFOV_DEG. Rojo: candidato persistente proyectado por borde inferior; claro: apariencia de suelo, no espacio libre verificado. No hay cierre de bucle ni relocalización; errores acumulativos. Obstáculos registrados no se borran automáticamente. Reiniciar mapa para cambiar escenario. DEMO es sintético. No hay garantía de detección de obstáculos, basura ni desniveles.
