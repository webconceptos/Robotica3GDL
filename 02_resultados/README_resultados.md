# Resultados

Gráficas y tabla comparativa exportadas desde MATLAB/Simulink, organizadas
en orden secuencial de trabajo:

- `02.0_simulink_validacion/`: capturas de la generación del modelo y de
  los Scopes de los tres subsistemas (evidencia de ejecución en Simulink
  real).
- `02.1_graficas_preview/`: dinámica libre y configuración cinemática de
  prueba (validación previa del modelo dinámico).
- `02.2_graficas_seguimiento/`: q deseado vs q real y trayectoria
  cartesiana del efector final (v3/v4).
- `02.3_graficas_error/`: error articular por controlador (Simulink, v3,
  v4).
- `02.4_graficas_torque/`: torque por controlador y por articulación (v3,
  v4).
- `02.5_graficas_trayectoria_obstaculos/`: mapa A* con obstáculos y
  trayectoria cartesiana seguida (v4).
- `tabla_comparativa_error_articular.csv`: métricas comparativas de los
  tres controladores (error RMS/máximo, tiempo de estabilización, torque
  RMS/máximo).
