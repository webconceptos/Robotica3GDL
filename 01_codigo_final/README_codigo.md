# Código final

Todos los scripts siguen la convención `robot3dof_TFinal_vN_<etapa>.m`, en
orden de ejecución:

- `robot3dof_TFinal_v1.m`: versión integrada preliminar (dinámica ad-hoc, ya
  superada por el método de Jacobianos; se conserva como evidencia de la
  primera iteración, no como referencia de dinámica).
- `robot3dof_TFinal_v2_dinamica_jacobianos.m`: **versión vigente de la
  dinámica**. M(q), C(q,qdot), G(q) obtenidos por Jacobianos lineales/
  angulares de los centros de masa + coeficientes de Christoffel (n=3), sin
  Lagrange, según instrucción explícita del docente. Incluye supuestos
  físicos documentados, prueba numérica, y una verificación simbólica
  opcional (requiere Symbolic Math Toolbox).
- `robot3dof_TFinal_v2_simulink_generador.m`: prepara el workspace
  (parámetros, trayectoria, ganancias) y genera los archivos de bloques
  `MATLAB Function` en `simulink_blocks/`; construye
  `Robot3GDL_Control_Final.slx` automáticamente y validado en Simulink real
  (los tres subsistemas compilan y simulan). Guía de armado manual de
  respaldo en `05_anexos/guia_armado_simulink_robot3gdl.md`.
- `robot3dof_TFinal_v3_comparar_controladores.m`: calcula el error
  articular y la tabla comparativa (error RMS/máximo, tiempo de
  estabilización, torque) a partir de las señales exportadas por Simulink.
- `robot3dof_TFinal_v3_controladores.m`: respaldo/comparación en MATLAB
  puro (sin Simulink) de los tres controladores, con la dinámica de
  Jacobianos, la misma trayectoria y las mismas ganancias que
  `robot3dof_TFinal_v2_simulink_generador.m`. Integra con `ode45` (un
  integrador Euler de paso fijo diverge con esta dinámica para PID/PD, ver
  comentarios en el archivo); reproduce los resultados de Simulink con <1%
  de diferencia.
- `robot3dof_TFinal_v4_astar_obstaculos.m`: versión final integrada.
  Agrega planeación A* con obstáculos en el plano XZ, conversión a
  trayectoria articular por cinemática inversa, y simula los tres
  controladores (dinámica de Jacobianos, `ode45`) siguiendo la ruta
  planeada.
- `simulink_blocks/`: código plano y autocontenido para copiar dentro de
  cada bloque `MATLAB Function` (planta + 3 controladores). Se genera al
  correr `robot3dof_TFinal_v2_simulink_generador.m`.
