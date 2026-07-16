# Código final

- `robot3dof_TFinal_v1.m`: versión integrada preliminar (dinámica ad-hoc, ya
  superada por el método de Jacobianos; se conserva como evidencia de la
  primera iteración, no como referencia de dinámica).
- `robot3dof_TFinal_v2_dinamica_jacobianos.m`: **versión vigente de la
  dinámica**. M(q), C(q,qdot), G(q) obtenidos por Jacobianos lineales/
  angulares de los centros de masa + coeficientes de Christoffel (n=3), sin
  Lagrange, según instrucción explícita del docente. Incluye supuestos
  físicos documentados, prueba numérica, y una verificación simbólica
  opcional (requiere Symbolic Math Toolbox).
- `crear_modelo_simulink_robot3gdl.m`: prepara el workspace (parámetros,
  trayectoria, ganancias) y genera los archivos de bloques `MATLAB Function`
  en `simulink_blocks/`; intenta construir `Robot3GDL_Control_Final.slx`
  automáticamente (mejor esfuerzo, pendiente de verificar corriéndolo en
  Simulink). Guía de armado manual garantizada en
  `05_anexos/guia_armado_simulink_robot3gdl.md`.
- `simulink_blocks/`: código plano y autocontenido para copiar dentro de
  cada bloque `MATLAB Function` (planta + 3 controladores). Se genera al
  correr `crear_modelo_simulink_robot3gdl.m`.

## Pendiente de limpieza (no tocado en esta iteración)

`robot3dof_TFinal_v2_dinamica.m` (sin sufijo `_jacobianos`), `v3_controladores.m`
y `v4_astar_obstaculos.m` quedaron de una iteración anterior, con dinámica
ad-hoc (no Jacobianos+Christoffel) y sin el modelo Simulink. Por instrucción
del docente, el trabajo actual se centró primero en la dinámica y la
estructura Simulink (`v2_dinamica_jacobianos.m` + `crear_modelo_simulink_robot3gdl.m`).
Estos tres archivos deben reescribirse en la siguiente etapa del trabajo,
reutilizando las funciones numéricas de `v2_dinamica_jacobianos.m` en vez de
la dinámica antigua.
