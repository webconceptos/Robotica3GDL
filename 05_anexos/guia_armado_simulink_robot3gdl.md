# Guía de armado manual — Robot3GDL_Control_Final.slx

Esta guía es el procedimiento de referencia para construir el modelo Simulink exigido por el docente, independientemente de si `crear_modelo_simulink_robot3gdl.m` logró generar el `.slx` automáticamente. Procedimiento en MATLAB + Simulink:

> **Estado actual:** `crear_modelo_simulink_robot3gdl.m` ya cablea automáticamente los **tres** subsistemas (`PID_NoLineal`, `PD_Precomp`, `Par_Calculado`) siguiendo exactamente el patrón descrito abajo, y reacomoda el diagrama con `Simulink.BlockDiagram.arrangeSystem`. `PID_NoLineal` fue **validado en Simulink real**: compila (`Ctrl+D`) y converge correctamente al punto deseado. `PD_Precomp` y `Par_Calculado` usan el mismo patrón pero aún no se probaron en Simulink real. Esta guía sigue siendo el respaldo si la generación automática falla o si algún subsistema necesita ajuste manual.

## 0. Preparación previa en MATLAB

Antes de abrir Simulink, ejecuta en la consola de MATLAB (con la carpeta `01_codigo_final/` en el path):

```matlab
crear_modelo_simulink_robot3gdl
```

Esto deja en el workspace base:

- `robot` — parámetros geométricos y físicos (ver supuestos en `robot3dof_TFinal_v2_dinamica_jacobianos.m`).
- `qd_ws`, `qd_dot_ws`, `qd_ddot_ws` — trayectoria de referencia en formato `From Workspace` (struct con `.time` y `.signals.values`).
- `q0_ic`, `qdot0_ic` — condición inicial del robot (con un pequeño error respecto a `qd(0)`, para que se vea el transitorio del controlador).
- `gains_pid`, `gains_pd`, `gains_ct` — ganancias de los tres controladores.

Y genera en `01_codigo_final/simulink_blocks/` cuatro archivos con el código EXACTO que va dentro de cada bloque `MATLAB Function`:

- `mlfb_planta_3gdl.m`
- `mlfb_pid_nolineal.m`
- `mlfb_pd_precomp.m`
- `mlfb_par_calculado.m`

Estos archivos son autocontenidos (no llaman a funciones de otros archivos), porque un bloque `MATLAB Function` de Simulink no puede depender de funciones locales definidas en un script externo.

## 1. Crear el modelo

En Simulink: `File > New > Model`, guardarlo como `Robot3GDL_Control_Final.slx` dentro de `01_codigo_final/`.

Vas a construir **tres subsistemas independientes** dentro del mismo modelo (uno por controlador), para poder simular y comparar los tres bajo la misma trayectoria `qd(t)`:

```text
Robot3GDL_Control_Final.slx
 ├── PID_NoLineal
 ├── PD_Precomp
 └── Par_Calculado
```

Arrastra tres bloques `Subsystem` (`Simulink > Ports & Subsystems > Subsystem`) al lienzo principal y renómbralos así. Entra a cada uno (doble clic) y bórrale los puertos `In1`/`Out1` por defecto — dentro de cada subsistema todo se conecta con bloques `From Workspace` / `To Workspace`, no con puertos del subsistema.

## 2. Bloques dentro de CADA subsistema (mismo patrón para los tres)

| Bloque Simulink | Nombre sugerido | Librería |
|---|---|---|
| From Workspace | `qd` | Sources |
| From Workspace | `qd_dot` | Sources |
| From Workspace (solo PD_Precomp y Par_Calculado) | `qd_ddot` | Sources |
| MATLAB Function | `Controlador` | User-Defined Functions |
| MATLAB Function | `Planta_3GDL` | User-Defined Functions |
| Integrator | `Int_qdot` | Continuous |
| Integrator | `Int_q` | Continuous |
| Subtract (Sum con signos `+-`) | `Error` | Math Operations |
| Integrator (solo PID_NoLineal) | `Int_error` | Continuous |
| Saturation | `Sat_tau` | Discontinuities |
| Constant ×3 (solo PID_NoLineal) | `Const_Kp`, `Const_Kd`, `Const_Ki` | Sources |
| To Workspace | `q_out` | Sinks |
| To Workspace | `tau_out` | Sinks |
| Scope | `Scope_q` | Sinks |

Configura cada `From Workspace`:

- `qd` → `Data`: `qd_ws`
- `qd_dot` → `Data`: `qd_dot_ws`
- `qd_ddot` (si aplica) → `Data`: `qd_ddot_ws`

Configura cada `Integrator`:

- `Int_qdot` → `Initial condition`: `qdot0_ic`
- `Int_q` → `Initial condition`: `q0_ic`
- `Int_error` (solo PID) → `Initial condition`: `[0;0;0]`

Configura `Sat_tau`:

- `Upper limit`: `tau_max`
- `Lower limit`: `-tau_max`
- (`tau_max` queda en el workspace base al ejecutar `crear_modelo_simulink_robot3gdl`; ver Sección 0)

Configura cada `To Workspace`:

- `q_out` → `Variable name`: `q_<controlador>_out` (p. ej. `q_pid_out`, `q_pd_out`, `q_ct_out`)
- `tau_out` → `Variable name`: `tau_<controlador>_out`
- `Save format`: `Structure With Time` o `Timeseries` (ambos formatos funcionan; `comparar_controladores.m` detecta cuál se usó).

## 3. Código dentro de cada bloque `MATLAB Function`

Abre cada bloque `MATLAB Function` con doble clic y **reemplaza todo el contenido** por el de su archivo correspondiente en `simulink_blocks/`.

### Planta_3GDL (igual en los tres subsistemas)

Copiar el contenido de `mlfb_planta_3gdl.m`. Firma:

```matlab
function qddot = mlfb_planta_3gdl(tau, q, qdot)
```

- Entradas: `tau` (3×1), `q` (3×1), `qdot` (3×1).
- Salida: `qddot` (3×1).
- Calcula `M(q)`, `C(q,qdot)`, `G(q)` por Jacobianos + Christoffel (código numérico, sin `sym`) y devuelve `qddot = M\(tau - C*qdot - G)`.

### Controlador — subsistema PID_NoLineal

Copiar `mlfb_pid_nolineal.m`. Firma:

```matlab
function tau = mlfb_pid_nolineal(q, qdot, qd, qd_dot, eint, Kp, Kd, Ki)
```

`eint` viene del bloque `Int_error` (integral de `e = qd - q`), **no** se calcula dentro de este bloque — así el integrador queda visible en el diagrama, que es justamente lo que pidió el docente ("se aprecia mejor el control articular").

`Kp`, `Kd`, `Ki` entran como puertos de señal normales (puertos 6, 7 y 8, en ese orden — coinciden con el orden de los argumentos de la función). Conectar tres bloques `Constant` (`Const_Kp`, `Const_Kd`, `Const_Ki`) con `Value` = `gains_pid.Kp`, `gains_pid.Kd`, `gains_pid.Ki` respectivamente a esos tres puertos. **Sin esta conexión el modelo no compila** (puertos de entrada sin fuente). Alternativa equivalente: convertir esos tres argumentos en `Parameter` vía `Editor > Ports and Data Manager` dentro del bloque, en vez de puertos de señal — pero entonces no van conectados con `add_line`/cables, así que hay que elegir un solo enfoque y ser consistente.

### Controlador — subsistema PD_Precomp

Copiar `mlfb_pd_precomp.m`. Firma:

```matlab
function tau = mlfb_pd_precomp(q, qdot, qd, qd_dot, qd_ddot, Kp, Kd)
```

Puertos: 1=q, 2=qdot, 3=qd, 4=qd_dot, 5=qd_ddot, 6=Kp, 7=Kd. No hay `eint` (no lleva integral de error) — `e = qd - q` se calcula dentro del propio bloque, así que **no** se necesitan los bloques `Error`/`Int_error` de `PID_NoLineal`. `Kp`, `Kd` van igual que en `PID_NoLineal`: bloques `Constant` (`Const_Kp`, `Const_Kd`) con `Value` = `gains_pd.Kp`, `gains_pd.Kd`, conectados a los puertos 6 y 7.

### Controlador — subsistema Par_Calculado

Copiar `mlfb_par_calculado.m`. Firma:

```matlab
function tau = mlfb_par_calculado(q, qdot, qd, qd_dot, qd_ddot, Kp, Kd)
```

Mismo patrón que `PD_Precomp` (puertos 1-7 en el mismo orden), con `Const_Kp`/`Const_Kd` = `gains_ct.Kp`, `gains_ct.Kd`. Este es el controlador principal de la comparación final.

## 4. Conexiones (igual patrón en los tres subsistemas)

```text
qd, qd_dot, (qd_ddot) ──────────────┐
                                     ▼
Int_q ──(q)──────────────────► Controlador ──(tau)──► Planta_3GDL ──(qddot)──► Int_qdot ──(qdot)──► Int_q
Int_qdot ──(qdot)──────────────────┘                        ▲                     │
                                                              │                     │
                                     Int_q ──(q)──────────────┘                     │
                                     Int_qdot ──(qdot)───────────────────────────────┘
```

En palabras (para no depender del diagrama ASCII):

1. `Int_q` (salida `q`) y `Int_qdot` (salida `qdot`) alimentan de vuelta **tanto** al `Controlador` **como** a `Planta_3GDL` (realimentación de estado — usa líneas ramificadas, clic derecho y arrastra desde la línea existente).
2. `Controlador` recibe además `qd`, `qd_dot` (y `qd_ddot` si es PD_Precomp/Par_Calculado) desde los bloques `From Workspace`.
3. Solo en `PID_NoLineal`: `Error = qd - q` (bloque `Subtract`) → `Int_error` → entrada `eint` del `Controlador`; y `Const_Kp`/`Const_Kd`/`Const_Ki` → puertos 6/7/8 del `Controlador`.
4. `Controlador` produce `tau` → entra a `Sat_tau` (saturación de torque) → `Planta_3GDL`.
5. `Planta_3GDL` produce `qddot` → `Int_qdot` → `Int_q` (integradores en cascada: `qddot → Integrator → qdot → Integrator → q`).
6. `q` (salida de `Int_q`) → `To Workspace (q_out)` y → `Scope_q`.
7. `tau` saturado (salida de `Sat_tau`) → `To Workspace (tau_out)`.

## 5. Bloques de comparación (fuera de los subsistemas, en el modelo principal)

**El docente confirmó: la comparación final se hace sobre error ARTICULAR (no cartesiano).**

Los tres subsistemas comparten el mismo modelo (`Robot3GDL_Control_Final`), así que un solo `Run` los simula a los tres a la vez. Al terminar, según la configuración de "Data Import/Export" del modelo, las señales exportadas quedan disponibles de una de estas dos formas:

- Directamente en el workspace base: `q_pid_out`, `q_pd_out`, `q_ct_out`, `tau_pid_out`, `tau_pd_out`, `tau_ct_out`.
- O agrupadas dentro de un objeto `Simulink.SimulationOutput` llamado `out` (comportamiento por defecto al correr desde la barra de herramientas de la app en versiones recientes de Simulink), accesibles como `out.q_pid_out`, etc.

`comparar_controladores.m` (Sección 6) detecta automáticamente cuál de los dos casos aplica.

No hace falta un bloque adicional en Simulink para las métricas: se calculan en MATLAB después de simular (ver Sección 6). Si quieres verlas en vivo dentro de Simulink, agrega en cada subsistema:

- `Error articular` → un bloque `Subtract` (`qd - q`) → `Scope`.
- `Torque` → conectar la salida de `Sat_tau` directo a un `Scope` adicional.

## 6. Qué graficar en MATLAB después de simular (para el informe)

Después de simular (`Run` en cualquiera de los tres subsistemas simula el modelo completo), ejecutar:

```matlab
comparar_controladores
```

Este script (`01_codigo_final/comparar_controladores.m`) hace automáticamente:

1. Extrae `q_*_out`/`tau_*_out` desde `out` (si existe como `Simulink.SimulationOutput`) o desde el workspace base.
2. Alinea cada señal con la malla de tiempo de `qd_ws` (interpolación).
3. Calcula el error articular `e(t) = qd(t) - q(t)` para los tres controladores.
4. Grafica el error por articulación (`q1`, `q2`, `q3`) y el error total (norma), los tres controladores superpuestos.
5. Calcula la tabla comparativa (error RMS, error máximo, tiempo de estabilización, torque RMS, torque máximo) e indica cuál controlador converge más rápido a cero.

El tiempo de estabilización se calcula como el último instante en que la norma del error sale de una banda del 2% de su valor máximo y ya no vuelve a salir.

Gráficas y métricas mínimas a exportar (según lo confirmado por el docente: **error articular**, no cartesiano):

1. `qd` vs `q` (una figura por articulación, los tres controladores superpuestos).
2. Error articular `e(t) = qd(t) - q(t)` (norma o por articulación) — comparar error de entrada (al inicio) vs error de salida (en régimen permanente) para cada controlador.
3. Torque `tau1, tau2, tau3` por controlador (ya saturado, salida de `Sat_tau`).
4. Tabla comparativa: `Controlador | Error RMS | Error máximo | Tiempo de estabilización | Torque RMS | Torque máximo | Comentario` — indicar explícitamente cuál controlador converge más rápido a cero.
5. (Trayectoria con obstáculos: se agrega en `robot3dof_TFinal_v4_astar_obstaculos.m`, etapa posterior — no es parte de este modelo Simulink todavía.)

## 7. Notas de trazabilidad (para el informe)

- **Viene del parcial:** geometría `L1, L2, L3`; la idea de validar `fk`/`ik` antes de construir dinámica.
- **Se agrega para el trabajo final:** centros de masa, Jacobianos lineales/angulares, `M(q)` por el método de Jacobianos, `C(q,qdot)` por Christoffel (`n=3`), `G(q)` desde energía potencial, los tres controladores, y este modelo Simulink.
- **Datos del paper base (Tabla 2, pág. 8):** `L1, L2, L3, m1, m2, m3, g` — reportados explícitamente, no son supuestos.
- **Supuestos físicos:** centros de masa (`lc_i = L_i/2`) y radio de cada eslabón (`r = 0.03 m`, para el modelo de cilindro sólido indicado por el docente) — el paper no reporta estos valores; están documentados en el encabezado de `robot3dof_TFinal_v2_dinamica_jacobianos.m` y deben citarse igual en el informe.

## 8. Si algo no conecta

- Si `add_line` falla en el script automático, es casi siempre porque los **números de puerto** de un bloque `MATLAB Function` dependen del **orden** de los argumentos de entrada/salida definidos en su firma — verificar en `Editor > Ports and Data Manager` dentro del bloque y conectar por nombre de puerto en caso de duda.
- Si `Planta_3GDL` produce un error de matriz singular, verificar que `q0_ic` no coincida con una configuración donde `M(q)` sea numéricamente inestable (no debería ocurrir con los parámetros por defecto; puede aparecer si se modifican masas/longitudes a valores extremos).
