# Guía de armado manual — Robot3GDL_Control_Final.slx

Esta guía es el procedimiento de referencia para construir el modelo Simulink exigido por el docente, independientemente de si `crear_modelo_simulink_robot3gdl.m` logró generar el `.slx` automáticamente. Procedimiento en MATLAB + Simulink:

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

Configura cada `To Workspace`:

- `q_out` → `Variable name`: `q_<controlador>_out` (p. ej. `q_pid_out`, `q_pd_out`, `q_ct_out`)
- `tau_out` → `Variable name`: `tau_<controlador>_out`
- `Save format`: `Structure With Time` (para que quede alineado con `qd_ws` al graficar el error después).

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

`Kp`, `Kd`, `Ki` deben quedar fijados como parámetros del bloque: en el editor del `MATLAB Function`, `Editor > Ports and Data Manager`, agrega `Kp`, `Kd`, `Ki` como argumentos de entrada de tipo `Parameter` con valor inicial `gains_pid.Kp`, `gains_pid.Kd`, `gains_pid.Ki` (o cablea tres bloques `Constant` con esos valores desde el workspace).

### Controlador — subsistema PD_Precomp

Copiar `mlfb_pd_precomp.m`. Firma:

```matlab
function tau = mlfb_pd_precomp(q, qdot, qd, qd_dot, qd_ddot, Kp, Kd)
```

Usa `gains_pd.Kp`, `gains_pd.Kd`.

### Controlador — subsistema Par_Calculado

Copiar `mlfb_par_calculado.m`. Firma:

```matlab
function tau = mlfb_par_calculado(q, qdot, qd, qd_dot, qd_ddot, Kp, Kd)
```

Usa `gains_ct.Kp`, `gains_ct.Kd`. Este es el controlador principal de la comparación final.

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
3. Solo en `PID_NoLineal`: `Error = qd - q` (bloque `Subtract`) → `Int_error` → entrada `eint` del `Controlador`.
4. `Controlador` produce `tau` → entra a `Planta_3GDL`.
5. `Planta_3GDL` produce `qddot` → `Int_qdot` → `Int_q` (integradores en cascada: `qddot → Integrator → qdot → Integrator → q`).
6. `q` (salida de `Int_q`) → `To Workspace (q_out)` y → `Scope_q`.
7. `tau` (salida de `Controlador`) → `To Workspace (tau_out)`.

## 5. Bloques de comparación (fuera de los subsistemas, en el modelo principal)

Después de simular los tres subsistemas (con el mismo `Stop time` = último valor de `qd_ws.time`, es decir `tf = 5.0 s` con la trayectoria por defecto), en el workspace base vas a tener:

- `q_pid_out`, `q_pd_out`, `q_ct_out` (estructura con `.time` y `.signals.values`)
- `tau_pid_out`, `tau_pd_out`, `tau_ct_out`

No hace falta un bloque adicional en Simulink para el error RMS: se calcula en MATLAB después de simular (ver Sección 6). Si quieres verlo en vivo dentro de Simulink, agrega en cada subsistema:

- `Error articular` → un bloque `Subtract` (`qd - q`) → `Scope`.
- `Torque` → conectar la salida de `Controlador` directo a un `Scope` adicional.

## 6. Qué graficar en MATLAB después de simular (para el informe)

```matlab
sim('Robot3GDL_Control_Final');   % o simular cada subsistema por separado

e_pid = qd_ws.signals.values - q_pid_out.signals.values;
e_pd  = qd_ws.signals.values - q_pd_out.signals.values;
e_ct  = qd_ws.signals.values - q_ct_out.signals.values;

error_rms_pid = rms(vecnorm(e_pid,2,2));
error_rms_pd  = rms(vecnorm(e_pd,2,2));
error_rms_ct  = rms(vecnorm(e_ct,2,2));
```

Gráficas mínimas a exportar:

1. `qd` vs `q` (una figura por articulación, los tres controladores superpuestos).
2. Error articular `e(t) = qd(t) - q(t)` (norma o por articulación).
3. Torque `tau1, tau2, tau3` por controlador.
4. Tabla comparativa: `Controlador | Error RMS | Error máximo | Torque RMS | Torque máximo | Comentario`.
5. (Trayectoria con obstáculos: se agrega en `robot3dof_TFinal_v4_astar_obstaculos.m`, etapa posterior — no es parte de este modelo Simulink todavía.)

## 7. Notas de trazabilidad (para el informe)

- **Viene del parcial:** geometría `L1, L2, L3`; la idea de validar `fk`/`ik` antes de construir dinámica.
- **Se agrega para el trabajo final:** centros de masa, Jacobianos lineales/angulares, `M(q)` por el método de Jacobianos, `C(q,qdot)` por Christoffel (`n=3`), `G(q)` desde energía potencial, los tres controladores, y este modelo Simulink.
- **Supuestos físicos:** masas, centros de masa y tensores de inercia (varilla delgada) — el paper base no los reporta; están documentados en el encabezado de `robot3dof_TFinal_v2_dinamica_jacobianos.m` y deben citarse igual en el informe.

## 8. Si algo no conecta

- Si `add_line` falla en el script automático, es casi siempre porque los **números de puerto** de un bloque `MATLAB Function` dependen del **orden** de los argumentos de entrada/salida definidos en su firma — verificar en `Editor > Ports and Data Manager` dentro del bloque y conectar por nombre de puerto en caso de duda.
- Si `Planta_3GDL` produce un error de matriz singular, verificar que `q0_ic` no coincida con una configuración donde `M(q)` sea numéricamente inestable (no debería ocurrir con los parámetros por defecto; puede aparecer si se modifican masas/longitudes a valores extremos).
