# Guía de estudio para la sustentación

Material de apoyo para dominar cada símbolo, cada fórmula y anticipar las
preguntas del docente. Pensado para repasar antes de la exposición, no
para leer en voz alta durante ella (eso ya está en las notas del orador
del `.pptx` y en `Distribucion_Sustentacion.txt`).

---

## 1. Glosario de símbolos

| Símbolo | Cómo se lee | Qué significa |
|---|---|---|
| `q` | "cu" | Vector de posición articular: `[q1, q2, q3]`, los tres ángulos de las articulaciones. |
| `q̇` (qdot) | "cu punto" | Velocidad articular, derivada de `q` respecto al tiempo. |
| `q̈` (qddot) | "cu dos puntos" | Aceleración articular, segunda derivada de `q`. |
| `qd`, `q̇d`, `q̈d` | "cu sub de" | Posición/velocidad/aceleración articular **deseada** (la trayectoria de referencia que el robot debe seguir). |
| `e` | "e" | Error de posición articular: `e = qd − q`. |
| `ė` (edot) | "e punto" | Error de velocidad: `ė = q̇d − q̇`. |
| `M(q)` | "eme de cu" | Matriz de inercia (o matriz de masa generalizada). 3×3, simétrica y definida positiva. Depende de la configuración `q`. |
| `C(q,q̇)` | "ce de cu, cu punto" | Matriz de Coriolis y fuerzas centrífugas. Depende de posición **y** velocidad. |
| `G(q)` | "ge de cu" | Vector de torques gravitacionales — cuánto torque hace falta solo para sostener el robot contra la gravedad en la configuración `q`. |
| `τ` (tau) | "tau" | Vector de torques articulares — lo que finalmente aplican los motores en cada articulación. |
| `Kp` | "ka pe" | Matriz (diagonal) de ganancias proporcionales del controlador. |
| `Kd` | "ka de" | Matriz de ganancias derivativas. |
| `Ki` | "ka i" | Matriz de ganancias integrales (solo en el PID no lineal). |
| `∫e` | "integral de e" | Acumulación del error en el tiempo — la acción integral del PID. |
| `Jv_i` | "jota ve sub i" | Jacobiano **lineal** del centro de masa del eslabón `i`: relaciona velocidades articulares con la velocidad lineal de ese centro de masa. |
| `Jw_i` | "jota doble-u sub i" | Jacobiano **angular** del eslabón `i`: relaciona velocidades articulares con la velocidad angular de ese eslabón. |
| `R_i` | "erre sub i" | Matriz de rotación del marco del eslabón `i` respecto a la base. |
| `I_i` | "i sub i" | Tensor de inercia del eslabón `i`, respecto a su propio centro de masa, expresado en su marco local. (No confundir con la matriz identidad.) |
| `pc_i` | "pe ce sub i" | Posición del centro de masa del eslabón `i`. |
| `m_i` | "eme sub i" | Masa del eslabón `i`. |
| `lc_i` | "ele ce sub i" | Distancia del centro de masa del eslabón `i` medida desde su articulación, a lo largo del eslabón. |
| `L1, L2, L3` | "ele uno, dos, tres" | Longitudes de los eslabones (parámetros geométricos DH, heredados del parcial). |
| `r_i` | "erre sub i" | Radio asumido de cada eslabón, para el modelo de cilindro sólido (0.03 m, supuesto — el paper no lo reporta). |
| `g` | "ge" | Aceleración de la gravedad, 9.81 m/s² (dato del paper). |
| `n` | "ene" | Número de grados de libertad del robot; `n = 3` en este trabajo. |
| `c_ijk` | "ce sub i, j, k" | Coeficientes de Christoffel de primer tipo — las piezas algebraicas con las que se construye `C(q,q̇)` a partir de `M(q)`. |
| `Ṁ` (Mdot) | "eme punto" | Derivada temporal de la matriz de inercia `M(q)`. |
| `z0, z1, z2` | "zeta cero, uno, dos" | Vectores unitarios de los ejes de giro de cada articulación; columnas de los Jacobianos angulares. |
| `a, α, d, θ` | "a, alfa, de, theta" | Los cuatro parámetros de Denavit-Hartenberg de cada articulación (longitud, torsión, desplazamiento, ángulo). |
| `≻ 0` | "sucede a cero" / "definida positiva" | Notación matemática para "matriz definida positiva" (todos sus autovalores son estrictamente positivos). |
| `λ` (lambda) | "lambda" | Autovalor — se usan los autovalores de `M(q)` para verificar que es definida positiva. |
| `h` | "hache" | Paso de diferenciación finita (`1e-6`) usado para derivar `M(q)` numéricamente sin símbolos. |
| `~` (en código MATLAB) | "tilde" | No es un símbolo matemático: en MATLAB descarta una salida de una función que no se va a usar (evita crear una variable solo para ignorarla). |
| `pchip` | se lee tal cual, es una sigla | *Piecewise Cubic Hermite Interpolating Polynomial* — método de interpolación que suaviza la ruta A* sin generar oscilaciones entre waypoints. |

---

## 2. Fórmulas explicadas

**Ecuación general de la dinámica del robot:**
```
M(q)·q̈ + C(q,q̇)·q̇ + G(q) = τ
```
La ecuación de Newton para un sistema articulado: inercia por aceleración, más
efectos centrífugos/Coriolis, más gravedad, igual al torque aplicado.

**Matriz de inercia (método de Jacobianos):**
```
M(q) = Σᵢ [ mᵢ·Jvᵢᵀ·Jvᵢ  +  Jwᵢᵀ·Rᵢ·Iᵢ·Rᵢᵀ·Jwᵢ ]
```
Por cada eslabón, se suma su contribución traslacional (masa por el Jacobiano
lineal al cuadrado) y su contribución rotacional (el tensor de inercia
"transportado" al marco base vía la rotación y el Jacobiano angular).

**Coeficientes de Christoffel → matriz de Coriolis:**
```
cᵢⱼₖ = ½·( ∂Mᵢⱼ/∂qₖ + ∂Mᵢₖ/∂qⱼ − ∂Mⱼₖ/∂qᵢ )
Cᵢⱼ  = Σₖ cᵢⱼₖ · q̇ₖ
```
`C(q,q̇)` no se inventa: se deriva mecánicamente de las derivadas parciales de
`M(q)`. Es el mismo resultado que daría Lagrange, obtenido por otro camino.

**Vector de gravedad (gradiente de energía potencial):**
```
G(q) = ∂P/∂q,      P(q) = Σᵢ mᵢ·g·pc_i,z(q)
```
La energía potencial es la suma de "masa por gravedad por altura" de cada
centro de masa; `G(q)` es cuánto cambia esa energía al mover cada articulación.

**Las tres leyes de control:**
```
PID no lineal:       τ = Kp·e + Kd·ė + Ki·∫e + G(q)
PD precompensado:    τ = M(qd)·q̈d + C(qd,q̇d)·q̇d + G(qd) + Kp·e + Kd·ė
Par calculado:        τ = M(q)·(q̈d + Kd·ė + Kp·e) + C(q,q̇)·q̇ + G(q)
```
La diferencia clave está en dónde se evalúa el modelo dinámico: el PID casi no
lo usa (solo `G(q)`), el PD lo evalúa en la trayectoria **deseada**, el par
calculado lo evalúa en el estado **real** medido.

**Dinámica de error del par calculado (por qué es el mejor):**
```
ë + Kd·ė + Kp·e = 0        (con Kd = 2√Kp → amortiguamiento crítico)
```
Al sustituir la ley del par calculado en la ecuación general, `C` y `G` se
cancelan exactamente y `M(q)` desaparece (es invertible), dejando una ecuación
lineal, desacoplada por articulación y exponencialmente estable.

**Propiedad de pasividad (base de la prueba de estabilidad):**
```
Ṁ(q) − 2·C(q,q̇)  es antisimétrica
```
Consecuencia automática de derivar `C` por Christoffel a partir de una `M(q)`
válida. Es lo que permite construir una función de Lyapunov válida para
demostrar estabilidad, no solo "verla" en una gráfica.

**Función de Lyapunov del PID no lineal:**
```
V = ½·q̇ᵀM(q)q̇ + ½·eᵀKp·e         (energía cinética + energía "elástica" del error)
V̇ = −ėᵀKd·ė ≤ 0                    (usando la antisimetría de Ṁ−2C)
```

**Rotaciones de la cadena cinemática (usadas en M(q)):**
```
R1 = Rz(q1)·Rx(π/2)
R2 = R1·Rz(q2)
R3 = R2·Rz(q3)
```

**Dinámica directa (para simular / integrar):**
```
q̈ = M(q)⁻¹·( τ − C(q,q̇)·q̇ − G(q) )
```

---

## 3. 20 preguntas probables del docente y las mejores respuestas

**1. ¿Por qué usar Jacobianos y no el método de Lagrange?**
Es indicación explícita del docente. Además, el método de Jacobianos permite
verificar cada término por separado (rotaciones, centros de masa, Jacobianos)
en vez de depender de una única expresión lagrangiana cerrada — así se
detectó, por ejemplo, una matriz de rotación mal orientada durante el
desarrollo.

**2. ¿Por qué el modelo de inercia es cilindro sólido y no varilla delgada?**
Indicación explícita del docente. Se usan las fórmulas estándar de cilindro
sólido: `I_axial = ½·m·r²`, `I_transversal = (1/12)·m·(3r²+L²)`.

**3. ¿De dónde sale el radio r = 0.03 m si el paper no lo reporta?**
Es un supuesto de simulación, documentado explícitamente como tal (no un dato
del paper). El paper solo reporta `L1,L2,L3`, `m1,m2,m3` y `g`.

**4. ¿Por qué las masas son 0.5, 0.5, 0.5 kg?**
Ese sí es un dato reportado directamente en la Tabla 2 del paper (Ashagrie et
al., 2021, pág. 8) — no es un supuesto nuestro.

**5. ¿Por qué G1 = 0 exactamente, y no solo "aproximadamente cero"?**
Es un resultado geométrico exacto: la primera articulación gira sobre el eje
vertical de la base, así que ningún centro de masa cambia de altura al mover
`q1`. Por lo tanto `∂P/∂q1 ≡ 0` para cualquier configuración.

**6. ¿Por qué q1 se mueve en la dinámica libre si G1 = 0 y parte del reposo?**
Por el acoplamiento de Coriolis (términos `C(1,2)` y `C(1,3)`): conservación
de momento angular transmitida desde `q2`/`q3` hacia la base. Si el modelo no
lo capturara, sería señal de un error de derivación.

**7. ¿Por qué M(q) tiene que ser simétrica y definida positiva?**
Porque representa el doble de la energía cinética del sistema,
`2T = q̇ᵀM(q)q̇`, que físicamente nunca puede ser negativa. Si `M(q)` no fuera
definida positiva, el modelo estaría mal derivado.

**8. ¿Qué es la propiedad de pasividad (Ṁ−2C antisimétrica) y por qué importa?**
Es la propiedad que permite construir una función de Lyapunov válida y
demostrar formalmente la estabilidad de los controladores, en vez de solo
observarla en una simulación. Es consecuencia automática de obtener `C` por
Christoffel a partir de una `M(q)` físicamente válida.

**9. ¿Por qué el par calculado converge más rápido y con menos torque?**
Porque cancela exactamente la dinámica no lineal evaluándola en el estado
real (no en el deseado), reduciendo el sistema en lazo cerrado a una ecuación
lineal de segundo orden desacoplada — no hay dinámica residual que combatir.

**10. ¿Por qué el torque RMS es casi igual en los tres controladores?**
Porque en régimen permanente el torque requerido es, en los tres casos, casi
enteramente compensación de gravedad — ahí los tres hacen básicamente lo
mismo. La diferencia real está en el pico del transitorio y en cuánto tardan
en llegar a ese régimen, no en el esfuerzo sostenido.

**11. ¿Por qué el error máximo es idéntico en los tres controladores?**
Ocurre en `t = 0`, antes de que cualquier controlador tenga oportunidad de
actuar. Depende solo de la condición inicial impuesta, no de la ley de
control.

**12. ¿Por qué el PID difiere en tiempo de estabilización entre Simulink y MATLAB puro?**
El PID no incorpora anti-windup; esa no linealidad lo hace sensible a
diferencias numéricas mínimas entre dos integradores distintos (aunque
equivalentes). Los otros dos controladores, sin ese elemento no lineal,
coinciden casi exactos en ambas plataformas.

**13. ¿Debería el PID tener anti-windup?**
Es una mejora natural y queda como pregunta abierta/recomendación explícita
al docente; no estaba en el alcance original pero explicaría la discrepancia
de la pregunta anterior.

**14. ¿Por qué usar diferencias finitas para C(q,q̇) en los bloques de Simulink, si ya tienen la versión simbólica?**
Porque un bloque `MATLAB Function` de Simulink no admite código del Symbolic
Math Toolbox en tiempo de ejecución — tiene que ser numérico puro. La versión
por diferencias finitas es la adaptación de la misma fórmula de Christoffel
para ese entorno.

**15. ¿Cómo se validó que la versión numérica (diferencias finitas) es correcta?**
Comparándola contra la derivación simbólica independiente en el mismo punto
de prueba: coinciden con un error del orden de `10⁻¹⁰`.

**16. ¿Por qué A* y no otro algoritmo de planeación (RRT, Dijkstra, campos de potencial)?**
El problema es de baja dimensión (una grilla 2D en el plano XZ), donde A* con
una heurística admisible garantiza la ruta óptima explorando el menor número
de nodos posible — es la elección estándar para este tipo de espacio de
búsqueda discreto y acotado.

**17. ¿Por qué conectividad de 8 vecinos y no 4?**
Permite movimientos diagonales, lo que da rutas más suaves y realistas y
evita el "efecto escalera" característico de una búsqueda con solo 4
direcciones.

**18. ¿La planeación A* puede ejecutarse dentro de Simulink, o solo en MATLAB?**
Se planifica en MATLAB y se entrega la trayectoria articular deseada `qd(t)`
a Simulink ya calculada. Queda como pregunta abierta al docente si desea que
el propio Simulink ejecute el algoritmo de búsqueda.

**19. ¿Por qué suavizar la trayectoria con pchip y no con splines cúbicos regulares?**
`pchip` preserva la monotonía entre waypoints y evita el efecto Runge
(oscilaciones no físicas entre puntos), garantizando continuidad `C¹` sin
sobrepasos, lo que es preferible para una trayectoria que después debe
ejecutar un robot físico.

**20. ¿Qué pasa si un waypoint de A* no fuera alcanzable por cinemática inversa?**
Se tendría que descartar o replantear esa parte de la ruta. En este trabajo
no fue necesario: los 13 waypoints resultaron todos alcanzables, cero puntos
descartados.

### Preguntas de reserva (por si sobra tiempo de preguntas)

**21. ¿Este método escala a un robot con más grados de libertad?**
Sí — el método de Jacobianos y Christoffel es general para `n` grados de
libertad; en el código solo cambia el valor de `n` en el triple bucle, no la
estructura del método.

**22. ¿Por qué no usar el Robotics System Toolbox de MATLAB?**
Por requisito del curso: la cinemática, la dinámica, los controladores y el
planeador A* están implementados desde cero, sin depender de toolboxes de
robótica de terceros.
