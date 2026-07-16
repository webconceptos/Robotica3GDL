%% ROBOT ANTROPOMORFICO 3GDL - TRABAJO FINAL v2 (DINAMICA POR JACOBIANOS)
% -------------------------------------------------------------------------
% Curso      : Robotica y Sistemas Autonomos
% Entregable : Modelo dinamico del robot 3GDL, metodo de Jacobianos + Christoffel
% Archivo    : robot3dof_TFinal_v2_dinamica_jacobianos.m
%
% Punto de partida (trazabilidad con el trabajo parcial):
% Este archivo parte de 00_base_parcial/robot3dof_paper_TParcial_g2.m, del
% cual se conservan SIN MODIFICAR:
%   - Parametros geometricos L1, L2, L3 (Tabla 2 del paper base).
%   - dh_standard, fk_3dof, ik_3dof, jacobian_3dof.
% Lo que se AGREGA en este archivo para el trabajo final:
%   - Centros de masa pc1, pc2, pc3.
%   - Jacobianos lineales Jv1, Jv2, Jv3 y angulares Jw1, Jw2, Jw3.
%   - Matriz de inercia M(q), matriz de Coriolis C(q,qdot) y vector de
%     gravedad G(q), obtenidos por el metodo de Jacobianos (NO Lagrange).
%
% Instruccion critica del docente:
%   "No obtener el modelo dinamico por el camino del Lagrangiano. En codigo
%    se debe encontrar el modelo dinamico usando el metodo alterno visto en
%    clase, centrado en Jacobianos. La matriz M depende de los Jacobianos
%    lineales y angulares. La matriz de Coriolis puede obtenerse con
%    coeficientes de Christoffel usando el codigo compartido en clase,
%    cambiando la variable a n = 3."
% Por tanto, en este archivo:
%   - M(q)      se calcula como  sum_i [ m_i*Jvi'*Jvi + Jwi'*Ri*Ii*Ri'*Jwi ]
%   - C(q,qdot) se calcula por coeficientes de Christoffel con n = 3.
%   - G(q)      se calcula como el gradiente de la energia potencial P(q).
%
% Paper base del parcial:
% Ashagrie, A., Salau, A. O., & Weldcherkos, T. (2021).
% Modeling and control of a 3-DOF articulated robotic manipulator using
% self-tuning fuzzy sliding mode controller. Cogent Engineering.
%
% Trazabilidad tecnica heredada del parcial:
% [P1] Paper, Pag. 5, Seccion 3: manipulador industrial de tres eslabones
%      con tres juntas revolutas.
% [P2] Paper, Pag. 6, Seccion 3.1 y Tabla 1: parametros DH del robot.
% [P5] Paper, Pag. 8, Tabla 2 "List of parameters of robot manipulator":
%      L1=0.15 m, L2=0.50 m, L3=0.50 m, m1=0.50 kg, m2=0.50 kg,
%      m3=0.50 kg, g=9.81 m/s^2. TODOS estos siete valores son datos
%      reportados explicitamente por el paper, no supuestos.
%
% -------------------------------------------------------------------------
% SUPUESTOS FISICOS
%
% La Tabla 2 del paper (Pag. 8, [P5]) reporta longitudes, masas y gravedad
% de los tres eslabones (L1,L2,L3,m1,m2,m3,g). Esos siete valores se toman
% tal cual del paper, no son supuestos.
%
% El paper NO reporta, en cambio, la posicion exacta del centro de masa de
% cada eslabon ni el tensor de inercia completo (solo aparecen simbolos
% genericos como lc2, lc3, r1 dentro de las ecuaciones de Lagrange (10)-
% (12), sin valores numericos en la Tabla 2). Por eso, lo siguiente SI son
% supuestos de simulacion, declarados y justificados:
%
% 1) Centros de masa (lc1, lc2, lc3): se asume que cada eslabon es
%    aproximadamente uniforme, por lo que su centro de masa esta a la
%    mitad de su longitud (lc_i = L_i/2). pc1 se ubica sobre el eje de la
%    columna base (ver deduccion geometrica en la Seccion 3).
%
% 2) Tensores de inercia (I1, I2, I3): cada eslabon se aproxima como un
%    CILINDRO SOLIDO UNIFORME (indicacion explicita del docente: "todos
%    los eslabones pueden modelarse como cilindros solidos"), con radio
%    asumido r = 0.03 m para los tres eslabones (el paper no reporta
%    radio de eslabon en ninguna parte). Formulas estandar de inercia de
%    un cilindro solido de masa m, radio r y longitud L:
%      I_axial      = (1/2)*m*r^2        (eje de simetria del cilindro)
%      I_transversal = (1/12)*m*(3*r^2 + L^2)  (eje perpendicular, por el centro)
%    Expresadas en la base ortonormal del propio marco DH del eslabon
%    (frame i), tal como exige la formula
%    M(q) = sum_i [ m_i*Jvi'*Jvi + Jwi'*Ri*Ii*Ri'*Jwi ].
%    La asignacion de cual eje local es el "axial" NO es arbitraria: se
%    deduce de la geometria DH real del robot (ver Seccion 3, nota sobre
%    y1 = z0 y x_i a lo largo del eslabon i).
%
% Estos dos supuestos (centros de masa y radio de cada eslabon) deben
% declararse explicitamente en el informe, diferenciandolos de L1,L2,L3,
% m1,m2,m3,g, que si provienen directamente de la Tabla 2 del paper base.
% -------------------------------------------------------------------------

clc; clear; close all;

%% ================================================================
% 1. PARAMETROS GEOMETRICOS Y FISICOS DEL ROBOT
% Objetivo: centralizar en robot.* toda constante geometrica/fisica usada
%           por la cinematica y la dinamica.
% Fuente/justificacion: L1,L2,L3,m1,m2,m3,g tomados de la Tabla 2 del paper
%           base [P5] (dato reportado, no supuesto). Centros de masa e
%           inercias son supuestos de simulacion documentados arriba (el
%           paper no reporta sus valores numericos).
% Resultado esperado: estructura "robot" lista para toda la derivacion
%           dinamica de este archivo.
%% ================================================================
robot.L1 = 0.15;        % altura/base [m]                          [P5]
robot.L2 = 0.50;        % longitud eslabon 2 [m]                   [P5]
robot.L3 = 0.50;        % longitud eslabon 3 [m]                   [P5]

% Dato del paper (Tabla 2, Pag. 8, [P5]): "Mass of link 1/2/3" = 0.5 kg cada uno.
robot.m1 = 0.50;        % masa eslabon 1 (columna base) [kg]       [P5]
robot.m2 = 0.50;        % masa eslabon 2 [kg]                      [P5]
robot.m3 = 0.50;        % masa eslabon 3 [kg]                      [P5]

% Supuesto de simulacion: el paper no reporta el valor numerico de lc_i.
robot.lc1 = robot.L1/2; % centro de masa eslabon 1 [m]
robot.lc2 = robot.L2/2; % centro de masa eslabon 2 [m]
robot.lc3 = robot.L3/2; % centro de masa eslabon 3 [m]

robot.g  = 9.81;        % gravedad [m/s^2]                         [P5]

% Tensores de inercia por eslabon: CILINDRO SOLIDO (indicacion del
% docente), radio asumido r=0.03 m para los tres eslabones (supuesto de
% simulacion; el paper no reporta radio). Formulas estandar de cilindro
% solido: I_axial=(1/2)*m*r^2, I_transversal=(1/12)*m*(3*r^2+L^2).
% Expresados en el marco local i: para los eslabones 2 y 3, el eje AXIAL
% es el eje local 1 (x_i, a lo largo del eslabon). Para el eslabon 1, el
% eje AXIAL corresponde al eje local 2 (y1), no al eje local 1, porque la
% torsion alpha=90 grados del DH del parcial hace que y1 quede alineado
% con z0 (el eje de la columna base). Ver deduccion en Seccion 3.
robot.r1 = 0.03; robot.r2 = 0.03; robot.r3 = 0.03; % [m] radio asumido de cada eslabon

I_axial1 = 0.5*robot.m1*robot.r1^2;
I_trans1 = (1/12)*robot.m1*(3*robot.r1^2 + robot.L1^2);
I_axial2 = 0.5*robot.m2*robot.r2^2;
I_trans2 = (1/12)*robot.m2*(3*robot.r2^2 + robot.L2^2);
I_axial3 = 0.5*robot.m3*robot.r3^2;
I_trans3 = (1/12)*robot.m3*(3*robot.r3^2 + robot.L3^2);

robot.I1 = diag([I_trans1, I_axial1, I_trans1]);
robot.I2 = diag([I_axial2, I_trans2, I_trans2]);
robot.I3 = diag([I_axial3, I_trans3, I_trans3]);

fprintf('============================================================\n');
fprintf(' ROBOT ANTROPOMORFICO 3GDL - v2: DINAMICA POR JACOBIANOS\n');
fprintf('============================================================\n');
fprintf('Geometria (paper base): L1=%.2f m, L2=%.2f m, L3=%.2f m\n', robot.L1, robot.L2, robot.L3);
fprintf('Masas (Tabla 2 del paper): m1=%.2f, m2=%.2f, m3=%.2f kg\n', robot.m1, robot.m2, robot.m3);
fprintf('Metodo dinamico: Jacobianos lineales/angulares + Christoffel (NO Lagrange).\n\n');

%% ================================================================
% 2. CINEMATICA DH HEREDADA DEL PARCIAL
% Objetivo: conservar intacta la cinematica validada en el parcial, ya que
%           la dinamica por Jacobianos se construye directamente sobre las
%           mismas matrices de transformacion homogenea T0i.
% Fuente/justificacion: dh_standard, fk_3dof, ik_3dof, jacobian_3dof vienen
%           del trabajo parcial [P3][P4], sin cambios.
% Resultado esperado: fk_3dof(ik_3dof(p)) ~= p dentro de tolerancia numerica.
%% ================================================================
q_test = deg2rad([30; 40; -25]);
[T03, p_test, ~] = fk_3dof(q_test, robot);
[q_up, ~, reachable] = ik_3dof(p_test, robot);
[~, p_check, ~] = fk_3dof(q_up, robot);
err_fk_ik = norm(p_test - p_check);

fprintf('================ VALIDACION CINEMATICA (heredada) ================\n');
fprintf('q prueba [deg] = [%.2f %.2f %.2f]\n', rad2deg(q_test));
fprintf('p efector [m]  = [%.4f %.4f %.4f]\n', p_test(1), p_test(2), p_test(3));
fprintf('Chequeo fk(ik(p)) ~= p : error = %.3e (reachable=%d)\n', err_fk_ik, reachable);
if err_fk_ik > 1e-6
    warning('El chequeo fk(ik(p)) ~= p supero la tolerancia esperada.');
end

%% ================================================================
% 3. CENTROS DE MASA Y JACOBIANOS (LINEALES Y ANGULARES)
% Objetivo: obtener, para cada eslabon i, la posicion de su centro de masa
%           pc_i(q), su Jacobiano lineal Jvi = d(pc_i)/dq y su Jacobiano
%           angular Jwi (columnas = ejes de giro que afectan al eslabon i).
%
% Fuente/justificacion (deduccion geometrica, NO arbitraria):
%   a) Para las juntas 2 y 3 (alpha=0 en su DH), se cumple que el vector
%      que va del origen del marco (i-1) al origen del marco i es
%      exactamente a_i * x_i (columna 1 de R_i). Por eso el centro de masa
%      del eslabon i se ubica en pc_i = p_{i-1} + lc_i * R_i(:,1).
%   b) Para la junta 1 (alpha=90 grados), a_1=0 y el eslabon 1 es la
%      columna vertical de altura L1 a lo largo de z0. Se verifica que
%      R1(:,2) = z0 para TODO q1 (la torsion de 90 grados hace que el eje
%      y1 quede permanentemente alineado con z0). Por eso pc1 = [0;0;lc1]
%      es constante (no depende de q1: la columna gira sobre si misma) y
%      el eje "axial" de I1 (inercia casi nula) se coloca en el eje local
%      2 de I1, no en el eje local 1.
%   c) Los ejes de giro de las juntas 2 y 3 son PARALELOS (alpha=0 entre
%      esos eslabones), por lo que z1 = z2 = R1(:,3) = R2(:,3). Esto es
%      fisicamente correcto: hombro y codo giran sobre el mismo eje
%      horizontal, como en un brazo antropomorfico real.
%
% Resultado esperado: Jv1 = 0 (pc1 no depende de q); Jv2, Jv3 y Jw1, Jw2,
%           Jw3 con la estructura triangular tipica de una cadena serial
%           (las columnas de juntas posteriores al eslabon i son cero).
%% ================================================================
function [pc1, pc2, pc3, Jv1, Jv2, Jv3, Jw1, Jw2, Jw3, R1, R2, R3] = com_kinematics_3dof(q, robot)
    % L3 no se usa directamente: pc3 llega hasta lc3 (centro de masa), no hasta L3.
    L1 = robot.L1; L2 = robot.L2;
    lc1 = robot.lc1; lc2 = robot.lc2; lc3 = robot.lc3;
    q1 = q(1); q2 = q(2); q3 = q(3);
    C1 = cos(q1); S1 = sin(q1);
    C2 = cos(q2); S2 = sin(q2);
    C23 = cos(q2+q3); S23 = sin(q2+q3);

    z0 = [0;0;1];
    zjoint23 = [S1; -C1; 0]; % eje comun de las juntas 2 y 3 (z1 = z2)

    % Matrices de rotacion base->marco i (derivadas de T0i = A01*A12*...).
    R1 = [C1, 0, S1; S1, 0, -C1; 0, 1, 0];                       % Rz(q1)*Rx(pi/2)
    R2 = [C1*C2, -C1*S2, S1; S1*C2, -S1*S2, -C1; S2, C2, 0];      % R1*Rz(q2)
    R3 = [C1*C23, -C1*S23, S1; S1*C23, -S1*S23, -C1; S23, C23, 0];% R2*Rz(q3)

    % Centros de masa (ver justificacion a,b arriba).
    pc1 = [0; 0; lc1];
    pc2 = [lc2*C1*C2;                    lc2*S1*C2;                    L1 + lc2*S2];
    pc3 = [L2*C1*C2 + lc3*C1*C23;        L2*S1*C2 + lc3*S1*C23;        L1 + L2*S2 + lc3*S23];

    % Jacobianos lineales Jvi = d(pc_i)/d[q1,q2,q3] (derivados analiticamente).
    Jv1 = zeros(3,3);

    Jv2 = [-lc2*S1*C2, -lc2*C1*S2, 0;
            lc2*C1*C2, -lc2*S1*S2, 0;
            0,          lc2*C2,    0];

    Jv3 = [-S1*(L2*C2+lc3*C23), -C1*(L2*S2+lc3*S23), -C1*lc3*S23;
            C1*(L2*C2+lc3*C23), -S1*(L2*S2+lc3*S23), -S1*lc3*S23;
            0,                    L2*C2+lc3*C23,        lc3*C23];

    % Jacobianos angulares: columna k = eje z de la junta k si esa junta
    % afecta al eslabon i (k<=i en una cadena serial), cero en otro caso.
    Jw1 = [z0,        [0;0;0],   [0;0;0]];
    Jw2 = [z0,        zjoint23,  [0;0;0]];
    Jw3 = [z0,        zjoint23,  zjoint23];
end

[pc1_t, pc2_t, pc3_t, Jv1_t, Jv2_t, Jv3_t, Jw1_t, Jw2_t, Jw3_t] = com_kinematics_3dof(q_test, robot);
fprintf('\n================ CENTROS DE MASA Y JACOBIANOS ================\n');
fprintf('pc1 [m] = [%.4f %.4f %.4f]\n', pc1_t);
fprintf('pc2 [m] = [%.4f %.4f %.4f]\n', pc2_t);
fprintf('pc3 [m] = [%.4f %.4f %.4f]\n', pc3_t);
fprintf('||Jv1|| (debe ser 0, pc1 no depende de q) = %.3e\n', norm(Jv1_t,'fro'));

%% ================================================================
% 4. MATRIZ DE INERCIA M(q) POR EL METODO DE JACOBIANOS
% Objetivo: ensamblar M(q) = sum_i [ m_i*Jvi'*Jvi + Jwi'*Ri*Ii*Ri'*Jwi ],
%           exactamente como exige la instruccion del docente, sin pasar
%           por una derivacion Lagrangiana manual.
% Fuente/justificacion: formula estandar de energia cinetica de un
%           manipulador serial expresada via Jacobianos de cada eslabon
%           (Spong, Hutchinson & Vidyasagar, "Robot Modeling and Control").
% Resultado esperado: M(q) simetrica y definida positiva para cualquier q
%           fisicamente valido; size(M)==[3 3].
%% ================================================================
function M = inertia_matrix_3dof(q, robot)
    [~, ~, ~, Jv1, Jv2, Jv3, Jw1, Jw2, Jw3, R1, R2, R3] = com_kinematics_3dof(q, robot);
    M = robot.m1*(Jv1.'*Jv1) + Jw1.'*R1*robot.I1*R1.'*Jw1 + ...
        robot.m2*(Jv2.'*Jv2) + Jw2.'*R2*robot.I2*R2.'*Jw2 + ...
        robot.m3*(Jv3.'*Jv3) + Jw3.'*R3*robot.I3*R3.'*Jw3;
    M = (M + M.')/2; % simetrizacion numerica (M es simetrica por construccion;
                      % esto solo elimina ruido de redondeo de punto flotante).
end

%% ================================================================
% 5. VECTOR DE GRAVEDAD G(q) DESDE LA ENERGIA POTENCIAL
% Objetivo: obtener G(q) = dP/dq, con P(q) = sum_i m_i*g*pc_i,z(q), es
%           decir, el gradiente de la energia potencial gravitacional de
%           los centros de masa, sin pasar por Lagrange.
% Fuente/justificacion: pc1_z = lc1 (constante) => no contribuye a G(q).
%           pc2_z = L1 + lc2*sin(q2); pc3_z = L1 + L2*sin(q2) + lc3*sin(q2+q3).
%           Derivando P(q) a mano respecto a q1,q2,q3 se obtiene la forma
%           cerrada usada abajo (verificada tambien de forma simbolica con
%           jacobian(P,q), ver Seccion 9).
% Resultado esperado: G1(q) = 0 (la junta de yaw no cambia energia
%           potencial); G2, G3 dependen de cos(q2) y cos(q2+q3).
%% ================================================================
function G = gravity_vector_3dof(q, robot)
    q2 = q(2); q3 = q(3);
    g = robot.g;
    L2 = robot.L2; lc2 = robot.lc2; lc3 = robot.lc3;
    m2 = robot.m2; m3 = robot.m3;

    G1 = 0;
    G2 = (m2*lc2 + m3*L2)*g*cos(q2) + m3*lc3*g*cos(q2+q3);
    G3 = m3*lc3*g*cos(q2+q3);
    G = [G1; G2; G3];
end

%% ================================================================
% 6. MATRIZ DE CORIOLIS C(q,qdot) POR COEFICIENTES DE CHRISTOFFEL (n=3)
% Objetivo: calcular C(q,qdot) usando EXACTAMENTE el metodo de Christoffel
%           indicado por el docente:
%             c_ijk = 1/2*(dM_ij/dq_k + dM_ik/dq_j - dM_jk/dq_i)
%             C_ij  = sum_k c_ijk * qdot_k,   con n = 3.
% Fuente/justificacion: se evaluan las derivadas parciales de M(q) por
%           diferencias centrales (paso h=1e-6) en vez de diferenciacion
%           simbolica en tiempo de ejecucion. Es la MISMA formula de
%           Christoffel; solo cambia como se obtiene dM/dq. Se eligio esta
%           forma numerica porque debe ejecutarse dentro de un bloque
%           MATLAB Function de Simulink, y Simulink NO admite codigo del
%           Symbolic Math Toolbox (sym, diff) en esos bloques. La Seccion 9
%           de este archivo re-deriva C(q,qdot) de forma simbolica (metodo
%           canonico confirmado por el docente: M(q) alimenta a
%           Christoffel, de ahi sale C) y confirma que ambos caminos
%           coinciden numericamente.
% Resultado esperado: (Mdot - 2*C) antisimetrica (propiedad estandar de
%           C obtenida por Christoffel a partir de una M valida).
%% ================================================================
function C = coriolis_matrix_3dof(q, qdot, robot)
    n = 3;
    h = 1e-6;
    dM = cell(1,n);
    for k = 1:n
        dq = zeros(n,1); dq(k) = h;
        Mp = inertia_matrix_3dof(q + dq, robot);
        Mm = inertia_matrix_3dof(q - dq, robot);
        dM{k} = (Mp - Mm)/(2*h);
    end
    C = zeros(n,n);
    for i = 1:n
        for j = 1:n
            for k = 1:n
                cijk = 0.5*(dM{k}(i,j) + dM{j}(i,k) - dM{i}(j,k));
                C(i,j) = C(i,j) + cijk*qdot(k);
            end
        end
    end
end

%% ================================================================
% 7. DINAMICA DIRECTA DEL ROBOT
% Objetivo: implementar qddot = M(q) \ (tau - C(q,qdot)*qdot - G(q)),
%           exactamente como pide el enunciado del trabajo final, lista para
%           usarse dentro de un bloque MATLAB Function de Simulink (codigo
%           puramente numerico, sin dependencias del Symbolic Math Toolbox).
%% ================================================================
function qddot = robot_dynamics_3dof(q, qdot, tau, robot)
    M = inertia_matrix_3dof(q, robot);
    C = coriolis_matrix_3dof(q, qdot, robot);
    G = gravity_vector_3dof(q, robot);
    qddot = M \ (tau - C*qdot - G);
end

%% ================================================================
% 8. PRUEBA NUMERICA DEL MODELO DINAMICO
% Objetivo: validar M(q), C(q,qdot), G(q) en una configuracion de prueba:
%           M debe ser simetrica y definida positiva; G debe anularse en
%           G1; y una simulacion libre (tau=0) debe mostrar un
%           comportamiento fisicamente razonable (los eslabones "caen" por
%           efecto de la gravedad).
%% ================================================================
qdot_test = deg2rad([5; -3; 4]);
M_test = inertia_matrix_3dof(q_test, robot);
C_test = coriolis_matrix_3dof(q_test, qdot_test, robot);
G_test = gravity_vector_3dof(q_test, robot);

fprintf('\n================ VALIDACION NUMERICA DEL MODELO ================\n');
disp('M(q) = '); disp(M_test);
disp('C(q,dq) = '); disp(C_test);
disp('G(q) = '); disp(G_test);
fprintf('Chequeo M simetrica: ||M-M^T|| = %.3e\n', norm(M_test-M_test','fro'));
eigM = eig(M_test);
fprintf('Autovalores de M(q): [%.4f %.4f %.4f] (deben ser > 0)\n', eigM(1), eigM(2), eigM(3));
if any(eigM <= 0)
    warning('M(q) no es definida positiva en la configuracion de prueba.');
end

% ---- Simulacion de dinamica libre (tau = 0): sanity check fisico ----
dt = 0.005;
tf_free = 2.0;
t_free = 0:dt:tf_free;
N_free = length(t_free);

q_free = zeros(3, N_free);
qdot_free = zeros(3, N_free);
q_free(:,1) = deg2rad([10; 20; -10]);
qdot_free(:,1) = [0; 0; 0];

for k = 1:N_free-1
    qddot_k = robot_dynamics_3dof(q_free(:,k), qdot_free(:,k), [0;0;0], robot);
    qdot_free(:,k+1) = qdot_free(:,k) + qddot_k*dt;
    q_free(:,k+1) = q_free(:,k) + qdot_free(:,k+1)*dt;
end

fprintf('\n================ DINAMICA LIBRE (tau=0, sanity check) ================\n');
fprintf('q inicial [deg] = [%.2f %.2f %.2f]\n', rad2deg(q_free(:,1)));
fprintf('q final   [deg] = [%.2f %.2f %.2f] (tras %.1f s de caida libre)\n', rad2deg(q_free(:,end)), tf_free);

figure('Name','Dinamica libre bajo gravedad (tau=0)');
names = {'q1','q2','q3'};
for i = 1:3
    subplot(3,1,i);
    plot(t_free, rad2deg(q_free(i,:)), 'LineWidth', 1.4);
    grid on; xlabel('Tiempo [s]'); ylabel([names{i}, ' [deg]']);
    title(['Respuesta libre de ', names{i}, ' (tau=0, solo gravedad)']);
end

%% ================================================================
% 9. DERIVACION SIMBOLICA DE VERIFICACION (OPCIONAL, Symbolic Math Toolbox)
% Objetivo: re-derivar M(q), C(q,qdot) y G(q) de forma puramente simbolica,
%           usando exactamente la funcion coriolis_christoffel(M,q,qdot)
%           que el docente compartio en clase (con n=3), para dejar en el
%           informe/anexos las expresiones cerradas y para verificar de
%           forma independiente el resultado numerico de las Secciones
%           4-6. Este bloque es OPCIONAL: si el Symbolic Math Toolbox no
%           esta disponible, se omite automaticamente y el resto del
%           archivo (usado por Simulink y por la simulacion) no depende
%           de el en absoluto.
% Resultado esperado: M_sym, C_sym, G_sym simplificadas coinciden, al
%           evaluarlas numericamente en q_test/qdot_test, con M_test,
%           C_test, G_test calculadas en la Seccion 8.
%% ================================================================
try
    run_symbolic_verification(robot, q_test, qdot_test, M_test, C_test, G_test);
catch err
    fprintf('\n[Aviso] Verificacion simbolica omitida (%s).\n', err.message);
    fprintf('        Motivo tipico: Symbolic Math Toolbox no disponible.\n');
    fprintf('        El modelo numerico (Secciones 4-8), que es el que usan la\n');
    fprintf('        simulacion y Simulink, NO depende de este bloque.\n');
end

function run_symbolic_verification(robot, q_test, qdot_test, M_test, C_test, G_test)
    syms q1 q2 q3 qd1 qd2 qd3 real
    q = [q1;q2;q3]; qdot = [qd1;qd2;qd3];

    A01 = dh_standard_sym(q1, robot.L1, 0, sym(pi)/2);
    A12 = dh_standard_sym(q2, 0, robot.L2, sym(0));
    A23 = dh_standard_sym(q3, 0, robot.L3, sym(0));
    T01 = A01; T02 = A01*A12; T03 = A01*A12*A23;
    R1 = T01(1:3,1:3); R2 = T02(1:3,1:3); R3 = T03(1:3,1:3);
    p1 = T01(1:3,4); p2 = T02(1:3,4);

    z0 = [0;0;1]; z1 = R1(:,3); z2 = R2(:,3);

    pc1 = [sym(0); sym(0); robot.lc1];
    pc2 = p1 + robot.lc2*R2(:,1);
    pc3 = p2 + robot.lc3*R3(:,1);

    Jv1 = jacobian(pc1, q);
    Jv2 = jacobian(pc2, q);
    Jv3 = jacobian(pc3, q);
    Jw1 = [z0, [0;0;0], [0;0;0]];
    Jw2 = [z0, z1, [0;0;0]];
    Jw3 = [z0, z1, z2];

    M_sym = robot.m1*(Jv1.')*Jv1 + Jw1.'*R1*robot.I1*R1.'*Jw1 + ...
            robot.m2*(Jv2.')*Jv2 + Jw2.'*R2*robot.I2*R2.'*Jw2 + ...
            robot.m3*(Jv3.')*Jv3 + Jw3.'*R3*robot.I3*R3.'*Jw3;
    M_sym = simplify(M_sym);

    C_sym = coriolis_christoffel(M_sym, q, qdot);

    P = robot.m1*robot.g*pc1(3) + robot.m2*robot.g*pc2(3) + robot.m3*robot.g*pc3(3);
    G_sym = jacobian(P, q).';
    G_sym = simplify(G_sym);

    fprintf('\n================ DERIVACION SIMBOLICA (verificacion) ================\n');
    disp('M(q) simbolica simplificada:'); disp(M_sym);
    disp('C(q,qdot) simbolica simplificada:'); disp(C_sym);
    disp('G(q) simbolica simplificada:'); disp(G_sym);

    M_sym_num = double(subs(M_sym, q, q_test));
    C_sym_num = double(subs(C_sym, [q;qdot], [q_test;qdot_test]));
    G_sym_num = double(subs(G_sym, q, q_test));

    fprintf('\nDiferencia ||M_numerica - M_simbolica|| = %.3e\n', norm(M_test - M_sym_num, 'fro'));
    fprintf('Diferencia ||C_numerica - C_simbolica|| = %.3e\n', norm(C_test - C_sym_num, 'fro'));
    fprintf('Diferencia ||G_numerica - G_simbolica|| = %.3e\n', norm(G_test - G_sym_num));
end

function A = dh_standard_sym(theta, d, a, alpha)
    % Igual que dh_standard, pero con literales envueltos en sym(...) para
    % evitar errores de construccion de matrices simbolicas con literales
    % numericos puros mezclados con expresiones simbolicas.
    ct = cos(theta); st = sin(theta);
    ca = cos(alpha); sa = sin(alpha);
    A = [ct, -st*ca,  st*sa, a*ct;
         st,  ct*ca, -ct*sa, a*st;
         sym(0),     sa,     ca,    sym(d);
         sym(0),      sym(0),      sym(0),    sym(1)];
end

function C = coriolis_christoffel(M, q, qdot)
    % Funcion compartida en clase por el docente, adaptada unicamente
    % fijando n = 3 (no se cambia el metodo).
    n = 3;
    C = sym(zeros(n,n));
    for i = 1:n
        for j = 1:n
            for k = 1:n
                cijk = 0.5*(diff(M(i,j),q(k)) + diff(M(i,k),q(j)) - diff(M(j,k),q(i)));
                C(i,j) = C(i,j) + cijk*qdot(k);
            end
        end
    end
    C = simplify(C);
end

fprintf('\n================ RESUMEN v2 (DINAMICA POR JACOBIANOS) ================\n');
fprintf('M(q), C(q,qdot) y G(q) obtenidos por Jacobianos + Christoffel (n=3), sin Lagrange.\n');
fprintf('Funciones numericas listas para bloques MATLAB Function de Simulink:\n');
fprintf('  inertia_matrix_3dof(q,robot), coriolis_matrix_3dof(q,qdot,robot),\n');
fprintf('  gravity_vector_3dof(q,robot), robot_dynamics_3dof(q,qdot,tau,robot)\n');
fprintf('Siguiente paso: crear_modelo_simulink_robot3gdl.m (Planta + 3 controladores).\n');

%% ================================================================
% RESPUESTAS DEL DOCENTE (preguntas ya resueltas)
% ================================================================
% R1. Modelo de inercia: usar CILINDRO SOLIDO para los tres eslabones (no
%     varilla delgada). "Con la formula M(q) saldra automaticamente el
%     modelo de cilindro solido." Implementado arriba (Seccion 1).
% R2. Matriz de Coriolis: usar el codigo compartido en clase. Flujo
%     confirmado: M(q) alimenta a Christoffel (funcion coriolis_christoffel,
%     Seccion 9) y de ahi sale la matriz C. Ese es el metodo canonico; la
%     version numerica por diferencias finitas (Seccion 6) es unicamente
%     una adaptacion de ESE MISMO metodo para poder ejecutarse dentro de
%     un bloque MATLAB Function de Simulink (que no admite sym/diff).
% R3. Comparacion final: reportar ERROR ARTICULAR (no cartesiano). Ademas
%     de error RMS/max, incluir tiempo de estabilizacion (cuanto tarda el
%     error en aproximarse a cero) y comparar que controlador converge
%     mas rapido. Ver Seccion 8 y compute_metrics en v3_controladores.m.
%
% PREGUNTAS PENDIENTES AL DOCENTE (aun sin responder)
% Q1. ¿El modelo Simulink puede usar bloques MATLAB Function que llamen a
%     estas mismas funciones numericas (inertia_matrix_3dof, etc.), o se
%     exige que el codigo dentro del bloque este completamente inline?
% Q2. ¿El PID no lineal debe incluir saturacion de torque y anti-windup?

%% ================================================================
% FUNCIONES LOCALES - CINEMATICA HEREDADA DEL PARCIAL (sin modificar)
% ================================================================

function A = dh_standard(theta, d, a, alpha)
    % Matriz DH estandar:
    % A = RotZ(theta)*TransZ(d)*TransX(a)*RotX(alpha)
    ct = cos(theta); st = sin(theta);
    ca = cos(alpha); sa = sin(alpha);
    A = [ct, -st*ca,  st*sa, a*ct;
         st,  ct*ca, -ct*sa, a*st;
          0,     sa,     ca,    d;
          0,      0,      0,    1];
end

function [T03, p, R] = fk_3dof(q, robot)
    % Cinematica directa del robot antropomorfico 3R usando DH del paper.
    L1 = robot.L1; L2 = robot.L2; L3 = robot.L3;
    q1 = q(1); q2 = q(2); q3 = q(3);
    A01 = dh_standard(q1, L1, 0,  pi/2);
    A12 = dh_standard(q2, 0,  L2, 0);
    A23 = dh_standard(q3, 0,  L3, 0);
    T03 = A01*A12*A23;
    p = T03(1:3,4);
    R = T03(1:3,1:3);
end

function [q_up, q_down, reachable] = ik_3dof(p, robot)
    % Cinematica inversa geometrica para robot 3R.
    L1 = robot.L1; L2 = robot.L2; L3 = robot.L3;
    px = p(1); py = p(2); pz = p(3);
    q1 = atan2(py, px);
    r = sqrt(px^2 + py^2);
    s = pz - L1;
    c3 = (r^2 + s^2 - L2^2 - L3^2)/(2*L2*L3);
    if abs(c3) > 1
        q_up = [NaN; NaN; NaN];
        q_down = [NaN; NaN; NaN];
        reachable = false;
        return;
    end
    c3 = max(min(c3,1),-1);
    reachable = true;
    q3_up = atan2( sqrt(1 - c3^2), c3);
    q2_up = atan2(s, r) - atan2(robot.L3*sin(q3_up), robot.L2 + robot.L3*cos(q3_up));
    q3_down = atan2(-sqrt(1 - c3^2), c3);
    q2_down = atan2(s, r) - atan2(robot.L3*sin(q3_down), robot.L2 + robot.L3*cos(q3_down));
    q_up = wrap_to_pi_local([q1; q2_up; q3_up]);
    q_down = wrap_to_pi_local([q1; q2_down; q3_down]);
end

function [J, detJ] = jacobian_3dof(q, robot)
    % Jacobiano translacional del efector final derivado de:
    % px = L2*C1*C2 + L3*C1*C23
    % py = L2*S1*C2 + L3*S1*C23
    % pz = L1 + L2*S2 + L3*S23
    L2 = robot.L2; L3 = robot.L3;
    q1 = q(1); q2 = q(2); q3 = q(3);
    C1 = cos(q1); S1 = sin(q1);
    C2 = cos(q2); S2 = sin(q2);
    C23 = cos(q2 + q3); S23 = sin(q2 + q3);
    J = [ -L3*S1*C23 - L2*S1*C2,  -L3*C1*S23 - L2*C1*S2,  -L3*C1*S23;
           L3*C1*C23 + L2*C1*C2,  -L3*S1*S23 - L2*S1*S2,  -L3*S1*S23;
           0,                       L3*C23 + L2*C2,          L3*C23];
    detJ = det(J);
end

%% ================================================================
% UTILIDADES
% ================================================================

function y = wrap_to_pi_local(x)
    y = mod(x + pi, 2*pi) - pi;
end
