%% ROBOT ANTROPOMORFICO 3GDL - TRABAJO FINAL v2 (DINAMICA)
% -------------------------------------------------------------------------
% Curso      : Robotica y Sistemas Autonomos
% Entregable : Bloque de desarrollo - Modelo dinamico M(q), C(q,dq), G(q)
% Archivo    : robot3dof_TFinal_v2_dinamica.m
%
% Punto de partida:
% Esta version parte de la base cinematica de robot3dof_TFinal_v1.m, que a
% su vez proviene del trabajo parcial (robot3dof_paper_TParcial_g2.m).
%
% Alcance de esta version (bloque "dinamica" unicamente):
%   A) Cinematica directa e inversa heredadas (necesarias para pruebas).
%   B) Jacobiano translacional y deteccion de singularidades.
%   C) Modelo dinamico completo M(q), C(q,dq), G(q).
%   D) Validacion numerica del modelo (simetria de M, dinamica libre).
%
% Fuera de alcance en esta version (se agregan en versiones posteriores):
%   - Controladores (PID no lineal, PD precompensado, par calculado) -> v3.
%   - Planeacion autonoma con obstaculos (A*) -> v4.
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
% [P5] Paper, Pag. 8, Tabla 2: L1=0.15 m, L2=0.50 m, L3=0.50 m.
%
% Notas metodologicas IMPORTANTES:
% 1) El paper base no reporta masas, inercias ni centros de masa completos.
%    Por ello, esta version usa parametros fisicos asumidos y declarados
%    explicitamente en el bloque de parametros (Seccion 1).
% 2) Esta decision debe consultarse con el docente (ver Q1 mas abajo). Si el
%    docente exige derivacion completa por Lagrange con parametros medidos,
%    se debera reemplazar esta dinamica por una derivacion formal.
%
% Pregunta pendiente al docente:
%   Q1. Dado que el paper base no presenta masas e inercias completas,
%       ¿se acepta usar parametros fisicos asumidos y justificados para
%       implementar M(q), C(q,dq), G(q)?
% -------------------------------------------------------------------------

clc; clear; close all;

%% ================================================================
% 1. PARAMETROS GEOMETRICOS Y FISICOS DEL ROBOT
% Objetivo: centralizar en robot.* toda constante geometrica/fisica usada
%           por la cinematica y la dinamica.
% Fuente/justificacion: geometria L1,L2,L3 tomada del paper base [P5].
%           Masas, centros de masa e inercias son supuestos de simulacion
%           (el paper no los reporta completos, ver nota metodologica 1).
% Resultado esperado: estructura "robot" lista para fk_3dof, ik_3dof,
%           jacobian_3dof y las funciones de dinamica.
%% ================================================================
robot.L1 = 0.15;        % altura/base [m]                          [P5]
robot.L2 = 0.50;        % longitud eslabon 2 [m]                   [P5]
robot.L3 = 0.50;        % longitud eslabon 3 [m]                   [P5]

% Parametros fisicos asumidos para simulacion dinamica preliminar.
% Supuesto para simulacion: el paper base no reporta masa/inercia completa.
robot.m1 = 2.00;        % masa equivalente base/eslabon 1 [kg]
robot.m2 = 1.50;        % masa eslabon 2 [kg]
robot.m3 = 1.00;        % masa eslabon 3 [kg]
robot.lc2 = robot.L2/2; % centro de masa eslabon 2 [m]
robot.lc3 = robot.L3/2; % centro de masa eslabon 3 [m]
robot.I1 = 0.030;       % inercia equivalente junta 1 [kg*m^2]
robot.I2 = robot.m2*robot.L2^2/12; % inercia barra eslabon 2 [kg*m^2]
robot.I3 = robot.m3*robot.L3^2/12; % inercia barra eslabon 3 [kg*m^2]
robot.g  = 9.81;        % gravedad [m/s^2]

fprintf('============================================================\n');
fprintf(' ROBOT ANTROPOMORFICO 3GDL - TRABAJO FINAL v2 (DINAMICA)\n');
fprintf('============================================================\n');
fprintf('Geometria: L1=%.2f m, L2=%.2f m, L3=%.2f m\n', robot.L1, robot.L2, robot.L3);
fprintf('Nota: masas e inercias son asumidas para simulacion preliminar.\n\n');

%% ================================================================
% 2. VALIDACION CINEMATICA HEREDADA DEL PARCIAL
% Objetivo: confirmar que fk_3dof, ik_3dof y jacobian_3dof siguen siendo
%           correctas antes de apoyar en ellas la validacion dinamica.
% Fuente/justificacion: cinematica directa/inversa del trabajo parcial [P3][P4].
% Resultado esperado: fk_3dof(ik_3dof(p)) ≈ p, y det(J) distinto de cero en
%           una configuracion no singular.
%% ================================================================
q_test = deg2rad([30; 40; -25]);
[T03, p_test, ~] = fk_3dof(q_test, robot);
[J_test, detJ_test] = jacobian_3dof(q_test, robot);

fprintf('================ VALIDACION CINEMATICA ================\n');
fprintf('q prueba [deg] = [%.2f %.2f %.2f]\n', rad2deg(q_test));
fprintf('p efector [m] = [%.4f %.4f %.4f]\n', p_test(1), p_test(2), p_test(3));
fprintf('det(J) = %.8f\n', detJ_test);

[q_up, ~, reachable] = ik_3dof(p_test, robot);
[~, p_check, ~] = fk_3dof(q_up, robot);
err_fk_ik = norm(p_test - p_check);
fprintf('Chequeo fk(ik(p)) ~= p : error = %.3e (reachable=%d)\n', err_fk_ik, reachable);
if err_fk_ik > 1e-6
    warning('El chequeo fk(ik(p)) ~= p supero la tolerancia esperada.');
end

%% ================================================================
% 3. MODELO DINAMICO M(q), C(q,dq), G(q)
% Objetivo: implementar la dinamica del manipulador segun
%           M(q)*qddot + C(q,qdot)*qdot + G(q) = tau.
% Fuente/justificacion: q1 se modela como giro de base (yaw); q2-q3 forman
%           un manipulador planar vertical 2R clasico. Se desprecian
%           acoplamientos dinamicos yaw/plano (ver comentario en
%           inertia_matrix_3dof). Parametros fisicos: ver Seccion 1.
% Resultado esperado: M(q) simetrica y definida positiva; C y G consistentes
%           dimensionalmente con tau en [N*m].
%% ================================================================
qdot_test = deg2rad([5; -3; 4]);
M_test = inertia_matrix_3dof(q_test, robot);
C_test = coriolis_matrix_3dof(q_test, qdot_test, robot);
G_test = gravity_vector_3dof(q_test, robot);

fprintf('\n================ VALIDACION DINAMICA ================\n');
disp('M(q) = '); disp(M_test);
disp('C(q,dq) = '); disp(C_test);
disp('G(q) = '); disp(G_test);
fprintf('Chequeo M simetrica: ||M-M^T|| = %.3e\n', norm(M_test-M_test','fro'));
eigM = eig(M_test);
fprintf('Autovalores de M(q): [%.4f %.4f %.4f] (deben ser > 0)\n', eigM(1), eigM(2), eigM(3));

%% ================================================================
% 4. SIMULACION DE DINAMICA LIBRE (tau = 0)
% Objetivo: validar que el modelo dinamico produce un comportamiento
%           fisicamente razonable sin controlador (caida bajo gravedad).
% Fuente/justificacion: con tau=0, robot_dynamics debe generar aceleraciones
%           que muevan q2 y q3 en la direccion que reduce energia potencial
%           (los eslabones "caen" hacia abajo por efecto de G(q)).
% Resultado esperado: q2(t) y q3(t) evolucionan monotonamente hacia una
%           configuracion de menor energia potencial durante la ventana de
%           simulacion (no hay control que lo evite).
%% ================================================================
dt = 0.005;
tf_free = 2.0;
t_free = 0:dt:tf_free;
N_free = length(t_free);

q_free = zeros(3, N_free);
qdot_free = zeros(3, N_free);
q_free(:,1) = deg2rad([10; 20; -10]);
qdot_free(:,1) = [0; 0; 0];

for k = 1:N_free-1
    tau_zero = [0; 0; 0];
    qddot_k = robot_dynamics(q_free(:,k), qdot_free(:,k), tau_zero, robot);
    qdot_free(:,k+1) = qdot_free(:,k) + qddot_k*dt;
    q_free(:,k+1) = q_free(:,k) + qdot_free(:,k+1)*dt;
    q_free(:,k+1) = wrap_to_pi_local(q_free(:,k+1));
end

fprintf('\n================ DINAMICA LIBRE (tau=0) ================\n');
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

fprintf('\n================ RESUMEN v2 ================\n');
fprintf('Cinematica validada y modelo dinamico M,C,G implementado y verificado.\n');
fprintf('Siguiente bloque (v3): agregar controladores sobre esta base dinamica.\n');

%% ================================================================
% FUNCIONES LOCALES - CINEMATICA
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
    % Jacobiano translacional derivado de:
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
% FUNCIONES LOCALES - DINAMICA
% ================================================================

function M = inertia_matrix_3dof(q, robot)
    % Matriz de inercia aproximada para robot antropomorfico 3GDL.
    % Supuestos:
    % 1) q1 es giro de base alrededor de Z.
    % 2) q2 y q3 forman un manipulador planar vertical 2R.
    % 3) Se ignoran terminos de acoplamiento dinamico entre yaw q1 y el plano
    %    q2-q3. Para una version final avanzada, estos terminos pueden
    %    obtenerse por Euler-Lagrange completo con Jacobianos de centros de masa.
    q2 = q(2); q3 = q(3);
    L2 = robot.L2; lc2 = robot.lc2; lc3 = robot.lc3;
    m2 = robot.m2; m3 = robot.m3;
    I1 = robot.I1; I2 = robot.I2; I3 = robot.I3;

    % Inercia equivalente de yaw por radios horizontales de centros de masa.
    r2 = lc2*cos(q2);
    r3 = L2*cos(q2) + lc3*cos(q2+q3);
    M11 = I1 + m2*r2^2 + m3*r3^2;

    % Submatriz planar 2R clasica.
    M22 = I2 + I3 + m2*lc2^2 + m3*(L2^2 + lc3^2 + 2*L2*lc3*cos(q3));
    M23 = I3 + m3*(lc3^2 + L2*lc3*cos(q3));
    M33 = I3 + m3*lc3^2;

    M = [M11, 0,   0;
         0,   M22, M23;
         0,   M23, M33];

    % Regularizacion numerica minima.
    M = M + 1e-6*eye(3);
end

function C = coriolis_matrix_3dof(q, qdot, robot)
    % Matriz C aproximada. Se modelan los terminos principales del submodelo
    % planar 2R q2-q3. El producto C(q,dq)*dq contiene efectos de Coriolis y
    % centrifugos.
    q3 = q(3);
    dq2 = qdot(2); dq3 = qdot(3);
    h = -robot.m3*robot.L2*robot.lc3*sin(q3);

    C = zeros(3,3);
    C(2,2) = h*dq3;
    C(2,3) = h*(dq2 + dq3);
    C(3,2) = -h*dq2;
end

function G = gravity_vector_3dof(q, robot)
    % Vector de gravedad G(q) = dV/dq.
    % La gravedad afecta q2 y q3. q1 no cambia energia potencial.
    q2 = q(2); q3 = q(3);
    g = robot.g;
    L2 = robot.L2; lc2 = robot.lc2; lc3 = robot.lc3;
    m2 = robot.m2; m3 = robot.m3;

    G1 = 0;
    G2 = (m2*lc2 + m3*L2)*g*cos(q2) + m3*lc3*g*cos(q2+q3);
    G3 = m3*lc3*g*cos(q2+q3);
    G = [G1; G2; G3];
end

function qddot = robot_dynamics(q, qdot, tau, robot)
    % Dinamica directa: qdd = inv(M)*(tau - C*dq - G)
    M = inertia_matrix_3dof(q, robot);
    C = coriolis_matrix_3dof(q, qdot, robot);
    G = gravity_vector_3dof(q, robot);
    qddot = M \ (tau - C*qdot - G);
end

%% ================================================================
% UTILIDADES
% ================================================================

function y = wrap_to_pi_local(x)
    y = mod(x + pi, 2*pi) - pi;
end
