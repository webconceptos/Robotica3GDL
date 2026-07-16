%% ROBOT ANTROPOMORFICO 3GDL - RECONSTRUCCION DEL PAPER
% -------------------------------------------------------------------------
% Paper base:
% Ashagrie, A., Salau, A. O., & Weldcherkos, T. (2021).
% Modeling and control of a 3-DOF articulated robotic manipulator using
% self-tuning fuzzy sliding mode controller. Cogent Engineering.
%
% Objetivo del script:
% 1) Cinematica directa mediante DH
% 2) Cinematica inversa geometrica
% 3) Cinematica diferencial mediante Jacobiano
% 4) Analisis de singularidades
% 5) Control cinematico por Jacobiano inverso / pseudoinversa
% 6) Simulacion 3D de trayectoria del efector final
%
% Nota metodologica:
% Este codigo reconstruye el modelo cinematico del paper y agrega el
% Jacobiano, singularidades y control cinematico para cumplir el trabajo parcial.
%
% TRAZABILIDAD GENERAL AL PAPER:
% [P1] Pag. 1, Resumen/Abstract: el paper declara el modelado y control de
%      trayectoria de un manipulador robotico articulado de 3 GDL usando
%      MATLAB/Simulink y controlador ST-FSMC.
% [P2] Pag. 5, Seccion 3: el paper indica que desarrolla el modelo cinematico
%      y dinamico de un manipulador industrial de tres eslabones y tres juntas
%      revolutas.
% [P3] Pag. 6, Seccion 3.1 y Tabla 1: se reportan los parametros DH del
%      manipulador 3GDL: a0=0, alpha0=90°, d1=L1, q1; a1=L2, alpha1=0,
%      d2=0, q2; a2=L3, alpha2=0, d3=0, q3.
% [P4] Pag. 7, Ecs. (4) y (5): se reportan la matriz homogenea T03 y las
%      ecuaciones de posicion px, py, pz del efector final.
% [P5] Pag. 7, Seccion 3.2, Ecs. (6)-(8): se reporta la cinematica inversa
%      geometrica para q1, q2 y q3.
% [P6] Pag. 8, Tabla 2: se reportan las dimensiones usadas en el modelo:
%      L1=0.15 m, L2=0.5 m, L3=0.5 m.
% [P7] Pag. 14 y Pag. 20, Fig. 11 y Seccion 5.1: se reporta una trayectoria
%      helicoidal 3D para evaluar el seguimiento del efector final.
% [P8] Pag. 19, Seccion 4, Ecs. (20)-(22): se usa estabilidad de Lyapunov
%      en el diseno de control del paper.
%
% DESARROLLO PROPIO SOBRE LA BASE DEL PAPER:
% [D1] Jacobiano translacional: derivado por diferenciacion parcial de las
%      ecuaciones px, py, pz del paper, Ecs. (5).
% [D2] Singularidades: derivadas a partir de det(J)=0.
% [D3] Control cinematico por pseudoinversa amortiguada: complementa el paper
%      para cubrir explicitamente control cinematico y seguimiento cartesiano.

clc; clear; close all;

%% ================================================================
% 1. PARAMETROS DEL ROBOT SEGUN EL PAPER
% ================================================================
% Fuente del paper:
% - Pag. 5, Seccion 3: manipulador industrial de tres eslabones con tres
%   juntas revolutas.
% - Pag. 8, Tabla 2: dimensiones fisicas del manipulador:
%   L1 = 0.15 m, L2 = 0.5 m, L3 = 0.5 m.

L1 = 0.15;     % altura/base del primer eslabon [m]
L2 = 0.50;     % longitud del segundo eslabon [m]
L3 = 0.50;     % longitud del tercer eslabon [m]

% Vector de parametros para pasar a funciones
robot.L1 = L1;
robot.L2 = L2;
robot.L3 = L3;

%% ================================================================
% 2. CINEMATICA DIRECTA - PRUEBA EN UNA CONFIGURACION
% ================================================================
% Variables articulares segun el modelo DH del paper:
% - Pag. 6, Seccion 3.1, Tabla 1: q1, q2 y q3 son las variables articulares
%   de las tres juntas revolutas.
% - Pag. 7, Ecs. (4)-(5): T03 y p=[px;py;pz] dependen de q1, q2 y q3.

q = deg2rad([30; 40; -25]);   % configuracion de prueba elegida para validar el codigo [rad]
                              % Nota: estos angulos NO son datos del paper;
                              % son un caso numerico propio para comprobar FK, IK y J.

[T03, p, R] = fk_3dof(q, robot);

fprintf('================ CINEMATICA DIRECTA ================\n');
fprintf('q [deg] = [%.3f %.3f %.3f]\n', rad2deg(q(1)), rad2deg(q(2)), rad2deg(q(3)));
fprintf('Posicion efector [m] = [%.4f %.4f %.4f]\n', p(1), p(2), p(3));
disp('Matriz homogenea T03 = ');
disp(T03);

%% ================================================================
% 3. CINEMATICA INVERSA - VALIDACION
% ================================================================
% Fuente del paper:
% - Pag. 7, Seccion 3.2, Ecs. (6)-(8): cinematica inversa geometrica para
%   obtener q1, q2 y q3 a partir de la posicion cartesiana del efector.
% En este script, se usa la posicion calculada por FK y se recuperan los
% angulos articulares para validar consistencia FK-IK.

[q_ik_up, q_ik_down, reachable] = ik_3dof(p, robot);

fprintf('\n================ CINEMATICA INVERSA ================\n');
if reachable
    fprintf('Solucion codo arriba  [deg] = [%.3f %.3f %.3f]\n', rad2deg(q_ik_up));
    fprintf('Solucion codo abajo   [deg] = [%.3f %.3f %.3f]\n', rad2deg(q_ik_down));
else
    fprintf('El punto esta fuera del espacio de trabajo.\n');
end

%% ================================================================
% 4. CINEMATICA DIFERENCIAL - JACOBIANO
% ================================================================
% Desarrollo propio basado en el paper:
% - El paper entrega px, py, pz en Pag. 7, Ec. (5).
% - El Jacobiano no aparece desarrollado explicitamente en el paper.
% - Aqui se obtiene derivando parcialmente p=[px;py;pz] respecto de
%   q=[q1;q2;q3].
% Relacion fundamental de cinematica diferencial:
%       x_dot = J(q) * q_dot

J = jacobian_3dof(q, robot);
detJ = det(J);
rankJ = rank(J);

fprintf('\n================ JACOBIANO ================\n');
disp('J(q) = ');
disp(J);
fprintf('det(J) = %.8f\n', detJ);
fprintf('rank(J) = %d\n', rankJ);

% Ejemplo de velocidades articulares
qdot = deg2rad([10; 5; -8]);      % [rad/s]
xdot = J*qdot;                    % velocidad cartesiana [m/s]

fprintf('qdot [deg/s] = [%.3f %.3f %.3f]\n', rad2deg(qdot));
fprintf('xdot [m/s]   = [%.5f %.5f %.5f]\n', xdot(1), xdot(2), xdot(3));

%% ================================================================
% 5. ANALISIS DE SINGULARIDADES
% ================================================================
% Desarrollo propio basado en el paper:
% - A partir del Jacobiano derivado de las Ecs. (5), se calcula det(J).
% - Las singularidades se identifican cuando el Jacobiano pierde rango,
%   es decir, det(J)=0 para este caso cuadrado 3x3.
% Para este robot, el determinante del Jacobiano translacional es:
% det(J) = -L2*L3*(L2*cos(q2) + L3*cos(q2+q3))*sin(q3)
%
% Hay singularidad cuando:
% 1) sin(q3) = 0
% 2) L2*cos(q2) + L3*cos(q2+q3) = 0

fprintf('\n================ SINGULARIDADES ================\n');
fprintf('Condicion 1: sin(q3) = 0  -> q3 = 0 o pi\n');
fprintf('Condicion 2: L2*cos(q2) + L3*cos(q2+q3) = 0\n');

% Barrido numerico para visualizar configuraciones cercanas a singularidad
q2_vals = deg2rad(linspace(-170,170,160));
q3_vals = deg2rad(linspace(-170,170,160));
[Q2, Q3] = meshgrid(q2_vals, q3_vals);
DET = -L2*L3.*(L2*cos(Q2) + L3*cos(Q2+Q3)).*sin(Q3);

figure('Name','Mapa de singularidades');
contourf(rad2deg(Q2), rad2deg(Q3), abs(DET), 30);
grid on;
xlabel('q2 [deg]');
ylabel('q3 [deg]');
title('|det(J)| - Zonas oscuras cercanas a singularidad');
colorbar;

%% ================================================================
% 6. GENERACION DE TRAYECTORIA CARTESIANA 3D
% ================================================================
% Fuente del paper:
% - Pag. 14, Fig. 11, y Pag. 20, Seccion 5.1: el paper evalua seguimiento
%   de una trayectoria helicoidal 3D.
% - El paper indica una trayectoria del tipo:
%     x = 70*sin(t) cm, y = 50*cos(t) cm, z = 15 + 3.5*t cm.
% Adaptacion en este script:
% - Se escala y desplaza la trayectoria para mantenerla dentro del workspace
%   del robot y evitar posiciones no alcanzables durante la simulacion.

ts = 0.02;
tf = 8;
time = 0:ts:tf;
N = length(time);

xd = zeros(3,N);
xd_dot = zeros(3,N);

for k = 1:N
    tt = time(k);
    xd(:,k) = [0.35 + 0.12*sin(1.2*tt);
               0.12*cos(1.2*tt);
               0.35 + 0.08*tt/tf];

    xd_dot(:,k) = [0.12*1.2*cos(1.2*tt);
                  -0.12*1.2*sin(1.2*tt);
                   0.08/tf];
end

%% ================================================================
% 7. CONTROL CINEMATICO POR JACOBIANO
% ================================================================
% Fuente / relacion con el paper:
% - Pag. 1, Resumen: el paper implementa control de seguimiento de trayectoria
%   en MATLAB/Simulink.
% - Pag. 19, Seccion 4, Ecs. (20)-(22): el paper analiza estabilidad usando
%   Lyapunov para su controlador ST-FSMC.
% Desarrollo propio:
% - El paper trabaja principalmente control dinamico/no lineal basado en
%   ST-FSMC, PID, SMC y FSMC.
% - Para cubrir explicitamente el requisito de control cinematico del parcial,
%   se implementa aqui una ley cartesiana basada en Jacobiano:
%       qdot = pinv(J(q)) * (xd_dot + K*(xd - x))
%
% Esta es una ley de control cinematico de seguimiento en espacio cartesiano.
% Se usa pseudoinversa amortiguada para evitar explosion numerica cerca de
% singularidades.

K = diag([4, 4, 4]);      % ganancia cartesiana
q_ctrl = zeros(3,N);
x_real = zeros(3,N);
err = zeros(3,N);

% Condicion inicial calculada por IK para el primer punto deseado
[q0_up, q0_down, ok] = ik_3dof(xd(:,1), robot);
if ~ok
    error('El punto inicial de la trayectoria esta fuera del workspace.');
end
q_ctrl(:,1) = q0_up;

for k = 1:N-1
    [~, xk, ~] = fk_3dof(q_ctrl(:,k), robot);
    x_real(:,k) = xk;
    err(:,k) = xd(:,k) - xk;

    Jk = jacobian_3dof(q_ctrl(:,k), robot);

    % Factor de amortiguamiento para robustez cerca de singularidades
    lambda = 0.01;
    J_damped_pinv = Jk'/(Jk*Jk' + lambda^2*eye(3));

    qdot_cmd = J_damped_pinv * (xd_dot(:,k) + K*err(:,k));

    % Integracion Euler
    q_ctrl(:,k+1) = q_ctrl(:,k) + qdot_cmd*ts;
end

[~, x_real(:,N), ~] = fk_3dof(q_ctrl(:,N), robot);
err(:,N) = xd(:,N) - x_real(:,N);

%% ================================================================
% 8. GRAFICAS DE RESULTADOS
% ================================================================

figure('Name','Seguimiento de trayectoria 3D');
plot3(xd(1,:), xd(2,:), xd(3,:), '--', 'LineWidth', 1.5); hold on;
plot3(x_real(1,:), x_real(2,:), x_real(3,:), 'LineWidth', 1.5);
grid on; axis equal;
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
title('Control cinematico: trayectoria deseada vs trayectoria real');
legend('Deseada','Real');

figure('Name','Errores cartesianos');
plot(time, err(1,:), 'LineWidth', 1.2); hold on;
plot(time, err(2,:), 'LineWidth', 1.2);
plot(time, err(3,:), 'LineWidth', 1.2);
grid on;
xlabel('Tiempo [s]'); ylabel('Error [m]');
title('Error de seguimiento cartesiano');
legend('e_x','e_y','e_z');

figure('Name','Variables articulares');
plot(time, rad2deg(q_ctrl(1,:)), 'LineWidth', 1.2); hold on;
plot(time, rad2deg(q_ctrl(2,:)), 'LineWidth', 1.2);
plot(time, rad2deg(q_ctrl(3,:)), 'LineWidth', 1.2);
grid on;
xlabel('Tiempo [s]'); ylabel('Angulo [deg]');
title('Evolucion de variables articulares');
legend('q1','q2','q3');

figure('Name','Animacion simple del robot');
for k = 1:20:N
    draw_robot_3dof(q_ctrl(:,k), robot);
    title(sprintf('Robot 3GDL - t = %.2f s', time(k)));
    pause(0.01);
end

fprintf('\n================ RESUMEN FINAL ================\n');
fprintf('Error RMS X = %.6f m\n', rms(err(1,:)));
fprintf('Error RMS Y = %.6f m\n', rms(err(2,:)));
fprintf('Error RMS Z = %.6f m\n', rms(err(3,:)));
fprintf('Simulacion finalizada correctamente.\n');

%% ================================================================
% FUNCIONES LOCALES
% ================================================================

function A = dh_standard(theta, d, a, alpha)
    % Matriz DH estandar usada segun Pag. 6, Seccion 3.1 y Tabla 1 del paper.
    % Convencion: RotZ(theta)*TransZ(d)*TransX(a)*RotX(alpha).
    ct = cos(theta); st = sin(theta);
    ca = cos(alpha); sa = sin(alpha);

    A = [ct, -st*ca,  st*sa, a*ct;
         st,  ct*ca, -ct*sa, a*st;
          0,     sa,     ca,    d;
          0,      0,      0,    1];
end

function [T03, p, R] = fk_3dof(q, robot)
    % Cinematica directa del robot antropomorfico 3R usando DH del paper.
    % Fuente: Pag. 6, Tabla 1 para parametros DH; Pag. 7, Ecs. (4)-(5)
    % para matriz T03 y posicion del efector final.
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
    % Cinematica inversa geometrica para el robot 3R.
    % Fuente: Pag. 7, Seccion 3.2, Ecs. (6)-(8) del paper.
    % Se implementan las dos soluciones geometricas: codo arriba/codo abajo.
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

    reachable = true;

    % Solucion codo arriba
    q3_up = atan2(sqrt(1 - c3^2), c3);
    q2_up = atan2(s, r) - atan2(L3*sin(q3_up), L2 + L3*cos(q3_up));

    % Solucion codo abajo
    q3_down = atan2(-sqrt(1 - c3^2), c3);
    q2_down = atan2(s, r) - atan2(L3*sin(q3_down), L2 + L3*cos(q3_down));

    q_up = [q1; q2_up; q3_up];
    q_down = [q1; q2_down; q3_down];
end

function J = jacobian_3dof(q, robot)
    % Jacobiano translacional del efector final.
    % Desarrollo propio: el paper no reporta explicitamente J(q).
    % Se deriva de px, py, pz reportados en Pag. 7, Ec. (5):
    %   px = L3*C1*C23 + L2*C1*C2
    %   py = L3*S1*C23 + L2*S1*C2
    %   pz = L3*S23 + L2*S2 + L1
    L2 = robot.L2; L3 = robot.L3;
    q1 = q(1); q2 = q(2); q3 = q(3);

    C1 = cos(q1); S1 = sin(q1);
    C2 = cos(q2); S2 = sin(q2);
    C23 = cos(q2 + q3); S23 = sin(q2 + q3);

    J = [ -L3*S1*C23 - L2*S1*C2,  -L3*C1*S23 - L2*C1*S2,  -L3*C1*S23;
           L3*C1*C23 + L2*C1*C2,  -L3*S1*S23 - L2*S1*S2,  -L3*S1*S23;
           0,                       L3*C23 + L2*C2,          L3*C23];
end

function draw_robot_3dof(q, robot)
    % Dibuja el robot 3GDL usando las matrices DH.
    % Fuente geometrica: Pag. 6, Fig. 1 y Tabla 1 del paper.
    L1 = robot.L1; L2 = robot.L2; L3 = robot.L3;

    A01 = dh_standard(q(1), L1, 0,  pi/2);
    A12 = dh_standard(q(2), 0,  L2, 0);
    A23 = dh_standard(q(3), 0,  L3, 0);

    T00 = eye(4);
    T01 = A01;
    T02 = A01*A12;
    T03 = A01*A12*A23;

    P0 = T00(1:3,4);
    P1 = T01(1:3,4);
    P2 = T02(1:3,4);
    P3 = T03(1:3,4);

    P = [P0 P1 P2 P3];

    plot3(P(1,:), P(2,:), P(3,:), '-o', 'LineWidth', 2, 'MarkerSize', 6);
    grid on; axis equal;
    xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
    xlim([-0.8 0.8]); ylim([-0.8 0.8]); zlim([0 1.2]);
    view(45,25);
end
