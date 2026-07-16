%% ROBOT ANTROPOMORFICO 3GDL - TRABAJO FINAL v3 (CONTROLADORES)
% -------------------------------------------------------------------------
% Curso      : Robotica y Sistemas Autonomos
% Entregable : Bloque de desarrollo - Controladores dinamicos
% Archivo    : robot3dof_TFinal_v3_controladores.m
%
% Punto de partida:
% Esta version parte de robot3dof_TFinal_v2_dinamica.m (cinematica +
% modelo dinamico M(q), C(q,dq), G(q) ya validados).
%
% Alcance de esta version (bloque "controladores"):
%   A) Cinematica y dinamica heredadas de v2 (sin cambios).
%   B) Trayectoria articular de referencia punto-a-punto (polinomio
%      quintico), usada UNICAMENTE para poner a prueba los controladores.
%      La planeacion con obstaculos (A*) se agrega en v4.
%   C) Tres controladores dinamicos:
%      - PID no lineal (Kp*e + Kd*de + Ki*int(e) + G(q)).
%      - PD con precompensacion (feedforward M,C,G evaluados en qd).
%      - Control por par calculado (linealizacion por realimentacion).
%   D) Metricas y graficas comparativas entre los tres controladores.
%
% Fuera de alcance en esta version (se agrega en v4):
%   - Planeacion autonoma con obstaculos (A*) en el plano cartesiano XZ.
%
% Paper base del parcial:
% Ashagrie, A., Salau, A. O., & Weldcherkos, T. (2021).
% Modeling and control of a 3-DOF articulated robotic manipulator using
% self-tuning fuzzy sliding mode controller. Cogent Engineering.
%
% Nota metodologica: masas, centros de masa e inercias son supuestos de
% simulacion (el paper base no los reporta completos). Ver robot.* abajo.
% -------------------------------------------------------------------------

clc; clear; close all;

%% ================================================================
% 1. PARAMETROS GEOMETRICOS Y FISICOS DEL ROBOT
% Objetivo: mismos parametros que v2, reutilizados aqui para que este
%           archivo corra de forma independiente.
% Fuente/justificacion: geometria del paper base; masas/inercias asumidas
%           (Supuesto para simulacion: el paper base no reporta masa/inercia
%           completa).
% Resultado esperado: estructura "robot" identica a v2_dinamica.m.
%% ================================================================
robot.L1 = 0.15;        % altura/base [m]
robot.L2 = 0.50;        % longitud eslabon 2 [m]
robot.L3 = 0.50;        % longitud eslabon 3 [m]

robot.m1 = 2.00;        % masa equivalente base/eslabon 1 [kg]
robot.m2 = 1.50;        % masa eslabon 2 [kg]
robot.m3 = 1.00;        % masa eslabon 3 [kg]
robot.lc2 = robot.L2/2; % centro de masa eslabon 2 [m]
robot.lc3 = robot.L3/2; % centro de masa eslabon 3 [m]
robot.I1 = 0.030;       % inercia equivalente junta 1 [kg*m^2]
robot.I2 = robot.m2*robot.L2^2/12; % inercia barra eslabon 2 [kg*m^2]
robot.I3 = robot.m3*robot.L3^2/12; % inercia barra eslabon 3 [kg*m^2]
robot.g  = 9.81;        % gravedad [m/s^2]

robot.tau_max = [80; 80; 60]; % [N*m] proteccion numerica de saturacion

fprintf('============================================================\n');
fprintf(' ROBOT ANTROPOMORFICO 3GDL - TRABAJO FINAL v3 (CONTROLADORES)\n');
fprintf('============================================================\n');
fprintf('Geometria: L1=%.2f m, L2=%.2f m, L3=%.2f m\n\n', robot.L1, robot.L2, robot.L3);

%% ================================================================
% 2. TRAYECTORIA ARTICULAR DE REFERENCIA (PUNTO A PUNTO)
% Objetivo: generar qd(t), dqd(t), ddqd(t) suaves entre una configuracion
%           inicial y una final, para probar los controladores bajo la
%           MISMA trayectoria (comparacion justa).
% Fuente/justificacion: polinomio quintico con velocidad y aceleracion nulas
%           en los extremos; es una eleccion estandar de generacion de
%           trayectoria punto-a-punto en robotica.
% Resultado esperado: qd_dot(0)=qd_dot(tf)=0 y qd_ddot(0)=qd_ddot(tf)=0.
%% ================================================================
q_start = deg2rad([10; 25; -20]);
q_goal  = deg2rad([-25; 55; -45]);

dt = 0.01;
tf = 5.0;
traj = make_quintic_trajectory(q_start, q_goal, tf, dt);

q0 = traj.qd(:,1) + deg2rad([8; -6; 5]); % pequeno error inicial
qdot0 = [0; 0; 0];

fprintf('================ TRAYECTORIA DE REFERENCIA ================\n');
fprintf('q_start [deg] = [%.2f %.2f %.2f]\n', rad2deg(q_start));
fprintf('q_goal  [deg] = [%.2f %.2f %.2f]\n', rad2deg(q_goal));
fprintf('Duracion: %.1f s, dt = %.3f s\n\n', tf, dt);

%% ================================================================
% 3. GANANCIAS DE CONTROL
% Objetivo: definir ganancias independientes para cada controlador.
% Fuente/justificacion: ganancias preliminares ajustadas empiricamente
%           para lograr estabilidad y seguimiento razonable sobre esta
%           trayectoria; deben afinarse segun desempeno.
% Resultado esperado: los tres controladores estabilizan el error hacia 0.
%% ================================================================
gains_pid.Kp = diag([80 90 70]);
gains_pid.Kd = diag([18 20 15]);
gains_pid.Ki = diag([8 8 6]);

gains_pd.Kp = diag([90 100 80]);
gains_pd.Kd = diag([20 22 18]);

gains_ct.Kp = diag([120 130 100]);
gains_ct.Kd = diag([25 28 22]);

%% ================================================================
% 4. SIMULACION DE LOS TRES CONTROLADORES
% Objetivo: simular PID no lineal, PD precompensado y par calculado sobre
%           la misma trayectoria qd(t) y con las mismas condiciones
%           iniciales q0, qdot0.
% Fuente/justificacion: leyes de control PID no lineal, PD precompensado y
%           par calculado, segun el enunciado del trabajo final.
% Resultado esperado: tres estructuras res_* con q, qdot, tau, err.
%% ================================================================
fprintf('================ SIMULANDO CONTROLADORES ================\n');
fprintf('1/3 PID no lineal...\n');
res_pid = simulate_robot_controller('PID_NO_LINEAL', robot, traj, q0, qdot0, gains_pid);

fprintf('2/3 PD con precompensacion...\n');
res_pd = simulate_robot_controller('PD_PRECOMP', robot, traj, q0, qdot0, gains_pd);

fprintf('3/3 Control por par calculado...\n');
res_ct = simulate_robot_controller('PAR_CALCULADO', robot, traj, q0, qdot0, gains_ct);

%% ================================================================
% 5. METRICAS COMPARATIVAS
% Objetivo: cuantificar desempeno de cada controlador con las metricas
%           obligatorias del trabajo final.
% Resultado esperado: tabla con error RMS/max, torque RMS/max y error final
%           por controlador.
%% ================================================================
metrics = [compute_metrics(res_pid, traj, 'PID no lineal');
           compute_metrics(res_pd,  traj, 'PD precompensado');
           compute_metrics(res_ct,  traj, 'Par calculado')];

disp('================ METRICAS COMPARATIVAS ================');
Tmetrics = struct2table(metrics);
disp(Tmetrics);

%% ================================================================
% 6. GRAFICAS COMPARATIVAS
% Objetivo: visualizar seguimiento articular, error y torque de los tres
%           controladores sobre la misma trayectoria.
% Resultado esperado: graficas equivalentes a las exigidas en los
%           resultados minimos del trabajo final (items 1-4), sin mapa de
%           obstaculos (eso es v4).
%% ================================================================
plot_joint_tracking(traj, res_pid, res_pd, res_ct);
plot_error_norms(traj, res_pid, res_pd, res_ct);
plot_torques(traj, res_pid, res_pd, res_ct);

fprintf('\n================ RESUMEN v3 ================\n');
fprintf('Controladores PID no lineal, PD precompensado y par calculado implementados y comparados.\n');
fprintf('Siguiente bloque (v4): reemplazar la trayectoria punto-a-punto por una ruta A* con obstaculos.\n');

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

%% ================================================================
% FUNCIONES LOCALES - DINAMICA
% ================================================================

function M = inertia_matrix_3dof(q, robot)
    % Matriz de inercia aproximada para robot antropomorfico 3GDL.
    % Supuestos: q1 giro de base; q2-q3 manipulador planar 2R; se ignoran
    % acoplamientos dinamicos yaw/plano (ver v2_dinamica.m para detalle).
    q2 = q(2); q3 = q(3);
    L2 = robot.L2; lc2 = robot.lc2; lc3 = robot.lc3;
    m2 = robot.m2; m3 = robot.m3;
    I1 = robot.I1; I2 = robot.I2; I3 = robot.I3;

    r2 = lc2*cos(q2);
    r3 = L2*cos(q2) + lc3*cos(q2+q3);
    M11 = I1 + m2*r2^2 + m3*r3^2;

    M22 = I2 + I3 + m2*lc2^2 + m3*(L2^2 + lc3^2 + 2*L2*lc3*cos(q3));
    M23 = I3 + m3*(lc3^2 + L2*lc3*cos(q3));
    M33 = I3 + m3*lc3^2;

    M = [M11, 0,   0;
         0,   M22, M23;
         0,   M23, M33];

    M = M + 1e-6*eye(3);
end

function C = coriolis_matrix_3dof(q, qdot, robot)
    % Matriz C aproximada del submodelo planar 2R q2-q3.
    q3 = q(3);
    dq2 = qdot(2); dq3 = qdot(3);
    h = -robot.m3*robot.L2*robot.lc3*sin(q3);

    C = zeros(3,3);
    C(2,2) = h*dq3;
    C(2,3) = h*(dq2 + dq3);
    C(3,2) = -h*dq2;
end

function G = gravity_vector_3dof(q, robot)
    % Vector de gravedad G(q) = dV/dq. La gravedad afecta q2 y q3.
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
% FUNCIONES LOCALES - CONTROLADORES
% ================================================================

function tau = control_pid_nonlinear(q, qdot, qd, qd_dot, eint, robot, gains)
    % Control de posicion: PID + compensacion gravitacional G(q).
    % El termino G(q) depende no linealmente de la configuracion articular,
    % de ahi el caracter "no lineal" del controlador.
    e = qd - q;
    de = qd_dot - qdot;
    tau = gains.Kp*e + gains.Kd*de + gains.Ki*eint + gravity_vector_3dof(q, robot);
end

function tau = control_pd_precomp(q, qdot, qd, qd_dot, qd_ddot, robot, gains)
    % PD con precompensacion: feedforward M,C,G evaluados en la trayectoria
    % deseada (qd, qd_dot, qd_ddot), mas realimentacion PD del error.
    e = qd - q;
    de = qd_dot - qdot;
    Mqd = inertia_matrix_3dof(qd, robot);
    Cqd = coriolis_matrix_3dof(qd, qd_dot, robot);
    Gqd = gravity_vector_3dof(qd, robot);
    tau_ff = Mqd*qd_ddot + Cqd*qd_dot + Gqd;
    tau = tau_ff + gains.Kp*e + gains.Kd*de;
end

function tau = control_computed_torque(q, qdot, qd, qd_dot, qd_ddot, robot, gains)
    % Control por par calculado (linealizacion por realimentacion): se usa
    % el modelo dinamico completo evaluado en el estado real (q, qdot), lo
    % que permite cancelar la dinamica no lineal del robot.
    e = qd - q;
    de = qd_dot - qdot;
    Mq = inertia_matrix_3dof(q, robot);
    Cq = coriolis_matrix_3dof(q, qdot, robot);
    Gq = gravity_vector_3dof(q, robot);
    v = qd_ddot + gains.Kd*de + gains.Kp*e;
    tau = Mq*v + Cq*qdot + Gq;
end

function tau_sat = saturate_torque(tau, robot)
    tau_sat = min(max(tau, -robot.tau_max), robot.tau_max);
end

%% ================================================================
% FUNCIONES LOCALES - TRAYECTORIA
% ================================================================

function traj = make_quintic_trajectory(q_start, q_goal, tf, dt)
    % Polinomio quintico por junta con velocidad y aceleracion nulas en los
    % extremos: q(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5.
    t = 0:dt:tf;
    N = length(t);
    qd = zeros(3,N);
    qd_dot = zeros(3,N);
    qd_ddot = zeros(3,N);
    for i = 1:3
        q0i = q_start(i); q1i = q_goal(i);
        a0 = q0i;
        a1 = 0;
        a2 = 0;
        a3 = 10*(q1i-q0i)/tf^3;
        a4 = -15*(q1i-q0i)/tf^4;
        a5 = 6*(q1i-q0i)/tf^5;
        qd(i,:)      = a0 + a1*t + a2*t.^2 + a3*t.^3 + a4*t.^4 + a5*t.^5;
        qd_dot(i,:)  = a1 + 2*a2*t + 3*a3*t.^2 + 4*a4*t.^3 + 5*a5*t.^4;
        qd_ddot(i,:) = 2*a2 + 6*a3*t + 12*a4*t.^2 + 20*a5*t.^3;
    end
    traj.t = t;
    traj.dt = dt;
    traj.qd = qd;
    traj.qd_dot = qd_dot;
    traj.qd_ddot = qd_ddot;
end

%% ================================================================
% FUNCIONES LOCALES - SIMULACION Y METRICAS
% ================================================================

function res = simulate_robot_controller(controller_name, robot, traj, q0, qdot0, gains)
    N = length(traj.t);
    dt = traj.dt;
    q = zeros(3,N);
    qdot = zeros(3,N);
    tau = zeros(3,N);
    qddot = zeros(3,N);
    eint = zeros(3,1);

    q(:,1) = q0;
    qdot(:,1) = qdot0;

    for k = 1:N-1
        qd = traj.qd(:,k);
        qd_dot = traj.qd_dot(:,k);
        qd_ddot = traj.qd_ddot(:,k);
        e = qd - q(:,k);
        eint = eint + e*dt;

        switch upper(controller_name)
            case 'PID_NO_LINEAL'
                tau(:,k) = control_pid_nonlinear(q(:,k), qdot(:,k), qd, qd_dot, eint, robot, gains);
            case 'PD_PRECOMP'
                tau(:,k) = control_pd_precomp(q(:,k), qdot(:,k), qd, qd_dot, qd_ddot, robot, gains);
            case 'PAR_CALCULADO'
                tau(:,k) = control_computed_torque(q(:,k), qdot(:,k), qd, qd_dot, qd_ddot, robot, gains);
            otherwise
                error('Controlador no reconocido: %s', controller_name);
        end

        tau(:,k) = saturate_torque(tau(:,k), robot);
        qddot(:,k) = robot_dynamics(q(:,k), qdot(:,k), tau(:,k), robot);

        % Integracion semi-implicita simple.
        qdot(:,k+1) = qdot(:,k) + qddot(:,k)*dt;
        q(:,k+1) = q(:,k) + qdot(:,k+1)*dt;
    end

    tau(:,N) = tau(:,N-1);
    qddot(:,N) = qddot(:,N-1);

    res.name = controller_name;
    res.q = q;
    res.qdot = qdot;
    res.qddot = qddot;
    res.tau = tau;
    res.err = traj.qd - q;
end

function m = compute_metrics(res, traj, label)
    e = traj.qd - res.q;
    e_norm = vecnorm(e,2,1);
    tau_norm = vecnorm(res.tau,2,1);
    m.Controlador = string(label);
    m.Error_RMS_rad = sqrt(mean(e_norm.^2));
    m.Error_Max_rad = max(e_norm);
    m.Torque_RMS_Nm = sqrt(mean(tau_norm.^2));
    m.Torque_Max_Nm = max(tau_norm);
    m.Error_Final_rad = e_norm(end);
end

%% ================================================================
% FUNCIONES LOCALES - GRAFICAS
% ================================================================

function plot_joint_tracking(traj, res_pid, res_pd, res_ct)
    names = {'q1','q2','q3'};
    for i = 1:3
        figure('Name',['Seguimiento articular ', names{i}]);
        plot(traj.t, rad2deg(traj.qd(i,:)), 'k--', 'LineWidth', 1.8); hold on;
        plot(traj.t, rad2deg(res_pid.q(i,:)), 'LineWidth', 1.2);
        plot(traj.t, rad2deg(res_pd.q(i,:)), 'LineWidth', 1.2);
        plot(traj.t, rad2deg(res_ct.q(i,:)), 'LineWidth', 1.2);
        grid on;
        xlabel('Tiempo [s]'); ylabel([names{i}, ' [deg]']);
        title(['Seguimiento de ', names{i}, ': deseado vs controladores']);
        legend('Deseado','PID no lineal','PD precomp','Par calculado','Location','best');
    end
end

function plot_error_norms(traj, res_pid, res_pd, res_ct)
    e_pid = vecnorm(traj.qd - res_pid.q, 2, 1);
    e_pd  = vecnorm(traj.qd - res_pd.q,  2, 1);
    e_ct  = vecnorm(traj.qd - res_ct.q,  2, 1);
    figure('Name','Comparacion de error articular');
    plot(traj.t, e_pid, 'LineWidth', 1.4); hold on;
    plot(traj.t, e_pd,  'LineWidth', 1.4);
    plot(traj.t, e_ct,  'LineWidth', 1.4);
    grid on;
    xlabel('Tiempo [s]'); ylabel('||e_q|| [rad]');
    title('Error articular total por controlador');
    legend('PID no lineal','PD precomp','Par calculado','Location','best');
end

function plot_torques(traj, res_pid, res_pd, res_ct)
    figure('Name','Norma de torque');
    plot(traj.t, vecnorm(res_pid.tau,2,1), 'LineWidth', 1.3); hold on;
    plot(traj.t, vecnorm(res_pd.tau,2,1),  'LineWidth', 1.3);
    plot(traj.t, vecnorm(res_ct.tau,2,1),  'LineWidth', 1.3);
    grid on;
    xlabel('Tiempo [s]'); ylabel('||tau|| [N*m]');
    title('Esfuerzo de control por controlador');
    legend('PID no lineal','PD precomp','Par calculado','Location','best');

    for i = 1:3
        figure('Name',sprintf('Torque junta %d', i));
        plot(traj.t, res_pid.tau(i,:), 'LineWidth', 1.2); hold on;
        plot(traj.t, res_pd.tau(i,:), 'LineWidth', 1.2);
        plot(traj.t, res_ct.tau(i,:), 'LineWidth', 1.2);
        grid on;
        xlabel('Tiempo [s]'); ylabel(sprintf('tau_%d [N*m]', i));
        title(sprintf('Torque en junta %d', i));
        legend('PID no lineal','PD precomp','Par calculado','Location','best');
    end
end
