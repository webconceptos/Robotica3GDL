%% ROBOT ANTROPOMORFICO 3GDL - TRABAJO FINAL v4 (A* Y OBSTACULOS)
% -------------------------------------------------------------------------
% Curso      : Robotica y Sistemas Autonomos
% Entregable : Version final integrada del Trabajo Final
% Archivo    : robot3dof_TFinal_v4_astar_obstaculos.m
%
% Punto de partida:
% Esta version parte de robot3dof_TFinal_v3_controladores.m (cinematica +
% dinamica + tres controladores ya validados sobre una trayectoria
% punto-a-punto simple).
%
% Alcance de esta version (bloque "planeacion con obstaculos", version final):
%   A) Cinematica, dinamica y controladores heredados de v2/v3 (sin cambios
%      en las leyes de control).
%   B) Planeacion autonoma en el plano cartesiano XZ mediante A*, con
%      obstaculos circulares.
%   C) Conversion de la ruta A* a trayectoria articular qd(t) mediante
%      cinematica inversa e interpolacion (pchip).
%   D) Los tres controladores siguen la MISMA trayectoria planeada, lo cual
%      permite una comparacion justa incluyendo evasion de obstaculos.
%   E) Metricas comparativas y graficas completas.
%
% Esta es la version que integra todos los bloques del trabajo final y es
% funcionalmente equivalente a robot3dof_TFinal_v1.m (la version preliminar
% integrada usada para la primera presentacion), pero documentada siguiendo
% la progresion incremental v2 -> v3 -> v4 del enunciado del trabajo final.
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
% [P3] Paper, Pag. 7, Ecs. (4)-(5): matriz homogenea T03 y posicion.
% [P4] Paper, Pag. 7, Seccion 3.2, Ecs. (6)-(8): cinematica inversa.
% [P5] Paper, Pag. 8, Tabla 2: L1=0.15 m, L2=0.50 m, L3=0.50 m.
%
% Notas metodologicas IMPORTANTES:
% 1) El paper base no reporta masas, inercias ni centros de masa completos.
%    Se usan parametros fisicos asumidos y declarados explicitamente
%    (Supuesto para simulacion: el paper base no reporta masa/inercia
%    completa).
% 2) La planeacion se realiza en el plano XZ manteniendo Y constante, para
%    facilitar la visualizacion; es una decision defendible para un robot
%    3GDL que debe posicionar su efector evitando obstaculos.
%
% Preguntas pendientes al docente:
%   Q1. ¿Se acepta usar parametros fisicos asumidos y justificados para
%       implementar M(q), C(q,dq), G(q)?
%   Q2. ¿La planeacion autonoma con obstaculos puede realizarse en espacio
%       cartesiano con A* y luego convertirse a espacio articular mediante
%       cinematica inversa?
% -------------------------------------------------------------------------

clc; clear; close all;

%% ================================================================
% 1. PARAMETROS GEOMETRICOS Y FISICOS DEL ROBOT
% Objetivo: mismos parametros que v2/v3, reutilizados para que este archivo
%           corra de forma independiente.
% Fuente/justificacion: geometria del paper base [P5]; masas/inercias
%           asumidas para simulacion (ver nota metodologica 1).
% Resultado esperado: estructura "robot" identica a las versiones previas.
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
fprintf(' ROBOT ANTROPOMORFICO 3GDL - TRABAJO FINAL v4 (A* Y OBSTACULOS)\n');
fprintf('============================================================\n');
fprintf('Geometria: L1=%.2f m, L2=%.2f m, L3=%.2f m\n\n', robot.L1, robot.L2, robot.L3);

%% ================================================================
% 2. PLANEACION AUTONOMA CON OBSTACULOS - A* EN PLANO XZ
% Objetivo: generar una ruta cartesiana libre de colisiones entre un punto
%           de inicio y una meta, evitando obstaculos circulares.
% Fuente/justificacion: A* 8-conectado sobre grilla de ocupacion en el
%           plano XZ (Y constante), decision defendible y facil de
%           visualizar para un robot 3GDL (ver nota metodologica 2).
% Resultado esperado: path_xz sin colisiones con los obstaculos definidos.
%% ================================================================
planner.y_const = 0.12;      % plano de trabajo Y constante [m]
planner.xlim = [0.20 0.82];  % limites del mapa [m]
planner.zlim = [0.16 0.78];  % limites del mapa [m]
planner.res  = 0.02;         % resolucion de grilla [m]

start_xz = [0.35 0.25];      % [x z]
goal_xz  = [0.70 0.65];      % [x z]

obstacles = [0.48 0.40 0.07;
             0.60 0.52 0.06];

[path_xz, map] = astar_plan_xz(start_xz, goal_xz, obstacles, planner);

path_cart = [path_xz(:,1)';
             planner.y_const*ones(1,size(path_xz,1));
             path_xz(:,2)'];

fprintf('================ PLANEACION AUTONOMA ================\n');
fprintf('Waypoints A*: %d\n', size(path_cart,2));

figure('Name','Planeacion autonoma A* con obstaculos');
plot_map_and_path(map, planner, obstacles, path_xz, start_xz, goal_xz);
title('Planeacion autonoma en plano XZ usando A*');

%% ================================================================
% 3. CONVERSION DE RUTA CARTESIANA A TRAYECTORIA ARTICULAR
% Objetivo: convertir cada punto de la ruta A* en una configuracion
%           articular qd(t), manteniendo continuidad de rama (up/down) de
%           la cinematica inversa, y luego suavizar con interpolacion.
% Fuente/justificacion: cinematica inversa geometrica del parcial [P4];
%           continuidad elegida por minima distancia articular entre
%           soluciones "up" y "down" consecutivas.
% Resultado esperado: qd(t), dqd(t), ddqd(t) que reproducen la ruta A* en
%           el espacio articular.
%% ================================================================
[q_waypoints, reachable_flags] = cartesian_path_to_joint_path(path_cart, robot);
if any(~reachable_flags)
    warning('Algunos waypoints no son alcanzables. Se conservaran los alcanzables.');
    q_waypoints = q_waypoints(:, reachable_flags);
    path_cart = path_cart(:, reachable_flags);
end
fprintf('Waypoints alcanzables por IK: %d\n', size(q_waypoints,2));

dt = 0.01;       % paso de simulacion [s]
tf = 8.0;        % duracion total [s]
traj = make_joint_trajectory(q_waypoints, tf, dt);

q0 = traj.qd(:,1) + deg2rad([8; -6; 5]); % pequeno error inicial
qdot0 = [0; 0; 0];

%% ================================================================
% 4. GANANCIAS DE CONTROL
% Objetivo: mismas ganancias validadas en v3, reutilizadas aqui sobre la
%           trayectoria con obstaculos.
% Resultado esperado: los tres controladores permanecen estables al seguir
%           la ruta planeada por A*.
%% ================================================================
gains_pid.Kp = diag([80 90 70]);
gains_pid.Kd = diag([18 20 15]);
gains_pid.Ki = diag([8 8 6]);

gains_pd.Kp = diag([90 100 80]);
gains_pd.Kd = diag([20 22 18]);

gains_ct.Kp = diag([120 130 100]);
gains_ct.Kd = diag([25 28 22]);

%% ================================================================
% 5. SIMULACION DE LOS TRES CONTROLADORES SOBRE LA RUTA CON OBSTACULOS
% Objetivo: comparar PID no lineal, PD precompensado y par calculado
%           siguiendo la trayectoria generada a partir de la ruta A*.
% Resultado esperado: tres estructuras res_* con q, qdot, tau, err.
%% ================================================================
fprintf('\n================ SIMULANDO CONTROLADORES ================\n');
fprintf('1/3 PID no lineal...\n');
res_pid = simulate_robot_controller('PID_NO_LINEAL', robot, traj, q0, qdot0, gains_pid);

fprintf('2/3 PD con precompensacion...\n');
res_pd = simulate_robot_controller('PD_PRECOMP', robot, traj, q0, qdot0, gains_pd);

fprintf('3/3 Control por par calculado...\n');
res_ct = simulate_robot_controller('PAR_CALCULADO', robot, traj, q0, qdot0, gains_ct);

%% ================================================================
% 6. METRICAS COMPARATIVAS
% Objetivo: cuantificar desempeno de cada controlador sobre la trayectoria
%           con obstaculos.
% Resultado esperado: tabla con error RMS/max, torque RMS/max y error final.
%% ================================================================
metrics = [compute_metrics(res_pid, traj, 'PID no lineal');
           compute_metrics(res_pd,  traj, 'PD precompensado');
           compute_metrics(res_ct,  traj, 'Par calculado')];

disp('================ METRICAS COMPARATIVAS ================');
Tmetrics = struct2table(metrics);
disp(Tmetrics);

%% ================================================================
% 7. GRAFICAS FINALES PARA LA PRESENTACION
% Objetivo: generar todas las graficas minimas exigidas por el enunciado
%           del trabajo final (seguimiento, error, torque, mapa+ruta,
%           trayectoria cartesiana del efector con obstaculos).
% Resultado esperado: conjunto completo de figuras para el informe/PPT.
%% ================================================================
plot_joint_tracking(traj, res_pid, res_pd, res_ct);
plot_error_norms(traj, res_pid, res_pd, res_ct);
plot_torques(traj, res_pid, res_pd, res_ct);
plot_ee_paths(robot, traj, res_pid, res_pd, res_ct, obstacles, planner);

fprintf('\n================ RESUMEN v4 (VERSION FINAL) ================\n');
fprintf('Se integraron dinamica, los tres controladores y planeacion A* con obstaculos.\n');
fprintf('Pendiente por confirmar con docente: parametros fisicos asumidos y obligatoriedad de Simulink.\n');
fprintf('Simulacion final completada correctamente.\n');

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
    % Matriz de inercia aproximada. Supuestos: q1 giro de base; q2-q3
    % manipulador planar 2R; se ignoran acoplamientos yaw/plano.
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
    % Control de posicion: PID + compensacion gravitacional G(q), que
    % introduce el caracter no lineal del controlador.
    e = qd - q;
    de = qd_dot - qdot;
    tau = gains.Kp*e + gains.Kd*de + gains.Ki*eint + gravity_vector_3dof(q, robot);
end

function tau = control_pd_precomp(q, qdot, qd, qd_dot, qd_ddot, robot, gains)
    % PD con precompensacion: feedforward M,C,G evaluados en la trayectoria
    % deseada, mas realimentacion PD del error.
    e = qd - q;
    de = qd_dot - qdot;
    Mqd = inertia_matrix_3dof(qd, robot);
    Cqd = coriolis_matrix_3dof(qd, qd_dot, robot);
    Gqd = gravity_vector_3dof(qd, robot);
    tau_ff = Mqd*qd_ddot + Cqd*qd_dot + Gqd;
    tau = tau_ff + gains.Kp*e + gains.Kd*de;
end

function tau = control_computed_torque(q, qdot, qd, qd_dot, qd_ddot, robot, gains)
    % Control por par calculado: se usa el modelo dinamico completo
    % evaluado en el estado real (q, qdot), lo que permite cancelar la
    % dinamica no lineal del robot.
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
        q(:,k+1) = wrap_to_pi_local(q(:,k+1));
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
% FUNCIONES LOCALES - PLANEACION A*
% ================================================================

function [path_xz, map] = astar_plan_xz(start_xz, goal_xz, obstacles, planner)
    xs = planner.xlim(1):planner.res:planner.xlim(2);
    zs = planner.zlim(1):planner.res:planner.zlim(2);
    [X,Z] = meshgrid(xs,zs);
    occ = false(size(X));

    for i = 1:size(obstacles,1)
        cx = obstacles(i,1); cz = obstacles(i,2); r = obstacles(i,3);
        occ = occ | ((X-cx).^2 + (Z-cz).^2 <= r^2);
    end

    start_idx = world_to_grid(start_xz, xs, zs);
    goal_idx = world_to_grid(goal_xz, xs, zs);

    occ(start_idx(2), start_idx(1)) = false;
    occ(goal_idx(2), goal_idx(1)) = false;

    path_idx = astar_grid(occ, start_idx, goal_idx);
    if isempty(path_idx)
        warning('A* no encontro ruta. Se usara interpolacion directa como respaldo.');
        path_xz = [linspace(start_xz(1), goal_xz(1), 25)', linspace(start_xz(2), goal_xz(2), 25)'];
    else
        path_xz = [xs(path_idx(:,1))', zs(path_idx(:,2))'];
        path_xz = simplify_path(path_xz, 2); % reduce puntos para interpolacion mas suave
    end

    map.xs = xs;
    map.zs = zs;
    map.occ = occ;
end

function idx = world_to_grid(p, xs, zs)
    [~, ix] = min(abs(xs - p(1)));
    [~, iz] = min(abs(zs - p(2)));
    idx = [ix iz];
end

function path = astar_grid(occ, start_idx, goal_idx)
    % A* 8-conectado sobre una matriz de ocupacion.
    [nz,nx] = size(occ);
    start_key = sub2ind([nz,nx], start_idx(2), start_idx(1));
    goal_key  = sub2ind([nz,nx], goal_idx(2),  goal_idx(1));

    gscore = inf(nz,nx);
    fscore = inf(nz,nx);
    came = zeros(nz,nx);
    open = false(nz,nx);
    closed = false(nz,nx);

    gscore(start_idx(2),start_idx(1)) = 0;
    fscore(start_idx(2),start_idx(1)) = heuristic(start_idx, goal_idx);
    open(start_idx(2),start_idx(1)) = true;

    neigh = [-1 -1; 0 -1; 1 -1; -1 0; 1 0; -1 1; 0 1; 1 1];

    while any(open(:))
        tmp = fscore;
        tmp(~open) = inf;
        [~, current_key] = min(tmp(:));
        [cy,cx] = ind2sub([nz,nx], current_key);

        if current_key == goal_key
            path = reconstruct_path(came, current_key, [nz,nx]);
            return;
        end

        open(cy,cx) = false;
        closed(cy,cx) = true;

        for i = 1:size(neigh,1)
            nx_i = cx + neigh(i,1);
            ny_i = cy + neigh(i,2);
            if nx_i < 1 || nx_i > nx || ny_i < 1 || ny_i > nz
                continue;
            end
            if occ(ny_i,nx_i) || closed(ny_i,nx_i)
                continue;
            end
            step_cost = norm(neigh(i,:));
            tentative_g = gscore(cy,cx) + step_cost;
            if ~open(ny_i,nx_i)
                open(ny_i,nx_i) = true;
            elseif tentative_g >= gscore(ny_i,nx_i)
                continue;
            end
            came(ny_i,nx_i) = current_key;
            gscore(ny_i,nx_i) = tentative_g;
            fscore(ny_i,nx_i) = tentative_g + heuristic([nx_i ny_i], goal_idx);
        end
    end

    path = [];
end

function h = heuristic(a,b)
    h = norm(a-b);
end

function path = reconstruct_path(came, current_key, dims)
    keys = current_key;
    while came(current_key) ~= 0
        current_key = came(current_key);
        keys = [current_key; keys]; %#ok<AGROW>
    end
    path = zeros(length(keys),2);
    for i = 1:length(keys)
        [y,x] = ind2sub(dims, keys(i));
        path(i,:) = [x y];
    end
end

function p2 = simplify_path(p, stride)
    if size(p,1) <= 2
        p2 = p;
        return;
    end
    idx = unique([1:stride:size(p,1), size(p,1)]);
    p2 = p(idx,:);
end

function [q_path, reachable] = cartesian_path_to_joint_path(path_cart, robot)
    N = size(path_cart,2);
    q_path = zeros(3,N);
    reachable = false(1,N);
    q_prev = [];
    for k = 1:N
        [q_up, q_down, ok] = ik_3dof(path_cart(:,k), robot);
        reachable(k) = ok;
        if ~ok
            q_path(:,k) = [NaN;NaN;NaN];
            continue;
        end
        if isempty(q_prev)
            q_sel = q_up;
        else
            if norm(wrap_to_pi_local(q_up - q_prev)) <= norm(wrap_to_pi_local(q_down - q_prev))
                q_sel = q_up;
            else
                q_sel = q_down;
            end
        end
        q_path(:,k) = q_sel;
        q_prev = q_sel;
    end
end

function traj = make_joint_trajectory(q_waypoints, tf, dt)
    nwp = size(q_waypoints,2);
    t_wp = linspace(0, tf, nwp);
    t = 0:dt:tf;
    qd = zeros(3,length(t));
    for i = 1:3
        qd(i,:) = interp1(t_wp, q_waypoints(i,:), t, 'pchip');
    end
    qd_dot = zeros(size(qd));
    qd_ddot = zeros(size(qd));
    for i = 1:3
        qd_dot(i,:) = gradient(qd(i,:), dt);
        qd_ddot(i,:) = gradient(qd_dot(i,:), dt);
    end
    traj.t = t;
    traj.dt = dt;
    traj.qd = qd;
    traj.qd_dot = qd_dot;
    traj.qd_ddot = qd_ddot;
end

%% ================================================================
% FUNCIONES LOCALES - GRAFICAS
% ================================================================

function plot_map_and_path(map, planner, obstacles, path_xz, start_xz, goal_xz)
    imagesc(map.xs, map.zs, map.occ); set(gca,'YDir','normal'); hold on;
    plot(path_xz(:,1), path_xz(:,2), 'w-', 'LineWidth', 2);
    plot(start_xz(1), start_xz(2), 'go', 'MarkerSize', 9, 'LineWidth', 2);
    plot(goal_xz(1), goal_xz(2), 'rx', 'MarkerSize', 10, 'LineWidth', 2);
    for i = 1:size(obstacles,1)
        th = linspace(0,2*pi,100);
        plot(obstacles(i,1)+obstacles(i,3)*cos(th), obstacles(i,2)+obstacles(i,3)*sin(th), 'k-', 'LineWidth', 1.5);
    end
    xlabel('X [m]'); ylabel('Z [m]'); grid on;
    xlim(planner.xlim); ylim(planner.zlim);
    legend('Ruta A*','Inicio','Meta','Obstaculos','Location','best');
end

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

function plot_ee_paths(robot, traj, res_pid, res_pd, res_ct, obstacles, planner)
    xd = joint_series_to_cartesian(robot, traj.qd);
    x_pid = joint_series_to_cartesian(robot, res_pid.q);
    x_pd  = joint_series_to_cartesian(robot, res_pd.q);
    x_ct  = joint_series_to_cartesian(robot, res_ct.q);

    figure('Name','Trayectoria cartesiana del efector final');
    plot3(xd(1,:), xd(2,:), xd(3,:), 'k--', 'LineWidth', 1.8); hold on;
    plot3(x_pid(1,:), x_pid(2,:), x_pid(3,:), 'LineWidth', 1.2);
    plot3(x_pd(1,:),  x_pd(2,:),  x_pd(3,:),  'LineWidth', 1.2);
    plot3(x_ct(1,:),  x_ct(2,:),  x_ct(3,:),  'LineWidth', 1.2);

    % Obstaculos dibujados como cilindros aproximados en el plano XZ.
    for i = 1:size(obstacles,1)
        [Xc,Yc,Zc] = cylinder(obstacles(i,3), 40);
        Xc = Xc + obstacles(i,1);
        Yc = 0.02*Yc + planner.y_const - 0.01;
        Zc = Zc*(planner.zlim(2)-planner.zlim(1)) + planner.zlim(1);
        surf(Xc,Yc,Zc, 'FaceAlpha',0.15, 'EdgeAlpha',0.1);
    end

    grid on; axis equal;
    xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
    title('Trayectoria cartesiana: ruta planeada y seguimiento');
    legend('Deseada A*','PID no lineal','PD precomp','Par calculado','Obstaculos','Location','best');
    view(45,25);
end

function X = joint_series_to_cartesian(robot, Q)
    N = size(Q,2);
    X = zeros(3,N);
    for k = 1:N
        [~, p, ~] = fk_3dof(Q(:,k), robot);
        X(:,k) = p;
    end
end

%% ================================================================
% UTILIDADES
% ================================================================

function y = wrap_to_pi_local(x)
    y = mod(x + pi, 2*pi) - pi;
end
