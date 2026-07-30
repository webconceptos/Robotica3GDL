%% ROBOT ANTROPOMORFICO 3GDL - TRABAJO FINAL v4 (A* Y OBSTACULOS)
% -------------------------------------------------------------------------
% Curso      : Robotica y Sistemas Autonomos
% Entregable : Version final integrada del Trabajo Final
% Archivo    : robot3dof_TFinal_v4_astar_obstaculos.m
%
% Punto de partida:
% Esta version parte de robot3dof_TFinal_v3_controladores.m: cinematica,
% dinamica por Jacobianos + Christoffel (n=3), y los tres controladores ya
% validados sobre una trayectoria punto-a-punto simple (ver README.md,
% Seccion 2.6, para el resultado de referencia en Simulink).
%
% Alcance de esta version (bloque "planeacion con obstaculos", version final):
%   A) Cinematica, dinamica y controladores heredados de v2/v3 (Jacobianos +
%      Christoffel, sin cambios en las leyes de control).
%   B) Planeacion autonoma en el plano cartesiano XZ mediante A*, con
%      obstaculos circulares.
%   C) Conversion de la ruta A* a trayectoria articular qd(t) mediante
%      cinematica inversa e interpolacion (pchip).
%   D) Los tres controladores siguen la MISMA trayectoria planeada, lo cual
%      permite una comparacion justa incluyendo evasion de obstaculos.
%   E) Metricas comparativas (error ARTICULAR, confirmado por el docente) y
%      graficas completas, incluyendo el mapa de obstaculos.
%
% Esta es la version que integra todos los bloques del trabajo final.
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
% [P5] Paper, Pag. 8, Tabla 2: L1=0.15 m, L2=0.50 m, L3=0.50 m, m1=m2=m3=0.5
%      kg, g=9.81 m/s^2 (dato reportado, no supuesto).
%
% Supuestos fisicos: ver encabezado de robot3dof_TFinal_v2_dinamica_jacobianos.m
% (centros de masa lc_i=L_i/2 y radio de cilindro r=0.03 m para los tres
% eslabones, no reportados por el paper).
%
% Nota metodologica sobre la planeacion:
% La planeacion se realiza en el plano XZ manteniendo Y constante, para
% facilitar la visualizacion; es una decision defendible para un robot
% 3GDL que debe posicionar su efector evitando obstaculos.
% -------------------------------------------------------------------------

clc; clear; close all;

%% ================================================================
% 1. PARAMETROS GEOMETRICOS Y FISICOS DEL ROBOT
% Objetivo: mismos parametros que v2/v3, reutilizados para que este archivo
%           corra de forma independiente.
% Fuente/justificacion: L1,L2,L3,m1,m2,m3,g de la Tabla 2 del paper base
%           (dato reportado); centros de masa y radio de cilindro son
%           supuestos de simulacion (cilindro solido, indicacion del docente).
% Resultado esperado: estructura "robot" identica a las versiones previas.
%% ================================================================
robot.L1 = 0.15; robot.L2 = 0.50; robot.L3 = 0.50;

robot.m1 = 0.50; robot.m2 = 0.50; robot.m3 = 0.50; % Tabla 2 del paper, Pag. 8

robot.lc1 = robot.L1/2; robot.lc2 = robot.L2/2; robot.lc3 = robot.L3/2; % supuesto
robot.g = 9.81;

robot.r1 = 0.03; robot.r2 = 0.03; robot.r3 = 0.03; % [m] radio asumido (cilindro solido)
I_axial1 = 0.5*robot.m1*robot.r1^2;
I_trans1 = (1/12)*robot.m1*(3*robot.r1^2 + robot.L1^2);
I_axial2 = 0.5*robot.m2*robot.r2^2;
I_trans2 = (1/12)*robot.m2*(3*robot.r2^2 + robot.L2^2);
I_axial3 = 0.5*robot.m3*robot.r3^2;
I_trans3 = (1/12)*robot.m3*(3*robot.r3^2 + robot.L3^2);
robot.I1 = diag([I_trans1, I_axial1, I_trans1]);
robot.I2 = diag([I_axial2, I_trans2, I_trans2]);
robot.I3 = diag([I_axial3, I_trans3, I_trans3]);

robot.tau_max = [80; 80; 60]; % [N*m] proteccion numerica de saturacion

fprintf('============================================================\n');
fprintf(' ROBOT ANTROPOMORFICO 3GDL - TRABAJO FINAL v4 (A* Y OBSTACULOS)\n');
fprintf('============================================================\n');
fprintf('Geometria (paper): L1=%.2f m, L2=%.2f m, L3=%.2f m\n', robot.L1, robot.L2, robot.L3);
fprintf('Masas (paper, Tabla 2): m1=%.2f, m2=%.2f, m3=%.2f kg\n\n', robot.m1, robot.m2, robot.m3);

%% ================================================================
% 2. PLANEACION AUTONOMA CON OBSTACULOS - A* EN PLANO XZ
% Objetivo: generar una ruta cartesiana libre de colisiones entre un punto
%           de inicio y una meta, evitando obstaculos circulares.
% Fuente/justificacion: A* 8-conectado sobre grilla de ocupacion en el
%           plano XZ (Y constante), decision defendible y facil de
%           visualizar para un robot 3GDL.
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

dt = 0.01;       % paso de muestreo de la trayectoria de referencia [s]
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
% 6. METRICAS COMPARATIVAS (ERROR ARTICULAR)
% Objetivo: cuantificar desempeno de cada controlador sobre la trayectoria
%           con obstaculos, con las metricas confirmadas por el docente:
%           error ARTICULAR (no cartesiano), RMS/maximo, tiempo de
%           estabilizacion, y cual controlador converge mas rapido.
% Resultado esperado: tabla comparativa.
%% ================================================================
tol = 0.02;
m_pid = compute_metrics(res_pid, traj, 'PID no lineal', tol);
m_pd  = compute_metrics(res_pd,  traj, 'PD precompensado', tol);
m_ct  = compute_metrics(res_ct,  traj, 'Par calculado', tol);

metrics = [m_pid; m_pd; m_ct];
disp('================ METRICAS COMPARATIVAS (error articular) ================');
Tmetrics = struct2table(metrics);
disp(Tmetrics);

[~, idx_best] = min([m_pid.TiempoEstabilizacion_s, m_pd.TiempoEstabilizacion_s, m_ct.TiempoEstabilizacion_s]);
nombres = {'PID no lineal','PD precompensado','Par calculado'};
fprintf('Controlador que converge mas rapido a cero: %s\n', nombres{idx_best});

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
fprintf('Se integraron dinamica por Jacobianos, los tres controladores y planeacion A* con obstaculos.\n');
fprintf('Simulacion final completada correctamente.\n');

%% ================================================================
% FUNCIONES LOCALES - CINEMATICA
% ================================================================

function A = dh_standard(theta, d, a, alpha)
    ct = cos(theta); st = sin(theta);
    ca = cos(alpha); sa = sin(alpha);
    A = [ct, -st*ca,  st*sa, a*ct;
         st,  ct*ca, -ct*sa, a*st;
          0,     sa,     ca,    d;
          0,      0,      0,    1];
end

function [T03, p, R] = fk_3dof(q, robot)
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

%% ================================================================
% FUNCIONES LOCALES - DINAMICA POR JACOBIANOS (identicas a v2/v3)
% ================================================================

function [pc1, pc2, pc3, Jv1, Jv2, Jv3, Jw1, Jw2, Jw3, R1, R2, R3] = com_kinematics_3dof(q, robot)
    L1 = robot.L1; L2 = robot.L2;
    lc1 = robot.lc1; lc2 = robot.lc2; lc3 = robot.lc3;
    q1 = q(1); q2 = q(2); q3 = q(3);
    C1 = cos(q1); S1 = sin(q1);
    C2 = cos(q2); S2 = sin(q2);
    C23 = cos(q2+q3); S23 = sin(q2+q3);

    z0 = [0;0;1];
    zjoint23 = [S1; -C1; 0];

    R1 = [C1, 0, S1; S1, 0, -C1; 0, 1, 0];
    R2 = [C1*C2, -C1*S2, S1; S1*C2, -S1*S2, -C1; S2, C2, 0];
    R3 = [C1*C23, -C1*S23, S1; S1*C23, -S1*S23, -C1; S23, C23, 0];

    pc1 = [0; 0; lc1];
    pc2 = [lc2*C1*C2; lc2*S1*C2; L1 + lc2*S2];
    pc3 = [L2*C1*C2 + lc3*C1*C23; L2*S1*C2 + lc3*S1*C23; L1 + L2*S2 + lc3*S23];

    Jv1 = zeros(3,3);
    Jv2 = [-lc2*S1*C2, -lc2*C1*S2, 0;
            lc2*C1*C2, -lc2*S1*S2, 0;
            0,          lc2*C2,    0];
    Jv3 = [-S1*(L2*C2+lc3*C23), -C1*(L2*S2+lc3*S23), -C1*lc3*S23;
            C1*(L2*C2+lc3*C23), -S1*(L2*S2+lc3*S23), -S1*lc3*S23;
            0,                    L2*C2+lc3*C23,        lc3*C23];

    Jw1 = [z0, [0;0;0], [0;0;0]];
    Jw2 = [z0, zjoint23, [0;0;0]];
    Jw3 = [z0, zjoint23, zjoint23];
end

function M = inertia_matrix_3dof(q, robot)
    [~, ~, ~, Jv1, Jv2, Jv3, Jw1, Jw2, Jw3, R1, R2, R3] = com_kinematics_3dof(q, robot);
    M = robot.m1*(Jv1.'*Jv1) + Jw1.'*R1*robot.I1*R1.'*Jw1 + ...
        robot.m2*(Jv2.'*Jv2) + Jw2.'*R2*robot.I2*R2.'*Jw2 + ...
        robot.m3*(Jv3.'*Jv3) + Jw3.'*R3*robot.I3*R3.'*Jw3;
    M = (M + M.')/2;
end

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

function qddot = robot_dynamics_3dof(q, qdot, tau, robot)
    M = inertia_matrix_3dof(q, robot);
    C = coriolis_matrix_3dof(q, qdot, robot);
    G = gravity_vector_3dof(q, robot);
    qddot = M \ (tau - C*qdot - G);
end

%% ================================================================
% FUNCIONES LOCALES - CONTROLADORES
% ================================================================

function tau = control_pid_nonlinear(q, qdot, qd, qd_dot, eint, robot, gains)
    e = qd - q;
    de = qd_dot - qdot;
    tau = gains.Kp*e + gains.Kd*de + gains.Ki*eint + gravity_vector_3dof(q, robot);
end

function tau = control_pd_precomp(q, qdot, qd, qd_dot, qd_ddot, robot, gains)
    e = qd - q;
    de = qd_dot - qdot;
    Mqd = inertia_matrix_3dof(qd, robot);
    Cqd = coriolis_matrix_3dof(qd, qd_dot, robot);
    Gqd = gravity_vector_3dof(qd, robot);
    tau_ff = Mqd*qd_ddot + Cqd*qd_dot + Gqd;
    tau = tau_ff + gains.Kp*e + gains.Kd*de;
end

function tau = control_computed_torque(q, qdot, qd, qd_dot, qd_ddot, robot, gains)
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
% FUNCIONES LOCALES - PLANEACION A* (geometria pura, sin cambios respecto
% a versiones anteriores: no depende del modelo dinamico)
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
        path_xz = simplify_path(path_xz, 2);
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
    % Interpola los waypoints articulares (de la ruta A* convertida por IK)
    % con pchip para obtener qd(t) suave; qd_dot, qd_ddot por diferenciacion
    % numerica (gradient). Se muestrea densamente (dt=0.01) para que la
    % interpolacion LINEAL usada despues por eval_traj_at (necesaria para
    % que ode45 pueda consultar el lazo cerrado en instantes continuos
    % arbitrarios) introduzca un error despreciable frente a la de pchip.
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

function [qd, qd_dot, qd_ddot] = eval_traj_at(traj, t)
    % Evalua qd/qd_dot/qd_ddot en un instante continuo arbitrario t,
    % interpolando linealmente sobre la malla densa (dt=0.01) de
    % make_joint_trajectory. Necesario para integrar con ode45.
    t = min(max(t, traj.t(1)), traj.t(end));
    qd = zeros(3,1); qd_dot = zeros(3,1); qd_ddot = zeros(3,1);
    for i = 1:3
        qd(i) = interp1(traj.t, traj.qd(i,:), t, 'linear');
        qd_dot(i) = interp1(traj.t, traj.qd_dot(i,:), t, 'linear');
        qd_ddot(i) = interp1(traj.t, traj.qd_ddot(i,:), t, 'linear');
    end
end

%% ================================================================
% FUNCIONES LOCALES - SIMULACION Y METRICAS
% ================================================================

function dx = closed_loop_ode(t, x, controller_name, robot, gains, traj)
    % Estado aumentado x = [q(3); qdot(3); eint(3)] (9x1), integrado con
    % ode45 (paso variable) en vez de Euler de paso fijo -- ver
    % robot3dof_TFinal_v3_controladores.m para la justificacion (Euler con
    % dt=0.01 diverge para PID_NO_LINEAL/PD_PRECOMP con esta dinamica mas
    % ligera; ode45 reproduce fielmente el resultado ya validado en Simulink).
    q = x(1:3); qdot = x(4:6); eint = x(7:9);
    [qd, qd_dot, qd_ddot] = eval_traj_at(traj, t);

    switch upper(controller_name)
        case 'PID_NO_LINEAL'
            tau = control_pid_nonlinear(q, qdot, qd, qd_dot, eint, robot, gains);
        case 'PD_PRECOMP'
            tau = control_pd_precomp(q, qdot, qd, qd_dot, qd_ddot, robot, gains);
        case 'PAR_CALCULADO'
            tau = control_computed_torque(q, qdot, qd, qd_dot, qd_ddot, robot, gains);
        otherwise
            error('Controlador no reconocido: %s', controller_name);
    end
    tau = saturate_torque(tau, robot);
    qddot = robot_dynamics_3dof(q, qdot, tau, robot);
    e = qd - q;
    dx = [qdot; qddot; e];
end

function res = simulate_robot_controller(controller_name, robot, traj, q0, qdot0, gains)
    x0 = [q0; qdot0; zeros(3,1)];
    opts = odeset('RelTol', 1e-6, 'AbsTol', 1e-8);
    ode_fun = @(t,x) closed_loop_ode(t, x, controller_name, robot, gains, traj);
    [~, X] = ode45(ode_fun, traj.t, x0, opts);

    N = length(traj.t);
    q = X(:,1:3)';
    qdot = X(:,4:6)';
    eint = X(:,7:9)';
    tau = zeros(3,N);
    qddot = zeros(3,N);
    for k = 1:N
        [qd, qd_dot, qd_ddot] = eval_traj_at(traj, traj.t(k));
        switch upper(controller_name)
            case 'PID_NO_LINEAL'
                tau(:,k) = control_pid_nonlinear(q(:,k), qdot(:,k), qd, qd_dot, eint(:,k), robot, gains);
            case 'PD_PRECOMP'
                tau(:,k) = control_pd_precomp(q(:,k), qdot(:,k), qd, qd_dot, qd_ddot, robot, gains);
            case 'PAR_CALCULADO'
                tau(:,k) = control_computed_torque(q(:,k), qdot(:,k), qd, qd_dot, qd_ddot, robot, gains);
        end
        tau(:,k) = saturate_torque(tau(:,k), robot);
        qddot(:,k) = robot_dynamics_3dof(q(:,k), qdot(:,k), tau(:,k), robot);
    end

    res.name = controller_name;
    res.q = q;
    res.qdot = qdot;
    res.qddot = qddot;
    res.tau = tau;
    res.err = traj.qd - q;
end

function m = compute_metrics(res, traj, label, tol)
    e = traj.qd - res.q;
    e_norm = vecnorm(e,2,1);
    tau_norm = vecnorm(res.tau,2,1);
    m.Controlador = string(label);
    m.Error_RMS_rad = sqrt(mean(e_norm.^2));
    m.Error_Max_rad = max(e_norm);
    m.TiempoEstabilizacion_s = settling_time(traj.t, e_norm, tol);
    m.Torque_RMS_Nm = sqrt(mean(tau_norm.^2));
    m.Torque_Max_Nm = max(tau_norm);
end

function ts = settling_time(t, e_norm, tol)
    band = tol * max(e_norm);
    outside = find(e_norm > band);
    if isempty(outside)
        ts = t(1);
    else
        idx = outside(end) + 1;
        if idx > numel(t)
            idx = numel(t);
        end
        ts = t(idx);
    end
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
