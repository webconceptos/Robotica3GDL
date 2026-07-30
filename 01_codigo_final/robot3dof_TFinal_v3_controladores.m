%% ROBOT ANTROPOMORFICO 3GDL - TRABAJO FINAL v3 (CONTROLADORES)
% -------------------------------------------------------------------------
% Curso      : Robotica y Sistemas Autonomos
% Entregable : Bloque de desarrollo - Controladores dinamicos (respaldo MATLAB)
% Archivo    : robot3dof_TFinal_v3_controladores.m
%
% Punto de partida:
% Esta version parte de robot3dof_TFinal_v2_dinamica_jacobianos.m: modelo
% dinamico M(q), C(q,qdot), G(q) obtenido por Jacobianos lineales/angulares
% de los centros de masa + coeficientes de Christoffel (n=3), sin Lagrange.
% Las funciones de dinamica de este archivo son copias autocontenidas de
% esas mismas funciones (mismo metodo, mismos supuestos fisicos).
%
% Proposito de este archivo: sirve como RESPALDO/COMPARACION en MATLAB
% puro del resultado ya obtenido en Simulink (ver README.md, Seccion 2.6).
% Usa la MISMA trayectoria, las MISMAS ganancias y el MISMO modelo
% dinamico que Robot3GDL_Control_Final.slx, asi que sus metricas deberian
% aproximarse a las ya validadas en Simulink (PID no lineal: ~1.89 s de
% estabilizacion; PD precompensado: ~0.85 s; Par calculado: ~0.70 s y
% menor torque). Sirve tambien para iterar mas rapido sobre ganancias sin
% depender de Simulink.
%
% Alcance de esta version (bloque "controladores"):
%   A) Cinematica y dinamica heredadas de v2 (Jacobianos + Christoffel).
%   B) Trayectoria articular de referencia punto-a-punto (polinomio
%      quintico) - la misma usada en crear_modelo_simulink_robot3gdl.m.
%      La planeacion con obstaculos (A*) se agrega en v4.
%   C) Tres controladores dinamicos:
%      - PID no lineal (Kp*e + Kd*de + Ki*int(e) + G(q)).
%      - PD con precompensacion (feedforward M,C,G evaluados en qd).
%      - Control por par calculado (linealizacion por realimentacion).
%   D) Metricas y graficas comparativas: error ARTICULAR (confirmado por
%      el docente, no cartesiano), incluyendo tiempo de estabilizacion y
%      cual controlador converge mas rapido a cero.
%
% Fuera de alcance en esta version (se agrega en v4):
%   - Planeacion autonoma con obstaculos (A*) en el plano cartesiano XZ.
%
% Paper base del parcial:
% Ashagrie, A., Salau, A. O., & Weldcherkos, T. (2021).
% Modeling and control of a 3-DOF articulated robotic manipulator using
% self-tuning fuzzy sliding mode controller. Cogent Engineering.
%
% Supuestos fisicos: ver encabezado de robot3dof_TFinal_v2_dinamica_jacobianos.m
% (L1,L2,L3,m1,m2,m3,g son dato del paper, Tabla 2, Pag. 8; centros de masa
% y radio de cilindro de cada eslabon son supuestos de simulacion).
% -------------------------------------------------------------------------

clc; clear; close all;

%% ================================================================
% 1. PARAMETROS GEOMETRICOS Y FISICOS DEL ROBOT
% Objetivo: mismos parametros que v2_dinamica_jacobianos.m, reutilizados
%           aqui para que este archivo corra de forma independiente.
% Fuente/justificacion: L1,L2,L3,m1,m2,m3,g de la Tabla 2 del paper base
%           (dato reportado); centros de masa y radio de cilindro son
%           supuestos de simulacion (cilindro solido, indicacion del
%           docente).
% Resultado esperado: estructura "robot" identica a v2_dinamica_jacobianos.m.
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
fprintf(' ROBOT ANTROPOMORFICO 3GDL - TRABAJO FINAL v3 (CONTROLADORES)\n');
fprintf('============================================================\n');
fprintf('Geometria (paper): L1=%.2f m, L2=%.2f m, L3=%.2f m\n', robot.L1, robot.L2, robot.L3);
fprintf('Masas (paper, Tabla 2): m1=%.2f, m2=%.2f, m3=%.2f kg\n\n', robot.m1, robot.m2, robot.m3);

%% ================================================================
% 2. TRAYECTORIA ARTICULAR DE REFERENCIA (PUNTO A PUNTO)
% Objetivo: generar qd(t), dqd(t), ddqd(t) suaves entre una configuracion
%           inicial y una final, para probar los controladores bajo la
%           MISMA trayectoria (comparacion justa). Es la misma trayectoria
%           usada en crear_modelo_simulink_robot3gdl.m, para comparabilidad
%           directa con los resultados ya obtenidos en Simulink.
% Fuente/justificacion: polinomio quintico con velocidad y aceleracion nulas
%           en los extremos; eleccion estandar de generacion de trayectoria
%           punto-a-punto en robotica.
% Resultado esperado: qd_dot(0)=qd_dot(tf)=0 y qd_ddot(0)=qd_ddot(tf)=0.
%% ================================================================
q_start = deg2rad([10; 25; -20]);
q_goal  = deg2rad([-25; 55; -45]);

dt = 0.01;
tf = 5.0;
traj = make_quintic_trajectory(q_start, q_goal, tf, dt);

q0 = traj.qd(:,1) + deg2rad([8; -6; 5]); % mismo error inicial que en Simulink (q0_ic)
qdot0 = [0; 0; 0];

fprintf('================ TRAYECTORIA DE REFERENCIA ================\n');
fprintf('q_start [deg] = [%.2f %.2f %.2f]\n', rad2deg(q_start));
fprintf('q_goal  [deg] = [%.2f %.2f %.2f]\n', rad2deg(q_goal));
fprintf('Duracion: %.1f s, dt = %.3f s\n\n', tf, dt);

%% ================================================================
% 3. GANANCIAS DE CONTROL
% Objetivo: mismas ganancias que crear_modelo_simulink_robot3gdl.m
%           (gains_pid, gains_pd, gains_ct), para que este resultado sea
%           directamente comparable con el de Simulink.
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
% 5. METRICAS COMPARATIVAS (ERROR ARTICULAR)
% Objetivo: cuantificar desempeno de cada controlador con las metricas
%           confirmadas por el docente: error ARTICULAR (no cartesiano),
%           error RMS/maximo, tiempo de estabilizacion, y cual controlador
%           converge mas rapido a cero.
% Resultado esperado: tabla comparativa + identificacion del controlador
%           mas rapido (deberia ser "Par calculado", igual que en Simulink).
%% ================================================================
tol = 0.02; % 2% de tolerancia sobre el error inicial, criterio de estabilizacion

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
% 6. GRAFICAS COMPARATIVAS
% Objetivo: visualizar trayectoria cartesiana, seguimiento articular,
%           error articular y torque de los tres controladores.
% Resultado esperado: graficas equivalentes a las exigidas en los
%           resultados minimos del trabajo final (items 1-4), sin mapa de
%           obstaculos (eso es v4).
%% ================================================================
plot_ee_path(robot, traj, res_pid, res_pd, res_ct);
plot_joint_tracking(traj, res_pid, res_pd, res_ct);
plot_error_norms(traj, res_pid, res_pd, res_ct);
plot_torques(traj, res_pid, res_pd, res_ct);

fprintf('\n================ RESUMEN v3 ================\n');
fprintf('Controladores PID no lineal, PD precompensado y par calculado, con dinamica de Jacobianos.\n');
fprintf('Comparar estas metricas contra README.md Seccion 2.6 (resultado de Simulink) como validacion cruzada.\n');
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
% FUNCIONES LOCALES - DINAMICA POR JACOBIANOS (identicas a v2)
% ================================================================

function [pc1, pc2, pc3, Jv1, Jv2, Jv3, Jw1, Jw2, Jw3, R1, R2, R3] = com_kinematics_3dof(q, robot)
    L1 = robot.L1; L2 = robot.L2;
    lc1 = robot.lc1; lc2 = robot.lc2; lc3 = robot.lc3;
    q1 = q(1); q2 = q(2); q3 = q(3);
    C1 = cos(q1); S1 = sin(q1);
    C2 = cos(q2); S2 = sin(q2);
    C23 = cos(q2+q3); S23 = sin(q2+q3);

    z0 = [0;0;1];
    zjoint23 = [S1; -C1; 0]; % eje comun de las juntas 2 y 3 (z1 = z2)

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
    % Coeficientes de Christoffel (n=3) con dM/dq por diferencias finitas
    % centrales (adaptacion numerica del metodo, ver v2_dinamica_jacobianos.m).
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
    traj.q_start = q_start;
    traj.q_goal = q_goal;
    traj.tf = tf;
end

function [qd, qd_dot, qd_ddot] = eval_quintic_at(q_start, q_goal, tf, t)
    % Evalua el mismo polinomio quintico de make_quintic_trajectory, pero en
    % un instante continuo arbitrario t (no solo en la malla fija de traj.t).
    % Necesario para integrar con ode45, que consulta el lazo cerrado en
    % instantes de tiempo elegidos adaptativamente por el solver, no en una
    % malla fija.
    t = min(max(t, 0), tf); % qd(t) se mantiene constante en q_goal para t>tf
    qd = zeros(3,1); qd_dot = zeros(3,1); qd_ddot = zeros(3,1);
    for i = 1:3
        q0i = q_start(i); q1i = q_goal(i);
        a3 = 10*(q1i-q0i)/tf^3;
        a4 = -15*(q1i-q0i)/tf^4;
        a5 = 6*(q1i-q0i)/tf^5;
        qd(i)      = q0i + a3*t^3 + a4*t^4 + a5*t^5;
        qd_dot(i)  = 3*a3*t^2 + 4*a4*t^3 + 5*a5*t^4;
        qd_ddot(i) = 6*a3*t + 12*a4*t^2 + 20*a5*t^3;
    end
end

%% ================================================================
% FUNCIONES LOCALES - SIMULACION Y METRICAS
% ================================================================

function dx = closed_loop_ode(t, x, controller_name, robot, gains, q_start, q_goal, tf)
    % Estado aumentado x = [q(3); qdot(3); eint(3)] (9x1). eint se integra
    % como un estado mas -- exactamente equivalente al bloque Int_error de
    % Simulink -- en vez de acumularse a mano con un paso fijo.
    q = x(1:3); qdot = x(4:6); eint = x(7:9);
    [qd, qd_dot, qd_ddot] = eval_quintic_at(q_start, q_goal, tf, t);

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
    % Integra el lazo cerrado con ode45 (solver de paso variable, igual
    % familia que el solver por defecto de Simulink) en vez de Euler de paso
    % fijo. Con esta dinamica (masas ligeras, cilindro solido) un Euler de
    % paso fijo dt=0.01 diverge para PID_NO_LINEAL y PD_PRECOMP -- ode45 fue
    % necesario para reproducir de forma fiel el resultado ya validado en
    % Simulink (ver README.md, Seccion 2.6: ErrRMS, ErrMax y Torque_Max
    % coinciden con Simulink dentro de <1% usando este metodo).
    x0 = [q0; qdot0; zeros(3,1)];
    opts = odeset('RelTol', 1e-6, 'AbsTol', 1e-8);
    ode_fun = @(t,x) closed_loop_ode(t, x, controller_name, robot, gains, traj.q_start, traj.q_goal, traj.tf);
    [~, X] = ode45(ode_fun, traj.t, x0, opts);

    N = length(traj.t);
    q = X(:,1:3)';
    qdot = X(:,4:6)';
    eint = X(:,7:9)';
    tau = zeros(3,N);
    qddot = zeros(3,N);
    for k = 1:N
        [qd, qd_dot, qd_ddot] = eval_quintic_at(traj.q_start, traj.q_goal, traj.tf, traj.t(k));
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
    % Ultimo instante en que |e_norm| sale de la banda tol*max(e_norm) y ya
    % no vuelve a salir (mismo criterio que comparar_controladores.m).
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

function plot_ee_path(robot, traj, res_pid, res_pd, res_ct)
    % Item 1 de "Resultados minimos": trayectoria deseada vs real (cartesiana).
    xd = joint_series_to_cartesian(robot, traj.qd);
    x_pid = joint_series_to_cartesian(robot, res_pid.q);
    x_pd  = joint_series_to_cartesian(robot, res_pd.q);
    x_ct  = joint_series_to_cartesian(robot, res_ct.q);

    figure('Name','Trayectoria cartesiana del efector final');
    plot3(xd(1,:), xd(2,:), xd(3,:), 'k--', 'LineWidth', 1.8); hold on;
    plot3(x_pid(1,:), x_pid(2,:), x_pid(3,:), 'LineWidth', 1.2);
    plot3(x_pd(1,:),  x_pd(2,:),  x_pd(3,:),  'LineWidth', 1.2);
    plot3(x_ct(1,:),  x_ct(2,:),  x_ct(3,:),  'LineWidth', 1.2);
    grid on; axis equal;
    xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
    title('Trayectoria cartesiana: deseada vs seguimiento de cada controlador');
    legend('Deseada','PID no lineal','PD precomp','Par calculado','Location','best');
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
