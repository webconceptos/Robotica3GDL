%% CREAR MODELO SIMULINK - ROBOT 3GDL (PLANTA + 3 CONTROLADORES)
% -------------------------------------------------------------------------
% Curso      : Robotica y Sistemas Autonomos
% Entregable : Generador/preparador del modelo Robot3GDL_Control_Final.slx
% Archivo    : crear_modelo_simulink_robot3gdl.m
%
% Punto de partida:
% Este script depende de robot3dof_TFinal_v2_dinamica_jacobianos.m, del cual
% reutiliza (redefiniendolas aqui para que este archivo corra de forma
% independiente) las funciones numericas:
%   inertia_matrix_3dof(q,robot), coriolis_matrix_3dof(q,qdot,robot),
%   gravity_vector_3dof(q,robot), robot_dynamics_3dof(q,qdot,tau,robot)
% obtenidas por el metodo de Jacobianos + Christoffel (NO Lagrange).
%
% Que hace este script:
%   1) Prepara en el workspace de MATLAB todo lo que Simulink necesita:
%      parametros robot.*, trayectoria de referencia qd/qd_dot/qd_ddot en
%      formato "From Workspace", y ganancias de los tres controladores.
%   2) Escribe archivos .m independientes con el codigo EXACTO que debe ir
%      dentro de cada bloque "MATLAB Function" (planta y los tres
%      controladores), listos para copiar/pegar o para apuntar el bloque
%      directamente a ellos.
%   3) Intenta construir automaticamente Robot3GDL_Control_Final.slx usando
%      la API de Simulink (new_system/add_block/add_line). Si Simulink no
%      esta instalado/licenciado en el equipo donde se ejecuta, este paso
%      se omite de forma segura (try/catch) y el script deja todo listo
%      para el armado manual descrito en:
%        05_anexos/guia_armado_simulink_robot3gdl.md
%
% NOTA: la generacion automatica del .slx (paso 3) sigue el patron de API
% de Simulink documentado por MathWorks (new_system, add_block, add_line,
% Stateflow.EMChart.Script para el codigo interno de los bloques MATLAB
% Function). Verificar/ajustar en la primera ejecucion con Simulink; usar
% la guia de 05_anexos como referencia si algun bloque no conecta como se
% espera.
% -------------------------------------------------------------------------

clc; clear; close all;

output_dir = fileparts(mfilename('fullpath'));
model_name = 'Robot3GDL_Control_Final';

%% ================================================================
% 1. PARAMETROS DEL ROBOT (identicos a v2_dinamica_jacobianos.m)
% Objetivo: tener robot.* disponible de forma independiente en este script.
% Fuente/justificacion: mismos supuestos fisicos documentados en
%           robot3dof_TFinal_v2_dinamica_jacobianos.m (varilla delgada,
%           masas asumidas, geometria del paper base).
% Resultado esperado: estructura "robot" identica a la de v2.
%% ================================================================
robot.L1 = 0.15; robot.L2 = 0.50; robot.L3 = 0.50;
robot.m1 = 2.00; robot.m2 = 1.50; robot.m3 = 1.00;
robot.lc1 = robot.L1/2; robot.lc2 = robot.L2/2; robot.lc3 = robot.L3/2;
robot.g = 9.81;

eps_ax = 1e-4;
robot.I1 = diag([robot.m1*robot.L1^2/12, eps_ax, robot.m1*robot.L1^2/12]);
robot.I2 = diag([eps_ax, robot.m2*robot.L2^2/12, robot.m2*robot.L2^2/12]);
robot.I3 = diag([eps_ax, robot.m3*robot.L3^2/12, robot.m3*robot.L3^2/12]);

fprintf('============================================================\n');
fprintf(' GENERADOR DE MODELO SIMULINK - ROBOT3GDL_CONTROL_FINAL\n');
fprintf('============================================================\n');

%% ================================================================
% 2. TRAYECTORIA DE REFERENCIA PARA SIMULINK (From Workspace)
% Objetivo: construir qd(t), qd_dot(t), qd_ddot(t) en el formato de
%           estructura con tiempo que exige el bloque "From Workspace"
%           de Simulink (campo .time y .signals.values).
% Fuente/justificacion: trayectoria punto-a-punto por polinomio quintico
%           (velocidad y aceleracion nulas en los extremos), la misma
%           usada como caso de prueba en las versiones de controladores.
% Resultado esperado: qd_ws, qd_dot_ws, qd_ddot_ws listos para asignarse a
%           bloques "From Workspace" en Simulink.
%% ================================================================
q_start = deg2rad([10; 25; -20]);
q_goal  = deg2rad([-25; 55; -45]);
dt = 0.01;
tf = 5.0;
t = (0:dt:tf)';
N = length(t);

qd = zeros(N,3); qd_dot = zeros(N,3); qd_ddot = zeros(N,3);
for i = 1:3
    q0i = q_start(i); q1i = q_goal(i);
    a3 = 10*(q1i-q0i)/tf^3; a4 = -15*(q1i-q0i)/tf^4; a5 = 6*(q1i-q0i)/tf^5;
    qd(:,i)      = q0i + a3*t.^3 + a4*t.^4 + a5*t.^5;
    qd_dot(:,i)  = 3*a3*t.^2 + 4*a4*t.^3 + 5*a5*t.^4;
    qd_ddot(:,i) = 6*a3*t + 12*a4*t.^2 + 20*a5*t.^3;
end

qd_ws = struct('time', t, 'signals', struct('values', qd, 'dimensions', 3));
qd_dot_ws = struct('time', t, 'signals', struct('values', qd_dot, 'dimensions', 3));
qd_ddot_ws = struct('time', t, 'signals', struct('values', qd_ddot, 'dimensions', 3));

q0_ic = qd(1,:)' + deg2rad([8; -6; 5]); % condicion inicial con error pequeno
qdot0_ic = [0;0;0];

fprintf('Trayectoria de referencia: %d muestras, tf=%.1f s, dt=%.3f s\n', N, tf, dt);
fprintf('Variables listas en el workspace: qd_ws, qd_dot_ws, qd_ddot_ws, q0_ic, qdot0_ic\n\n');

%% ================================================================
% 3. GANANCIAS DE LOS TRES CONTROLADORES
% Objetivo: dejar en el workspace gains_pid, gains_pd, gains_ct, para que
%           los bloques MATLAB Function de Simulink las lean como
%           parametros (Model Workspace / Base Workspace).
%% ================================================================
gains_pid.Kp = diag([80 90 70]);
gains_pid.Kd = diag([18 20 15]);
gains_pid.Ki = diag([8 8 6]);

gains_pd.Kp = diag([90 100 80]);
gains_pd.Kd = diag([20 22 18]);

gains_ct.Kp = diag([120 130 100]);
gains_ct.Kd = diag([25 28 22]);

tau_max = [80; 80; 60]; %#ok<NASGU> % limite de saturacion sugerido, ver guia

%% ================================================================
% 4. FUNCIONES NUMERICAS PARA LOS BLOQUES "MATLAB FUNCTION"
% Objetivo: generar, como archivos .m independientes en esta misma
%           carpeta, el codigo EXACTO que debe usarse dentro de cada
%           bloque MATLAB Function de Simulink (planta + 3 controladores).
%           Cada archivo es codigo numerico puro (sin sym/diff), por lo
%           tanto es compatible con la generacion de codigo de Simulink.
% Fuente/justificacion: mismas leyes de control y mismo modelo dinamico de
%           robot3dof_TFinal_v2_dinamica_jacobianos.m, reescritos como
%           funciones "planas" (todo en un solo archivo, sin depender de
%           funciones locales externas) porque los bloques MATLAB Function
%           de Simulink no pueden depender de funciones locales definidas
%           en OTRO archivo de script.
% Resultado esperado: cuatro archivos .m en 01_codigo_final/simulink_blocks/
%           listos para usarse como "Bloque MATLAB Function -> User-defined
%           functions -> Import from file" o para copiar/pegar su cuerpo.
%% ================================================================
blocks_dir = fullfile(output_dir, 'simulink_blocks');
if ~exist(blocks_dir, 'dir')
    mkdir(blocks_dir);
end

write_mlfb_planta(blocks_dir, robot);
write_mlfb_pid_nolineal(blocks_dir);
write_mlfb_pd_precomp(blocks_dir, robot);
write_mlfb_par_calculado(blocks_dir, robot);

fprintf('Archivos de bloques MATLAB Function generados en:\n  %s\n', blocks_dir);
fprintf('  - mlfb_planta_3gdl.m\n  - mlfb_pid_nolineal.m\n  - mlfb_pd_precomp.m\n  - mlfb_par_calculado.m\n\n');

%% ================================================================
% 5. CONSTRUCCION AUTOMATICA DEL MODELO SIMULINK (mejor esfuerzo)
% Objetivo: intentar crear Robot3GDL_Control_Final.slx con la Planta
%           Dinamica y los tres subsistemas de control, usando la API de
%           Simulink. Si Simulink no esta disponible, se omite con
%           try/catch y se deja la guia manual como camino garantizado.
% Fuente/justificacion: estructura de bloques exigida por el enunciado
%           ("Simulink obligatorio"): Planta (MATLAB Function) + 3
%           subsistemas de control (MATLAB Function) + integradores en
%           cascada + Scopes/To Workspace de comparacion.
% Resultado esperado: Robot3GDL_Control_Final.slx en 01_codigo_final/, con
%           al menos un subsistema (PID no lineal) completamente cableado
%           como referencia; si la construccion falla, mensaje claro
%           remitiendo a 05_anexos/guia_armado_simulink_robot3gdl.md.
%% ================================================================
try
    build_robot3gdl_simulink_model(model_name, output_dir, blocks_dir);
    fprintf('\n[OK] %s.slx generado en %s\n', model_name, output_dir);
catch err
    fprintf('\n[Aviso] No se pudo generar %s.slx automaticamente (%s).\n', model_name, err.message);
    fprintf('        Esto es normal si Simulink no esta instalado/licenciado en este equipo.\n');
    fprintf('        Todas las variables y funciones necesarias ya estan preparadas.\n');
    fprintf('        Arma el modelo manualmente siguiendo:\n');
    fprintf('        05_anexos/guia_armado_simulink_robot3gdl.md\n');
end

fprintf('\n================ RESUMEN ================\n');
fprintf('Workspace preparado: robot, qd_ws, qd_dot_ws, qd_ddot_ws, q0_ic, qdot0_ic,\n');
fprintf('                     gains_pid, gains_pd, gains_ct.\n');
fprintf('Bloques MATLAB Function listos en: %s\n', blocks_dir);
fprintf('Guia de armado manual: 05_anexos/guia_armado_simulink_robot3gdl.md\n');

%% ================================================================
% FUNCIONES LOCALES - GENERACION DE CODIGO DE BLOQUES MATLAB FUNCTION
% ================================================================

function write_mlfb_planta(blocks_dir, robot)
    % Objetivo: escribir el codigo de la Planta Dinamica Robot 3GDL como
    % funcion MATLAB plana y autocontenida (sin llamar a funciones locales
    % de otro archivo), tal como lo requiere un bloque MATLAB Function.
    fid = fopen(fullfile(blocks_dir, 'mlfb_planta_3gdl.m'), 'w');
    fprintf(fid, '%s\n', '%% BLOQUE MATLAB FUNCTION: Planta Dinamica Robot 3GDL');
    fprintf(fid, '%s\n', '% Entrada  : tau (3x1), q (3x1), qdot (3x1)');
    fprintf(fid, '%s\n', '% Salida   : qddot (3x1)');
    fprintf(fid, '%s\n', '% Pegar el CUERPO de esta funcion dentro del bloque MATLAB Function');
    fprintf(fid, '%s\n', '% llamado "Planta_3GDL" (ver guia_armado_simulink_robot3gdl.md).');
    fprintf(fid, '%s\n\n', '%%');
    fprintf(fid, 'function qddot = mlfb_planta_3gdl(tau, q, qdot)\n');
    write_robot_params_block(fid, robot);
    fprintf(fid, '\n    M = local_inertia_matrix(q, L1,L2,L3,lc1,lc2,lc3,m1,m2,m3,I1,I2,I3);\n');
    fprintf(fid, '    C = local_coriolis_matrix(q, qdot, L1,L2,L3,lc1,lc2,lc3,m1,m2,m3,I1,I2,I3);\n');
    fprintf(fid, '    G = local_gravity_vector(q, L2,lc2,lc3,m2,m3,g);\n');
    fprintf(fid, '    qddot = M \\ (tau - C*qdot - G);\n');
    fprintf(fid, 'end\n\n');
    write_shared_dynamics_functions(fid);
    fclose(fid);
end

function write_mlfb_pid_nolineal(blocks_dir)
    fid = fopen(fullfile(blocks_dir, 'mlfb_pid_nolineal.m'), 'w');
    fprintf(fid, '%s\n', '%% BLOQUE MATLAB FUNCTION: Control PID no lineal');
    fprintf(fid, '%s\n', '% Entrada : q, qdot, qd, qd_dot, eint (todos 3x1), Kp,Kd,Ki (3x3 diag)');
    fprintf(fid, '%s\n', '% Salida  : tau (3x1)');
    fprintf(fid, '%s\n', '% eint (integral del error) se calcula con un bloque Integrator de');
    fprintf(fid, '%s\n\n', '% Simulink ANTES de este bloque, no dentro de el (ver guia).');
    fprintf(fid, 'function tau = mlfb_pid_nolineal(q, qdot, qd, qd_dot, eint, Kp, Kd, Ki)\n');
    fprintf(fid, '    %% Parametros de gravedad (identicos a robot3dof_TFinal_v2_dinamica_jacobianos.m)\n');
    fprintf(fid, '    L2 = 0.50; lc2 = 0.25; lc3 = 0.25; m2 = 1.50; m3 = 1.00; g = 9.81;\n');
    fprintf(fid, '    e = qd - q;\n');
    fprintf(fid, '    edot = qd_dot - qdot;\n');
    fprintf(fid, '    q2 = q(2); q3 = q(3);\n');
    fprintf(fid, '    G1 = 0;\n');
    fprintf(fid, '    G2 = (m2*lc2 + m3*L2)*g*cos(q2) + m3*lc3*g*cos(q2+q3);\n');
    fprintf(fid, '    G3 = m3*lc3*g*cos(q2+q3);\n');
    fprintf(fid, '    G = [G1; G2; G3];\n');
    fprintf(fid, '    tau = Kp*e + Kd*edot + Ki*eint + G;\n');
    fprintf(fid, 'end\n');
    fclose(fid);
end

function write_mlfb_pd_precomp(blocks_dir, robot) %#ok<INUSD>
    fid = fopen(fullfile(blocks_dir, 'mlfb_pd_precomp.m'), 'w');
    fprintf(fid, '%s\n', '%% BLOQUE MATLAB FUNCTION: Control PD con precompensacion');
    fprintf(fid, '%s\n', '% Entrada : q, qdot, qd, qd_dot, qd_ddot (3x1), Kp,Kd (3x3 diag)');
    fprintf(fid, '%s\n\n', '% Salida  : tau (3x1)');
    fprintf(fid, 'function tau = mlfb_pd_precomp(q, qdot, qd, qd_dot, qd_ddot, Kp, Kd)\n');
    write_robot_params_block(fid, robot);
    fprintf(fid, '\n    e = qd - q;\n');
    fprintf(fid, '    edot = qd_dot - qdot;\n');
    fprintf(fid, '    Mqd = local_inertia_matrix(qd, L1,L2,L3,lc1,lc2,lc3,m1,m2,m3,I1,I2,I3);\n');
    fprintf(fid, '    Cqd = local_coriolis_matrix(qd, qd_dot, L1,L2,L3,lc1,lc2,lc3,m1,m2,m3,I1,I2,I3);\n');
    fprintf(fid, '    Gqd = local_gravity_vector(qd, L2,lc2,lc3,m2,m3,g);\n');
    fprintf(fid, '    tau_ff = Mqd*qd_ddot + Cqd*qd_dot + Gqd;\n');
    fprintf(fid, '    tau = tau_ff + Kp*e + Kd*edot;\n');
    fprintf(fid, 'end\n\n');
    write_shared_dynamics_functions(fid);
    fclose(fid);
end

function write_mlfb_par_calculado(blocks_dir, robot) %#ok<INUSD>
    fid = fopen(fullfile(blocks_dir, 'mlfb_par_calculado.m'), 'w');
    fprintf(fid, '%s\n', '%% BLOQUE MATLAB FUNCTION: Control por par calculado');
    fprintf(fid, '%s\n', '% Entrada : q, qdot, qd, qd_dot, qd_ddot (3x1), Kp,Kd (3x3 diag)');
    fprintf(fid, '%s\n\n', '% Salida  : tau (3x1)');
    fprintf(fid, 'function tau = mlfb_par_calculado(q, qdot, qd, qd_dot, qd_ddot, Kp, Kd)\n');
    write_robot_params_block(fid, robot);
    fprintf(fid, '\n    e = qd - q;\n');
    fprintf(fid, '    edot = qd_dot - qdot;\n');
    fprintf(fid, '    Mq = local_inertia_matrix(q, L1,L2,L3,lc1,lc2,lc3,m1,m2,m3,I1,I2,I3);\n');
    fprintf(fid, '    Cq = local_coriolis_matrix(q, qdot, L1,L2,L3,lc1,lc2,lc3,m1,m2,m3,I1,I2,I3);\n');
    fprintf(fid, '    Gq = local_gravity_vector(q, L2,lc2,lc3,m2,m3,g);\n');
    fprintf(fid, '    v = qd_ddot + Kd*edot + Kp*e;\n');
    fprintf(fid, '    tau = Mq*v + Cq*qdot + Gq;\n');
    fprintf(fid, 'end\n\n');
    write_shared_dynamics_functions(fid);
    fclose(fid);
end

function write_robot_params_block(fid, robot)
    fprintf(fid, '    %% Parametros del robot (identicos a robot3dof_TFinal_v2_dinamica_jacobianos.m)\n');
    fprintf(fid, '    L1=%.6g; L2=%.6g; L3=%.6g;\n', robot.L1, robot.L2, robot.L3);
    fprintf(fid, '    m1=%.6g; m2=%.6g; m3=%.6g;\n', robot.m1, robot.m2, robot.m3);
    fprintf(fid, '    lc1=%.6g; lc2=%.6g; lc3=%.6g;\n', robot.lc1, robot.lc2, robot.lc3);
    fprintf(fid, '    g=%.6g;\n', robot.g);
    fprintf(fid, '    I1 = diag([%.6g, %.6g, %.6g]);\n', robot.I1(1,1), robot.I1(2,2), robot.I1(3,3));
    fprintf(fid, '    I2 = diag([%.6g, %.6g, %.6g]);\n', robot.I2(1,1), robot.I2(2,2), robot.I2(3,3));
    fprintf(fid, '    I3 = diag([%.6g, %.6g, %.6g]);\n', robot.I3(1,1), robot.I3(2,2), robot.I3(3,3));
end

function write_shared_dynamics_functions(fid)
    % Copias "planas" (sin dependencias externas) de com_kinematics_3dof /
    % inertia_matrix_3dof / coriolis_matrix_3dof / gravity_vector_3dof de
    % robot3dof_TFinal_v2_dinamica_jacobianos.m, necesarias porque un
    % bloque MATLAB Function no puede llamar funciones locales de OTRO
    % archivo de script.
    fprintf(fid, [ ...
        'function M = local_inertia_matrix(q, L1,L2,L3,lc1,lc2,lc3,m1,m2,m3,I1,I2,I3) %%#ok<INUSL>\n' ...
        '    q1=q(1); q2=q(2); q3=q(3);\n' ...
        '    C1=cos(q1); S1=sin(q1); C2=cos(q2); S2=sin(q2); C23=cos(q2+q3); S23=sin(q2+q3);\n' ...
        '    R1 = [C1,0,S1; S1,0,-C1; 0,1,0];\n' ...
        '    R2 = [C1*C2,-C1*S2,S1; S1*C2,-S1*S2,-C1; S2,C2,0];\n' ...
        '    R3 = [C1*C23,-C1*S23,S1; S1*C23,-S1*S23,-C1; S23,C23,0];\n' ...
        '    z0=[0;0;1]; zj=[S1;-C1;0];\n' ...
        '    Jv1 = zeros(3,3);\n' ...
        '    Jv2 = [-lc2*S1*C2,-lc2*C1*S2,0; lc2*C1*C2,-lc2*S1*S2,0; 0,lc2*C2,0];\n' ...
        '    Jv3 = [-S1*(L2*C2+lc3*C23),-C1*(L2*S2+lc3*S23),-C1*lc3*S23; C1*(L2*C2+lc3*C23),-S1*(L2*S2+lc3*S23),-S1*lc3*S23; 0,L2*C2+lc3*C23,lc3*C23];\n' ...
        '    Jw1 = [z0,[0;0;0],[0;0;0]];\n' ...
        '    Jw2 = [z0,zj,[0;0;0]];\n' ...
        '    Jw3 = [z0,zj,zj];\n' ...
        '    M = m1*(Jv1.''*Jv1) + Jw1.''*R1*I1*R1.''*Jw1 + ...\n' ...
        '        m2*(Jv2.''*Jv2) + Jw2.''*R2*I2*R2.''*Jw2 + ...\n' ...
        '        m3*(Jv3.''*Jv3) + Jw3.''*R3*I3*R3.''*Jw3;\n' ...
        '    M = (M + M.'')/2;\n' ...
        'end\n\n' ...
        'function C = local_coriolis_matrix(q, qdot, L1,L2,L3,lc1,lc2,lc3,m1,m2,m3,I1,I2,I3)\n' ...
        '    n = 3; h = 1e-6; dM = cell(1,n);\n' ...
        '    for k = 1:n\n' ...
        '        dq = zeros(n,1); dq(k) = h;\n' ...
        '        Mp = local_inertia_matrix(q+dq, L1,L2,L3,lc1,lc2,lc3,m1,m2,m3,I1,I2,I3);\n' ...
        '        Mm = local_inertia_matrix(q-dq, L1,L2,L3,lc1,lc2,lc3,m1,m2,m3,I1,I2,I3);\n' ...
        '        dM{k} = (Mp - Mm)/(2*h);\n' ...
        '    end\n' ...
        '    C = zeros(n,n);\n' ...
        '    for i=1:n\n' ...
        '        for j=1:n\n' ...
        '            for k=1:n\n' ...
        '                cijk = 0.5*(dM{k}(i,j) + dM{j}(i,k) - dM{i}(j,k));\n' ...
        '                C(i,j) = C(i,j) + cijk*qdot(k);\n' ...
        '            end\n' ...
        '        end\n' ...
        '    end\n' ...
        'end\n\n' ...
        'function G = local_gravity_vector(q, L2,lc2,lc3,m2,m3,g)\n' ...
        '    q2=q(2); q3=q(3);\n' ...
        '    G1 = 0;\n' ...
        '    G2 = (m2*lc2 + m3*L2)*g*cos(q2) + m3*lc3*g*cos(q2+q3);\n' ...
        '    G3 = m3*lc3*g*cos(q2+q3);\n' ...
        '    G = [G1; G2; G3];\n' ...
        'end\n' ...
    ]);
end

%% ================================================================
% FUNCION LOCAL - CONSTRUCCION DEL MODELO SIMULINK (mejor esfuerzo)
% ================================================================

function build_robot3gdl_simulink_model(model_name, output_dir, blocks_dir) %#ok<INUSD>
    % Cierra el modelo si ya estuviera abierto de una corrida previa.
    if bdIsLoaded(model_name)
        close_system(model_name, 0);
    end
    new_system(model_name);
    open_system(model_name);

    % ---- Subsistema de referencia: PID no lineal (completamente cableado) ----
    sub = [model_name '/PID_NoLineal'];
    add_block('built-in/Subsystem', sub);
    % Limpiar puertos de entrada/salida por defecto si el subsistema los trae
    % (no todas las versiones de Simulink los crean automaticamente).
    try, delete_block([sub '/In1']); catch, end %#ok<CTCH>
    try, delete_block([sub '/Out1']); catch, end %#ok<CTCH>

    add_block('simulink/Sources/From Workspace', [sub '/qd'], 'VariableName', 'qd_ws');
    add_block('simulink/Sources/From Workspace', [sub '/qd_dot'], 'VariableName', 'qd_dot_ws');

    add_block('simulink/User-Defined Functions/MATLAB Function', [sub '/Controlador_PID']);
    add_block('simulink/User-Defined Functions/MATLAB Function', [sub '/Planta_3GDL']);

    add_block('simulink/Continuous/Integrator', [sub '/Int_qdot'], 'InitialCondition', 'qdot0_ic');
    add_block('simulink/Continuous/Integrator', [sub '/Int_q'], 'InitialCondition', 'q0_ic');

    add_block('simulink/Math Operations/Subtract', [sub '/Error']);
    add_block('simulink/Continuous/Integrator', [sub '/Int_error']);

    add_block('simulink/Sinks/To Workspace', [sub '/q_out'], 'VariableName', 'q_pid_out');
    add_block('simulink/Sinks/To Workspace', [sub '/tau_out'], 'VariableName', 'tau_pid_out');
    add_block('simulink/Sinks/Scope', [sub '/Scope_q']);

    % Codigo de los bloques MATLAB Function (ver Seccion 4 de este script).
    set_matlab_function_script(sub, 'Controlador_PID', fileread(fullfile(blocks_dir,'mlfb_pid_nolineal.m')));
    set_matlab_function_script(sub, 'Planta_3GDL', fileread(fullfile(blocks_dir,'mlfb_planta_3gdl.m')));

    % Conexiones principales (mejor esfuerzo; revisar en Simulink real).
    % Orden de puertos de Controlador_PID = orden de argumentos de
    % mlfb_pid_nolineal(q, qdot, qd, qd_dot, eint, ...):
    %   puerto1=q, puerto2=qdot, puerto3=qd, puerto4=qd_dot, puerto5=eint
    % Orden de puertos de Planta_3GDL = mlfb_planta_3gdl(tau, q, qdot):
    %   puerto1=tau, puerto2=q, puerto3=qdot
    try
        set_param([sub '/Error'], 'Inputs', '+-'); % Error = qd - q
        add_line(sub, 'qd/1', 'Error/1', 'autorouting', 'on');
        add_line(sub, 'Int_q/1', 'Error/2', 'autorouting', 'on');
        add_line(sub, 'Error/1', 'Int_error/1', 'autorouting', 'on');
        add_line(sub, 'Int_error/1', 'Controlador_PID/5', 'autorouting', 'on'); % eint
        add_line(sub, 'Controlador_PID/1', 'Planta_3GDL/1', 'autorouting', 'on'); % tau
        add_line(sub, 'Planta_3GDL/1', 'Int_qdot/1', 'autorouting', 'on'); % qddot
        add_line(sub, 'Int_qdot/1', 'Int_q/1', 'autorouting', 'on'); % qdot
        add_line(sub, 'Int_q/1', 'Planta_3GDL/2', 'autorouting', 'on'); % q -> planta
        add_line(sub, 'Int_qdot/1', 'Planta_3GDL/3', 'autorouting', 'on'); % qdot -> planta
        add_line(sub, 'Int_q/1', 'Controlador_PID/1', 'autorouting', 'on'); % q -> controlador
        add_line(sub, 'Int_qdot/1', 'Controlador_PID/2', 'autorouting', 'on'); % qdot -> controlador
        add_line(sub, 'qd/1', 'Controlador_PID/3', 'autorouting', 'on'); % qd -> controlador
        add_line(sub, 'qd_dot/1', 'Controlador_PID/4', 'autorouting', 'on'); % qd_dot -> controlador
        add_line(sub, 'Int_q/1', 'q_out/1', 'autorouting', 'on');
        add_line(sub, 'Int_q/1', 'Scope_q/1', 'autorouting', 'on');
        add_line(sub, 'Controlador_PID/1', 'tau_out/1', 'autorouting', 'on');
    catch errWire
        fprintf('  [Aviso] Cableado automatico parcial en %s (%s). Revisar/ajustar en Simulink.\n', sub, errWire.message);
    end

    % ---- Los subsistemas PD_Precomp y Par_Calculado se dejan como copias
    % del mismo patron (planta + integradores) para armar manualmente con
    % los bloques MATLAB Function ya generados en simulink_blocks/. Ver
    % guia_armado_simulink_robot3gdl.md para el cableado completo de los
    % tres controladores y los bloques de comparacion (qd vs q, error,
    % torque, error RMS).
    add_block('built-in/Subsystem', [model_name '/PD_Precomp']);
    add_block('built-in/Subsystem', [model_name '/Par_Calculado']);

    save_system(model_name, fullfile(output_dir, [model_name '.slx']));
    close_system(model_name, 0);
end

function set_matlab_function_script(sub, block_name, script_text)
    % Establece el codigo fuente de un bloque MATLAB Function via la API
    % de Stateflow (patron documentado por MathWorks para generar bloques
    % MATLAB Function programaticamente).
    blk_path = [sub '/' block_name];
    rt = sfroot;
    chart = rt.find('-isa', 'Stateflow.EMChart', 'Path', blk_path);
    if isempty(chart)
        error('No se encontro el chart Stateflow del bloque %s', blk_path);
    end
    chart.Script = script_text;
end
