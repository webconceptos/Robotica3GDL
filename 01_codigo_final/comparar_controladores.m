%% COMPARACION DE LOS TRES CONTROLADORES - ERROR ARTICULAR
% -------------------------------------------------------------------------
% Curso      : Robotica y Sistemas Autonomos
% Entregable : Graficas comparativas para el informe (error articular)
% Archivo    : comparar_controladores.m
%
% Punto de partida:
% Este script se ejecuta DESPUES de simular Robot3GDL_Control_Final.slx en
% Simulink (los tres subsistemas: PID_NoLineal, PD_Precomp, Par_Calculado).
% Requiere en el workspace base las variables exportadas por los bloques
% "To Workspace" de cada subsistema:
%   q_pid_out, tau_pid_out, q_pd_out, tau_pd_out, q_ct_out, tau_ct_out
% y la trayectoria de referencia qd_ws (generada por
% crear_modelo_simulink_robot3gdl.m).
%
% Nota sobre el formato de las senales: segun la configuracion de "Data
% Import/Export" del modelo, Simulink puede devolver estas variables
% directamente al workspace base, o agrupadas dentro de un objeto
% Simulink.SimulationOutput llamado "out" (comportamiento por defecto al
% correr desde la barra de herramientas de la app en versiones recientes).
% Este script detecta automaticamente cual de los dos casos aplica.
%
% Que hace este script:
%   1) Extrae q_*_out y tau_*_out (desde "out" o desde el workspace base).
%   2) Alinea cada senal con la malla de tiempo de qd_ws (interpolacion).
%   3) Calcula el error articular e(t) = qd(t) - q(t) por controlador.
%   4) Grafica el error por articulacion (q1,q2,q3) con los tres
%      controladores superpuestos, y el error total (norma).
%   5) Calcula la tabla comparativa: Error RMS, Error maximo, Tiempo de
%      estabilizacion, Torque RMS, Torque maximo (error ARTICULAR, segun
%      lo confirmado por el docente - ver "Respuestas del
%      docente").
% -------------------------------------------------------------------------

%% ================================================================
% 1. EXTRACCION DE SENALES (maneja "out" o workspace base)
% Objetivo: obtener q_pid_out, tau_pid_out, q_pd_out, tau_pd_out,
%           q_ct_out, tau_ct_out sin importar donde haya quedado la
%           salida de la simulacion.
% Resultado esperado: 6 variables numericas [t, valores] listas para usar.
%% ================================================================
if exist('out', 'var') && isa(out, 'Simulink.SimulationOutput')
    fprintf('Usando variables logueadas en el objeto "out" (Simulink.SimulationOutput).\n');
    get_sig = @(name) out.get(name);
else
    fprintf('Usando variables directamente del workspace base.\n');
    get_sig = @(name) evalin('base', name);
end

[t_pid, q_pid]   = normalize_signal(get_sig('q_pid_out'));
[t_pd,  q_pd]    = normalize_signal(get_sig('q_pd_out'));
[t_ct,  q_ct]    = normalize_signal(get_sig('q_ct_out'));
[t_tpid, tau_pid] = normalize_signal(get_sig('tau_pid_out'));
[t_tpd,  tau_pd]  = normalize_signal(get_sig('tau_pd_out'));
[t_tct,  tau_ct]  = normalize_signal(get_sig('tau_ct_out'));

%% ================================================================
% 2. ALINEACION TEMPORAL Y CALCULO DEL ERROR ARTICULAR
% Objetivo: interpolar cada q_*_out sobre la malla de tiempo de qd_ws
%           (pueden diferir levemente por el solver de paso variable) y
%           calcular e(t) = qd(t) - q(t).
%% ================================================================
t_qd = qd_ws.time;
qd = qd_ws.signals.values;

q_pid_i = interp1(t_pid, q_pid, t_qd, 'linear', 'extrap');
q_pd_i  = interp1(t_pd,  q_pd,  t_qd, 'linear', 'extrap');
q_ct_i  = interp1(t_ct,  q_ct,  t_qd, 'linear', 'extrap');

e_pid = qd - q_pid_i;
e_pd  = qd - q_pd_i;
e_ct  = qd - q_ct_i;

% tau_*_out viene con la malla de tiempo NATIVA del solver de Simulink (paso
% variable), que no es uniforme y normalmente tiene mas muestras durante
% los transitorios rapidos (torque cerca de saturacion). Si se calcula
% sqrt(mean(tau.^2)) directamente sobre esa malla no uniforme, el resultado
% queda sesgado (sobrepondera los tramos donde el solver dio pasos mas
% pequenos). Se interpola tau sobre la MISMA malla uniforme t_qd que q,
% igual que arriba, para que Torque_RMS sea un promedio temporal correcto.
tau_pid_i = interp1(t_tpid, tau_pid, t_qd, 'linear', 'extrap');
tau_pd_i  = interp1(t_tpd,  tau_pd,  t_qd, 'linear', 'extrap');
tau_ct_i  = interp1(t_tct,  tau_ct,  t_qd, 'linear', 'extrap');

%% ================================================================
% 3. GRAFICAS: ERROR ARTICULAR POR JUNTA Y ERROR TOTAL
% Resultado esperado: 4 figuras (q1,q2,q3, y norma total) con los tres
%           controladores superpuestos.
%% ================================================================
names = {'q1','q2','q3'};
for i = 1:3
    figure('Name', ['Error articular ' names{i}]);
    plot(t_qd, e_pid(:,i), 'LineWidth', 1.4); hold on;
    plot(t_qd, e_pd(:,i),  'LineWidth', 1.4);
    plot(t_qd, e_ct(:,i),  'LineWidth', 1.4);
    grid on;
    xlabel('Tiempo [s]'); ylabel(['e_{' names{i} '} [rad]']);
    title(['Error articular - ' names{i}]);
    legend('PID no lineal', 'PD precompensado', 'Par calculado', 'Location', 'best');
end

figure('Name', 'Error articular total (norma)');
plot(t_qd, vecnorm(e_pid,2,2), 'LineWidth', 1.4); hold on;
plot(t_qd, vecnorm(e_pd,2,2),  'LineWidth', 1.4);
plot(t_qd, vecnorm(e_ct,2,2),  'LineWidth', 1.4);
grid on;
xlabel('Tiempo [s]'); ylabel('||e_q|| [rad]');
title('Error articular total por controlador (Par Calculado es el principal de la comparacion)');
legend('PID no lineal', 'PD precompensado', 'Par calculado', 'Location', 'best');

%% ================================================================
% 4. TABLA COMPARATIVA (error articular, tiempo de estabilizacion)
% Fuente/justificacion: metricas confirmadas por el docente (error
%           ARTICULAR, no cartesiano; incluir tiempo de estabilizacion y
%           cual controlador converge mas rapido).
%% ================================================================
tol = 0.02; % 2% de tolerancia sobre el error inicial, criterio de estabilizacion

m_pid = compute_row('PID no lineal',      t_qd, e_pid, tau_pid_i, tol);
m_pd  = compute_row('PD precompensado',   t_qd, e_pd,  tau_pd_i,  tol);
m_ct  = compute_row('Par calculado',      t_qd, e_ct,  tau_ct_i,  tol);

Tcomp = struct2table([m_pid; m_pd; m_ct]);
disp('================ TABLA COMPARATIVA (error articular) ================');
disp(Tcomp);

[~, idx_best] = min([m_pid.TiempoEstabilizacion_s, m_pd.TiempoEstabilizacion_s, m_ct.TiempoEstabilizacion_s]);
nombres = {'PID no lineal','PD precompensado','Par calculado'};
fprintf('\nControlador que converge mas rapido a cero: %s\n', nombres{idx_best});

%% ================================================================
% FUNCIONES LOCALES
% ================================================================

function [t, Y] = normalize_signal(sig)
    % Normaliza una senal exportada por Simulink (timeseries o "Structure
    % With Time") a un par [t, Y] con Y de tamano Nxm. Una senal vectorial
    % [3x1] logueada por Simulink suele quedar como arreglo 3D [3 x 1 x N]
    % (una "pagina" por instante de tiempo) en vez de [N x 3]; se reduce
    % con squeeze() antes de alinear con el vector de tiempo.
    if isa(sig, 'timeseries')
        t = sig.Time;
        Y = sig.Data;
    elseif isstruct(sig) && isfield(sig, 'time') && isfield(sig, 'signals')
        t = sig.time;
        Y = sig.signals.values;
    else
        error(['Formato de senal no reconocido (%s). Revisa el "Save format" ' ...
               'del bloque To Workspace (Timeseries o Structure With Time) ' ...
               'en 05_anexos/guia_armado_simulink_robot3gdl.md.'], class(sig));
    end

    if ndims(Y) == 3
        Y = squeeze(Y); % [M x 1 x N] -> [M x N]
    end
    if size(Y,1) ~= numel(t) && size(Y,2) == numel(t)
        Y = Y.'; % [M x N] -> [N x M]
    end
    if size(Y,1) ~= numel(t)
        error(['normalize_signal: no se pudo alinear el tiempo (%d muestras) ' ...
               'con los datos (%d x %d). Revisa el formato de la senal en Simulink.'], ...
               numel(t), size(Y,1), size(Y,2));
    end
end

function m = compute_row(label, t, e, tau, tol)
    e_norm = vecnorm(e, 2, 2);
    tau_norm = vecnorm(tau, 2, 2);
    m.Controlador = string(label);
    m.Error_RMS_rad = sqrt(mean(e_norm.^2));
    m.Error_Max_rad = max(e_norm);
    m.TiempoEstabilizacion_s = settling_time(t, e_norm, tol);
    m.Torque_RMS_Nm = sqrt(mean(tau_norm.^2));
    m.Torque_Max_Nm = max(tau_norm);
end

function ts = settling_time(t, e_norm, tol)
    % Tiempo de estabilizacion: ultimo instante en que |e_norm| sale de la
    % banda tol*max(e_norm) y ya no vuelve a salir.
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
