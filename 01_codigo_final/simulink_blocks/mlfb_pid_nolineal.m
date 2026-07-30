%% BLOQUE MATLAB FUNCTION: Control PID no lineal
% Entrada : q, qdot, qd, qd_dot, eint (todos 3x1), Kp,Kd,Ki (3x3 diag)
% Salida  : tau (3x1)
% eint (integral del error) se calcula con un bloque Integrator de
% Simulink ANTES de este bloque, no dentro de el (ver guia).

function tau = mlfb_pid_nolineal(q, qdot, qd, qd_dot, eint, Kp, Kd, Ki)
    % Parametros de gravedad (identicos a robot3dof_TFinal_v2_dinamica_jacobianos.m)
    L2 = 0.5; lc2 = 0.25; lc3 = 0.25; m2 = 0.5; m3 = 0.5; g = 9.81;
    e = qd - q;
    edot = qd_dot - qdot;
    q2 = q(2); q3 = q(3);
    G1 = 0;
    G2 = (m2*lc2 + m3*L2)*g*cos(q2) + m3*lc3*g*cos(q2+q3);
    G3 = m3*lc3*g*cos(q2+q3);
    G = [G1; G2; G3];
    tau = Kp*e + Kd*edot + Ki*eint + G;
end
