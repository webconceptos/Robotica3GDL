%% BLOQUE MATLAB FUNCTION: Control PD con precompensacion
% Entrada : q, qdot, qd, qd_dot, qd_ddot (3x1), Kp,Kd (3x3 diag)
% Salida  : tau (3x1)

function tau = mlfb_pd_precomp(q, qdot, qd, qd_dot, qd_ddot, Kp, Kd)
    % Parametros del robot (identicos a robot3dof_TFinal_v2_dinamica_jacobianos.m)
    L1=0.15; L2=0.5; L3=0.5;
    m1=0.5; m2=0.5; m3=0.5;
    lc1=0.075; lc2=0.25; lc3=0.25;
    g=9.81;
    I1 = diag([0.00105, 0.000225, 0.00105]);
    I2 = diag([0.000225, 0.0105292, 0.0105292]);
    I3 = diag([0.000225, 0.0105292, 0.0105292]);

    e = qd - q;
    edot = qd_dot - qdot;
    Mqd = local_inertia_matrix(qd, L1,L2,L3,lc1,lc2,lc3,m1,m2,m3,I1,I2,I3);
    Cqd = local_coriolis_matrix(qd, qd_dot, L1,L2,L3,lc1,lc2,lc3,m1,m2,m3,I1,I2,I3);
    Gqd = local_gravity_vector(qd, L2,lc2,lc3,m2,m3,g);
    tau_ff = Mqd*qd_ddot + Cqd*qd_dot + Gqd;
    tau = tau_ff + Kp*e + Kd*edot;
end

function M = local_inertia_matrix(q, L1,L2,L3,lc1,lc2,lc3,m1,m2,m3,I1,I2,I3) %#ok<INUSL>
    q1=q(1); q2=q(2); q3=q(3);
    C1=cos(q1); S1=sin(q1); C2=cos(q2); S2=sin(q2); C23=cos(q2+q3); S23=sin(q2+q3);
    R1 = [C1,0,S1; S1,0,-C1; 0,1,0];
    R2 = [C1*C2,-C1*S2,S1; S1*C2,-S1*S2,-C1; S2,C2,0];
    R3 = [C1*C23,-C1*S23,S1; S1*C23,-S1*S23,-C1; S23,C23,0];
    z0=[0;0;1]; zj=[S1;-C1;0];
    Jv1 = zeros(3,3);
    Jv2 = [-lc2*S1*C2,-lc2*C1*S2,0; lc2*C1*C2,-lc2*S1*S2,0; 0,lc2*C2,0];
    Jv3 = [-S1*(L2*C2+lc3*C23),-C1*(L2*S2+lc3*S23),-C1*lc3*S23; C1*(L2*C2+lc3*C23),-S1*(L2*S2+lc3*S23),-S1*lc3*S23; 0,L2*C2+lc3*C23,lc3*C23];
    Jw1 = [z0,[0;0;0],[0;0;0]];
    Jw2 = [z0,zj,[0;0;0]];
    Jw3 = [z0,zj,zj];
    M = m1*(Jv1.'*Jv1) + Jw1.'*R1*I1*R1.'*Jw1 + ...
        m2*(Jv2.'*Jv2) + Jw2.'*R2*I2*R2.'*Jw2 + ...
        m3*(Jv3.'*Jv3) + Jw3.'*R3*I3*R3.'*Jw3;
    M = (M + M.')/2;
end

function C = local_coriolis_matrix(q, qdot, L1,L2,L3,lc1,lc2,lc3,m1,m2,m3,I1,I2,I3)
    n = 3; h = 1e-6; dM = cell(1,n);
    for k = 1:n
        dq = zeros(n,1); dq(k) = h;
        Mp = local_inertia_matrix(q+dq, L1,L2,L3,lc1,lc2,lc3,m1,m2,m3,I1,I2,I3);
        Mm = local_inertia_matrix(q-dq, L1,L2,L3,lc1,lc2,lc3,m1,m2,m3,I1,I2,I3);
        dM{k} = (Mp - Mm)/(2*h);
    end
    C = zeros(n,n);
    for i=1:n
        for j=1:n
            for k=1:n
                cijk = 0.5*(dM{k}(i,j) + dM{j}(i,k) - dM{i}(j,k));
                C(i,j) = C(i,j) + cijk*qdot(k);
            end
        end
    end
end

function G = local_gravity_vector(q, L2,lc2,lc3,m2,m3,g)
    q2=q(2); q3=q(3);
    G1 = 0;
    G2 = (m2*lc2 + m3*L2)*g*cos(q2) + m3*lc3*g*cos(q2+q3);
    G3 = m3*lc3*g*cos(q2+q3);
    G = [G1; G2; G3];
end
