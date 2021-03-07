Q = sym('q%d', [6 1], 'real');
Qp = sym('q%dp', [6 1], 'real');
Qpp = sym('q%dpp', [6 1], 'real');
syms([Q Qp Qpp], 'real');

syms(sym('L', [15 1], 'positive'), 'positive');

syms(sym('m', [6 1], 'positive'), 'positive');
% syms(sym('k',[6 1],'positive'),'positive');
syms(sym('b', [6 1], 'positive'), 'positive');
syms(sym('I1%d%d', [3 3], 'positive'), 'positive');
syms(sym('I2%d%d', [3 3], 'positive'), 'positive');
syms(sym('I3%d%d', [3 3], 'positive'), 'positive');
syms(sym('I4%d%d', [3 3], 'positive'), 'positive');
syms(sym('I5%d%d', [3 3], 'positive'), 'positive');
syms(sym('I6%d%d', [3 3], 'positive'), 'positive');

syms g real

g_axis = sym([0; 0; 1]);

I1 = [I111 I112 I113;
    I112 I122 I123;
    I113 I123 I133];

I2 = [I211 I212 I213;
    I212 I222 I223;
    I213 I223 I233];

I3 = [I311 I312 I313;
    I312 I322 I323;
    I313 I323 I333];

I4 = [I411 I412 I413;
    I412 I422 I423;
    I413 I423 I433];

I5 = [I511 I512 I513;
    I512 I522 I523;
    I513 I523 I533];

I6 = [I611 I612 I613;
    I612 I622 I623;
    I613 I623 I633];

mi = [m1, m2, m3, m4, m5, m6];

Ii(:, :, 1) = I1;
Ii(:, :, 2) = I2;
Ii(:, :, 3) = I3;
Ii(:, :, 4) = I4;
Ii(:, :, 5) = I5;
Ii(:, :, 6) = I6;

qi = [q1, q2, q3, q4, q5, q6];
qip = [q1p, q2p, q3p, q4p, q5p, q6p];

m_size = length(mi);
q_size = length(qi);

spi = sym(pi);

%% Forward Kinematics
relDH_Links = [
            q1 L1 0 spi / 2;
            q2 + spi / 2 0 L3 0;
            q3 0 L5 0;
            q4 + spi / 2 L2 0 spi / 2;
            q5 L6 0 -spi / 2;
            q6 L4 0 0];

relDH_com = [
        q1 L7 0 spi / 2;
        q2 + spi / 2 L8 L9 0;
        q3 L10 L11 0;
        q4 L12 L13 0;
        q5 L14 0 0;
        q6 L15 0 0];

[H_abs_stack, X_abs_stack, Ti_0_stack, Tcmi_0_stack, H_link_rel_stack, ...
        H_cm_rel_stack, X_link_abs_stack, X_cm_abs_stack] = FwdKinematics(relDH_Links, relDH_com, 0);

Xef_0 = X_link_abs_stack(:, :, end);
Xef_0_str = char(simplify(Xef_0));

% compute the transformations regarding to the world coordinate frame
T0_W = rotm_trvec2tf(RotX(spi / 2), [0; 0; 0]);

Tef_W = T0_W * Ti_0_stack(:, :, end);

Xef_W = tf2pose(Tef_W);
Xef_W_char = char(simplify(expand(Xef_W)));

T1_0 = Ti_0_stack(:, :, 1);
T2_0 = Ti_0_stack(:, :, 2);
T3_0 = Ti_0_stack(:, :, 3);
T4_0 = Ti_0_stack(:, :, 4);
T5_0 = Ti_0_stack(:, :, 5);
T6_0 = Ti_0_stack(:, :, 6);

Tcm1_0 = Tcmi_0_stack(:, :, 1);
Tcm2_0 = Tcmi_0_stack(:, :, 2);
Tcm3_0 = Tcmi_0_stack(:, :, 3);
Tcm4_0 = Tcmi_0_stack(:, :, 4);
Tcm5_0 = Tcmi_0_stack(:, :, 5);
Tcm6_0 = Tcmi_0_stack(:, :, 6);

T1_0_char = char(simplify(expand(T1_0)));
T2_0_char = char(simplify(expand(T2_0)));
T3_0_char = char(simplify(expand(T3_0)));
T4_0_char = char(simplify(expand(T4_0)));
T5_0_char = char(simplify(expand(T5_0)));
T6_0_char = char(simplify(expand(T6_0)));

Tcm1_0_char = char(simplify(expand(Tcm1_0)));
Tcm2_0_char = char(simplify(expand(Tcm2_0)));
Tcm3_0_char = char(simplify(expand(Tcm3_0)));
Tcm4_0_char = char(simplify(expand(Tcm4_0)));
Tcm5_0_char = char(simplify(expand(Tcm5_0)));
Tcm6_0_char = char(simplify(expand(Tcm6_0)));

%% Differential Kinematics
[jacobi_abs_stack, Ji_0_stack, Jcmi_0_stack] = DiffKinematics(H_abs_stack, 'rrrrrr');

Jcmi_vi_stack = sym(zeros(3,q_size,m_size));
Jcmi_wi_stack = sym(zeros(3,q_size,m_size));

for idx = 1:m_size
    Jcmi_vi_stack(:, :, idx) = expand(Jcmi_0_stack(1:3, :, idx));
    Jcmi_wi_stack(:, :, idx) = expand(Jcmi_0_stack(4:6, :, idx));
end

Jef_0 = Ji_0_stack(:, :, end);
Jef_0_str = char(simplify(expand(Jef_0)));

%% M,C,G calculation

Rcmi_0_stack = sym(zeros(3,3,m_size));
xcmi_0 = sym(zeros(3,m_size));

% precalculation
for idx = 1:m_size
    Rcmi_0_stack(:, :, idx) = expand(tf2rotm(Tcmi_0_stack(:, :, idx)));
    xcmi_0(:, idx) = expand(X_cm_abs_stack(1:3,:,idx));
end

% Complete Gravitational Torques Vector (symbolic form)
G = sym(zeros(q_size, 1));

P = sym(0);

for idx = 1:m_size
    hi = xcmi_0(:, idx)' * g_axis;
    P = P + mi(idx) * g * hi;
end

for kdx = 1:q_size
    pPqk = expand(diff(P, qi(kdx)));
    G(kdx) = pPqk;
end

G = simplify(expand(G))
G_str = char(simplify(G));

% Complete the Inertia Matrix (symbolic form)
M = sym(zeros(m_size));

for idx = 1:m_size
    M_cmi_vi = mi(idx) * Jcmi_vi_stack(:, :, idx)' * Jcmi_vi_stack(:, :, idx);

    Ii_0 = Rcmi_0_stack(:, :, idx) * Ii(:, :, idx) * Rcmi_0_stack(:, :, idx)';
    M_cmi_wi = Jcmi_wi_stack(:, :, idx)' * Ii_0 * Jcmi_wi_stack(:, :, idx);

    Mi = M_cmi_vi + M_cmi_wi;
    M = M + Mi;
end

% M = simplify(M)
% M = expand(M)

%Complete Centripetal and Coriolis Matrix (symbolic form)
C = sym(zeros(length(qi)));
M = expand(M);

for kdx = 1:q_size

    for jdx = 1:q_size

        for idx = 1:q_size
            %             pM_kj_qi = simplify(diff(M(kdx,jdx), qi(idx)));
            pM_kj_qi = expand(diff(M(kdx, jdx), qi(idx)));
            %             pM_ki_qj = simplify(diff(M(kdx,idx), qi(jdx)));
            pM_ki_qj = expand(diff(M(kdx, idx), qi(jdx)));
            %             pM_ij_qk = simplify(diff(M(idx,jdx), qi(kdx)));
            pM_ij_qk = expand(diff(M(idx, jdx), qi(kdx)));
            %             C(kdx,jdx) = simplify(C(kdx,jdx) + 1/2 * (pM_kj_qi + pM_ki_qj - pM_ij_qk) * qip(idx));
            C(kdx, jdx) = C(kdx, jdx) + (1/2) * (pM_kj_qi + pM_ki_qj - pM_ij_qk) * qip(idx);
        end

        % or here multiply 1/2
    end

end

C = expand(C)

% the DM-2*C must be skewsymmetric
DM = zeros(size(M));

for idx = 1:q_size
    DM = DM + diff(M, qi(idx)) * qip(idx) + diff(M,qip(idx)) * qipp(idx);
end

% DM = diff(M, q1) * q1p + diff(M, q2) * q2p + diff(M, q3) * q3p ...
%      + diff(M, q1p) * q1pp + diff(M, q2p) * q2pp + diff(M, q3p) * q3pp;
% DM = simplify(DM);

DM = expand(DM);

N = DM - 2 * C;
N = expand(N);

if isSym(M)
    disp("M matrix is symmetric"); % ok
end

if isSkewSym(N)
    disp("N matrix is skew symmetric"); % ok
end

M_char = char(simplify(expand(M)));
C_char = char(simplify(expand(C)));
G_char = char(simplify(expand(G)));

%% Robot regressor
Qrp = sym('q%drp', [q_size 1], 'real');
Qrpp = sym('q%drpp', [q_size 1], 'real');

state_vars = [Q' Qp' Qrp' Qrpp' g g_axis'];

eqn = M * Qrpp + C * Qrp + G;

[Yr, Theta] = RobotRegressor(eqn, state_vars);

if sum(expand(eqn - Yr * Theta), 'all') == 0
    disp("Robot regressor correctly calculated");
end

Theta_str = char(Theta);
Yr_str = char(Yr);

disp('Done!');
