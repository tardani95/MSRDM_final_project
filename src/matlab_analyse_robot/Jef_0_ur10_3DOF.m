function [Jef_0, Jef_0_v, Jef_0_w] = Jef_0_ur10_3DOF(u)
    %JEF_0_UR10_3DOF Summary of this function goes here
    %   Detailed explanation goes here
    %% Robot Parameters
    robot_param = u(1:38);
    L_param = robot_param(1:10);
    m_param = robot_param(11:13);
    I_param = robot_param(14:31);
    b_param = robot_param(32:34);

    g_vec_param = robot_param(35:37);
    g_param = robot_param(38);

    L1 = L_param(1);
    L2 = L_param(2);
    L4 = L_param(3);
    L6 = L_param(4);
    L7 = L_param(5);
    L9 = L_param(6);
    L3 = L_param(7);
    L5 = L_param(8);
    L8 = L_param(9);
    L10 = L_param(10);

    m1 = m_param(1);
    m2 = m_param(2);
    m3 = m_param(3);

    I1_param = I_param(1:6);
    I111 = I1_param(1);
    I112 = I1_param(2);
    I113 = I1_param(3);
    I122 = I1_param(4);
    I123 = I1_param(5);
    I133 = I1_param(6);

    I2_param = I_param(7:12);
    I211 = I2_param(1);
    I212 = I2_param(2);
    I213 = I2_param(3);
    I222 = I2_param(4);
    I223 = I2_param(5);
    I233 = I2_param(6);

    I3_param = I_param(13:18);
    I311 = I3_param(1);
    I312 = I3_param(2);
    I313 = I3_param(3);
    I322 = I3_param(4);
    I323 = I3_param(5);
    I333 = I3_param(6);

    Beta(1, 1) = b_param(1);
    Beta(2, 2) = b_param(2);
    Beta(3, 3) = b_param(3);

    gx = g_vec_param(1);
    gy = g_vec_param(2);
    gz = g_vec_param(3);
    g = g_param;

    %% Joint variables
    Q = u(39:41);
    q1 = Q(1);
    q2 = Q(2);
    q3 = Q(3);

    %% Jacobian Calculation
    Jef_0 = [L2 * cos(q1) + L4 * cos(q1) + L3 * sin(q1) * sin(q2) + L5 * cos(q2) * sin(q1) * sin(q3) + L5 * cos(q3) * sin(q1) * sin(q2), -cos(q1) * (L5 * cos(q2 + q3) + L3 * cos(q2)), -L5 * cos(q2 + q3) * cos(q1); L2 * sin(q1) + L4 * sin(q1) - L3 * cos(q1) * sin(q2) - L5 * cos(q1) * cos(q2) * sin(q3) - L5 * cos(q1) * cos(q3) * sin(q2), -sin(q1) * (L5 * cos(q2 + q3) + L3 * cos(q2)), -L5 * cos(q2 + q3) * sin(q1); 0, - L5 * sin(q2 + q3) - L3 * sin(q2), -L5 * sin(q2 + q3); 0, sin(q1), sin(q1); 0, -cos(q1), -cos(q1); 1, 0, 0];
    Jef_0_v = Jef_0(1:3, :);
    Jef_0_w = Jef_0(4:6, :);
end
