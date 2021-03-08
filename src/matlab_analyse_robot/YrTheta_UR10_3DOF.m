function [YrTheta,Yr,Theta] = YrTheta_UR10_3DOF(u)

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

    Qp = u(42:44);
    q1p = Qp(1);
    q2p = Qp(2);
    q3p = Qp(3);

    Qrp = u(45:47);
    q1rp = Qrp(1);
    q2rp = Qrp(2);
    q3rp = Qrp(3);

    Qrpp = u(48:50);
    q1rpp = Qrpp(1);
    q2rpp = Qrpp(2);
    q3rpp = Qrpp(3);

    % Define the vector of parameters(symbolic form)
    Theta = [I122; I211; I212; I213; I222; I223; I233; I311; I312; I313; I322; I323; I333;
        L3 * m3; L7 * m2; L8 * m2; L9 * m3; L10 * m3; L3^2 * m3; L7^2 * m2; L8^2 * m2;
        L9^2 * m3; L10^2 * m3; L3 * L9 * m3; L3 * L10 * m3; L7 * L8 * m2; L9 * L10 * m3];

    % Define the matrix of states(symbolic form)
    Yr = [q1rpp, ...
            q1rpp / 2 + (q1rpp * cos(2 * q2)) / 2 - (q1p * q2rp * sin(2 * q2)) / 2 - (q2p * q1rp * sin(2 * q2)) / 2, ...
            -q1rpp * sin(2 * q2) - q1p * q2rp * cos(2 * q2) - q2p * q1rp * cos(2 * q2), ...
            q2rpp * cos(q2) - q2p * q2rp * sin(q2), ...
            q1rpp / 2 - (q1rpp * cos(2 * q2)) / 2 + (q1p * q2rp * sin(2 * q2)) / 2 + (q2p * q1rp * sin(2 * q2)) / 2, ...
            -q2rpp * sin(q2) - q2p * q2rp * cos(q2), ...
            0, ...
            q1rpp / 2 + (q1rpp * cos(2 * q2 + 2 * q3)) / 2 - (q1p * q2rp * sin(2 * q2 + 2 * q3)) / 2 - (q2p * q1rp * sin(2 * q2 + 2 * q3)) / 2 - (q1p * q3rp * sin(2 * q2 + 2 * q3)) / 2 - (q3p * q1rp * sin(2 * q2 + 2 * q3)) / 2, ...
            -q1rpp * sin(2 * q2 + 2 * q3) - q1p * q2rp * cos(2 * q2 + 2 * q3) - q2p * q1rp * cos(2 * q2 + 2 * q3) - q1p * q3rp * cos(2 * q2 + 2 * q3) - q3p * q1rp * cos(2 * q2 + 2 * q3), ...
            q2rpp * cos(q2 + q3) + q3rpp * cos(q2 + q3) - q2p * q2rp * sin(q2 + q3) - q2p * q3rp * sin(q2 + q3) - q3p * q2rp * sin(q2 + q3) - q3p * q3rp * sin(q2 + q3), ...
            q1rpp / 2 - (q1rpp * cos(2 * q2 + 2 * q3)) / 2 + (q1p * q2rp * sin(2 * q2 + 2 * q3)) / 2 + (q2p * q1rp * sin(2 * q2 + 2 * q3)) / 2 + (q1p * q3rp * sin(2 * q2 + 2 * q3)) / 2 + (q3p * q1rp * sin(2 * q2 + 2 * q3)) / 2, ...
            -q2rpp * sin(q2 + q3) - q3rpp * sin(q2 + q3) - q2p * q2rp * cos(q2 + q3) - q2p * q3rp * cos(q2 + q3) - q3p * q2rp * cos(q2 + q3) - q3p * q3rp * cos(q2 + q3), ...
            0, ...
            (L3 * q1rpp) / 2 - L10 * q1rpp * cos(2 * q2 + q3) + (g * gx * cos(q1 - q2)) / 2 + (g * gy * sin(q1 - q2)) / 2 - (L3 * q1rpp * cos(2 * q2)) / 2 - (g * gx * cos(q1 + q2)) / 2 - (g * gy * sin(q1 + q2)) / 2 - L9 * q2rpp * cos(q2) + L10 * q1rpp * cos(q3) + L9 * q2p * q2rp * sin(q2) - (L10 * q1p * q3rp * sin(q3)) / 2 - (L10 * q3p * q1rp * sin(q3)) / 2 + L10 * q1p * q2rp * sin(2 * q2 + q3) + L10 * q2p * q1rp * sin(2 * q2 + q3) + (L10 * q1p * q3rp * sin(2 * q2 + q3)) / 2 + (L10 * q3p * q1rp * sin(2 * q2 + q3)) / 2 + (L3 * q1p * q2rp * sin(2 * q2)) / 2 + (L3 * q2p * q1rp * sin(2 * q2)) / 2, ...
            L7 * q1rpp + g * gx * cos(q1) + g * gy * sin(q1) - L8 * q2rpp * cos(q2) + L8 * q2p * q2rp * sin(q2), ...
            L8 * q1rpp - L8 * q1rpp * cos(q2)^2 + (L8 * q1p * q2rp * sin(2 * q2)) / 2 + (L8 * q2p * q1rp * sin(2 * q2)) / 2 - g * gy * cos(q1) * sin(q2) + g * gx * sin(q1) * sin(q2), ...
            L9 * q1rpp + g * gx * cos(q1) + g * gy * sin(q1) - L10 * q2rpp * cos(q2 + q3) - L10 * q3rpp * cos(q2 + q3) + L10 * q2p * q2rp * sin(q2 + q3) + L10 * q2p * q3rp * sin(q2 + q3) + L10 * q3p * q2rp * sin(q2 + q3) + L10 * q3p * q3rp * sin(q2 + q3), ...
            (L10 * q1rpp) / 2 - (g * gx * cos(q1 + q2 + q3)) / 2 - (g * gy * sin(q1 + q2 + q3)) / 2 - (L10 * q1rpp * cos(2 * q2 + 2 * q3)) / 2 + (g * gx * cos(q2 - q1 + q3)) / 2 - (g * gy * sin(q2 - q1 + q3)) / 2 + (L10 * q1p * q2rp * sin(2 * q2 + 2 * q3)) / 2 + (L10 * q2p * q1rp * sin(2 * q2 + 2 * q3)) / 2 + (L10 * q1p * q3rp * sin(2 * q2 + 2 * q3)) / 2 + (L10 * q3p * q1rp * sin(2 * q2 + 2 * q3)) / 2, ...
            0, 0, 0, 0, 0, 0, 0, 0, 0;
            0, ...
            (q1p * q1rp * sin(2 * q2)) / 2, ...
            q1p * q1rp * cos(2 * q2), ...
            q1rpp * cos(q2), ...
            -(q1p * q1rp * sin(2 * q2)) / 2, ...
            -q1rpp * sin(q2), ...
            q2rpp, ...
            (q1p * q1rp * sin(2 * q2 + 2 * q3)) / 2, ...
            q1p * q1rp * cos(2 * q2 + 2 * q3), ...
            q1rpp * cos(q2 + q3), ...
            -(q1p * q1rp * sin(2 * q2 + 2 * q3)) / 2, ...
            -q1rpp * sin(q2 + q3), ...
            q2rpp + q3rpp, ...
            L3 * q2rpp - g * gz * sin(q2) - L9 * q1rpp * cos(q2) + 2 * L10 * q2rpp * cos(q3) + L10 * q3rpp * cos(q3) + L10 * q1p * q1rp * sin(q3) - L10 * q2p * q3rp * sin(q3) - L10 * q3p * q2rp * sin(q3) - L10 * q3p * q3rp * sin(q3) - (L3 * q1p * q1rp * sin(2 * q2)) / 2 - g * gx * cos(q1) * cos(q2) - g * gy * cos(q2) * sin(q1) - 2 * L10 * q1p * q1rp * cos(q2)^2 * sin(q3) - 2 * L10 * q1p * q1rp * cos(q2) * cos(q3) * sin(q2), ...
            -L8 * q1rpp * cos(q2), ...
            L8 * q2rpp - g * gz * sin(q2) - g * gx * cos(q1) * cos(q2) - g * gy * cos(q2) * sin(q1) - L8 * q1p * q1rp * cos(q2) * sin(q2), ...
            -L10 * q1rpp * cos(q2 + q3), ...
            L10 * q2rpp + L10 * q3rpp - (g * gx * cos(q1 + q2 + q3)) / 2 - (g * gy * sin(q1 + q2 + q3)) / 2 - (g * gx * cos(q2 - q1 + q3)) / 2 + (g * gy * sin(q2 - q1 + q3)) / 2 - g * gz * sin(q2 + q3) - (L10 * q1p * q1rp * sin(2 * q2 + 2 * q3)) / 2, ...
            0, 0, 0, 0, 0, 0, 0, 0, 0;
            0, ...
            0, ...
            0, ...
            0, ...
            0, ...
            0, ...
            0, ...
            (q1p * q1rp * sin(2 * q2 + 2 * q3)) / 2, ...
            q1p * q1rp * cos(2 * q2 + 2 * q3), ...
            q1rpp * cos(q2 + q3), ...
            -(q1p * q1rp * sin(2 * q2 + 2 * q3)) / 2, ...
            -q1rpp * sin(q2 + q3), ...
            q2rpp + q3rpp, ...
            (L10 * (2 * q2rpp * cos(q3) + q1p * q1rp * sin(q3) + 2 * q2p * q2rp * sin(q3) - q1p * q1rp * sin(2 * q2 + q3))) / 2, ...
            0, ...
            0, ...
            -L10 * q1rpp * cos(q2 + q3), ...
            L10 * q2rpp + L10 * q3rpp - (g * gx * cos(q1 + q2 + q3)) / 2 - (g * gy * sin(q1 + q2 + q3)) / 2 - (g * gx * cos(q2 - q1 + q3)) / 2 + (g * gy * sin(q2 - q1 + q3)) / 2 - g * gz * sin(q2 + q3) - (L10 * q1p * q1rp * sin(2 * q2 + 2 * q3)) / 2, ...
            0, 0, 0, 0, 0, 0, 0, 0, 0];

    YrTheta = Yr * Theta;

end