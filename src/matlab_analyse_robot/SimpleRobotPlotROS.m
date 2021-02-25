function [Out] = SimpleRobotPlotROS(u)
    %SIMPLEROBOTPLOT Summary of this function goes here
    persistent jointpub jointmsg counter tftree tfStampedMsg tfStampedMsg1 tfStampedMsg2 tfStampedMsg3 tfStampedMsg4 tfStampedMsg5 tfStampedMsg6
    % persistent  tfStampedMsgcm1 tfStampedMsgcm2 tfStampedMsgcm3

    %% Input parameters
    Q_param = u(1:6);
    %Joint Position
    q1 = Q_param(1);
    q2 = Q_param(2);
    q3 = Q_param(3);
    q4 = Q_param(4);
    q5 = Q_param(5);
    q6 = Q_param(6);
    %Joint Position Vector
    Q = [q1 q2 q3 q4 q5 q6]';

    p_offset = length(Q_param);
    Qp_param = u((1:6) + p_offset);
    %Joint Velocity
    qp1 = Qp_param(1);
    qp2 = Qp_param(2);
    qp3 = Qp_param(3);
    qp4 = Qp_param(4);
    qp5 = Qp_param(5);
    qp6 = Qp_param(6);
    %Joint Velocity Vector
    Qp = [qp1 qp2 qp3 qp4 qp5 qp6]';

    % Robot Parameters
    p_offset = p_offset + length(Qp_param);
    L_size = 15;
    m_size = 6;
    I_size = 6 * 6;
    b_size = 6;
    robot_param_size = L_size + m_size + I_size + b_size + 3 + 1;
    robot_param = u(p_offset + (1:(robot_param_size)));

    L_param = robot_param(1:L_size);
    m_param = robot_param((1:m_size) + L_size);
    I_param = robot_param((1:I_size) + L_size + m_size);
    b_param = robot_param((1:b_size) + L_size + m_size + I_size));

    g_vec_param = robot_param(end-4:end-1);
    g_param = robot_param(end);

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
    L11 = L_param(11);
    L12 = L_param(12);
    L13 = L_param(13);
    L14 = L_param(14);
    L15 = L_param(15);

    m1 = m_param(1);
    m2 = m_param(2);
    m3 = m_param(3);
    m4 = m_param(4);
    m5 = m_param(5);
    m6 = m_param(6);

    I1_param = I_param(1:6);
    I111 = I1_param(1);
    I112 = I1_param(2);
    I113 = I1_param(3);
    I122 = I1_param(4);
    I123 = I1_param(5);
    I133 = I1_param(6);

    I2_param = I_param((1:6)+6);
    I211 = I2_param(1);
    I212 = I2_param(2);
    I213 = I2_param(3);
    I222 = I2_param(4);
    I223 = I2_param(5);
    I233 = I2_param(6);

    I3_param = I_param((1:6)+6*2);
    I311 = I3_param(1);
    I312 = I3_param(2);
    I313 = I3_param(3);
    I322 = I3_param(4);
    I323 = I3_param(5);
    I333 = I3_param(6);

    I4_param = I_param((1:6)+6*3);
    I411 = I4_param(1);
    I412 = I4_param(2);
    I413 = I4_param(3);
    I422 = I4_param(4);
    I423 = I4_param(5);
    I433 = I4_param(6);

    I5_param = I_param((1:6)+6*4);
    I511 = I5_param(1);
    I512 = I5_param(2);
    I513 = I5_param(3);
    I522 = I5_param(4);
    I523 = I5_param(5);
    I533 = I5_param(6);

    I6_param = I_param((1:6)+6*5);
    I611 = I6_param(1);
    I612 = I6_param(2);
    I613 = I6_param(3);
    I622 = I6_param(4);
    I623 = I6_param(5);
    I633 = I6_param(6);

    Beta(1, 1) = b_param(1);
    Beta(2, 2) = b_param(2);
    Beta(3, 3) = b_param(3);
    Beta(4, 4) = b_param(4);
    Beta(5, 5) = b_param(5);
    Beta(6, 6) = b_param(6);

    gx = g_vec_param(1);
    gy = g_vec_param(2);
    gz = g_vec_param(3);
    g = g_param;

    p_offset = p_offset + robot_param_size
    Tao = u(p_offset+(1:3));
    %Time
    t = u(p_offset+4);

    %% Homogenous Transformations
    % Specify the Robot Base (with respect to the world coordinate frame in ROS)
    T0_W = rotm_trvec2tf(RotZ(pi), [0 0 0]');

    % Compute the Homogeneous Transformations
    T1_0 = [cos(q1), 0, sin(q1), 0; sin(q1), 0, -cos(q1), 0; 0, 1, 0, L1; 0, 0, 0, 1];
    T2_0 = [-cos(q1)*sin(q2), -cos(q1)*cos(q2), sin(q1), -L3*cos(q1)*sin(q2); -sin(q1)*sin(q2), -cos(q2)*sin(q1), -cos(q1), -L3*sin(q1)*sin(q2); cos(q2), -sin(q2), 0, L1 + L3*cos(q2); 0, 0, 0, 1];
    T3_0 = [-sin(q2 + q3)*cos(q1), -cos(q2 + q3)*cos(q1), sin(q1), -cos(q1)*(L5*sin(q2 + q3) + L3*sin(q2)); -sin(q2 + q3)*sin(q1), -cos(q2 + q3)*sin(q1), -cos(q1), -sin(q1)*(L5*sin(q2 + q3) + L3*sin(q2)); cos(q2 + q3), -sin(q2 + q3), 0, L1 + L5*cos(q2 + q3) + L3*cos(q2); 0, 0, 0, 1];
    T4_0 = [- cos(q1 + q2 + q3 + q4)/2 - cos(q2 - q1 + q3 + q4)/2, sin(q1), - sin(q1 + q2 + q3 + q4)/2 - sin(q2 - q1 + q3 + q4)/2, L2*sin(q1) - L3*cos(q1)*sin(q2) - L5*cos(q1)*cos(q2)*sin(q3) - L5*cos(q1)*cos(q3)*sin(q2); sin(q2 - q1 + q3 + q4)/2 - sin(q1 + q2 + q3 + q4)/2, -cos(q1), cos(q1 + q2 + q3 + q4)/2 - cos(q2 - q1 + q3 + q4)/2, - L2*cos(q1) - L3*sin(q1)*sin(q2) - L5*cos(q2)*sin(q1)*sin(q3) - L5*cos(q3)*sin(q1)*sin(q2); -sin(q2 + q3 + q4), 0, cos(q2 + q3 + q4), L1 + L5*cos(q2 + q3) + L3*cos(q2); 0, 0, 0, 1];
    T5_0 = [sin(q1)*sin(q5) - cos(q1)*cos(q2)*cos(q3)*cos(q4)*cos(q5) + cos(q1)*cos(q2)*cos(q5)*sin(q3)*sin(q4) + cos(q1)*cos(q3)*cos(q5)*sin(q2)*sin(q4) + cos(q1)*cos(q4)*cos(q5)*sin(q2)*sin(q3), sin(q1 + q2 + q3 + q4)/2 + sin(q2 - q1 + q3 + q4)/2, cos(q5)*sin(q1) + cos(q1)*cos(q2)*cos(q3)*cos(q4)*sin(q5) - cos(q1)*cos(q2)*sin(q3)*sin(q4)*sin(q5) - cos(q1)*cos(q3)*sin(q2)*sin(q4)*sin(q5) - cos(q1)*cos(q4)*sin(q2)*sin(q3)*sin(q5), L2*sin(q1) - L3*cos(q1)*sin(q2) - L5*cos(q1)*cos(q2)*sin(q3) - L5*cos(q1)*cos(q3)*sin(q2) - L6*cos(q1)*cos(q2)*cos(q3)*sin(q4) - L6*cos(q1)*cos(q2)*cos(q4)*sin(q3) - L6*cos(q1)*cos(q3)*cos(q4)*sin(q2) + L6*cos(q1)*sin(q2)*sin(q3)*sin(q4); cos(q2)*cos(q5)*sin(q1)*sin(q3)*sin(q4) - cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q1) - cos(q1)*sin(q5) + cos(q3)*cos(q5)*sin(q1)*sin(q2)*sin(q4) + cos(q4)*cos(q5)*sin(q1)*sin(q2)*sin(q3), cos(q2 - q1 + q3 + q4)/2 - cos(q1 + q2 + q3 + q4)/2, cos(q2)*cos(q3)*cos(q4)*sin(q1)*sin(q5) - cos(q1)*cos(q5) - cos(q2)*sin(q1)*sin(q3)*sin(q4)*sin(q5) - cos(q3)*sin(q1)*sin(q2)*sin(q4)*sin(q5) - cos(q4)*sin(q1)*sin(q2)*sin(q3)*sin(q5), L6*sin(q1)*sin(q2)*sin(q3)*sin(q4) - L3*sin(q1)*sin(q2) - L5*cos(q2)*sin(q1)*sin(q3) - L5*cos(q3)*sin(q1)*sin(q2) - L6*cos(q2)*cos(q3)*sin(q1)*sin(q4) - L6*cos(q2)*cos(q4)*sin(q1)*sin(q3) - L6*cos(q3)*cos(q4)*sin(q1)*sin(q2) - L2*cos(q1); - sin(q2 + q3 + q4 + q5)/2 - sin(q2 + q3 + q4 - q5)/2, -cos(q2 + q3 + q4), cos(q2 + q3 + q4 - q5)/2 - cos(q2 + q3 + q4 + q5)/2, L1 + L5*cos(q2 + q3) + L3*cos(q2) + L6*cos(q2 + q3 + q4); 0, 0, 0, 1];
    T6_0 = [cos(q6)*sin(q1)*sin(q5) + cos(q1)*cos(q2)*cos(q3)*sin(q4)*sin(q6) + cos(q1)*cos(q2)*cos(q4)*sin(q3)*sin(q6) + cos(q1)*cos(q3)*cos(q4)*sin(q2)*sin(q6) - cos(q1)*sin(q2)*sin(q3)*sin(q4)*sin(q6) - cos(q1)*cos(q2)*cos(q3)*cos(q4)*cos(q5)*cos(q6) + cos(q1)*cos(q2)*cos(q5)*cos(q6)*sin(q3)*sin(q4) + cos(q1)*cos(q3)*cos(q5)*cos(q6)*sin(q2)*sin(q4) + cos(q1)*cos(q4)*cos(q5)*cos(q6)*sin(q2)*sin(q3), cos(q1)*cos(q2)*cos(q3)*cos(q6)*sin(q4) - sin(q1)*sin(q5)*sin(q6) + cos(q1)*cos(q2)*cos(q4)*cos(q6)*sin(q3) + cos(q1)*cos(q3)*cos(q4)*cos(q6)*sin(q2) - cos(q1)*cos(q6)*sin(q2)*sin(q3)*sin(q4) + cos(q1)*cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q6) - cos(q1)*cos(q2)*cos(q5)*sin(q3)*sin(q4)*sin(q6) - cos(q1)*cos(q3)*cos(q5)*sin(q2)*sin(q4)*sin(q6) - cos(q1)*cos(q4)*cos(q5)*sin(q2)*sin(q3)*sin(q6), cos(q5)*sin(q1) + cos(q1)*cos(q2)*cos(q3)*cos(q4)*sin(q5) - cos(q1)*cos(q2)*sin(q3)*sin(q4)*sin(q5) - cos(q1)*cos(q3)*sin(q2)*sin(q4)*sin(q5) - cos(q1)*cos(q4)*sin(q2)*sin(q3)*sin(q5), L2*sin(q1) - L3*cos(q1)*sin(q2) + L4*cos(q5)*sin(q1) - L5*cos(q1)*cos(q2)*sin(q3) - L5*cos(q1)*cos(q3)*sin(q2) - L6*cos(q1)*cos(q2)*cos(q3)*sin(q4) - L6*cos(q1)*cos(q2)*cos(q4)*sin(q3) - L6*cos(q1)*cos(q3)*cos(q4)*sin(q2) + L6*cos(q1)*sin(q2)*sin(q3)*sin(q4) - L4*cos(q1)*cos(q2)*sin(q3)*sin(q4)*sin(q5) - L4*cos(q1)*cos(q3)*sin(q2)*sin(q4)*sin(q5) - L4*cos(q1)*cos(q4)*sin(q2)*sin(q3)*sin(q5) + L4*cos(q1)*cos(q2)*cos(q3)*cos(q4)*sin(q5); cos(q2)*cos(q3)*sin(q1)*sin(q4)*sin(q6) - cos(q1)*cos(q6)*sin(q5) + cos(q2)*cos(q4)*sin(q1)*sin(q3)*sin(q6) + cos(q3)*cos(q4)*sin(q1)*sin(q2)*sin(q6) - sin(q1)*sin(q2)*sin(q3)*sin(q4)*sin(q6) - cos(q2)*cos(q3)*cos(q4)*cos(q5)*cos(q6)*sin(q1) + cos(q2)*cos(q5)*cos(q6)*sin(q1)*sin(q3)*sin(q4) + cos(q3)*cos(q5)*cos(q6)*sin(q1)*sin(q2)*sin(q4) + cos(q4)*cos(q5)*cos(q6)*sin(q1)*sin(q2)*sin(q3), cos(q1)*sin(q5)*sin(q6) + cos(q2)*cos(q3)*cos(q6)*sin(q1)*sin(q4) + cos(q2)*cos(q4)*cos(q6)*sin(q1)*sin(q3) + cos(q3)*cos(q4)*cos(q6)*sin(q1)*sin(q2) - cos(q6)*sin(q1)*sin(q2)*sin(q3)*sin(q4) + cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q1)*sin(q6) - cos(q2)*cos(q5)*sin(q1)*sin(q3)*sin(q4)*sin(q6) - cos(q3)*cos(q5)*sin(q1)*sin(q2)*sin(q4)*sin(q6) - cos(q4)*cos(q5)*sin(q1)*sin(q2)*sin(q3)*sin(q6), cos(q2)*cos(q3)*cos(q4)*sin(q1)*sin(q5) - cos(q1)*cos(q5) - cos(q2)*sin(q1)*sin(q3)*sin(q4)*sin(q5) - cos(q3)*sin(q1)*sin(q2)*sin(q4)*sin(q5) - cos(q4)*sin(q1)*sin(q2)*sin(q3)*sin(q5), L6*sin(q1)*sin(q2)*sin(q3)*sin(q4) - L4*cos(q1)*cos(q5) - L3*sin(q1)*sin(q2) - L5*cos(q2)*sin(q1)*sin(q3) - L5*cos(q3)*sin(q1)*sin(q2) - L6*cos(q2)*cos(q3)*sin(q1)*sin(q4) - L6*cos(q2)*cos(q4)*sin(q1)*sin(q3) - L6*cos(q3)*cos(q4)*sin(q1)*sin(q2) - L2*cos(q1) - L4*cos(q2)*sin(q1)*sin(q3)*sin(q4)*sin(q5) - L4*cos(q3)*sin(q1)*sin(q2)*sin(q4)*sin(q5) - L4*cos(q4)*sin(q1)*sin(q2)*sin(q3)*sin(q5) + L4*cos(q2)*cos(q3)*cos(q4)*sin(q1)*sin(q5); cos(q2)*sin(q3)*sin(q4)*sin(q6) - cos(q2)*cos(q3)*cos(q4)*sin(q6) + cos(q3)*sin(q2)*sin(q4)*sin(q6) + cos(q4)*sin(q2)*sin(q3)*sin(q6) - cos(q2)*cos(q3)*cos(q5)*cos(q6)*sin(q4) - cos(q2)*cos(q4)*cos(q5)*cos(q6)*sin(q3) - cos(q3)*cos(q4)*cos(q5)*cos(q6)*sin(q2) + cos(q5)*cos(q6)*sin(q2)*sin(q3)*sin(q4), cos(q2)*cos(q6)*sin(q3)*sin(q4) - cos(q2)*cos(q3)*cos(q4)*cos(q6) + cos(q3)*cos(q6)*sin(q2)*sin(q4) + cos(q4)*cos(q6)*sin(q2)*sin(q3) + cos(q2)*cos(q3)*cos(q5)*sin(q4)*sin(q6) + cos(q2)*cos(q4)*cos(q5)*sin(q3)*sin(q6) + cos(q3)*cos(q4)*cos(q5)*sin(q2)*sin(q6) - cos(q5)*sin(q2)*sin(q3)*sin(q4)*sin(q6), cos(q2 + q3 + q4 - q5)/2 - cos(q2 + q3 + q4 + q5)/2, L1 - (L4*cos(q2 + q3 + q4 + q5))/2 + L5*cos(q2 + q3) + L3*cos(q2) + (L4*cos(q2 + q3 + q4 - q5))/2 + L6*cos(q2 + q3 + q4); 0, 0, 0, 1];


    Tcm1_0 = [cos(q1), 0, sin(q1), 0; sin(q1), 0, -cos(q1), 0; 0, 1, 0, L7; 0, 0, 0, 1];
    Tcm2_0 = [-cos(q1)*sin(q2), -cos(q1)*cos(q2), sin(q1), L8*sin(q1) - L9*cos(q1)*sin(q2); -sin(q1)*sin(q2), -cos(q2)*sin(q1), -cos(q1), - L8*cos(q1) - L9*sin(q1)*sin(q2); cos(q2), -sin(q2), 0, L1 + L9*cos(q2); 0, 0, 0, 1];
    Tcm3_0 = [-sin(q2 + q3)*cos(q1), -cos(q2 + q3)*cos(q1), sin(q1), L10*sin(q1) - L3*cos(q1)*sin(q2) - L11*cos(q1)*cos(q2)*sin(q3) - L11*cos(q1)*cos(q3)*sin(q2); -sin(q2 + q3)*sin(q1), -cos(q2 + q3)*sin(q1), -cos(q1), - L10*cos(q1) - L3*sin(q1)*sin(q2) - L11*cos(q2)*sin(q1)*sin(q3) - L11*cos(q3)*sin(q1)*sin(q2); cos(q2 + q3), -sin(q2 + q3), 0, L1 + L11*cos(q2 + q3) + L3*cos(q2); 0, 0, 0, 1];
    Tcm4_0 = [- sin(q1 + q2 + q3 + q4)/2 - sin(q2 - q1 + q3 + q4)/2, - cos(q1 + q2 + q3 + q4)/2 - cos(q2 - q1 + q3 + q4)/2, sin(q1), L12*sin(q1) - L3*cos(q1)*sin(q2) - L5*cos(q1)*cos(q2)*sin(q3) - L5*cos(q1)*cos(q3)*sin(q2) - L13*cos(q1)*cos(q2)*cos(q3)*sin(q4) - L13*cos(q1)*cos(q2)*cos(q4)*sin(q3) - L13*cos(q1)*cos(q3)*cos(q4)*sin(q2) + L13*cos(q1)*sin(q2)*sin(q3)*sin(q4); cos(q1 + q2 + q3 + q4)/2 - cos(q2 - q1 + q3 + q4)/2, sin(q2 - q1 + q3 + q4)/2 - sin(q1 + q2 + q3 + q4)/2, -cos(q1), L13*sin(q1)*sin(q2)*sin(q3)*sin(q4) - L3*sin(q1)*sin(q2) - L5*cos(q2)*sin(q1)*sin(q3) - L5*cos(q3)*sin(q1)*sin(q2) - L13*cos(q2)*cos(q3)*sin(q1)*sin(q4) - L13*cos(q2)*cos(q4)*sin(q1)*sin(q3) - L13*cos(q3)*cos(q4)*sin(q1)*sin(q2) - L12*cos(q1); cos(q2 + q3 + q4), -sin(q2 + q3 + q4), 0, L1 + L5*cos(q2 + q3) + L3*cos(q2) + L13*cos(q2 + q3 + q4); 0, 0, 0, 1];
    Tcm5_0 = [sin(q1)*sin(q5) - cos(q1)*cos(q2)*cos(q3)*cos(q4)*cos(q5) + cos(q1)*cos(q2)*cos(q5)*sin(q3)*sin(q4) + cos(q1)*cos(q3)*cos(q5)*sin(q2)*sin(q4) + cos(q1)*cos(q4)*cos(q5)*sin(q2)*sin(q3), cos(q5)*sin(q1) + cos(q1)*cos(q2)*cos(q3)*cos(q4)*sin(q5) - cos(q1)*cos(q2)*sin(q3)*sin(q4)*sin(q5) - cos(q1)*cos(q3)*sin(q2)*sin(q4)*sin(q5) - cos(q1)*cos(q4)*sin(q2)*sin(q3)*sin(q5), - sin(q1 + q2 + q3 + q4)/2 - sin(q2 - q1 + q3 + q4)/2, L2*sin(q1) - L3*cos(q1)*sin(q2) - L5*cos(q1)*cos(q2)*sin(q3) - L5*cos(q1)*cos(q3)*sin(q2) - L14*cos(q1)*cos(q2)*cos(q3)*sin(q4) - L14*cos(q1)*cos(q2)*cos(q4)*sin(q3) - L14*cos(q1)*cos(q3)*cos(q4)*sin(q2) + L14*cos(q1)*sin(q2)*sin(q3)*sin(q4); cos(q2)*cos(q5)*sin(q1)*sin(q3)*sin(q4) - cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q1) - cos(q1)*sin(q5) + cos(q3)*cos(q5)*sin(q1)*sin(q2)*sin(q4) + cos(q4)*cos(q5)*sin(q1)*sin(q2)*sin(q3), cos(q2)*cos(q3)*cos(q4)*sin(q1)*sin(q5) - cos(q1)*cos(q5) - cos(q2)*sin(q1)*sin(q3)*sin(q4)*sin(q5) - cos(q3)*sin(q1)*sin(q2)*sin(q4)*sin(q5) - cos(q4)*sin(q1)*sin(q2)*sin(q3)*sin(q5), cos(q1 + q2 + q3 + q4)/2 - cos(q2 - q1 + q3 + q4)/2, L14*sin(q1)*sin(q2)*sin(q3)*sin(q4) - L3*sin(q1)*sin(q2) - L5*cos(q2)*sin(q1)*sin(q3) - L5*cos(q3)*sin(q1)*sin(q2) - L14*cos(q2)*cos(q3)*sin(q1)*sin(q4) - L14*cos(q2)*cos(q4)*sin(q1)*sin(q3) - L14*cos(q3)*cos(q4)*sin(q1)*sin(q2) - L2*cos(q1); - sin(q2 + q3 + q4 + q5)/2 - sin(q2 + q3 + q4 - q5)/2, cos(q2 + q3 + q4 - q5)/2 - cos(q2 + q3 + q4 + q5)/2, cos(q2 + q3 + q4), L1 + L5*cos(q2 + q3) + L3*cos(q2) + L14*cos(q2 + q3 + q4); 0, 0, 0, 1];
    Tcm6_0 = [cos(q6)*sin(q1)*sin(q5) + cos(q1)*cos(q2)*cos(q3)*sin(q4)*sin(q6) + cos(q1)*cos(q2)*cos(q4)*sin(q3)*sin(q6) + cos(q1)*cos(q3)*cos(q4)*sin(q2)*sin(q6) - cos(q1)*sin(q2)*sin(q3)*sin(q4)*sin(q6) - cos(q1)*cos(q2)*cos(q3)*cos(q4)*cos(q5)*cos(q6) + cos(q1)*cos(q2)*cos(q5)*cos(q6)*sin(q3)*sin(q4) + cos(q1)*cos(q3)*cos(q5)*cos(q6)*sin(q2)*sin(q4) + cos(q1)*cos(q4)*cos(q5)*cos(q6)*sin(q2)*sin(q3), cos(q1)*cos(q2)*cos(q3)*cos(q6)*sin(q4) - sin(q1)*sin(q5)*sin(q6) + cos(q1)*cos(q2)*cos(q4)*cos(q6)*sin(q3) + cos(q1)*cos(q3)*cos(q4)*cos(q6)*sin(q2) - cos(q1)*cos(q6)*sin(q2)*sin(q3)*sin(q4) + cos(q1)*cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q6) - cos(q1)*cos(q2)*cos(q5)*sin(q3)*sin(q4)*sin(q6) - cos(q1)*cos(q3)*cos(q5)*sin(q2)*sin(q4)*sin(q6) - cos(q1)*cos(q4)*cos(q5)*sin(q2)*sin(q3)*sin(q6), cos(q5)*sin(q1) + cos(q1)*cos(q2)*cos(q3)*cos(q4)*sin(q5) - cos(q1)*cos(q2)*sin(q3)*sin(q4)*sin(q5) - cos(q1)*cos(q3)*sin(q2)*sin(q4)*sin(q5) - cos(q1)*cos(q4)*sin(q2)*sin(q3)*sin(q5), L2*sin(q1) - L3*cos(q1)*sin(q2) + L15*cos(q5)*sin(q1) - L5*cos(q1)*cos(q2)*sin(q3) - L5*cos(q1)*cos(q3)*sin(q2) - L6*cos(q1)*cos(q2)*cos(q3)*sin(q4) - L6*cos(q1)*cos(q2)*cos(q4)*sin(q3) - L6*cos(q1)*cos(q3)*cos(q4)*sin(q2) + L6*cos(q1)*sin(q2)*sin(q3)*sin(q4) - L15*cos(q1)*cos(q2)*sin(q3)*sin(q4)*sin(q5) - L15*cos(q1)*cos(q3)*sin(q2)*sin(q4)*sin(q5) - L15*cos(q1)*cos(q4)*sin(q2)*sin(q3)*sin(q5) + L15*cos(q1)*cos(q2)*cos(q3)*cos(q4)*sin(q5); cos(q2)*cos(q3)*sin(q1)*sin(q4)*sin(q6) - cos(q1)*cos(q6)*sin(q5) + cos(q2)*cos(q4)*sin(q1)*sin(q3)*sin(q6) + cos(q3)*cos(q4)*sin(q1)*sin(q2)*sin(q6) - sin(q1)*sin(q2)*sin(q3)*sin(q4)*sin(q6) - cos(q2)*cos(q3)*cos(q4)*cos(q5)*cos(q6)*sin(q1) + cos(q2)*cos(q5)*cos(q6)*sin(q1)*sin(q3)*sin(q4) + cos(q3)*cos(q5)*cos(q6)*sin(q1)*sin(q2)*sin(q4) + cos(q4)*cos(q5)*cos(q6)*sin(q1)*sin(q2)*sin(q3), cos(q1)*sin(q5)*sin(q6) + cos(q2)*cos(q3)*cos(q6)*sin(q1)*sin(q4) + cos(q2)*cos(q4)*cos(q6)*sin(q1)*sin(q3) + cos(q3)*cos(q4)*cos(q6)*sin(q1)*sin(q2) - cos(q6)*sin(q1)*sin(q2)*sin(q3)*sin(q4) + cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q1)*sin(q6) - cos(q2)*cos(q5)*sin(q1)*sin(q3)*sin(q4)*sin(q6) - cos(q3)*cos(q5)*sin(q1)*sin(q2)*sin(q4)*sin(q6) - cos(q4)*cos(q5)*sin(q1)*sin(q2)*sin(q3)*sin(q6), cos(q2)*cos(q3)*cos(q4)*sin(q1)*sin(q5) - cos(q1)*cos(q5) - cos(q2)*sin(q1)*sin(q3)*sin(q4)*sin(q5) - cos(q3)*sin(q1)*sin(q2)*sin(q4)*sin(q5) - cos(q4)*sin(q1)*sin(q2)*sin(q3)*sin(q5), L6*sin(q1)*sin(q2)*sin(q3)*sin(q4) - L15*cos(q1)*cos(q5) - L3*sin(q1)*sin(q2) - L5*cos(q2)*sin(q1)*sin(q3) - L5*cos(q3)*sin(q1)*sin(q2) - L6*cos(q2)*cos(q3)*sin(q1)*sin(q4) - L6*cos(q2)*cos(q4)*sin(q1)*sin(q3) - L6*cos(q3)*cos(q4)*sin(q1)*sin(q2) - L2*cos(q1) - L15*cos(q2)*sin(q1)*sin(q3)*sin(q4)*sin(q5) - L15*cos(q3)*sin(q1)*sin(q2)*sin(q4)*sin(q5) - L15*cos(q4)*sin(q1)*sin(q2)*sin(q3)*sin(q5) + L15*cos(q2)*cos(q3)*cos(q4)*sin(q1)*sin(q5); cos(q2)*sin(q3)*sin(q4)*sin(q6) - cos(q2)*cos(q3)*cos(q4)*sin(q6) + cos(q3)*sin(q2)*sin(q4)*sin(q6) + cos(q4)*sin(q2)*sin(q3)*sin(q6) - cos(q2)*cos(q3)*cos(q5)*cos(q6)*sin(q4) - cos(q2)*cos(q4)*cos(q5)*cos(q6)*sin(q3) - cos(q3)*cos(q4)*cos(q5)*cos(q6)*sin(q2) + cos(q5)*cos(q6)*sin(q2)*sin(q3)*sin(q4), cos(q2)*cos(q6)*sin(q3)*sin(q4) - cos(q2)*cos(q3)*cos(q4)*cos(q6) + cos(q3)*cos(q6)*sin(q2)*sin(q4) + cos(q4)*cos(q6)*sin(q2)*sin(q3) + cos(q2)*cos(q3)*cos(q5)*sin(q4)*sin(q6) + cos(q2)*cos(q4)*cos(q5)*sin(q3)*sin(q6) + cos(q3)*cos(q4)*cos(q5)*sin(q2)*sin(q6) - cos(q5)*sin(q2)*sin(q3)*sin(q4)*sin(q6), cos(q2 + q3 + q4 - q5)/2 - cos(q2 + q3 + q4 + q5)/2, L1 - (L15*cos(q2 + q3 + q4 + q5))/2 + L5*cos(q2 + q3) + L3*cos(q2) + (L15*cos(q2 + q3 + q4 - q5))/2 + L6*cos(q2 + q3 + q4); 0, 0, 0, 1];

    % Get the position of the end-effector
    [~, Xef_0pos, ~] = FKef_0_ur10_3DOF([robot_param' Q']);
    Xef_Wpos = T0_W * [Xef_0pos'; 1];
    Xef_Wpos = Xef_Wpos(1:3);
    %Initialize the publishers and messages
    if t == 0

        %% TF publisher
        tftree = rostf;

        tfStampedMsg = rosmessage('geometry_msgs/TransformStamped');
        tfStampedMsg.Header.FrameId = 'world';
        tfStampedMsg.ChildFrameId = 'DH_0';

        tfStampedMsg1 = rosmessage('geometry_msgs/TransformStamped');
        tfStampedMsg1.Header.FrameId = 'ursa_base';
        tfStampedMsg1.ChildFrameId = 'DH_1';

        tfStampedMsg2 = rosmessage('geometry_msgs/TransformStamped');
        tfStampedMsg2.Header.FrameId = 'ursa_base';
        tfStampedMsg2.ChildFrameId = 'DH_2';

        tfStampedMsg3 = rosmessage('geometry_msgs/TransformStamped');
        tfStampedMsg3.Header.FrameId = 'ursa_base';
        tfStampedMsg3.ChildFrameId = 'DH_3';

        tfStampedMsg4 = rosmessage('geometry_msgs/TransformStamped');
        tfStampedMsg4.Header.FrameId = 'ursa_base';
        tfStampedMsg4.ChildFrameId = 'DH_4';

        tfStampedMsg5 = rosmessage('geometry_msgs/TransformStamped');
        tfStampedMsg5.Header.FrameId = 'ursa_base';
        tfStampedMsg5.ChildFrameId = 'DH_5';

        tfStampedMsg6 = rosmessage('geometry_msgs/TransformStamped');
        tfStampedMsg6.Header.FrameId = 'ursa_base';
        tfStampedMsg6.ChildFrameId = 'DH_6';

        % tfStampedMsgcm1 = rosmessage('geometry_msgs/TransformStamped');
        % tfStampedMsgcm1.Header.FrameId = 'ursa_base';
        % tfStampedMsgcm1.ChildFrameId = 'DH_cm1';
        % tfStampedMsgcm1.ChildFrameId = strcat('DH_cm',int2str(i));
        % tfStampedMsgcm1.Header.Stamp = rostime('now');
        % tfStampedMsgcm1.Header.Seq=counter;

        %% Joint State Publisher
        %Use here the correct topic name --see bringup launch file--
        jointpub = rospublisher('/ursa_joint_states', 'sensor_msgs/JointState');
        jointmsg = rosmessage(jointpub);

        % specific names of the joints --see urdf file--
        jointmsg.Name = {'ursa_shoulder_pan_joint', ...
                        'ursa_shoulder_lift_joint', ...
                        'ursa_elbow_joint', ...
                        'ursa_wrist_1_joint', ...
                        'ursa_wrist_2_joint', ...
                        'ursa_wrist_3_joint'};

        for i = 1:6
            jointmsg.Velocity(i) = 0.0;
            jointmsg.Effort(i) = 0.0;
        end

        counter = 0;

    end

    %% JOINT STATE MSG and TF MSG
    joint_offsets = [0, -pi / 2, 0, -pi / 2, 0, 0];

    sampleTime = 0.06;

    if (~mod(t, sampleTime))
        rT = rostime('now');

        jointmsg.Header.Stamp = rT;
        jointmsg.Header.Seq = counter;
        jointmsg.Position = [Q', 0, 0, 0] + joint_offsets;
        send(jointpub, jointmsg);

        getTF(tfStampedMsg, T0_W, counter, rT);
        getTF(tfStampedMsg1, T1_0, counter, rT);
        getTF(tfStampedMsg2, T2_0, counter, rT);
        getTF(tfStampedMsg3, T3_0, counter, rT);
        getTF(tfStampedMsg4, T4_0, counter, rT);
        getTF(tfStampedMsg5, T5_0, counter, rT);
        getTF(tfStampedMsg6, T6_0, counter, rT);

        arrayTFs = [tfStampedMsg;
                tfStampedMsg1;
                tfStampedMsg2;
                tfStampedMsg3;
                tfStampedMsg4;
                tfStampedMsg5;
                tfStampedMsg6];

        counter = counter + 1;

        sendTransform(tftree, arrayTFs);
    end

    %%  Lyapanov function

    %Inertia Matrix
    M = [I122 + I211 / 2 + I222 / 2 + I311 / 2 + I322 / 2 + (I311 * cos(2 * q2 + 2 * q3)) / 2 - (I322 * cos(2 * q2 + 2 * q3)) / 2 - I312 * sin(2 * q2 + 2 * q3) + (L3^2 * m3) / 2 + L7^2 * m2 + (L8^2 * m2) / 2 + L9^2 * m3 + (L10^2 * m3) / 2 + (I211 * cos(2 * q2)) / 2 - (I222 * cos(2 * q2)) / 2 - I212 * sin(2 * q2) - (L3^2 * m3 * cos(2 * q2)) / 2 - (L8^2 * m2 * cos(2 * q2)) / 2 - (L10^2 * m3 * cos(2 * q2 + 2 * q3)) / 2 + L3 * L10 * m3 * cos(q3) - L3 * L10 * m3 * cos(2 * q2 + q3), I313 * cos(q2 + q3) - I323 * sin(q2 + q3) + I213 * cos(q2) - I223 * sin(q2) - L9 * L10 * m3 * cos(q2 + q3) - L3 * L9 * m3 * cos(q2) - L7 * L8 * m2 * cos(q2), I313 * cos(q2 + q3) - I323 * sin(q2 + q3) - L9 * L10 * m3 * cos(q2 + q3); I313 * cos(q2 + q3) - I323 * sin(q2 + q3) + I213 * cos(q2) - I223 * sin(q2) - L9 * L10 * m3 * cos(q2 + q3) - L3 * L9 * m3 * cos(q2) - L7 * L8 * m2 * cos(q2), I233 + I333 + L3^2 * m3 + L8^2 * m2 + L10^2 * m3 + 2 * L3 * L10 * m3 * cos(q3), I333 + L10^2 * m3 + L3 * L10 * m3 * cos(q3); I313 * cos(q2 + q3) - I323 * sin(q2 + q3) - L9 * L10 * m3 * cos(q2 + q3), I333 + L10^2 * m3 + L3 * L10 * m3 * cos(q3), I333 + L10^2 * m3];

    % V = 1/2 * Qp' * M * Qp;
    V = 0;
    
    %% Output: vector [Xef] (size 3X1)
    Out = [Xef_Wpos; Q; Qp; V];

end
