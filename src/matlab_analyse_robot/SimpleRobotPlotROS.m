function [Out] = SimpleRobotPlotROS(u)
    %SIMPLEROBOTPLOT Summary of this function goes here
    persistent jointpub jointmsg counter tftree tfStampedMsg tfStampedMsg1 tfStampedMsg2 tfStampedMsg3
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
    
    Q_param = u(length(Q_param)+1:length(Q_param)+6);
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
    robot_param = u(7:44);
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
    
    Tao = u(45:47);
    %Time
    t = u(48);

    %% Homogenous Transformations
    % Specify the Robot Base (with respect to the world coordinate frame in ROS)
    T0_W = rotm_trvec2tf(RotZ(pi), [0 0 0]');

    % Compute the Homogeneous Transformations
    T1_0 = [cos(q1), 0, sin(q1), 0; sin(q1), 0, -cos(q1), 0; 0, 1, 0, L1; 0, 0, 0, 1];
    T2_0 = [-cos(q1) * sin(q2), -cos(q1) * cos(q2), sin(q1), -L3 * cos(q1) * sin(q2); -sin(q1) * sin(q2), -cos(q2) * sin(q1), -cos(q1), -L3 * sin(q1) * sin(q2); cos(q2), -sin(q2), 0, L1 + L3 * cos(q2); 0, 0, 0, 1];
    T3_0 = [-sin(q2 + q3) * cos(q1), -cos(q2 + q3) * cos(q1), sin(q1), L2 * sin(q1) + L4 * sin(q1) - L3 * cos(q1) * sin(q2) - L5 * cos(q1) * cos(q2) * sin(q3) - L5 * cos(q1) * cos(q3) * sin(q2); -sin(q2 + q3) * sin(q1), -cos(q2 + q3) * sin(q1), -cos(q1), - L2 * cos(q1) - L4 * cos(q1) - L3 * sin(q1) * sin(q2) - L5 * cos(q2) * sin(q1) * sin(q3) - L5 * cos(q3) * sin(q1) * sin(q2); cos(q2 + q3), -sin(q2 + q3), 0, L1 + L5 * cos(q2 + q3) + L3 * cos(q2); 0, 0, 0, 1];

    Tcm1_0 = [cos(q1), 0, sin(q1), 0; sin(q1), 0, -cos(q1), 0; 0, 1, 0, L6; 0, 0, 0, 1];
    Tcm2_0 = [-cos(q1) * sin(q2), -cos(q1) * cos(q2), sin(q1), L7 * sin(q1) - L8 * cos(q1) * sin(q2); -sin(q1) * sin(q2), -cos(q2) * sin(q1), -cos(q1), - L7 * cos(q1) - L8 * sin(q1) * sin(q2); cos(q2), -sin(q2), 0, L1 + L8 * cos(q2); 0, 0, 0, 1];
    Tcm3_0 = [-sin(q2 + q3) * cos(q1), -cos(q2 + q3) * cos(q1), sin(q1), L9 * sin(q1) - L3 * cos(q1) * sin(q2) - L10 * cos(q1) * cos(q2) * sin(q3) - L10 * cos(q1) * cos(q3) * sin(q2); -sin(q2 + q3) * sin(q1), -cos(q2 + q3) * sin(q1), -cos(q1), - L9 * cos(q1) - L3 * sin(q1) * sin(q2) - L10 * cos(q2) * sin(q1) * sin(q3) - L10 * cos(q3) * sin(q1) * sin(q2); cos(q2 + q3), -sin(q2 + q3), 0, L1 + L10 * cos(q2 + q3) + L3 * cos(q2); 0, 0, 0, 1];

    % Get the position of the end-effector
    [~, Xef_0pos,~] = FKef_0_ur10_3DOF([robot_param' Q']);
    Xef_Wpos = T0_W * [Xef_0pos';1];
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

        tfStampedMsg3 = rosmessage('geometry_msgs/TransformStamped');
        tfStampedMsg3.Header.FrameId = 'ursa_base';
        tfStampedMsg3.ChildFrameId = 'DH_4';
        
        tfStampedMsg3 = rosmessage('geometry_msgs/TransformStamped');
        tfStampedMsg3.Header.FrameId = 'ursa_base';
        tfStampedMsg3.ChildFrameId = 'DH_5';
        
        tfStampedMsg3 = rosmessage('geometry_msgs/TransformStamped');
        tfStampedMsg3.Header.FrameId = 'ursa_base';
        tfStampedMsg3.ChildFrameId = 'DH_6';
        


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

        arrayTFs = [tfStampedMsg;
                tfStampedMsg1;
                tfStampedMsg2;
                tfStampedMsg3];

        counter = counter + 1;

        sendTransform(tftree, arrayTFs);
    end

    %%  Lyapanov function

    %Inertia Matrix
    M = [I122 + I211 / 2 + I222 / 2 + I311 / 2 + I322 / 2 + (I311 * cos(2 * q2 + 2 * q3)) / 2 - (I322 * cos(2 * q2 + 2 * q3)) / 2 - I312 * sin(2 * q2 + 2 * q3) + (L3^2 * m3) / 2 + L7^2 * m2 + (L8^2 * m2) / 2 + L9^2 * m3 + (L10^2 * m3) / 2 + (I211 * cos(2 * q2)) / 2 - (I222 * cos(2 * q2)) / 2 - I212 * sin(2 * q2) - (L3^2 * m3 * cos(2 * q2)) / 2 - (L8^2 * m2 * cos(2 * q2)) / 2 - (L10^2 * m3 * cos(2 * q2 + 2 * q3)) / 2 + L3 * L10 * m3 * cos(q3) - L3 * L10 * m3 * cos(2 * q2 + q3), I313 * cos(q2 + q3) - I323 * sin(q2 + q3) + I213 * cos(q2) - I223 * sin(q2) - L9 * L10 * m3 * cos(q2 + q3) - L3 * L9 * m3 * cos(q2) - L7 * L8 * m2 * cos(q2), I313 * cos(q2 + q3) - I323 * sin(q2 + q3) - L9 * L10 * m3 * cos(q2 + q3); I313 * cos(q2 + q3) - I323 * sin(q2 + q3) + I213 * cos(q2) - I223 * sin(q2) - L9 * L10 * m3 * cos(q2 + q3) - L3 * L9 * m3 * cos(q2) - L7 * L8 * m2 * cos(q2), I233 + I333 + L3^2 * m3 + L8^2 * m2 + L10^2 * m3 + 2 * L3 * L10 * m3 * cos(q3), I333 + L10^2 * m3 + L3 * L10 * m3 * cos(q3); I313 * cos(q2 + q3) - I323 * sin(q2 + q3) - L9 * L10 * m3 * cos(q2 + q3), I333 + L10^2 * m3 + L3 * L10 * m3 * cos(q3), I333 + L10^2 * m3];

    V = 1/2 * Qp' * M * Qp;

    %% Output: vector [Xef] (size 3X1)
    Out = [Xef_Wpos; Q; Qp; V];

end
