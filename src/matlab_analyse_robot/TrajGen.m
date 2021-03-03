function [ret] = TrajGen(u)
    %TRAJGEN Summary of this function goes here
    %   DetAmp_jc_tracled explanation goes here
    persistent counter tftree tfStampedMsg path_desired_pub path_ef_pub path_desired_msg path_ef_msg Q_init
    global state_machine anti_windup sumDeltaQ sumDeltaQp

    %% Input Parameters
    %robot params
    robot_param = u(1:38);
    %desired position
    Qd = deg2rad(u(39:41));
    Qppd = zeros(3, 1);

    %current position
    Q = u(42:44)';
    Qp = u(45:47)';
    %time
    t = u(48);

    %% Settings
    % time settings
    t_reg = 10.0;
    t_track1 = 20.0;
    t_track2 = 20.0;

    t_break = 1.0;

    t_pa = 3.0;
    t_pb = 2.5;

    t_pc = 2.5;
    t_oc_track1 = 25;
    t_oc_track2 = 25;

    % zero desired velocity
    Qpd_zero = zeros(3, 1);

    %point a
    Qd_pa = Qd;
    %point b
    Qd_pb = [0; 0; pi / 2];
    [~, Xd_pb, ~] = FKef_0_ur10_3DOF([robot_param' Qd_pb']);
    %point c
    Xd_pc = [-0.55; 0.8; 0.45];

    % joint control
    Amp_jc_track = deg2rad([30; 20; 20]);
    W_jc_track1 = 0.75 * 1.5 * [0.5; 0.2; 0.3];
    W_jc_track2 = 0.75 * 4 * [0.5; 0.2; 0.3];
    phi_jc_offset = [0; 0; 0];

    % operational control
    % normal circle
    circle_center = [-0.5; 0.55; 0.4];
    Amp_op_track = [0.25; 0.25; 0.05];
    % circle to singularity
    %circle_center = [-0.5; 0.6; 0.4];
    %Amp_op_track = [0.4; 0.4; 0.05];
    W_op_track1 = 1 .* [1; 1; 4.0];
    W_op_track2 = 1 .* [0.5; 0.5; 6.0];
    phi_op_offset = [0; pi / 2; 0];

    %% Init
    if (t == 0)
        % initialize state machine
        state_machine = StateMachine();
        state_machine.change_state(TaskState.jc_pd_reg, t_reg);
        anti_windup = ones(3, 1);
        sumDeltaQ = zeros(3, 1);
        sumDeltaQp = zeros(3, 1);
        Q_init = zeros(3, 1);

        %% TF publisher
        tftree = rostf;

        tfStampedMsg = rosmessage('geometry_msgs/TransformStamped');
        tfStampedMsg.Header.FrameId = 'ursa_base';
        tfStampedMsg.ChildFrameId = 'Xdes';
        counter = 0;

        path_desired_pub = rospublisher('/path_des', 'nav_msgs/Path');
        path_ef_pub = rospublisher('/path_ef', 'nav_msgs/Path');

        path_desired_msg = rosmessage(path_desired_pub);
        path_desired_msg.Header.FrameId = 'ursa_base';
        path_ef_msg = rosmessage(path_ef_pub);
        path_ef_msg.Header.FrameId = 'ursa_base';
    end

    %% State machine
    if state_machine.state == TaskState.jc_pd_reg
        Qd = Qd;
        Qpd = zeros(3, 1);
        Qppd = zeros(3, 1);

        if ~state_machine.isrunning()

            path_desired_msg = rosmessage(path_desired_pub);
            path_desired_msg.Header.FrameId = 'ursa_base';
            path_ef_msg = rosmessage(path_ef_pub);
            path_ef_msg.Header.FrameId = 'ursa_base';

            state_machine.change_state(TaskState.jc_pd_track1, t_track1);
        end

    elseif state_machine.state == TaskState.jc_pd_track1
        tm = state_machine.man_time();

        [Qd, Qpd, Qppd] = sinusoid_traj_gen(Amp_jc_track, W_jc_track1, phi_jc_offset, Qd, tm);

        if ~state_machine.isrunning()
            state_machine.change_state(TaskState.jc_pd_track2, t_track2);
        end

    elseif state_machine.state == TaskState.jc_pd_track2
        tm = state_machine.man_time();

        [Qd, Qpd, Qppd] = sinusoid_traj_gen(Amp_jc_track, W_jc_track2, phi_jc_offset, Qd, tm);

        if ~state_machine.isrunning()

            path_desired_msg = rosmessage(path_desired_pub);
            path_desired_msg.Header.FrameId = 'ursa_base';
            path_ef_msg = rosmessage(path_ef_pub);
            path_ef_msg.Header.FrameId = 'ursa_base';

            state_machine.change_state(TaskState.jc_pid_reg, t_reg);
        end

    elseif state_machine.state == TaskState.jc_pid_reg
        Qd = Qd;
        Qpd = zeros(3, 1);
        Qppd = zeros(3, 1);

        if ~state_machine.isrunning()

            path_desired_msg = rosmessage(path_desired_pub);
            path_desired_msg.Header.FrameId = 'ursa_base';
            path_ef_msg = rosmessage(path_ef_pub);
            path_ef_msg.Header.FrameId = 'ursa_base';

            state_machine.change_state(TaskState.jc_pid_track1, t_track1);
        end

    elseif state_machine.state == TaskState.jc_pid_track1
        tm = state_machine.man_time();

        [Qd, Qpd, Qppd] = sinusoid_traj_gen(Amp_jc_track, W_jc_track1, phi_jc_offset, Qd, tm);

        if ~state_machine.isrunning()
            state_machine.change_state(TaskState.jc_pid_track2, t_track2);
        end

    elseif state_machine.state == TaskState.jc_pid_track2
        tm = state_machine.man_time();

        [Qd, Qpd, Qppd] = sinusoid_traj_gen(Amp_jc_track, W_jc_track2, phi_jc_offset, Qd, tm);

        if ~state_machine.isrunning()

            path_desired_msg = rosmessage(path_desired_pub);
            path_desired_msg.Header.FrameId = 'ursa_base';
            path_ef_msg = rosmessage(path_ef_pub);
            path_ef_msg.Header.FrameId = 'ursa_base';

            state_machine.change_state(TaskState.jc_pa, t_pa);
            Q_init = Q;
        end

    elseif state_machine.state == TaskState.jc_pa

        % [Qd, Qdp, ~] = traj_gen(Q_init, Qd_pa, 0, state_machine.t_end, state_machine.man_time());
        Qd = Qd_pa;
        Qpd = zeros(3, 1);
        Qppd = zeros(3, 1);

        if ~state_machine.isrunning()
            state_machine.change_state(TaskState.jc_pb, t_pb);
            Q_init = Q;
        end

    elseif state_machine.state == TaskState.jc_pb

        [Qd, Qpd, Qppd] = traj_gen(Q_init, Qd_pb, 0, state_machine.t_end, state_machine.man_time());
%         Qd = Qd_pb;
%         Qpd = zeros(3, 1);
%         Qppd = zeros(3, 1);

        if ~state_machine.isrunning()
            state_machine.change_state(TaskState.oc_pd_pc, t_pc);

            path_desired_msg = rosmessage(path_desired_pub);
            path_desired_msg.Header.FrameId = 'ursa_base';
            path_ef_msg = rosmessage(path_ef_pub);
            path_ef_msg.Header.FrameId = 'ursa_base';
        end

    elseif state_machine.state == TaskState.oc_pd_pc

        [Xd, Xdp, Xdpp] = traj_gen(Xd_pb, Xd_pc, 0, state_machine.t_end, state_machine.man_time());
        Qd = Xd;
        Qpd = Xdp;
        Qppd = Xdpp;
        [Q, Qp, issingular] = X_Xp_ur10_3DOF(robot_param, Q, Qp);

        if issingular
            state_machine.change_state(TaskState.jc_break, t_break);
        elseif ~state_machine.isrunning()
            state_machine.change_state(TaskState.oc_pd_track1, t_oc_track1);
        end

    elseif state_machine.state == TaskState.oc_pd_track1
        tm = state_machine.man_time();

        [Qd, Qpd, Qppd] = sinusoid_traj_gen(Amp_op_track, W_op_track1, phi_op_offset, circle_center, tm);
        [Q, Qp, issingular] = X_Xp_ur10_3DOF(robot_param, Q, Qp);

        if issingular
            state_machine.change_state(TaskState.jc_break, t_break);
        elseif ~state_machine.isrunning()
            state_machine.change_state(TaskState.oc_pd_track2, t_oc_track2);
        end

    elseif state_machine.state == TaskState.oc_pd_track2
        tm = state_machine.man_time();

        [Qd, Qpd, Qppd] = sinusoid_traj_gen(Amp_op_track, W_op_track2, phi_op_offset, circle_center, tm);
        [Q, Qp, issingular] = X_Xp_ur10_3DOF(robot_param, Q, Qp);

        if issingular
            state_machine.change_state(TaskState.jc_break, t_break);
        elseif ~state_machine.isrunning()
            state_machine.change_state(TaskState.jc_pb2, t_pb);
            Q_init = Q;
        end

    elseif state_machine.state == TaskState.jc_pb2
        [Qd, Qpd, Qppd] = traj_gen(Q_init, Qd_pb, 0, state_machine.t_end, state_machine.man_time());
%         Qd = Qd_pb;
%         Qpd = zeros(3, 1);
%         Qppd = zeros(3, 1);

        if ~state_machine.isrunning()
            state_machine.change_state(TaskState.oc_pid_pc, t_pc);

            path_desired_msg = rosmessage(path_desired_pub);
            path_desired_msg.Header.FrameId = 'ursa_base';
            path_ef_msg = rosmessage(path_ef_pub);
            path_ef_msg.Header.FrameId = 'ursa_base';
        end

    elseif state_machine.state == TaskState.oc_pid_pc

        [Xd, Xdp, Xdpp] = traj_gen(Xd_pb, Xd_pc, 0, state_machine.t_end, state_machine.man_time());
        Qd = Xd;
        Qpd = Xdp;
        Qppd = Xdpp;
        [Q, Qp, issingular] = X_Xp_ur10_3DOF(robot_param, Q, Qp);

        if issingular
            state_machine.change_state(TaskState.jc_break, t_break);
        elseif ~state_machine.isrunning()
            state_machine.change_state(TaskState.oc_pid_track1, t_oc_track1);
        end

    elseif state_machine.state == TaskState.oc_pid_track1
        tm = state_machine.man_time();

        [Qd, Qpd, Qppd] = sinusoid_traj_gen(Amp_op_track, W_op_track1, phi_op_offset, circle_center, tm);
        [Q, Qp, issingular] = X_Xp_ur10_3DOF(robot_param, Q, Qp);

        if issingular
            state_machine.change_state(TaskState.jc_break, t_break);
        elseif ~state_machine.isrunning()
            state_machine.change_state(TaskState.oc_pid_track2, t_oc_track2);
        end

    elseif state_machine.state == TaskState.oc_pid_track2
        tm = state_machine.man_time();

        [Qd, Qpd, Qppd] = sinusoid_traj_gen(Amp_op_track, W_op_track2, phi_op_offset, circle_center, tm);
        [Q, Qp, issingular] = X_Xp_ur10_3DOF(robot_param, Q, Qp);

        if issingular
            state_machine.change_state(TaskState.jc_break, t_break);
        elseif ~state_machine.isrunning()
            state_machine.change_state(TaskState.jc_pa, t_pa);
        end

    elseif state_machine.state == TaskState.jc_break
        Qd = zeros(3, 1);
        Qpd = zeros(3, 1);
        Qppd = zeros(3, 1);

        if ~state_machine.isrunning()
            state_machine.change_state(TaskState.jc_pa, t_pa);
            Q_init = Q;
        end

    end

    isjointcontrol = ~state_machine.operational_control;
    ret = [Qd' Qpd' Qppd' Q Qp isjointcontrol];

    sampleTime = 0.06;

    %plot desired position
    if (~mod(t, sampleTime))

        if state_machine.operational_control
            rT = rostime('now');
            HXdes = eye(4);
            HXdes(1:3, 4) = Qd;
            getTF(tfStampedMsg, HXdes, counter, rT);
            sendTransform(tftree, tfStampedMsg);

            des_pose = rosmessage('geometry_msgs/PoseStamped');
            des_pose.Pose.Position.X = Qd(1);
            des_pose.Pose.Position.Y = Qd(2);
            des_pose.Pose.Position.Z = Qd(3);
            path_desired_msg.Poses = [path_desired_msg.Poses; des_pose];
            send(path_desired_pub, path_desired_msg);

            pose = rosmessage('geometry_msgs/PoseStamped');
            pose.Pose.Position.X = Q(1);
            pose.Pose.Position.Y = Q(2);
            pose.Pose.Position.Z = Q(3);
            path_ef_msg.Poses = [path_ef_msg.Poses; pose];
            send(path_ef_pub, path_ef_msg);

        elseif state_machine.control == control_type.jp_track1 || state_machine.control == control_type.jp_track2

            rT = rostime('now');
            HXdes = eye(4);
            [~, Xd, ~] = FKef_0_ur10_3DOF([robot_param' Qd']);
            HXdes(1:3, 4) = Xd';
            getTF(tfStampedMsg, HXdes, counter, rT);
            sendTransform(tftree, tfStampedMsg);

            des_pose = rosmessage('geometry_msgs/PoseStamped');
            des_pose.Pose.Position.X = Xd(1);
            des_pose.Pose.Position.Y = Xd(2);
            des_pose.Pose.Position.Z = Xd(3);
            path_desired_msg.Poses = [path_desired_msg.Poses; des_pose];
            send(path_desired_pub, path_desired_msg);

            [~, X, ~] = FKef_0_ur10_3DOF([robot_param' Q]);
            pose = rosmessage('geometry_msgs/PoseStamped');
            pose.Pose.Position.X = X(1);
            pose.Pose.Position.Y = X(2);
            pose.Pose.Position.Z = X(3);
            path_ef_msg.Poses = [path_ef_msg.Poses; pose];
            send(path_ef_pub, path_ef_msg);
        end

        if size(path_ef_msg.Poses, 1) > 30
            path_ef_msg.Poses = path_ef_msg.Poses(4:end);
            path_desired_msg.Poses = path_desired_msg.Poses(4:end);
        end

    end

end

function [X, Xp, issingular] = X_Xp_ur10_3DOF(robot_param, Q, Qp)
    [~, X, ~] = FKef_0_ur10_3DOF([robot_param' Q]);
    [~, Jef_0, ~] = Jef_0_ur10_3DOF([robot_param' Q]);
    Xp = Jef_0 * Qp';

    % Manipulability Analisys

    % Compute the manipulability Index (w)
    w = sqrt(det(Jef_0 * Jef_0'));
    wThreshold = 0.07;
    issingular = w < wThreshold;

    Xp = Xp';
end

function [Qd, Qpd, Qppd] = sinusoid_traj_gen(amp, w, phase_shift, zero_offset, tm)

    Qd = amp .* sin(w * tm + phase_shift) + zero_offset;
    Qpd = w .* amp .* cos(w * tm + phase_shift);
    Qppd = -w.^2 .* amp .* sin(w * tm + phase_shift);

end
