classdef StateMachine < handle
    %STATE_MACHINE Summary of this class goes here
    %   Detailed explanation goes here
    properties (Access = public)
        state = TaskState.jc_pd_reg;
        controller = controller_type.pd;
        control = control_type.jp_reg;
        operational_control = false;
        reg_pos;
        t_start;
        t_end;
    end

    methods

        function man_time = man_time(obj)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            man_time = toc(obj.t_start);
        end

        function running = isrunning(obj)
            running = toc(obj.t_start) < obj.t_end;
        end

        function obj = change_state(obj, to_state, task_time)
            obj.t_start = tic;
            obj.state = to_state;

            if obj.state == TaskState.jc_pd_reg
                disp("PD joint control regulation");
                obj.controller = controller_type.pd;
                obj.control = control_type.jp_reg;
                obj.operational_control = false;
                obj.t_end = task_time;

            elseif obj.state == TaskState.jc_pd_track1
                disp("PD joint control tracking 1");
                obj.controller = controller_type.pd;
                obj.control = control_type.jp_track1;
                obj.operational_control = false;
                obj.t_end = task_time;

            elseif obj.state == TaskState.jc_pd_track2
                disp("PD joint control tracking 2");
                obj.controller = controller_type.pd;
                obj.control = control_type.jp_track2;
                obj.operational_control = false;
                obj.t_end = task_time;

            elseif obj.state == TaskState.jc_pid_reg
                disp("PID joint control regulation");
                obj.controller = controller_type.pid;
                obj.control = control_type.jp_reg;
                obj.operational_control = false;
                obj.t_end = task_time;

            elseif obj.state == TaskState.jc_pid_track1
                disp("PID joint control tracking 1");
                obj.controller = controller_type.pid;
                obj.control = control_type.jp_track1;
                obj.operational_control = false;
                obj.t_end = task_time;

            elseif obj.state == TaskState.jc_pid_track2
                disp("PID joint control tracking 2");
                obj.controller = controller_type.pid;
                obj.control = control_type.jp_track2;
                obj.operational_control = false;
                obj.t_end = task_time;

            elseif obj.state == TaskState.jc_pa
                disp("PID joint control - GO TO Pa");
                obj.controller = controller_type.pid;
                obj.control = control_type.jp_reg;
                obj.operational_control = false;
                obj.t_end = task_time;
                obj.reg_pos = [0 0 0];

            elseif obj.state == TaskState.jc_pb
                disp("PID joint control - GO TO Pb");
                obj.controller = controller_type.pid;
                obj.control = control_type.jp_reg;
                obj.operational_control = false;
                obj.t_end = task_time;
                obj.reg_pos = [0 0 pi / 2];

            elseif obj.state == TaskState.jc_pb2
                disp("PID joint control - GO TO Pb");
                obj.controller = controller_type.pid;
                obj.control = control_type.jp_reg;
                obj.operational_control = false;
                obj.t_end = task_time;
                obj.reg_pos = [0 0 pi / 2];

            elseif obj.state == TaskState.oc_pd_pc
                disp("PD operational control line tracking FROM Pb TO Pc");
                obj.controller = controller_type.pd;
                obj.control = control_type.op_track_lin;
                obj.operational_control = true;
                obj.t_end = task_time;

            elseif obj.state == TaskState.oc_pd_track1
                disp("PD operational control crown tracking1");
                obj.controller = controller_type.pd;
                obj.control = control_type.op_track1;
                obj.operational_control = true;
                obj.t_end = task_time;

            elseif obj.state == TaskState.oc_pd_track2
                disp("PD operational control crown tracking2");
                obj.controller = controller_type.pd;
                obj.control = control_type.op_track2;
                obj.operational_control = true;
                obj.t_end = task_time;

            elseif obj.state == TaskState.oc_pid_pc
                disp("PID operational control line tracking FROM Pb TO Pc");
                obj.controller = controller_type.pid;
                obj.control = control_type.op_track_lin;
                obj.operational_control = true;
                obj.t_end = task_time;

            elseif obj.state == TaskState.oc_pid_track1
                disp("PID operational control crown tracking1");
                obj.controller = controller_type.pid;
                obj.control = control_type.op_track1;
                obj.operational_control = true;
                obj.t_end = task_time;

            elseif obj.state == TaskState.oc_pid_track2
                disp("PID operational control crown tracking2");
                obj.controller = controller_type.pid;
                obj.control = control_type.op_track2;
                obj.operational_control = true;
                obj.t_end = task_time;

            elseif obj.state == TaskState.jc_break
                disp("Joint control BREAK!");
                obj.controller = controller_type.stop;
                obj.control = control_type.jp_reg;
                obj.operational_control = false;
                obj.t_end = task_time;

            else
                error("StateMachine: Unknown state");
            end

        end

    end

end
