function [ret] = ReferenceVelocity(u)
    %REFERENCEVELOCITY Summary of this function goes here
    %   Detailed explanation goes here
    global state_machine anti_windup sumDeltaQ sumDeltaQp

    %%Desired values
    Qd = u(1:3);
    Qdp = u(4:6);
    Qdpp = u(7:9);

    %Current values
    Q = u(10:12);
    Qp = u(13:15);

    %errors
    DeltaQ = Q - Qd;
    DeltaQp = Qp - Qdp;

    %Controller params
    Kp = diag(u(16:18));
    Ki = diag(u(19:21));

%     if state_machine.operational_control
%         Kp = 0.5 * Kp;
%         Ki = 0.0 * Ki;
%     end

    %Define controllers PD, PID else break
    if state_machine.controller == controller_type.pd
        Qrp = Qdp - Kp * DeltaQ;
        Qrpp = Qdpp - Kp * DeltaQp;
        sumDeltaQ = [0 0 0]';
        sumDeltaQp = [0 0 0]';
    elseif state_machine.controller == controller_type.pid
        % TODO: scale with dt?
        sumDeltaQ = sumDeltaQ + anti_windup .* DeltaQ * 0.004;
        sumDeltaQp = sumDeltaQp + anti_windup .* DeltaQp * 0.004;
        Qrp = Qdp - Kp * DeltaQ - Ki * sumDeltaQ;
        Qrpp = Qdpp - Kp * DeltaQp - Ki * sumDeltaQp;
    elseif state_machine.controller == controller_type.stop
        sumDeltaQ = [0 0 0]';
        sumDeltaQp = [0 0 0]';
        Qrp = [0 0 0]';
        Qrpp = [0 0 0]';
    else
        error("Unknown controller!");
    end

    ret = [Qd' Qdp' DeltaQ' DeltaQp' Qrp' Qrpp' Qp'];

end
