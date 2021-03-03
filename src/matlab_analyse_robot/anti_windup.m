function [tau] = anti_windup(tau)
    %ANTI_WINDUP Summary of this function goes here
    %   Detailed explanation goes here
    global anti_windup

    joint_effort_limits = [330; 330; 150];

    anti_windup = abs(tau ./ joint_effort_limits) < 1;

    limit_tau = ~anti_windup;

    if any(anti_windup == 0)
        tau(limit_tau) = sign(tau(limit_tau)) .* joint_effort_limits(limit_tau);
    end

end
