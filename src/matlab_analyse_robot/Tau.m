function [ret] = Tau(u)
    %TAU Summary of this function goes here
    %   Detailed explanation goes here
    global state_machine
    
    %% Robot Parameters
    robot_param = u(1:38);
    
    %% variables
    Qrp = u(39:41);
    Qrpp = u(42:44);
    
    QXp = u(45:47);
    
    Q = u(48:50);
    Qp = u(51:53);
    
    %% Controller params
    Kd = diag(u(54:56));
    
    %% Estimated Theta
    [~, Yr, Theta] = YrTheta_UR10_3DOF([robot_param' Q' Qp' Qrp' Qrpp']);
    
    Theta_e = u(57:57+27-1);
    t = u(57+27);
    
    % initialize parameter
    if t==0
        a = 0.9;
        b = 1.1;
        r = a + (b-a).*rand(27,1);
        Theta_e = r.*Theta;
    end
    
    %% Controller
    Sq = QXp - Qrp;
    
    if state_machine.operational_control
        % only linear velocity jacobian
        [~, Jqv, ~] = Jef_0_ur10_3DOF([robot_param' Q']);
        Sq = Jqv \ Sq;
    end
    
    tau = -Kd*Sq + Yr*Theta_e;
    
    %% parameter estimation
    gamma = 0.0002* eye(27);
    gamma(14,14) = gamma(14,14) * 10;
    gamma(15,15) = gamma(15,15) * 10;
    gamma(16,16) = gamma(16,16) * 10;
    
    gamma(18,18) = gamma(18,18) * 10;
    gamma(19,19) = gamma(19,19) * 10;
    gamma(20,20) = gamma(20,20) * 10;
    gamma(21,21) = gamma(21,21) * 10;
    
    gamma(23,23) = gamma(23,23) * 10;
    gamma(24,24) = gamma(24,24) * 10;
    gamma(25,25) = gamma(25,25) * 10;
    gamma(26,26) = gamma(26,26) * 10;
    gamma(27,27) = gamma(27,27) * 10;
    
    thetap_e = -gamma*Yr'*Sq;
    
    DeltaTheta = Theta-Theta_e;
    
    ret = [tau' thetap_e' DeltaTheta'];
end
