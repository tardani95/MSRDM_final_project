function X = tf2pose(HT)
%TF2POSE Summary of this function goes here
%   Detailed explanation goes here
    % X = [x y z theta1 theta2 theta3]'
    X_posH = HT * [0 0 0 1]';
    [phi, theta, psi] = R2EulerA(tf2rotm(HT));
    X = [X_posH(1:3);[phi, theta, psi]'];
end

