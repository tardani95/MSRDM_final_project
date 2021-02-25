function [ phi_d,theta_d,psi_d ] = R2EulerA( R )

% TODO: Fill with the proper rotation Euler angles computation
d = sqrt(1-R(3,1)^2);
theta_d = atan2(-R(3,1),d);
phi_d = atan2(R(2,1),R(1,1));
psi_d = atan2(R(3,2),R(3,3));

end

