function [iHT] = invHT(HT)

% TODO: Fill with the proper inverse transformation (from the course)
% Do not use the inv function of matlab !

rot_mat = HT(1:3,1:3);
t_vec = HT(1:3,4);

iHT = [rot_mat', -rot_mat' * t_vec;[0,0,0,1]];

end

