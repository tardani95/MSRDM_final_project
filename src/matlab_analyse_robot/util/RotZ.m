function [ Rz ] = RotZ( gammaZ )

% TODO: Fill with the proper rotation matrix
c = cos(gammaZ);
s = sin(gammaZ);

Rz=[c,-s,0;
    s,c,0;
    0,0,1];

end

