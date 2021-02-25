function [ Rx ] = RotX( alphaX )

% TODO: Fill with the proper rotation matrix
c = cos(alphaX);
s = sin(alphaX);

Rx=[1,0,0;
    0,c,-s;
    0,s,c];

end

