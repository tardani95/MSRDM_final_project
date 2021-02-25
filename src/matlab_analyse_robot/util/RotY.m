function [ Ry ] = RotY( betaY )

% TODO: Fill with the proper rotation matrix

c = cos(betaY);
s = sin(betaY);

Ry=[c,0,s;
    0,1,0;
    -s,0,c];

end

