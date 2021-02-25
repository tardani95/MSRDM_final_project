function HT = dh2rel_tf(dh)
%DH2REL_TF Summary of this function goes here
%   Detailed explanation goes here
%   size(dh) = [1,4]
    Hz = [RotZ(dh(1)) [0 0 dh(2)]'; 0 0 0 1];
    Hx = [RotX(dh(4)) [dh(3) 0 0]'; 0 0 0 1];
    HT = Hz*Hx;
end

