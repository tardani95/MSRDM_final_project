function HT = rotm_trvec2tf(rotm,trvec)
%ROTM_TV2TF Summary of this function goes here
%   Detailed explanation goes here

    HT = [rotm, trvec;
          0 0 0 1];

end

