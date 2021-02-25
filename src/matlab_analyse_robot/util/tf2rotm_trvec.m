function [rotm, trvec] = tf2rotm_trvec(HT)
%TF2ROTM_TV Summary of this function goes here
%   Detailed explanation goes here
    rotm = tf2rotm(HT);
    trvec = tf2trvec(HT);
end

