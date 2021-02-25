function matquat = rosquat2matquat(rosquat)
%ROSQUAT2MATQUAT Summary of this function goes here
    x = rosquat(1);
    y = rosquat(2);
    z = rosquat(3);
    w = rosquat(4);
    matquat = [w,x,y,z];
end

