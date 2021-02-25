function is_sym = isSym(M)
%ISSYM Summary of this function goes here
%   Detailed explanation goes here
    is_sym = all(abs(M-M') < 1e-10, 'all');
end

