function [isposdef,ispossemidef] = isPd(M)
%ISPD Summary of this function goes here
%   Detailed explanation goes here    
    if all(abs(M-M') < 1e-10, 'all')
        d = eig(M);
        
        if ~isa(M,'sym')
            tol = length(d) * eps(max(d));
        else
            tol = 1e-10;
        end
        
        isposdef = all(d > tol);
        ispossemidef = all(d > -tol);
    else
        isposdef = false;
        ispossemidef = false;
    end
end

