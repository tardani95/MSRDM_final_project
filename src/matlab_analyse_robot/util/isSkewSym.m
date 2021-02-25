function skew_symmteric = isSkewSym(matrix)
%ISSKEWSYM Summary of this function goes here
%   Detailed explanation goes here

    condition1 = logical(sum(matrix+matrix','all') < 1e-10);
    condition2 = logical(det(matrix) < 1e-10);

    skew_symmteric = condition1 && condition2;

end

