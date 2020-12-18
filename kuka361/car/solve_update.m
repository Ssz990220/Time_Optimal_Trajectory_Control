function [Dx, Dnu] = solve_update(hess, A, r)
%SOLVE_UPDATE Solve the newton step for x and nu
%   Detailed explanation goes here
dimA = size(A,1);
KKT = [hess A.';A,zeros(dimA,dimA)];
[Dx,Dnu]=solve_rearrange_matrix(KKT,r);


end

