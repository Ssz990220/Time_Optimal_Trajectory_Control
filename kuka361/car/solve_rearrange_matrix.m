function [Dx, Dnu] = solve_rearrange_matrix(KKT,r)
%REARRANGE_MATRIX Rearrage the KKT matrix to make it in banded form
%   Detailed explanation goes here
re_KKT_c =sparse(sgrid*7,sgrid*7,(18+13)*sgrid);
re_KKT = sparse(sgrid*7,sgrid*7,(18+13)*sgrid);
re_r = zeros(size(r));
sizehessian = size(KKT,1)/7*4;
Dx = zeros(sizehessian,1);
Dnu = zeros(size(KKT,1)/7*3,1);
%% Column Exchange then Row exchange
for k = 1:size(KKT,1)/7       %in the car model, the denominator is 7
%     re_KKT_c(:,[(k-1)*7+1:(k-1)*7+3])=[KKT(:,sizehessian+(k-1)*3+3),KKT(:,sizehessian+(k-1)*3+[1,2])];
%     re_KKT_c(:,[(k-1)*7+4:(k-1)*7+7])=KKT(:,(k-1)*4+1:(k-1)*4+4);
    re_KKT_c(:,(k-1)*7+1:(k-1)*7+4) = KKT(:,(k-1)*4+1:(k-1)*4+4);
    re_KKT_c(:,(k-1)*7+5:(k-1)*7+7) = KKT(:,sizehessian+(k-1)*3+1:sizehessian+(k-1)*3+3);
end

for k = 1:size(KKT,1)/7
    re_KKT((k-1)*7+5:(k-1)*7+7,:)=re_KKT_c(sizehessian+(k-1)*3+[1:3],:);
    re_KKT((k-1)*7+1:(k-1)*7+4,:)=re_KKT_c((k-1)*4+1:(k-1)*4+4,:);
    re_r((k-1)*7+5:(k-1)*7+7)=r(sizehessian+(k-1)*3+[1:3],:);
    re_r((k-1)*7+1:(k-1)*7+4)=r((k-1)*4+1:(k-1)*4+4,:);
end
% tic;
% [l,u,P,Q]=lu(re_KKT);
% toc;
%% Solve
% tic;
sol = -re_KKT\re_r;
% toc;
%% reset the arrangement
for k = 1:size(KKT,1)/7
%     Dx((k-1)*4+[1:4])=sol((k-1)*7+[4:7]);
%     Dnu((k-1)*3+[1:3])=[sol((k-1)*7+[2:3]);sol((k-1)*7+1)];
    Dx((k-1)*4+[1:4])=sol((k-1)*7+[1:4]);
    Dnu((k-1)*3+[1:3])=sol((k-1)*7+[5:7]);
end

end