function gf = infeasible_grad(x,b0,s_phi,d_phi)
%INFEASIBLE_GRAD Summary of this function goes here
%   Detailed explanation goes here
global Mu FN Wf;
gf = grad(x(1:end-1),b0,s_phi,d_phi);
dfs=0;
for k=1:(size(x)-1)/4
   dfs = dfs - 1/(x(end)+ (Mu*FN)^2-x((k-1)*4+3)^2-x((k-1)*4+4)^2)...
       -1/(x(end)+(Wf*Mu*FN)^2-x((k-1)*4+3)^2)...
       -1/(x(end)+x((k-1)*4+1));
end
gf(end+1,1)=dfs;
end

