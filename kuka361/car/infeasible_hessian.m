function h = infeasible_hessian(x,b0,s_phi,d_phi)
%INFEASIBLE_HESSIAN Summary of this function goes here
%   Detailed explanation goes here
global Mu FN Wf;
h = hessian(x(1:end-1),b0,s_phi,d_phi);
h = [h,zeros(size(h,1),1);zeros(1,size(h,2)),0];
dfs2 = 0;
for k=1:(size(x)-1)/4-1
    h((k-1)*4+1,end)=1/(x(end)+x((k-1)*4+1))^2;
    h(end,(k-1)*4+1)=h((k-1)*4+1,end);
    h((k-1)*4+3,end)=2*x((k-1)*4+3)/(x(end)+(Mu*FN)^2-x((k-1)*4+3)^2-x((k-1)*4+4)^2)...
        +2*x((k-1)*4+3)/(x(end)+(Wf*Mu*FN)^2-x((k-1)*4+3)^2)^2;
    h(end,(k-1)*4+3)=h((k-1)*4+3,end);
    h((k-1)*4+4,end)=2*x((k-1)*4+4)/(x(end)+(Mu*FN)^2-x((k-1)*4+3)^2-x((k-1)*4+4)^2);
    h(end,(k-1)*4+4)=h((k-1)*4+4,end);
    dfs2=1/(x(end)+x((k-1)*4+1))^2+1/(x(end)+(Mu*FN)^2-x((k-1)*4+3)^2-x((k-1)*4+4)^2)...
        +1/(x(end)+(Wf*Mu*FN)^2-x((k-1)*4+3)^2)^2+dfs2;
end
h(end-4,end)=1/(x(end)+x(end-4))^2;
h(end,end-4)=h(end-4,end);
h(end-2,end)=2*x(end-2)/(x(end)+(Mu*FN)^2-x(end-2)^2-x(end-1)^2)...
        +2*x(end-2)/(x(end)+(Wf*Mu*FN)^2-x(end-2)^2)^2;
h(end,end-2)=h(end-2,end);
h(end-1,end)=2*x(end-1)/(x(end)+(Mu*FN)^2-x(end-2)^2-x(end-1)^2);
h(end,end-1)=h(end-1,end);
h(end,end)=dfs2;
end

