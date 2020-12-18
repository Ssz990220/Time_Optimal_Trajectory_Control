function h = hessian_final(x,b0,s_phi,d_phi,t)
%HESSIAN Summary of this function goes here
%   Detailed explanation goes here
global Mu FN Wf;
h = sparse(size(x,1),size(x,1));
for k = 1:size(x)/4
    h((k-1)*4+3,(k-1)*4+3) = (4*(x((k-1)*4+3))^2)/(t*((x((k-1)*4+3))^2 + (x((k-1)*4+4))^2 - (Mu*FN)^2)^2)...
                            - 2/(t*((x((k-1)*4+3))^2 - (Wf*Mu*FN)^2))...
                            - 2/(t*((x((k-1)*4+3))^2 + (x((k-1)*4+4))^2 - (Mu*FN)^2)) ...
                            + (4*(x((k-1)*4+3))^2)/(t*((x((k-1)*4+3))^2 - (Wf*Mu*FN)^2)^2);
    h((k-1)*4+4,(k-1)*4+4) = (4*(x((k-1)*4+4))^2)/(t*((x((k-1)*4+3))^2 + (x((k-1)*4+4))^2 - (Mu*FN)^2)^2) ...
                            - 2/(t*((x((k-1)*4+3))^2 + (x((k-1)*4+4))^2 - (Mu*FN)^2));
    h((k-1)*4+3,(k-1)*4+4) = (4*(x((k-1)*4+3))*(x((k-1)*4+4)))/(t*((x((k-1)*4+3))^2 + (x((k-1)*4+4))^2 - (Mu*FN)^2)^2);
    h((k-1)*4+4,(k-1)*4+3)=h((k-1)*4+3,(k-1)*4+4);
end
h(1,1) = 1/(x(1)^2*t) + 2*(d_phi/(2*x(1)*(x(1)^(1/2) + x(5)^(1/2))^3) + d_phi/(4*x(1)^(3/2)*(x(1)^(1/2) + x(5)^(1/2))^2) + d_phi/(2*x(1)*(x(1)^(1/2) + b0^(1/2))^3) + d_phi/(4*x(1)^(3/2)*(x(1)^(1/2) + b0^(1/2))^2));
h(1,5) = 2*(d_phi/(2*x(1)^(1/2)*x(5)^(1/2)*(x(1)^(1/2) + x(5)^(1/2))^3));
h(5,1) = h(1,5);
for k = 2:(size(x)/4-1)
    h((k-1)*4+1,(k-1)*4+1)=1/((x((k-1)*4+1))^2*t) +2*( d_phi/(2*(x((k-1)*4+1))*((x((k-1)*4+1))^(1/2) + (x((k-1)*4+5))^(1/2))^3) + d_phi/(4*(x((k-1)*4+1))^(3/2)*((x((k-1)*4+1))^(1/2) + (x((k-1)*4+5))^(1/2))^2) + d_phi/(2*(x((k-1)*4+1))*((x((k-1)*4+1))^(1/2) + (x((k-1)*4-3))^(1/2))^3) + d_phi/(4*(x((k-1)*4+1))^(3/2)*((x((k-1)*4+1))^(1/2) + (x((k-1)*4-3))^(1/2))^2));
    h((k-1)*4+5,(k-1)*4+1)=2*(d_phi/(2*(x((k-1)*4+1))^(1/2)*(x((k-1)*4+5))^(1/2)*((x((k-1)*4+1))^(1/2) + (x((k-1)*4+5))^(1/2))^3));
    h((k-1)*4+1,(k-1)*4+5)=h((k-1)*4+5,(k-1)*4+1);
end
h(end-3,end-3) = 1/(x(end-3)^2*t) + 2*(d_phi/(2*x(end-3)*(x(end-3)^(1/2) + x(end-7)^(1/2))^3) + d_phi/(4*x(end-3)^(3/2)*(x(end-3)^(1/2) + x(end-7)^(1/2))^2));
end

