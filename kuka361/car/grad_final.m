function gf = grad_final(x,b0,s_phi,d_phi,t)
%GRAD Summary of this function goes here
%   Detailed explanation goes here
global Mu FN Wf;
gf = zeros(size(x,1),1);
for k = 1:size(x)/4
    gf((k-1)*4+3) = - (2*(x((k-1)*4+3)))/(t*((x((k-1)*4+3))^2 - (Wf*Mu*FN)^2)) - (2*(x((k-1)*4+3)))/(t*((x((k-1)*4+3))^2 + (x((k-1)*4+4))^2 - (Mu*FN)^2));
    gf((k-1)*4+4) = -(2*(x((k-1)*4+4)))/(t*((x((k-1)*4+3))^2 + (x((k-1)*4+4))^2 - (Mu*FN)^2));
end
gf(1) = - 1/(x(1)*t) - 2*(d_phi/(2*x(1)^(1/2)*(x(1)^(1/2) + x(5)^(1/2))^2) + d_phi/(2*x(1)^(1/2)*(x(1)^(1/2) + b0^(1/2))^2));
for k = 2:(size(x)/4-1)
    gf((k-1)*4+1)=- 1/((x((k-1)*4+1))*t) - 2*(d_phi/(2*(x((k-1)*4+1))^(1/2)*((x((k-1)*4+1))^(1/2) + (x((k-1)*4+5))^(1/2))^2) + d_phi/(2*(x((k-1)*4+1))^(1/2)*((x((k-1)*4+1))^(1/2) + (x((k-1)*4-3))^(1/2))^2));
end
gf(end-3) = - 1/(x(end-3)*t) - 2*(d_phi/(2*x(end-3)^(1/2)*(x(end-3)^(1/2) + x(end-7)^(1/2))^2)) ;
end

