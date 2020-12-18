function gf = grad_2(x,b0,s_phi,d_phi,t)
%GRAD Summary of this function goes here
%   Detailed explanation goes here
global Mu FN Wf;
gf = zeros(size(x,1),1);
% tic;
gf(1)=d_phi*(- 1/(2*x(1)^(1/2)*(x(1)^(1/2) + b0^(1/2))^2)) -1/(x(1)*t);
gf(2)=0;
gf(3)=- (2*x(3))/(t*(x(3)^2 - (Wf*Mu*FN)^2)) - (2*x(3))/(t*(x(3)^2 + x(4)^2 - (Mu*FN)^2));
gf(4)=-(2*x(4))/(t*(x(3)^2 + x(4)^2 - (Mu*FN)^2));
for k=2:(size(x)/4-1)
    gf((k-1)*4+1)= d_phi*(- 1/(2*(x((k-1)*4+1))^(1/2)*((x((k-1)*4+1))^(1/2) + (x((k-1)*4+5))^(1/2))^2) - 1/(2*(x((k-1)*4+1))^(1/2)*((x((k-1)*4+1))^(1/2) + (x((k-1)*4-3))^(1/2))^2))-1/((x((k-1)*4-3))*t);
    gf((k-1)*4+2)=0;
    gf((k-1)*4+3)=-(2*(x((k-1)*4+3)))/(t*((x((k-1)*4+3))^2 - (Wf*Mu*FN)^2)) - (2*(x((k-1)*4+3)))/(t*((x((k-1)*4+3))^2 + (x((k-1)*4+4))^2 - (Mu*FN)^2));
    gf((k-1)*4+4)=-(2*(x((k-1)*4+4)))/(t*((x((k-1)*4+3))^2 + (x((k-1)*4+4))^2 - (Mu*FN)^2));
end
gf(end-3)=d_phi*(- 1/(2*x(end-3)^(1/2)*(x(end-3)^(1/2) + x(end-7)^(1/2))^2)) -1/(x(end-3)*t);
gf(end-2)=0;
gf(end-1)=- (2*x(end-1))/(t*(x(end-1)^2 - (Wf*Mu*FN)^2)) - (2*x(end-1))/(t*(x(end-1)^2 + x(end)^2 - (Mu*FN)^2));
gf(end)=-(2*x(end))/(t*(x(end-1)^2 + x(end)^2 - (Mu*FN)^2));
% toc;
end

