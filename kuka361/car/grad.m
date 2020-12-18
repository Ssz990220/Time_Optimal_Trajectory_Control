function gf = grad(x,b0,s_phi,d_phi,t)
%GRAD Summary of this function goes here
%   Detailed explanation goes here
global Mu FN Wf;
gf = zeros(size(x,1),1);
% tic;
gf(1)=-(d_phi)/(x(5)^(1/2)+x(1)^(1/2))^2/x(1)^(1/2)...
    -(d_phi)/(x(1)^(1/2)+b0^(1/2))^2/x(1)^(1/2)...
    -1/x(1)/t;
gf(2)=0;
gf(3)=(2*x(3)/((Mu*FN)^2-(x(3)^2+x(4)^2))...
      +2*x(3)/((Wf*Mu*FN)^2-x(3)^2))*1/t;
gf(4)=(2*x(4)/((Mu*FN)^2-(x(3)^2+x(4)^2)))*1/t;
for k=2:(size(x)/4-1)
    gf((k-1)*4+1)= -(d_phi)/(x((k-1)*4+5)^(1/2)+x((k-1)*4+1)^(1/2))^2/x((k-1)*4+1)^(1/2)...
                   -(d_phi)/(x((k-1)*4+1)^(1/2)+x((k-1)*4-3)^(1/2))^2/x((k-1)*4+1)^(1/2)...
                   -1/(x((k-1)*4+1)*t);
    gf((k-1)*4+2)=0;
    num = (Mu*FN)^2-(x((k-1)*4+3)^2+x((k-1)*4+4)^2);
    gf((k-1)*4+3)=(2*x((k-1)*4+3)/(num)...
                  +2*x((k-1)*4+3)/((Wf*Mu*FN)^2-x((k-1)*4+3)^2))*1/t;
    gf((k-1)*4+4)=(2*x((k-1)*4+4)/(num))*1/t;
end
gf(end-3)=-(d_phi)/(x(end-3)^(1/2)+x(end-7)^(1/2))^2/x(end-3)^(1/2)...
          -1/x(end-3)/t;
gf(end-2)=0;
gf(end-1)=(2*x(end-1)/((Mu*FN)^2-(x(end-1)^2+x(end)^2))...
          +2*x(end-1)/((Wf*Mu*FN)^2-x(end-1)^2))*1/t;
gf(end)=(2*x(end)/((Mu*FN)^2-(x(end-1)^2+x(end)^2)))*1/t;
% toc;
end

