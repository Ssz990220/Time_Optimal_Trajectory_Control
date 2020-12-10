function gf = grad(x,b0,s_phi,d_phi)
%GRAD Summary of this function goes here
%   Detailed explanation goes here
global Mu FN Wf;
gf = zeros(size(x,1),1);
% tic;
gf(1)=-(s_phi(3)-s_phi(2))/(x(5)^(1/2)*x(1)^(1/2)+1)-(s_phi(2)-s_phi(1))/(x(1)^(1/2)*b0^(1/2)+1)+1/x(1);
gf(2)=0;
gf(3)=-2*x(3)/((Mu*FN)^2-(x(3)^2+x(4)^2))-2*x(3)/((Wf*Mu*FN)^2-x(3)^2);
gf(4)=-2*x(4)/((Mu*FN)^2-(x(3)^2+x(4)^2));
for k=2:(size(x)/4-1)
    gf((k-1)*4+1)= -(s_phi(k+2)-s_phi(k+1))/((x(k*4+1)^(1/2)*x((k-1)*4+1)^(1/2))+1)...
        -(s_phi(k+1)-s_phi(k))/((x((k-1)*4+1)^(1/2)*(x((k-1)*4-3))^(1/2))+1)+1/(x((k-1)*4+1));
    gf((k-1)*4+2)=0;
    num = (Mu*FN)^2-(x((k-1)*4+3)^2+x((k-1)*4+4)^2);
    gf((k-1)*4+3)=-2*x((k-1)*4+3)/(num)-2*x((k-1)*4+3)/((Wf*Mu*FN)^2-x((k-1)*4+3)^2);
    gf((k-1)*4+4)=-2*x((k-1)*4+4)/(num);
end
gf(end-3)=-(s_phi(end)-s_phi(end-1))/((x(end-3)^(1/2)*x(end-7)^(1/2))+1)+1/x(end-3);
gf(end-2)=0;
gf(end-1)=-2*x(end-1)/((Mu*FN)^2-(x(end-1)^2+x(end)^2))-2*x(end-1)/((Wf*Mu*FN)^2-x(end-1)^2);
gf(end)=-2*x(end)/((Mu*FN)^2-(x(end-1)^2+x(end)^2));
% toc;
end

