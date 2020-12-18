function obj = getobjfromx(x)
%GETTFROMX Summary of this function goes here
%   Detailed explanation goes here
global d_phi Mu FN Wf;
[b,a,u] = get_bau(x);
obj = sum(2*d_phi'./(sqrt(b(2:end))+sqrt(b(1:end-1))));

obj = obj -sum(log((Mu*FN)^2-u(:,1).^2-u(:,2).^2)+log((Mu*FN*Wf)^2-u(:,1).^2)+log(b));

end

