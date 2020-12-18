function ok = checkU(x)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
global Mu FN Wf
[b,a,u] = get_bau(x);
U = sqrt(u(:,2).^2+u(:,1).^2);
ok = ~(min(U<Mu*FN) & min(u(:,1)<Mu*FN*Wf));

end

