function obj = mygetobjfromx(x)
%GETTFROMX Summary of this function goes here
%   Detailed explanation goes here
global d_phi ub r;
[b,a,u] = mygetbau(x);
obj = sum(2*d_phi'./(sqrt(b(2:end))+sqrt(b(1:end-1))));
for i = 1:r
    obj = obj - sum(log((ub)^2-u(:,i).^2));
end
obj = obj -sum(log(b));

end

