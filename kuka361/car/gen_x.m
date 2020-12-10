function [x,s] = gen_x(b,a,u)
%GEN_X Summary of this function goes here
%   Detailed explanation goes here
global Mu FN;
x = zeros(size(b,1)+size(a,1)+size(u,1),1);
for k = 1:size(b)
    x((k-1)*4+1:(k-1)*4+4,1)=[b(k);a(k);u((k-1)*2+1);u((k-1)*2+2)];
end
s=max([b;-Mu*FN]);
    
end

