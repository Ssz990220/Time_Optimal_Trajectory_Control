function [b,a,u] = get_bau(x)
%GET_BAU Summary of this function goes here
%   Detailed explanation goes here
b = zeros(size(x,1)/4,1);
a = zeros(size(x,1)/4,1);
u = zeros(size(x,1)/4*2,1);
for k=1:size(x)/4
    b(k) = x((k-1)*4+1);
    a(k) = x((k-1)*4+2);
    u((k-1)*2+1:(k-1)*2+2) = x((k-1)*4+3:(k-1)*4+4);
end
end

