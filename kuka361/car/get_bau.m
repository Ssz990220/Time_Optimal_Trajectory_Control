function [b,a,u] = get_bau(x)
%GET_BAU Summary of this function goes here
%   Detailed explanation goes here
b = zeros(size(x,1)/4,1);
a = zeros(size(x,1)/4,1);
u = zeros(size(x,1)/4,2);
for k=1:size(x)/4
     b(k) = x((k-1)*4+1);
    a(k) = x((k-1)*4+2);
    u(k,:) = [x((k-1)*4+3);x((k-1)*4+4)];
end
end

