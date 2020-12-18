function [b,a,u] = mygetbau(x)
%GET_BAU Summary of this function goes here
%   Detailed explanation goes here
global r sgrid;
b = zeros(sgrid,1);
a = zeros(sgrid,1);
u = zeros(sgrid,r);
for k=1:sgrid
     b(k) = x((k-1)*(2+r)+1);
    a(k) = x((k-1)*(2+r)+2);
    uk =  x((k-1)*(2+r)+[3:2+r]);
    u(k,:) = uk';
end
end

