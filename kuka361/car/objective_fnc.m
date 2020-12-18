function time = objective_fnc(x)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
global d_phi b0;
time = d_phi/(x(1)^(1/2)+b0^(1/2));
for k =1:(max(size(x))/4-1)
    time = time+d_phi/(x((k-1)*4+1)^(1/2)+x((k-1)*4+5)^(1/2));
end
end

