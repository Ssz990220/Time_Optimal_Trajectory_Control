function [x,y,t] = pathgenerator()
%PATHGENERATOR Summary of this function goes here
%   Detailed explanation goes here
omega =20* pi;
r = 1;
vx = 5;
scale = 2000;
t = (1:scale)/scale;
x = r*sin(omega*t);
y = r*cos(omega*t);
figure;
plot(x,y);
title('original path')
end

