function gf = mygrad(x,b0,d_phi,t)
%GRAD Summary of this function goes here
%   Detailed explanation goes here
global ub sgrid r
gf = zeros(sgrid*(2+r),1);
for k = 1:sgrid
    for i = 1:r
        gf((k-1)*(2+r)+i+2) = - (2*(x((k-1)*(2+r)+i+2)))/(t*((x((k-1)*(2+r)+i+2))^2 - (ub(i))^2));
    end
end
gf(1) = - 1/(x(1)*t) - 2*(d_phi/(2*x(1)^(1/2)*(x(1)^(1/2) + x(9)^(1/2))^2) + d_phi/(2*x(1)^(1/2)*(x(1)^(1/2) + b0^(1/2))^2));
for k = 2:(sgrid-1)
    gf((k-1)*(2+r)+1)=- 1/((x((k-1)*(2+r)+1))*t) - 2*(d_phi/(2*(x((k-1)*(2+r)+1))^(1/2)*((x((k-1)*(2+r)+1))^(1/2) + (x((k-1)*(2+r)+9))^(1/2))^2) + d_phi/(2*(x((k-1)*(2+r)+1))^(1/2)*((x((k-1)*(2+r)+1))^(1/2) + (x((k-1)*(2+r)-7))^(1/2))^2));
end
gf(end-7) = - 1/(x(end-7)*t) - 2*(d_phi/(2*x(end-7)^(1/2)*(x(end-7)^(1/2) + x(end-15)^(1/2))^2)) ;
end

