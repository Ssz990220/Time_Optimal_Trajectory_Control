function h = myhess(x,b0,d_phi,t)
%HESSIAN Summary of this function goes here
%   Detailed explanation goes here
global r sgrid ub
h = sparse(sgrid*(2+r),sgrid*(2+r));
for k = 1:sgrid
    for i = 1:r
        h((k-1)*(2+r)+2+i,(k-1)*(2+r)+2+i) =- 2/(t*((x((k-1)*(2+r)+2+i))^2 - (ub(i))^2))+ (4*(x((k-1)*(2+r)+2+i))^2)/(t*((x((k-1)*(2+r)+2+i))^2 - (ub(i))^2)^2);
    end
end
h(1,1) = 1/(x(1)^2*t) + 2*(d_phi/(2*x(1)*(x(1)^(1/2) + x(9)^(1/2))^3) + d_phi/(4*x(1)^(3/2)*(x(1)^(1/2) + x(9)^(1/2))^2) + d_phi/(2*x(1)*(x(1)^(1/2) + b0^(1/2))^3) + d_phi/(4*x(1)^(3/2)*(x(1)^(1/2) + b0^(1/2))^2));
h(1,9) = 2*(d_phi/(2*x(1)^(1/2)*x(9)^(1/2)*(x(1)^(1/2) + x(9)^(1/2))^3));
h(9,1) = h(1,9);
for k = 2:(sgrid-1)
    h((k-1)*(2+r)+1,(k-1)*(2+r)+1)=1/((x((k-1)*(2+r)+1))^2*t) +2*( d_phi/(2*(x((k-1)*(2+r)+1))*((x((k-1)*(2+r)+1))^(1/2) + (x((k-1)*(2+r)+9))^(1/2))^3) + d_phi/(4*(x((k-1)*(2+r)+1))^(3/2)*((x((k-1)*(2+r)+1))^(1/2) + (x((k-1)*(2+r)+9))^(1/2))^2) + d_phi/(2*(x((k-1)*(2+r)+1))*((x((k-1)*(2+r)+1))^(1/2) + (x((k-1)*(2+r)-7))^(1/2))^3) + d_phi/(4*(x((k-1)*(2+r)+1))^(3/2)*((x((k-1)*(2+r)+1))^(1/2) + (x((k-1)*(2+r)-7))^(1/2))^2));
    h((k-1)*(2+r)+9,(k-1)*(2+r)+1)=2*(d_phi/(2*(x((k-1)*(2+r)+1))^(1/2)*(x((k-1)*(2+r)+9))^(1/2)*((x((k-1)*(2+r)+1))^(1/2) + (x((k-1)*(2+r)+9))^(1/2))^3));
    h((k-1)*(2+r)+1,(k-1)*(2+r)+9)=h((k-1)*(2+r)+9,(k-1)*(2+r)+1);
end
h(end-7,end-7) = 1/(x(end-7)^2*t) + 2*(d_phi/(2*x(end-7)*(x(end-7)^(1/2) + x(end-15)^(1/2))^3) + d_phi/(4*x(end-7)^(3/2)*(x(end-7)^(1/2) + x(end-15)^(1/2))^2));
end

