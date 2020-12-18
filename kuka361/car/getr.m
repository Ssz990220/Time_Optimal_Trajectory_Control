function r = getr(A,x,nu,T,b0,s_phi,d_phi)
%GETR Summary of this function goes here
%   Detailed explanation goes here
gradient = grad_final(x,b0,s_phi,d_phi,T);
r = [gradient+A'*nu;A*x];

end

