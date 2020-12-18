function r = mygetr(A,b,x,nu,T,b0,d_phi)
%MYGETR Summary of this function goes here
%   Detailed explanation goes here
gradient = mygrad(x,b0,d_phi,T);
r = [gradient+A'*nu;A*x+b];

end

