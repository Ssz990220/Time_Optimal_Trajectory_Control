function [At,b,c,K] = rls(P,q)

[m,n]=size(P);
b = -sparse([1;1;zeros(n,1)]);
At = sparse([-1,zeros(1,1+n);...
    zeros(m,2),P]);
c = [0;q];
K.q = [1+m];
At = [At;0,-1,zeros(1,n);...
    zeros(1,2+n);...
    zeros(n,2),-eye(n)];
c = sparse([c;0;1;zeros(n,1)]);
K.q = [K.q 2+n];

