function [b,a,tau] = mysolver(s, sm, ds, q, qp, qpp, R, Mv, Cv, gv, torquelb, torqueub, gamma1, gamma2)
%MYSOLVER Summary of this function goes here
%   Detailed explanation goes here
global sgrid ub r p;
ub = torqueub;
sgrid = size(sm,2);
p = size(Mv,1);
r = size(R,2);
dimA=sgrid*(p+1);
d_phi = 1/sgrid;
b0 = 0;
b = 0.1*ones(sgrid,1);
a = zeros(sgrid,1);
u = zeros(sgrid*r,1);
x = mygenx(b,a,u);
MAXITER = 100;
ALPHA = 0.01;
BETA = 0.5;
RESTOL=1e-6;
THER=1e-8;
TOL = 1e-5;
t_update = 20;
T = 100;
nu=zeros(dimA,1);
%%
for i=1:MAXITER
    [A,b] = myAb(Mv,Cv,gv,d_phi);
    res = mygetr(A,b,x,nu,T,b0,d_phi);
    n = norm(res);
    hess = myhess(x,b0,d_phi,T);
    KKT = sparse(sgrid*(r+2+p+1),sgrid*(r+2+p+1));
    KKT(1:sgrid*(r+2),:)=[hess A.'];
    KKT(sgrid*(r+2)+1:sgrid*(r+2)+sgrid*(p+1),1:sgrid*(r+2))=A;
    sol = -KKT\res;
    Dx = sol(1:size(x,1),1);Dnu=sol(size(x,1)+[1:dimA],1); 
    t=1;
    while mycheckb(x+t*Dx)
        t=BETA*t;
    end
    while norm(mygetr(A,b,x+t*Dx,nu+t*Dnu,T,b,d_phi))>(1-ALPHA*t)*n
        t = BETA*t;
    end
    x = x + t*Dx; nu=nu+t*Dnu; 
    
    if (n<RESTOL)&&(norm(A*x)<THER)
        gap = 7*sgrid/T;
        if (gap<TOL),break;end
        T = t_update * T; 
        fprintf("Update T at iteration %d ...\n",i);
    end
end
[b,a,u]=mygetbau(x);
b = b';
a = a';
tau = u';
end

