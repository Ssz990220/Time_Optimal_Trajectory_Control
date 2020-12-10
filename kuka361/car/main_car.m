clear all;
close all;
global savefigs;
savefigs = 0; % switch to 1 if you want to save all the result plots


%% paramaters
% Number of grids
sgrid = 200;   %2000 segments

global m Mu FN Wf;
m = 2;
g = 9.8;
Mu = 0.6;
FN = m*g;
Wf = 0.6;
% The path function.
pathfun = @optecpath;
% The manipulator dynamics.
invdynfun = @invdynkuka361;

coulomb = 1;
[phi,s_phi,s_mid,sd_mid,sdd_mid,phi_mid,d_phi,R,M_tilde,C_tilde] = CarDynPathFunction(sgrid);

dimA=sgrid*3;
%% Initailization & infeasible start newton

b0 = 0;
b = ones(sgrid,1);
a = zeros(sgrid,1);
u = zeros(sgrid*2,1);
[x,s] = gen_x(b,a,u);
%% A small test
% A = get_A(x,M_tilde,C_tilde,0,R,0,d_phi);
% max(A*x);


%% Continue
MAXITER = 50;
ALPHA = 0.01;
BETA = 0.5;
RESTOL=1e-7;
x_s = [x;s];
nu=zeros(dimA+1,1);
rs = [];
for i=1:MAXITER
    gradient = infeasible_grad(x_s,b0,s_phi,d_phi);
    A = infeasible_get_A(x_s,M_tilde,C_tilde,0,R,0,d_phi);
    r = [gradient+A'*nu;A*x_s]; rs = [rs,norm(r)];
    hessian = infeasible_hessian(x_s,b0,s_phi,d_phi);
    sol = -[hessian A';A,zeros(dimA+1,dimA+1)]\r;
    Dx = sol(1:size(x_s,1),1);Dnu=sol(size(x_s,1)+[1:dimA+1],1);
    if (norm(r)<RESTOL),break;end
    t=1;
    while norm([infeasible_grad(x_s+t*Dx,b0,s_phi,d_phi)+A'*(nu+t*Dnu);A*(x_s+Dx)])>...
            (1-ALPHA*t)*norm(r), t=BETA*t; end
    x_s = x_s + t*Dx; nu=nu+t*Dnu;
    
    fprintf('%d iter is done...\t',i);
    fprintf('residual is %.2f...\n',norm(r));
end

%% Optimization
% Suppose we already have a feasible starting point...
x = x_s(1:size(x_s,1)-1);

nu=zeros(dimA,1);
rs = [];
for i=1:MAXITER
    gradient = grad(x,b0,s_phi,d_phi);
    A = get_A(x,M_tilde,C_tilde,0,R,0,d_phi);
    r = [gradient+A.'*nu;A*x]; rs = [rs,norm(r)];
    hess = hessian(x,b0,s_phi,d_phi);
    sol = -[hess A.';A,zeros(dimA,dimA)]\r;
    Dx = sol(1:size(x,1),1);Dnu=sol(size(x,1)+[1:dimA],1);
    if (norm(r)<RESTOL),break;end
    t=1;
    while norm([grad(x+t*Dx,b0,s_phi,d_phi)+A'*(nu+t*Dnu);A*(x+Dx)])>...
            (1-ALPHA*t)*norm(r), t=BETA*t; end
    x = x + t*Dx; nu=nu+t*Dnu;
 
    fprintf('%d iter is done...\t',i);
    fprintf('residual is %.6f...\n',norm(r));
end
 

%% Visualization

[b,a,u]=get_bau(x);
max(b)
min(b)
b=[b0;b];
t = cumsum(2*d_phi./(sqrt(b(2:end))+sqrt(b(1:end-1))));

