clear all;
close all;
clc;
global savefigs;
savefigs = 0; % switch to 1 if you want to save all the result plots

% Number of grids
sgrid = 2000;   %2000 segments

global m Mu FN Wf d_phi b0;
m = 2;
g = 9.8;
Mu = 0.6;
FN = m*g;
Wf = 0.6;
t_update = 50;

[phi,s_phi,s_mid,sd_mid,sdd_mid,phi_mid,d_phi,R,M_tilde,C_tilde] = CarDynPathFunction(sgrid);
b0 = 0;
b = 0.1*ones(sgrid,1);
a = zeros(sgrid,1);
u = zeros(sgrid*2,1);
d_phi = 1/sgrid;

[x0,s] = gen_x(b,a,u);

Aeq = get_A(sgrid,M_tilde,C_tilde,0,R,0,d_phi);
beq = zeros(size(Aeq,1),1);
lb_b = zeros(sgrid,1);
lb_a = ones(sgrid,1)*(-inf);
lb_u = ones(sgrid*2,1)*(-Mu*FN);
[lb,s] = gen_x(lb_b, lb_a, lb_u);
ub_b = ones(sgrid,1)*(inf);
ub_a = ones(sgrid,1)*(inf);
ub_u = ones(sgrid*2,1)*(Mu*FN);
[ub,s] = gen_x(ub_b, ub_a, ub_u);

objective = @objective_fnc;
A=[];
b=[];
x = fmincon(objective, x0, A, b ,Aeq, beq, lb, ub);