% clear all;
% close all;
% clc;
global savefigs;
savefigs = 0; % switch to 1 if you want to save all the result plots


%% paramaters
% Number of grids
sgrid =4000;   %2000 segments

global m Mu FN Wf d_phi;
m = 2;
g = 9.8;
Mu = 0.2;
FN = m*g;
Wf = 0.6;


[phi,s_phi,s_mid,sd_mid,sdd_mid,phi_mid,d_phi,R,M_tilde,C_tilde] = CarDynPathFunction(sgrid);
d_phi = 1/sgrid;
dimA=sgrid*3;

b0 = 0;
b = 0.1*ones(sgrid,1);
a = zeros(sgrid,1);
u = ones(sgrid*2,1);
[x,s] = gen_x(b,a,u);


MAXITER = 100;
ALPHA = 0.01;
BETA = 0.5;
RESTOL=1e-6;
THER=1e-8;
TOL = 1e-5;
t_update = 20;
T = 10;
%%  infeasible start newton
% x_s = [x;s];
% nu=zeros(dimA+1,1);
% rs = [];
% for i=1:MAXITER
%     gradient = infeasible_grad(x_s,b0,s_phi,d_phi,T);
%     A = infeasible_get_A(x_s,M_tilde,C_tilde,0,R,0,d_phi);
%     r = [gradient+A'*nu;A*x_s]; rs = [rs,norm(r)];
%     hessian = infeasible_hessian(x_s,b0,s_phi,d_phi,T);
%     sol = -[hessian A';A,zeros(dimA+1,dimA+1)]\r;
%     Dx = sol(1:size(x_s,1),1);Dnu=sol(size(x_s,1)+[1:dimA+1],1);
%     if (norm(r)<RESTOL),break;end
%     t=1;
%     while norm([infeasible_grad(x_s+t*Dx,b0,s_phi,d_phi,t)+A'*(nu+t*Dnu);A*(x_s+Dx)])>...
%             (1-ALPHA*t)*norm(r), t=BETA*t; end
%     x_s = x_s + t*Dx; nu=nu+t*Dnu;
%     
%     fprintf('%d iter is done...\t',i);
%     fprintf('residual is %.2f...\n',norm(r));
% end

%% Optimization

nu=zeros(dimA,1);
ress = [];
tic;
for i=1:MAXITER
    val = getobjfromx(x);
    A = get_A(sgrid,M_tilde,C_tilde,0,R,0,d_phi);
    r = getr(A,x,nu,T,b0,s_phi,d_phi);
    n = norm(r);
    hess = hessian_final(x,b0,s_phi,d_phi,T);
    KKT = sparse(sgrid*7,sgrid*7);
    KKT(1:sgrid*4,:)=[hess A.'];
    KKT(sgrid*4+1:sgrid*4+sgrid*3,1:sgrid*4)=A;
    sol = -KKT\r;
    Dx = sol(1:size(x,1),1);Dnu=sol(size(x,1)+[1:dimA],1);
%     tic;
%     [Dx, Dnu] = solve_rearrange_matrix(KKT, r);
%     toc;
%     fprintf("Back tracking...\n");
%     while (getobjfromx(x+t*Dx)>=val+ALPHA*t*fprime)
%         t = BETA*t;
%     end   
    t=1;
    while checkU(x+t*Dx)
        t=BETA*t;
    end
    while norm(getr(A,x+t*Dx,nu+t*Dnu,T,b0,s_phi,d_phi))>(1-ALPHA*t)*norm(r)
        t = BETA*t;
    end
    x = x + t*Dx; nu=nu+t*Dnu; 
    
    if (norm(r)<RESTOL)&&(norm(A*x)<THER)
        gap = 3*sgrid/T;
        fprintf('the gap at iter %d is %.6f\n',i,gap);
        if (gap<TOL),break;end
        T = t_update * T; 
        fprintf("Update T at iteration %d ...\n",i);
        
    end
end
toc;
%% Visualization

[b,a,u]=get_bau(x);
b=[b0;b];
t = cumsum(2*d_phi'./(sqrt(b(2:end))+sqrt(b(1:end-1))));
t(end)
bm = (b(2:end)+b(1:end-1))/2;
qdot = repmat(sqrt(bm'),2,1).*sd_mid;
qddot = repmat(a',2,1).*sd_mid + repmat(bm',2,1).*sdd_mid;


% Plot interesting things.
figure
plot(phi_mid,u(:,1),phi_mid,u(:,2))
setunits('s','n');
title('Joint Force')
legend('joint torque 1','joint torque 2','location','SouthEast');
setfigtempl;
as = axis;
axis([as(1) as(2) -1.1*Mu*FN 1.1*Mu*FN]);
    
figure
plot(t,phi_mid);
title('Path coordinate - time relation');
xlabel('time (s)');
ylabel('path coordinate s (-)');
setfigtempl;

figure;
plot(phi_mid, a);
title('Pseudo-acceleration');

figure;
plot(phi,b);
title('Pseudo-velocity');

figure
plot(t,s_phi(1,2:end),t,s_phi(2,2:end));
xlabel('time (s)')
ylabel('joint angle')
title('Joint angles')
legend('joint angle 1','joint angle 2','location','SouthEast');
setfigtempl;

figure
plot(t,sd_mid(1,:),'b',t,sd_mid(2,:));
xlabel('time (s)')
ylabel('joint velocity')
title('Joint velocity')
legend('joint velocity 1','joint velocity 2','location','SouthEast');
setfigtempl;

figure
plot(t,sdd_mid(1,:),'b',t,sdd_mid(2,:));
xlabel('time (s)')
ylabel('joint acceleration')
title('Joint acceleration')
legend('joint acceleration 1','joint acceleration 2','location','SouthEast');
setfigtempl;