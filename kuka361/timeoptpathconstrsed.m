function [a, b, c, d, tau, t, diagnostics] = timeoptpathconstrsed(s, sm, ds, q, qp, qpp, Mv, Cv, gv, torquelb, torqueub, gamma1, gamma2, solver)
%***************************************************************************
%  tag: Diederik Verscheure  di jun 12 15:28:22 CEST 2007  timeoptpathconstrsed.m
%
%                           timeoptpathconstrsed.m -  description
%                           ----------------------------
%    begin                : di juni 12 2007
%    copyright            : (C) 2007 K.U.Leuven
%    email                : diederik <dot> verscheure <at> mech <dot> kuleuven <dot> be
%
% This program is free software: you can redistribute it and/or modify
% it under the terms of the GNU General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% 
% This program is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU General Public License for more details.
% 
% You should have received a copy of the GNU General Public License
% along with this program.  If not, see <http://www.gnu.org/licenses/>.

% ***************************************************************************
% Purpose
% ---------------------------------------------------------------------------
% Solves the time-optimal trajectory planning problem.
% 
% ***************************************************************************
% [At, B, C, D, TAU, BVAR, CVAR, DVAR, F, T, DIAGNOSTICS] = TIMEOPTPATHCONSTRSED(S, SM, DS, Q, QP, QPP, MV, CV, GV, TORQUELB, TORQUEUB, GAMMA1, GAMMA2, SOLVER)
%
% INPUTS
% ----------------------
% The first 9 inputs should be generated by DYNPATHPARAMS.
% S: s grid row vector.
% SM: s midpoints row vector.
% DS: ds row vector.
% Q, QP, QPP: q, q' and q'' as a function of sm.
% MV, CV, GV: the mass, coriolis and gravity term as a function of
%             q, q' and q'' evaluated on sm.
%
%
% TORQUELB: torque lower bound (column vector). Dimension should be equal to system DOFS.
% TORQUEUB: torque upper bound (column vector). Dimension should be equal to system DOFS.
% GAMMA1: weighting on the integral of the square of the torques.
% GAMMA2: weighting on the integral of the absolute value of the rate of change of the torque.
% SOLVER: the solver to be used.
%
% OUTPUTS
% ----------------------
% A: sddot evaluated on the midpoints of the s grid (row vector).
% B: sdot^2 evaluated on the s grid (row vector) with beginning and end equal to zero.
% C: slack variables evaluated on the s grid (row vector) except at beginning and end.
% D: slack variables evaluated on the midpoints of the s grid (row vector)
% TAU: tau evaluated on the midpoints of the s grid (matrix).
% T: the objective function

% ------------------------------------------------------
% |                   INITIALISATION                   |
% ------------------------------------------------------

% System DOFS
nrdof = size(torquelb,1);
setuptime = 0;
solvertime = 0;
dtime = 0;
	
% Number of optimization variables
% --------------------------------
% 1) The b's.
% 2) Slack variables c's.
% 3) Slack variables d's.
% 4) Slack variables e's.
nrb = length(s)-2;
% For each b one slack variable c, except for begin and end.
nrc = nrb;
nrd = length(s)-1;
% nrdof x nre is total number of e variables!
if gamma2 ~= 0
	nre = nrb;
else
	nre = 0;
end

% Linear inequalities
% -------------------
% 1) Lower bounds on torques
% 2) Upper bounds on torques
% 3) Related to slack variables e
nrtorquelbineq = nrdof*nrd;
nrtorqueubineq = nrdof*nrd;
nrslackeineq = nrdof*nre*2;

% Linear matrix or cone inequalities
% ----------------------------------
% 1) Related to slack variables c
% 2) Related to slack variables d
nrslackcineq = nrc;
nrslackdineq = nrd;

% ------------------------------------------------------
% |                     SOLUTION                       |
% ------------------------------------------------------
% Generate variables
% ------------------
tic
fprintf('Allocating optimization variables...\n');
% Variable vector
nrvar = nrb + nrc + nrd + nrdof*nre;

dtime = toc; setuptime = setuptime + dtime;
fprintf('Done allocating optimization variables (%.3f s)...\n',dtime);

% Set implementation
% ------------------

tic
fprintf('Generating torque constraints...\n');
% 1) Inequality constraints
% In the form At*x + ct >= 0
% tau(k) <= torqueub <=> -tau(k) + torqueub >= 0
%                 <=> -(Mv(k)/(2*ds(k))*(b(k)-b(k-1)) + Cv(k)/2*(b(k-1)+b(k)) + gv(k))./torqueub + torqueub./torqueub >= 0
%                 <=> -[Mv(k)/(2*ds(k))+Cv(k)/2]./torqueub*b(k) - [-Mv(k)/(2*ds(k))+Cv(k)/2]./torqueub*b(k-1) + [-gv(k) + torqueub]./torqueub >= 0
% tau(k) >= torquelb <=> tau(k) - torqueub >= 0
%                 <=> (Mv(k)/(2*ds(k))*(b(k)-b(k-1)) + Cv(k)/2*(b(k-1)+b(k)) + gv(k))./torqueub - torqueub./torqueub >= 0
%                 <=> [Mv(k)/(2*ds(k))+Cv(k)/2]./torqueub*b(k) + [-Mv(k)/(2*ds(k))+Cv(k)/2]./torqueub*b(k-1) + [gv(k) - torqueub]./torqueub >= 0
% ct contains torqueub, then torquelb
nzmax = (nrd-2)*nrdof*4 + 2*nrdof*2 + (nre-2)*nrdof*6 + 2*nrdof*4 + nre*nrdof*2 + (nrd-2)*(nrdof*2+6) + 2*(nrdof+4) + nrc*3;
At = spalloc(nrtorquelbineq+nrtorqueubineq+nrslackeineq+nrslackdineq*(nrdof+3)+nrc*3,nrvar,nzmax);
ct = zeros(nrtorquelbineq+nrtorqueubineq+nrslackeineq+nrslackdineq*(nrdof+3)+nrc*3,1);

% Note: b is zero at the boundaries
At(1:nrdof,1) = -(Mv(:,1)/(2*ds(1))+Cv(:,1)/2)./torqueub;
ct(1:nrdof) = (-gv(:,1) + torqueub)./torqueub;
At(nrdof+1:2*nrdof,1) = (Mv(:,1)/(2*ds(1))+Cv(:,1)/2)./torqueub;
ct(nrdof+1:2*nrdof) = (gv(:,1) - torquelb)./torqueub;

for k = 2:nrd-1
	At(2*(k-1)*nrdof+1:2*(k-1)*nrdof+nrdof,k) = -(Mv(:,k)/(2*ds(k))+Cv(:,k)/2)./torqueub;
	At(2*(k-1)*nrdof+1:2*(k-1)*nrdof+nrdof,k-1) = -(-Mv(:,k)/(2*ds(k))+Cv(:,k)/2)./torqueub;
	ct(2*(k-1)*nrdof+1:2*(k-1)*nrdof+nrdof) = (-gv(:,k) + torqueub)./torqueub;
	At(2*(k-1)*nrdof+nrdof+1:2*(k-1)*nrdof+2*nrdof,k) = (Mv(:,k)/(2*ds(k))+Cv(:,k)/2)./torqueub;
	At(2*(k-1)*nrdof+nrdof+1:2*(k-1)*nrdof+2*nrdof,k-1) = (-Mv(:,k)/(2*ds(k))+Cv(:,k)/2)./torqueub;
	ct(2*(k-1)*nrdof+nrdof+1:2*(k-1)*nrdof+2*nrdof) = (gv(:,k) - torquelb)./torqueub;
end

% Note: b is zero at the boundaries
At(2*(nrd-1)*nrdof+1:2*(nrd-1)*nrdof+nrdof,nrd-1) = -(-Mv(:,nrd)/(2*ds(nrd))+Cv(:,nrd)/2)./torqueub;
ct(2*(nrd-1)*nrdof+1:2*(nrd-1)*nrdof+nrdof) = (-gv(:,nrd) + torqueub)./torqueub;
At(2*(nrd-1)*nrdof+nrdof+1:2*(nrd-1)*nrdof+2*nrdof,nrd-1) = (-Mv(:,nrd)/(2*ds(nrd))+Cv(:,nrd)/2)./torqueub;
ct(2*(nrd-1)*nrdof+nrdof+1:2*(nrd-1)*nrdof+2*nrdof) = (gv(:,nrd) - torquelb)./torqueub;

dtime = toc; setuptime = setuptime + dtime;
fprintf('Done generating torque constraints (%.3f s)...\n',dtime);

tic
fprintf('Generating absolute value constraints...\n');
% -e <= dtau <= e <=> [dtau./torqueub;e] >=0 and [-dtau./torqueub;e] >= 0 
% [dtau./torqueub;e]>=0 <=> [tau(k+1)./torqueub-tau(k)./torqueub; e(k)] >= 0
%                       <=> [(Mv(k+1)/(2*ds(k+1))*(b(k+1)-b(k)) + Cv(k+1)/2*(b(k)+b(k+1)) + gv(k+1))./torqueub - (Mv(k)/(2*ds(k))*(b(k)-b(k-1)) + Cv(k)/2*(b(k-1)+b(k)) + gv(k))./torqueub; e(k)] >= 0
%                       <=> [[Mv(k+1)/(2*ds(k+1))+Cv(k+1)/2]./torqueub*b(k+1) + [-Mv(k+1)/(2*ds(k+1))+Cv(k+1)/2-Mv(k)/(2*ds(k))-Cv(k)/2]./torqueub*b(k) + [Mv(k)/(2*ds(k))-Cv(k)/2]./torqueub*b(k-1); e(k)] + [[gv(k+1)-gv(k)]./torqueub;0]>= 0
% [-dtau./torqueub;e]>=0 <=> [-tau(k+1)./torqueub+tau(k)./torqueub; e(k)] >= 0
%                        <=> [-(Mv(k+1)/(2*ds(k+1))*(b(k+1)-b(k)) + Cv(k+1)/2*(b(k)+b(k+1)) + gv(k+1))./torqueub + (Mv(k)/(2*ds(k))*(b(k)-b(k-1)) + Cv(k)/2*(b(k-1)+b(k)) + gv(k))./torqueub; e(k)] >= 0
%                        <=> [[-Mv(k+1)/(2*ds(k+1))-Cv(k+1)/2]./torqueub*b(k+1) + [Mv(k+1)/(2*ds(k+1))-Cv(k+1)/2+Mv(k)/(2*ds(k))+Cv(k)/2]./torqueub*b(k) + [-Mv(k)/(2*ds(k))+Cv(k)/2]./torqueub*b(k-1); e(k)] + [[-gv(k+1)+gv(k)]./torqueub;0]>= 0
if gamma2 ~= 0
	At(nrtorquelbineq+nrtorqueubineq+1:nrtorquelbineq+nrtorqueubineq+nrdof,1) = (-Mv(:,2)/(2*ds(2))+Cv(:,2)/2-Mv(:,1)/(2*ds(1))-Cv(:,1)/2)./torqueub;
	At(nrtorquelbineq+nrtorqueubineq+1:nrtorquelbineq+nrtorqueubineq+nrdof,2) = (Mv(:,2)/(2*ds(2))+Cv(:,2)/2)./torqueub;
	At(nrtorquelbineq+nrtorqueubineq+1:nrtorquelbineq+nrtorqueubineq+nrdof,nrb+nrc+nrd+1:nrb+nrc+nrd+nrdof) = eye(nrdof); 
	ct(nrtorquelbineq+nrtorqueubineq+1:nrtorquelbineq+nrtorqueubineq+nrdof) = (gv(:,2)-gv(:,1))./torqueub;
	At(nrtorquelbineq+nrtorqueubineq+1+nrdof:nrtorquelbineq+nrtorqueubineq+2*nrdof,1) = (Mv(:,2)/(2*ds(2))-Cv(:,2)/2+Mv(:,1)/(2*ds(1))+Cv(:,1)/2)./torqueub;
	At(nrtorquelbineq+nrtorqueubineq+1+nrdof:nrtorquelbineq+nrtorqueubineq+2*nrdof,2) = (-Mv(:,2)/(2*ds(2))-Cv(:,2)/2)./torqueub;
	At(nrtorquelbineq+nrtorqueubineq+1+nrdof:nrtorquelbineq+nrtorqueubineq+2*nrdof,nrb+nrc+nrd+1:nrb+nrc+nrd+nrdof) = eye(nrdof);
	ct(nrtorquelbineq+nrtorqueubineq+1+nrdof:nrtorquelbineq+nrtorqueubineq+2*nrdof) = (-gv(:,2)+gv(:,1))./torqueub;
	for k = 2:nre-1
		At(nrtorquelbineq+nrtorqueubineq+2*(k-1)*nrdof+1:nrtorquelbineq+nrtorqueubineq+2*(k-1)*nrdof+nrdof,k-1) = (Mv(:,k)/(2*ds(k))-Cv(:,k)/2)./torqueub;
		At(nrtorquelbineq+nrtorqueubineq+2*(k-1)*nrdof+1:nrtorquelbineq+nrtorqueubineq+2*(k-1)*nrdof+nrdof,k) = (-Mv(:,k+1)/(2*ds(k+1))+Cv(:,k+1)/2-Mv(:,k)/(2*ds(k))-Cv(:,k)/2)./torqueub;
		At(nrtorquelbineq+nrtorqueubineq+2*(k-1)*nrdof+1:nrtorquelbineq+nrtorqueubineq+2*(k-1)*nrdof+nrdof,k+1) = (Mv(:,k+1)/(2*ds(k+1))+Cv(:,k+1)/2)./torqueub;
		At(nrtorquelbineq+nrtorqueubineq+2*(k-1)*nrdof+1:nrtorquelbineq+nrtorqueubineq+2*(k-1)*nrdof+nrdof,nrb+nrc+nrd+(k-1)*nrdof+1:nrb+nrc+nrd+(k-1)*nrdof+nrdof) = eye(nrdof); 
		ct(nrtorquelbineq+nrtorqueubineq+2*(k-1)*nrdof+1:nrtorquelbineq+nrtorqueubineq+2*(k-1)*nrdof+nrdof) = (gv(:,k+1)-gv(:,k))./torqueub;
		At(nrtorquelbineq+nrtorqueubineq+2*(k-1)*nrdof+nrdof+1:nrtorquelbineq+nrtorqueubineq+2*(k-1)*nrdof+2*nrdof,k-1) = (-Mv(:,k)/(2*ds(k))+Cv(:,k)/2)./torqueub;
		At(nrtorquelbineq+nrtorqueubineq+2*(k-1)*nrdof+nrdof+1:nrtorquelbineq+nrtorqueubineq+2*(k-1)*nrdof+2*nrdof,k) = (Mv(:,k+1)/(2*ds(k+1))-Cv(:,k+1)/2+Mv(:,k)/(2*ds(k))+Cv(:,k)/2)./torqueub;
		At(nrtorquelbineq+nrtorqueubineq+2*(k-1)*nrdof+nrdof+1:nrtorquelbineq+nrtorqueubineq+2*(k-1)*nrdof+2*nrdof,k+1) = (-Mv(:,k+1)/(2*ds(k+1))-Cv(:,k+1)/2)./torqueub;
		At(nrtorquelbineq+nrtorqueubineq+2*(k-1)*nrdof+nrdof+1:nrtorquelbineq+nrtorqueubineq+2*(k-1)*nrdof+2*nrdof,nrb+nrc+nrd+(k-1)*nrdof+1:nrb+nrc+nrd+(k-1)*nrdof+nrdof) = eye(nrdof); 
		ct(nrtorquelbineq+nrtorqueubineq+2*(k-1)*nrdof+nrdof+1:nrtorquelbineq+nrtorqueubineq+2*(k-1)*nrdof+2*nrdof) = (-gv(:,k+1)+gv(:,k))./torqueub;
	end
	At(nrtorquelbineq+nrtorqueubineq+2*(nre-1)*nrdof+1:nrtorquelbineq+nrtorqueubineq+2*(nre-1)*nrdof+nrdof,nre-1) = (Mv(:,nre)/(2*ds(nre))-Cv(:,nre)/2)./torqueub;
	At(nrtorquelbineq+nrtorqueubineq+2*(nre-1)*nrdof+1:nrtorquelbineq+nrtorqueubineq+2*(nre-1)*nrdof+nrdof,nre) = (-Mv(:,nre+1)/(2*ds(nre+1))+Cv(:,nre+1)/2-Mv(:,nre)/(2*ds(nre))-Cv(:,nre)/2)./torqueub;
	At(nrtorquelbineq+nrtorqueubineq+2*(nre-1)*nrdof+1:nrtorquelbineq+nrtorqueubineq+2*(nre-1)*nrdof+nrdof,nrb+nrc+nrd+(nre-1)*nrdof+1:nrb+nrc+nrd+(nre-1)*nrdof+nrdof) = eye(nrdof); 
	ct(nrtorquelbineq+nrtorqueubineq+2*(nre-1)*nrdof+1:nrtorquelbineq+nrtorqueubineq+2*(nre-1)*nrdof+nrdof) = (gv(:,nre+1)-gv(:,nre))./torqueub;
	At(nrtorquelbineq+nrtorqueubineq+2*(nre-1)*nrdof+nrdof+1:nrtorquelbineq+nrtorqueubineq+2*(nre-1)*nrdof+2*nrdof,nre-1) = (-Mv(:,nre)/(2*ds(nre))+Cv(:,nre)/2)./torqueub;
	At(nrtorquelbineq+nrtorqueubineq+2*(nre-1)*nrdof+nrdof+1:nrtorquelbineq+nrtorqueubineq+2*(nre-1)*nrdof+2*nrdof,nre) = (Mv(:,nre+1)/(2*ds(nre+1))-Cv(:,nre+1)/2+Mv(:,nre)/(2*ds(nre))+Cv(:,nre)/2)./torqueub;
	At(nrtorquelbineq+nrtorqueubineq+2*(nre-1)*nrdof+nrdof+1:nrtorquelbineq+nrtorqueubineq+2*(nre-1)*nrdof+2*nrdof,nrb+nrc+nrd+(nre-1)*nrdof+1:nrb+nrc+nrd+(nre-1)*nrdof+nrdof) = eye(nrdof); 
	ct(nrtorquelbineq+nrtorqueubineq+2*(nre-1)*nrdof+nrdof+1:nrtorquelbineq+nrtorqueubineq+2*(nre-1)*nrdof+2*nrdof) = (-gv(:,nre+1)+gv(:,nre))./torqueub;
end
dtime = toc; setuptime = setuptime + dtime;
fprintf('Done generating absolute value constraints (%.3f s)...\n',dtime);

tic
fprintf('Generating cone constraints...\n');
% 3) Cone constraints
% Note: b and c is zero at the boundaries

% First set
% Implicit: a zero row
At(nrtorquelbineq+nrtorqueubineq+nrslackeineq+1,nrb+1) = 1;
At(nrtorquelbineq+nrtorqueubineq+nrslackeineq+1,nrb+nrc+1) = 1;
ct(nrtorquelbineq+nrtorqueubineq+nrslackeineq+1) = 0;
ct(nrtorquelbineq+nrtorqueubineq+nrslackeineq+2) = 2;
At(nrtorquelbineq+nrtorqueubineq+nrslackeineq+2+1:nrtorquelbineq+nrtorqueubineq+nrslackeineq+2+nrdof,1) = 2*sqrt(gamma1)*(Mv(:,1)/(2*ds(1))+Cv(:,1)/2)./torqueub;
ct(nrtorquelbineq+nrtorqueubineq+nrslackeineq+2+1:nrtorquelbineq+nrtorqueubineq+nrslackeineq+2+nrdof) = 2*sqrt(gamma1)*(gv(:,1))./torqueub;
At(nrtorquelbineq+nrtorqueubineq+nrslackeineq+2+nrdof+1,nrb+1) = 1;
At(nrtorquelbineq+nrtorqueubineq+nrslackeineq+2+nrdof+1,nrb+nrc+1) = -1;
ct(nrtorquelbineq+nrtorqueubineq+nrslackeineq+2+nrdof+1) = 0;
for k = 2:nrd-1
	At(nrtorquelbineq+nrtorqueubineq+nrslackeineq+(k-1)*(nrdof+3)+1,nrb+k-1) = 1;
	At(nrtorquelbineq+nrtorqueubineq+nrslackeineq+(k-1)*(nrdof+3)+1,nrb+k) = 1;
	At(nrtorquelbineq+nrtorqueubineq+nrslackeineq+(k-1)*(nrdof+3)+1,nrb+nrc+k) = 1;
	ct(nrtorquelbineq+nrtorqueubineq+nrslackeineq+(k-1)*(nrdof+3)+1) = 0;
	ct(nrtorquelbineq+nrtorqueubineq+nrslackeineq+(k-1)*(nrdof+3)+2) = 2;
	At(nrtorquelbineq+nrtorqueubineq+nrslackeineq+(k-1)*(nrdof+3)+2+1:nrtorquelbineq+nrtorqueubineq+nrslackeineq+(k-1)*(nrdof+3)+2+nrdof,k) = 2*sqrt(gamma1)*(Mv(:,k)/(2*ds(k))+Cv(:,k)/2)./torqueub;
	At(nrtorquelbineq+nrtorqueubineq+nrslackeineq+(k-1)*(nrdof+3)+2+1:nrtorquelbineq+nrtorqueubineq+nrslackeineq+(k-1)*(nrdof+3)+2+nrdof,k-1) = 2*sqrt(gamma1)*(-Mv(:,k)/(2*ds(k))+Cv(:,k)/2)./torqueub;
	ct(nrtorquelbineq+nrtorqueubineq+nrslackeineq+(k-1)*(nrdof+3)+2+1:nrtorquelbineq+nrtorqueubineq+nrslackeineq+(k-1)*(nrdof+3)+2+nrdof) = 2*sqrt(gamma1)*(gv(:,k))./torqueub;
	At(nrtorquelbineq+nrtorqueubineq+nrslackeineq+(k-1)*(nrdof+3)+2+nrdof+1,nrb+k-1) = 1;
	At(nrtorquelbineq+nrtorqueubineq+nrslackeineq+(k-1)*(nrdof+3)+2+nrdof+1,nrb+k) = 1;
	At(nrtorquelbineq+nrtorqueubineq+nrslackeineq+(k-1)*(nrdof+3)+2+nrdof+1,nrb+nrc+k) = -1;
	ct(nrtorquelbineq+nrtorqueubineq+nrslackeineq+(k-1)*(nrdof+3)+2+nrdof+1) = 0;
end
At(nrtorquelbineq+nrtorqueubineq+nrslackeineq+(nrd-1)*(nrdof+3)+1,nrb+nrd-1) = 1;
At(nrtorquelbineq+nrtorqueubineq+nrslackeineq+(nrd-1)*(nrdof+3)+1,nrb+nrc+nrd) = 1;
ct(nrtorquelbineq+nrtorqueubineq+nrslackeineq+(nrd-1)*(nrdof+3)+1) = 0;
ct(nrtorquelbineq+nrtorqueubineq+nrslackeineq+(nrd-1)*(nrdof+3)+2) = 2;
At(nrtorquelbineq+nrtorqueubineq+nrslackeineq+(nrd-1)*(nrdof+3)+2+1:nrtorquelbineq+nrtorqueubineq+nrslackeineq+(nrd-1)*(nrdof+3)+2+nrdof,nrd-1) = 2*sqrt(gamma1)*(-Mv(:,nrd)/(2*ds(nrd))+Cv(:,nrd)/2)./torqueub;
ct(nrtorquelbineq+nrtorqueubineq+nrslackeineq+(nrd-1)*(nrdof+3)+2+1:nrtorquelbineq+nrtorqueubineq+nrslackeineq+(nrd-1)*(nrdof+3)+2+nrdof) = 2*sqrt(gamma1)*(gv(:,nrd))./torqueub;
At(nrtorquelbineq+nrtorqueubineq+nrslackeineq+(nrd-1)*(nrdof+3)+2+nrdof+1,nrb+nrd-1) = 1;
At(nrtorquelbineq+nrtorqueubineq+nrslackeineq+(nrd-1)*(nrdof+3)+2+nrdof+1,nrb+nrc+nrd) = -1;
ct(nrtorquelbineq+nrtorqueubineq+nrslackeineq+(nrd-1)*(nrdof+3)+2+nrdof+1) = 0;

% Second set
for k = 1:nrc
	At(nrtorquelbineq+nrtorqueubineq+nrslackeineq+nrslackdineq*(nrdof+3)+(k-1)*3+1,k) = 1;
	At(nrtorquelbineq+nrtorqueubineq+nrslackeineq+nrslackdineq*(nrdof+3)+(k-1)*3+2,nrb+k) = 2;
	At(nrtorquelbineq+nrtorqueubineq+nrslackeineq+nrslackdineq*(nrdof+3)+(k-1)*3+3,k) = 1;
	ct(nrtorquelbineq+nrtorqueubineq+nrslackeineq+nrslackdineq*(nrdof+3)+(k-1)*3+1) = 1;
	ct(nrtorquelbineq+nrtorqueubineq+nrslackeineq+nrslackdineq*(nrdof+3)+(k-1)*3+2) = 0;
	ct(nrtorquelbineq+nrtorqueubineq+nrslackeineq+nrslackdineq*(nrdof+3)+(k-1)*3+3) = -1;
end

dtime = toc; setuptime = setuptime + dtime;
fprintf('Done generating cone constraints (%.3f s)...\n',dtime);

% Transpose
At = -At';

tic
fprintf('Generating objective function...\n');
% 4) Objective function
if gamma2 ~= 0 
	bt = [zeros(nrb,1); zeros(nrc,1); ds'; gamma2*ones(nre*nrdof,1)];
else
	bt = [zeros(nrb,1); zeros(nrc,1); ds'];
end
bt = -bt;

dtime = toc; setuptime = setuptime + dtime;
fprintf('Done generating objective function (%.3f s)...\n',dtime);

K.l = nrtorquelbineq+nrtorqueubineq+nrslackeineq;
K.q = [(nrdof+3)*ones(1,nrslackdineq) 3*ones(1,nrslackcineq)];
pars.eps=1e-9;
tic
fprintf('---------------------------------------------------------------------------\n');
fprintf('Starting solver...\n');
fprintf('---------------------------------------------------------------------------\n');

if strcmp(lower(solver),'sdpt3') == 1
	% To use SDPT3
	[blk,At,C,b] = read_sedumi(At,bt,ct,K);
	[obj,xs,ys,zs,diagnostics] = sqlp(blk,At,C,b);
	x = ys;
else
	[xs,ys,diagnostics]=sedumi(At,bt,ct,K,pars);
	diagnostics
	x = ys;
end

dtime = toc; solvertime = solvertime + dtime;
fprintf('---------------------------------------------------------------------------\n');
fprintf('Solver ended (setup time = %.3f, solver time = %.3f s)...\n',setuptime,solvertime);
fprintf('---------------------------------------------------------------------------\n');

b = x(1:nrb)';
tau = zeros(nrdof,nrd);
tau(:,1) = (Mv(:,1)*(b(1,1)-0)/(2*ds(1)) + Cv(:,1)*(0 + b(1,1))/2 + gv(:,1));
for k = 2:nrd-1
        tau(:,k) = (Mv(:,k)*(b(1,k)-b(1,k-1))/(2*ds(k)) + Cv(:,k)*(b(1,k-1)+b(1,k))/2 + gv(:,k));
end
tau(:,nrd) = (Mv(:,nrd)*(0-b(1,nrd-1))/(2*ds(nrd)) + Cv(:,nrd)*(b(1,nrd-1)+0)/2 + gv(:,nrd));

b = [0 b 0];
a = (b(2:end)-b(1:end-1))./ds;
c = x(nrb+1:nrb+nrc)';
d = x(nrb+nrc+1:nrb+nrc+nrd);

t = 0;
%if diagnostics.problem == 0
%	fprintf('Succesfully solved problem: objective value %.4f...\n', double(t));
%else
%	fprintf('Error obtaining solution: objective value %.4f...\n', double(t));
%end
%
