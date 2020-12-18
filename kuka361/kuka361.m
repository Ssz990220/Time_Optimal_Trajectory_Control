%***************************************************************************
%  tag: Diederik Verscheure  di jun 12 18:12:36 CEST 2007  kuka361.m
%
%                           kuka361.m -  description
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
% Tests the time-optimal trajectory planning algorithm
% for a 6-DOF KUKA 361 manipulator.
% 
% ***************************************************************************

clear all;
close all;
global savefigs;
% When set to 1, this will save all images in "./images/" directory.
savefigs = 0;

% Use mosek, csdp, sdpt3, sedumi.
% Only tested extensively with sedumi so far.
solver = 'mosek';
solver = 'csdp';
solver = 'sdpt3';
solver = 'sedumi';
    
% Number of grid points.
sgrid = 2000;

% Load KUKA 361 parameters.
params361;
% Lower and upper bounds for the torques.
torquelb = -taumax*0.8;
torqueub = taumax*0.8;
nrdof = size(torquelb,1);

% The weighting.
% gamma1 or gamma2 can be vectors (but not both of them).
% In that case, it is possible to plot trade-off curves.
% Use kuka361tradeoff.m to process the results.
gamma1 = [0 logspace(-3,0.6,20)]; gamma1 = 0;
gamma2 = [0 logspace(-3,1,20)]; gamma2 = 0;

% If gamma1 or gamma2 are vectors, this vector contains the trajectory times
% as a function of gamma1 or gamma2.
ts = zeros(1,max(length(gamma1),length(gamma2)));
% If gamma1 is a vector, this vector contains the integral of the squares of the torques
% or if gamma2 is a vector, this vector contains the integral of the absolute value
% of the rate of change of the torques.
tausqs = zeros(1,max(length(gamma1),length(gamma2)));

% Cell arrays to save all results if gamma1 or gamma2 are vectors.
tc = cell(1,max(length(gamma1),length(gamma2)));
sc = cell(1,max(length(gamma1),length(gamma2)));
smc = cell(1,max(length(gamma1),length(gamma2)));
tauc = cell(1,max(length(gamma1),length(gamma2)));
qc = cell(1,max(length(gamma1),length(gamma2)));
qdotc = cell(1,max(length(gamma1),length(gamma2)));
qddotc = cell(1,max(length(gamma1),length(gamma2)));
xeec = cell(1,max(length(gamma1),length(gamma2)));
yeec = cell(1,max(length(gamma1),length(gamma2)));
zeec = cell(1,max(length(gamma1),length(gamma2)));
bc = cell(1,max(length(gamma1),length(gamma2)));
ac = cell(1,max(length(gamma1),length(gamma2)));

% The path function.
pathfun = @optecpath;
% The manipulator dynamics.
invdynfun = @invdynkuka361;

% Set to 1 to allow Coulomb friction in the dynamic model.
coulomb = 1;	
% Fetch all path-dependent quantities.
[s, sm, ds, q, qp, qpp, Mv, Cv, gv] = dynpathparams(invdynfun, pathfun, sgrid, coulomb);

plot(sm,qp(1,:));
for l = 1:length(gamma1)
for m = 1:length(gamma2)
	close all
	% Solve the time-optimal trajectory planning problem.
	% ---------------------------------------
	% Basic version with torques as variables
	% [a, b, c, d, tau, bvar, cvar, dvar, tauvar, F, t, diagnostics] = timeoptpathconstr(s, sm, ds, q, qp, qpp, Mv, Cv, gv, torquelb, torqueub, gamma1(l), gamma2(m), solver);
	% ---------------------------------------
	% Optimized version with torques eliminated as optimization variables.
	%[a, b, c, d, tau, bvar, cvar, dvar, F, t, diagnostics] = timeoptpathconstropt(s, sm, ds, q, qp, qpp, Mv, Cv, gv, torquelb, torqueub, gamma1(l), gamma2(m), solver);
	% ---------------------------------------
	% Optimized version with torques eliminated as optimization variables which does not use YALMIP.
	[a, b, c, d, tau, diagnostics] = timeoptpathconstrsed(s, sm, ds, q, qp, qpp, Mv, Cv, gv, torquelb, torqueub, gamma1(l), gamma2(m), solver);
	% ---------------------------------------

	t = zeros(1,length(s)-1);
	% Obtain the time vector.
	t = cumsum(2*ds./(sqrt(b(2:end))+sqrt(b(1:end-1))));
	[xee,yee,zee] = fwkinkuka361(q);
	bm = (b(2:end)+b(1:end-1))/2;
	qdot = repmat(sqrt(bm),6,1).*qp;
	qddot = repmat(a,6,1).*qp + repmat(bm,6,1).*qpp;

	% Save result for trade-off curve.
	if length(gamma1) >= length(gamma2)
		ts(l) = t(end);
		% This is the integral of the torques squared
		tausqs(l) = sum(sum(repmat(2*ds./(sqrt(b(2:end))+sqrt(b(1:end-1))),6,1).*(tau./repmat(torqueub,1,length(s)-1)).^2));
	else
		ts(m) = t(end);
		% This is the integral of the absolute value of the torque rates of change!!!
		tausqs(m) = sum(sum(abs(tau(:,2:end) - tau(:,1:end-1))./repmat(torqueub,1,length(s)-2))) + sum(abs(tau(:,1))./torqueub) + sum(abs(tau(:,end))./torqueub);
	end
	
	% Plot interesting things.
	figure
	plot(t,tau(1,:),'b',t,tau(2,:),'g-.',t,tau(3,:),'k--',t,tau(4,:),'c',t,tau(5,:),'m-.',t,tau(6,:),'r--')
	setunits('s','nm');
	title('Joint torques')
	legend('joint torque 1','joint torque 2','joint torque 3','joint torque 4','joint torque 5','joint torque 6','location','SouthEast');
	setfigtempl;
	as = axis;
	axis([as(1) as(2) -1.1*max(torqueub) 1.1*max(torqueub)]);
	savepls(gcf,sprintf('kuka361_torquetime_%s_%d_%d',func2str(pathfun),gamma1(l)*100,gamma2(m)*100));

	
	% Plot first three torques as a function of the path coordinate.
	% The other torques are less interesting.
	figure
	lh = plot(sm,tau(1,:),'b',sm,tau(2,:),'g-.',sm,tau(3,:),'k--'); set(lh(2),'color',get(lh(2),'color')*0.4);
	setunits('-','nm');
	xlabel('path coordinate (-)')
	title('Joint torques')
	legend('joint torque 1','joint torque 2','joint torque 3','location','SouthEast');
	setfigtempl;
	as = axis;
	axis([as(1) as(2) -1.1*max(torqueub) 1.1*max(torqueub)]);
	axis([0.55 0.8 -1.1*max(torqueub) 1.1*max(torqueub)]);
	set(gcf,'position',[175 188 853 400]);
	set(gca,'xtick',[0.55:0.02:0.8])
	savepls(gcf,sprintf('kuka361_torquepath_%s_%d_%d',func2str(pathfun),gamma1(l)*100,gamma2(m)*100));
	
	figure
	plot(t,sm);
	title('Path coordinate - time relation');
	xlabel('time (s)');
	ylabel('path coordinate s (-)');
	setfigtempl;
	savepls(gcf,sprintf('kuka361_pathtime_%s_%d_%d',func2str(pathfun),gamma1(l)*100,gamma2(m)*100));
	
	% Plot all joint angles.
	figure
	plot(t,q(1,:),'b',t,q(2,:),'g-.',t,q(3,:),'k--',t,q(4,:),'c',t,q(5,:),'m-.',t,q(6,:),'r--');
	xlabel('time (s)')
	ylabel('joint angle (rad)')
	title('Joint angles')
	legend('joint angle 1','joint angle 2','joint angle 3','joint angle 4','joint angle 5','joint angle 6','location','SouthEast');
	setfigtempl;
	savepls(gcf,sprintf('kuka361_jointtime_%s_%d_%d',func2str(pathfun),gamma1(l)*100,gamma2(m)*100));
	
	figure
	plot3(xee,yee,zee,'ro');
	xlabel('x-coordinate (m)');
	ylabel('y-coordinate (m)');
	zlabel('z-coordinate (m)');
	title('Spatial trajectory');
	setfigtempl;
	as = axis;
	axis([as(1:4) 0 as(6)]);
	savepls(gcf,sprintf('kuka361_%s_3d',func2str(pathfun)));

	% Plot the end-effector path with indications of path coordinate
	% in steps of 0.05
	figure
	plot(xee,yee,'r');
	hold on;
	sdiv = [0:0.05:1];
	sdividx = zeros(size(sdiv));
	for p = 1:length(sdiv)
		[ms, sdividx(p)] = min((sm-sdiv(p)).^2);
	end
	xyasdiv = [-0.1638   -0.5291;
			-0.2086   -0.3960;
			-0.2919   -0.5873;
			-0.3688   -0.5846;
			-0.3942   -0.3627;
			-0.3239   -0.4209;
			-0.4862   -0.5818;
			-0.4200   -0.4681;
			-0.5872   -0.6733;
			-0.5660   -0.5319;
			-0.6443   -0.4847;
			-0.7781   -0.5291;
			-0.7233   -0.4154
			-0.8990   -0.6595;
			-0.5490   -0.8897;
			-0.1509   -0.7704;
			-0.2107   -0.2850;
			-0.6529   -0.3044;
			-0.4981   -0.2018;
			-0.4302   -0.1020;
			-0.5554   -0.0521];
	plot(xee(sdividx),yee(sdividx),'ro');
	for p = 1:length(sdiv)
		text(xyasdiv(p,1),xyasdiv(p,2),sprintf('%.2f',sdiv(p)),'fontsize',12);
	end
	xlabel('x-coordinate (m)');
	ylabel('y-coordinate (m)');
	title('Trajectory in the plane parallel to the XY-plane');
	setfigtempl;
	as = axis;
	axis([as(1)-0.07 as(2)+0.07 as(3)-0.07 as(4)+0.07]);
	view(180,90);
	savepls(gcf,sprintf('kuka361_%s_2d',func2str(pathfun)));
	
	% Plot all joint velocities.
	figure
	plot(t,qdot(1,:),'b',t,qdot(2,:),'g-.',t,qdot(3,:),'k--',t,qdot(4,:),'c',t,qdot(5,:),'m-.',t,qdot(6,:),'r--');
	xlabel('time (s)')
	ylabel('joint velocity (rad/s)')
	title('Joint velocities')
	legend('joint velocity 1','joint velocity 2','joint velocity 3','joint velocity 4','joint velocity 5','joint velocity 6','location','SouthEast');
	setfigtempl;
	as = axis;
	savepls(gcf,sprintf('kuka361_jointveltime_%s_%d_%d',func2str(pathfun),gamma1(l)*100,gamma2(m)*100));

	% Plot first three joint velocities as a function of the path coordinate.
	% Other velocities are less interesting.
	figure
	lh = plot(sm,qdot(1,:),'b',sm,qdot(2,:),'g-.',sm,qdot(3,:),'k--'); set(lh(2),'color',get(lh(2),'color')*0.4);
	xlabel('path coordinate (-)')
	ylabel('joint velocity (rad/s)')
	title('Joint velocities')
	legend('joint velocity 1','joint velocity 2','joint velocity 3','location','NorthWest');
	setfigtempl;
	as = axis;
	axis([as(1) as(2) as(3) as(4)]);
	axis([0.55 0.8 as(3) as(4)]);
	set(gcf,'position',[175 188 853 400]);
	set(gca,'xtick',[0.55:0.02:0.8])
	savepls(gcf,sprintf('kuka361_jointvelpath_%s_%d_%d',func2str(pathfun),gamma1(l)*100,gamma2(m)*100));

	figure
	plot(s,sqrt(b),'b-x');
	xlabel('path coordinate (-)');
	ylabel('pseudo-velocity (1/s)');
	title('Pseudo-velocity');
	setfigtempl;
	as = axis;
	axis([as(1) as(2) as(3) as(4)*1.05]);
	axis([0.55 0.8 as(3) as(4)*1.05]);
	set(gcf,'position',[175 188 853 400]);
	savepls(gcf,sprintf('kuka361_pseudovelpath_%s_%d_%d',func2str(pathfun),gamma1(l)*100,gamma2(m)*100));
	
	figure
	plot(sm,a);
	xlabel('path coordinate (-)')
	ylabel('pseudo-acceleration (1/s^2)');
	title('Pseudo-acceleration');
	setfigtempl;
	as = axis;
	axis([as(1) as(2) as(3)*1.05 as(4)*1.05]);
	axis([0.55 0.8 as(3)/1.5 as(4)/1.5]);
	savepls(gcf,sprintf('kuka361_pseudoaccpath_%s_%d_%d',func2str(pathfun),gamma1(l)*100,gamma2(m)*100));

	% Save everything for kuka361tradeoff.m.
	if length(gamma1) > 1
		tc{1,l} = t; sc{1,l} = s; smc{1,l} = sm; tauc{1,l} = tau;
		qc{1,l} = q; qdotc{1,l} = qdot; qddotc{1,l} = qddot;
		xeec{1,l} = xee; yeec{1,l} = yee; zeec{1,l} = zee;
		bc{1,l} = b; ac{1,l} = a;
	elseif length(gamma2) > 1
		tc{1,k} = t; sc{1,k} = s; smc{1,k} = sm; tauc{1,k} = tau;
		qc{1,k} = q; qdotc{1,k} = qdot; qddotc{1,k} = qddot;
		xeec{1,k} = xee; yeec{1,k} = yee; zeec{1,k} = zee;
		bc{1,k} = b; ac{1,k} = a;
	end
end
end

% Save everything for kuka361tradeoff.m.
if ((length(gamma1) > 1) | (length(gamma2) > 1))
	save ../mat/kuka361_tradeoff.mat tc sc smc tauc qc qdotc qddotc xeec yeec zeec bc ac ts tausqs gamma1 gamma2 torquelb torqueub
	% Run kuka361tradeoff
	kuka361tradeoff
end




	
	
