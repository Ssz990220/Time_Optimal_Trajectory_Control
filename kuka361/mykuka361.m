clear all;
close all;
global savefigs;
% When set to 1, this will save all images in "./images/" directory.
savefigs = 0;

% Number of grid points.
sgrids = 10;

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

pathfun = @optecpath;
% The manipulator dynamics.
invdynfun = @invdynkuka361;

% Set to 1 to allow Coulomb friction in the dynamic model.
coulomb = 1;	
% Fetch all path-dependent quantities.
% [s, sm, ds, q, qp, qpp, Mv, Cv, gv] = dynpathparams(invdynfun, pathfun, sgrids, coulomb);
% save('workspace');
load('workspace');
%%
R = eye(6);
for l = 1:length(gamma1)
for m = 1:length(gamma2)
[a,b,tau]=mysolver(s, sm, ds, q, qp, qpp, R, Mv, Cv, gv, torquelb, torqueub, gamma1(l), gamma2(m));
end
end