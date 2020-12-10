clear all;
close all;
global savefigs;
savefigs = 0; % switch to 1 if you want to save all the result plots

% Number of grids
sgrid = 2000;   %2000 segments

global Mu m Wf FN
% coefficient of friction on the ground
Mu=0.2;
% weight of the car
m=20;
g=9.8;
FN=m*g;
Wf=0.60;

[phi,s_phi,s_mid,sd_mid,sdd_mid,phi_mid,d_phi,R_tilde,M_tilde,C_tilde] = CarDynPathFunction(sgrid);

    


