%***************************************************************************
%  tag: Diederik Verscheure  vr jan  6 10:11:28 CET 2006  params361.m
%
%                           params361.m -  description
%                           ----------------------------
%    begin                : vr januari 06 2006
%    copyright            : (C) 2006 Diederik Verscheure
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
% This file contains the controller transfer functions
% and the dynamic model for the first three axes of the Kuka 361 robot.
% Based on the Master's thesis:
% Verdonck - Kuka361/cd3/TrajectCompensation/Studenten/Tekst PDF/
% 
% ***************************************************************************

% Parameters for current regulator
% I = (a*UN + b)/R
%         Parameter a      Parameter b       Shuntweerstand R 
% As 1    -0.7874          -0.0248           0.1733
% As 2    -0.7904           0.0091           0.1709
% As 3    -0.7890          -0.0624           0.1730
%
% Axis 1

ai1 = -0.7874; 
bi1 = -0.0248;
Ri1 = 0.1733;

% Axis 2
ai2 = -0.7904;
bi2 = 0.0091;
Ri2 = 0.1709;

% Axis 3
ai3 = -0.7890;
bi3 = -0.0624;
Ri3 = 0.1730;

% Axis 4
ai4 = ai3;
bi4 = bi3;
Ri4 = Ri3;

% Axis 5
ai5 = ai3;
bi5 = bi3;
Ri5 = Ri3;

% Axis 6
ai6 = ai3;
bi6 = bi3;
Ri6 = Ri3;

ai = [ai1; ai2; ai3; ai4; ai5; ai6];
bi = [bi1; bi2; bi3; bi4; bi5; bi6];
Ri = [Ri1; Ri2; Ri3; Ri4; Ri5; Ri6];

% Velocity controller
% Lag model
% (SW - (UT*alpha + beta))*(P*(tau1*s + 1)/(tau2*s + 1)) = UN
%
%         P                 tau1       tau2         alpha        beta
% As 1    -174.1887         0.1834     0.6646       0.2475       -0.0093
% As 2    -624.8807         0.2367     2.4077       0.2380       -0.0029
% As 3     697.3184         0.3451   -18.9884       0.2554       -0.0238
%
% As 3 is wrong, non-minimum phase pole!!
%
% A0*SW(k) + A1*SW(k - 1) + A2*UT(k) + A3*UT(k - 1) + A4*1(k) + A5*UN(k - 1) = UN(k)
% Axis 1
Pc1 = -174.1887;
tau1c1 = 0.1834;
tau2c1 = 0.6646;
alphac1 = 0.2475;
betac1 = -0.0093;
%A0c1 = (Pc1*tau1c1*2 + Pc1*ts)/(tau2c1*2+ts);
%A1c1 = (-Pc1*tau1c1*2 + Pc1*ts)/(tau2c1*2+ts);
%A2c1 = (-Pc1*tau1c1*alphac1*2 + Pc1*alphac1*ts)/(tau2c1*2+ts);
%A3c1 = (Pc1*tau1c1*alphac1*2 + Pc1*alphac1*ts)/(tau2c1*2+ts);
%A4c1 = -2*Pc1*betac1*ts/(tau2c1*2+ts); 
%A5c1 = (tau2c1*2-ts)/(tau2c1*2+ts);

% Axis 2
Pc2 = -624.8807;
tau1c2 = 0.2367;
tau2c2 = 2.4077;
alphac2 = 0.2380;
betac2 = -0.0029;
%A0c2 = (Pc2*tau1c2*2 + Pc2*ts)/(tau2c2*2+ts);
%A1c2 = (-Pc2*tau1c2*2 + Pc2*ts)/(tau2c2*2+ts);
%A2c2 = (-Pc2*tau1c2*alphac2*2 + Pc2*alphac2*ts)/(tau2c2*2+ts);
%A3c2 = (Pc2*tau1c2*alphac2*2 + Pc2*alphac2*ts)/(tau2c2*2+ts);
%A4c2 = -2*Pc2*betac2*ts/(tau2c2*2+ts); 
%A5c2 = (tau2c2*2-ts)/(tau2c2*2+ts);

% Axis 3
Pc3 = -697.3184;
tau1c3 = 0.3451;
tau2c3 = 18.9884;
alphac3 = 0.2554;
betac3 = -0.0238;
%A0c3 = (Pc3*tau1c3*2 + Pc3*ts)/(tau2c3*2+ts);
%A1c3 = (-Pc3*tau1c3*2 + Pc3*ts)/(tau2c3*2+ts);
%A2c3 = (-Pc3*tau1c3*alphac3*2 + Pc3*alphac3*ts)/(tau2c3*2+ts);
%A3c3 = (Pc3*tau1c3*alphac3*2 + Pc3*alphac3*ts)/(tau2c3*2+ts);
%A4c3 = -2*Pc3*betac3*ts/(tau2c3*2+ts); 
%A5c3 = (tau2c3*2-ts)/(tau2c3*2+ts);

% Axis 4
alphac4 = 0.2554;
betac4 = -0.0238;

% Axis 5
alphac5 = 0.2554;
betac5 = -0.0238;

% Axis 6
alphac6 = 0.2554;
betac6 = -0.0238;

Pc4 = Pc3*7.85/3.41;
Pc5 = Pc3*7.85/3.255;
Pc6 = Pc3*7.85/2.24;

tau1c4 = tau1c3;
tau1c5 = tau1c3;
tau1c6 = tau1c3;
tau2c4 = tau2c3;
tau2c5 = tau2c3;
tau2c6 = tau2c3;

Pc = [Pc1; Pc2; Pc3; Pc4; Pc5; Pc6];
tau1c = [tau1c1; tau1c2; tau1c3; tau1c4; tau1c5; tau1c6];
tau2c = [tau2c1; tau2c2; tau2c3; tau1c4; tau1c5; tau1c6];
alphac = [alphac1; alphac2; alphac3; alphac4; alphac5; alphac6];
betac = [betac1; betac2; betac3; betac4; betac5; betac6];

%A0c = [A0c1; A0c2; A0c3];
%A1c = [A1c1; A1c2; A1c3];
%A2c = [A2c1; A2c2; A2c3];
%A3c = [A3c1; A3c2; A3c3];
%A4c = [A4c1; A4c2; A4c3];
%A5c = [A5c1; A5c2; A5c3];

% The velocity controllers
lag1 = tf(Pc(1,1)*[tau1c(1,1) 1],[tau2c(1,1) 1]);
lag2 = tf(Pc(2,1)*[tau1c(2,1) 1],[tau2c(2,1) 1]);
lag3 = tf(Pc(3,1)*[tau1c(3,1) 1],[tau2c(3,1) 1]);

% This is a temporary hack until I known the real controllers
% This is wrong
%lag4 = 7.85/3.41*tf(Pc(3,1)*[tau1c(3,1) 1],[tau2c(3,1) 1]);
%lag5 = 7.85/3.255*tf(Pc(3,1)*[tau1c(3,1) 1],[tau2c(3,1) 1]);
%lag6 = 7.85/2.24*tf(Pc(3,1)*[tau1c(3,1) 1],[tau2c(3,1) 1]);

lag4 = 3.41/5.8*tf(Pc(3,1)*[tau1c(3,1) 1],[tau2c(3,1) 1]);
lag5 = 3.255/5.8*tf(Pc(3,1)*[tau1c(3,1) 1],[tau2c(3,1) 1]);
lag6 = 2.24/5.8*tf(Pc(3,1)*[tau1c(3,1) 1],[tau2c(3,1) 1]);

% According to the thesis measurements
% UNRELIABLE!!!!!!!!!!!!!!!
% SW=matrix*parameters
% matrix(t,:,as)=[ SW(t-1,as) UT(t,as) UT(t-1,as) 1  UN(t,as) UN(t-1,as) ];
% A0*SW(k) + A1*SW(k - 1) + A2*UT(k) + A3*UT(k - 1) + A4*1(k) + A5*UN(k - 1) = UN(k)
%
% So rewrite:
% SW(k) = - A1/A0*SW(k - 1) - A2/A0*UT(k) - A3/A0*UT(k - 1) - A4/A0*1(k) - A5/A0*UN(k - 1) + 1/A0*UN(k)
%
% parameters =
%
% -A1/A0     0.9573    0.9771    0.9807
% -A2/A0     0.2473    0.2376    0.2608
% -A3/A0    -0.2367   -0.2321   -0.2560
% -A4/A0     0.0004    0.0001    0.0005
% +1/A0     -0.0207   -0.0167   -0.0722
% -A5/A0     0.0204    0.0167    0.0721
%Apc =[0.95732789122422   0.97706937970487   0.98071942744617;
%     0.24728223742672   0.23758630832566   0.26082538250347;
%     -0.23670351209264  -0.23211743799368  -0.25599018162217;
%     0.00039471916664   0.00005474169699   0.00049131121581;
%     -0.02068280192894  -0.01673850919610  -0.07222421902017;
%     0.02041434551637   0.01672246842919   0.07210772428051];
%A0cm = (1./Apc(5, :))';
%A1cm = -Apc(1, :)'.*A0cm;
%A2cm = -Apc(2, :)'.*A0cm;
%A3cm = -Apc(3, :)'.*A0cm;
%A4cm = -Apc(4, :)'.*A0cm;
%A5cm = -Apc(6, :)'.*A0cm;


% Tacho
%        Parameter    Parameter
%       a [V.sec/rad]   b [V]
% As 1    9.2750      0.0112
% As 2    10.0285     0.0083
% As 3    4.9633      0.0056
%          
% UT = a * q + b
%
% Axis 1
at1 = 9.2750;
bt1 = 0.0112;

% Axis 2
at2 = 10.0285;
bt2 = 0.0083;

% Axis 3
at3 = 4.9633;
bt3 = 0.0056;

% Axis 4
at4 = at3;
bt4 = bt3;

% Axis 5
at5 = at3;
bt5 = bt3;

% Axis 6
at6 = at3;
bt6 = bt3;

at = [at1; at2; at3; at4; at5; at6];
bt = [bt1; bt2; bt3; bt4; bt5; bt6];

% D/A parameters
% SW = ((dq/dt)/vs - vo)*10/2048
vs1 = 1.8846e-03;
vo1 = 0;
ad1 = 10/2048;

vs2 = 1.7858e-03;
vo2 = 1;
ad2 = 10/2048;

vs3 = 3.469e-03;
vo3 = -2;
ad3 = 10/2048;

vs4 = vs3;
vo4 = vo3;
ad4 = ad3;

vs5 = vs3;
vo5 = vo3;
ad5 = ad3;

vs6 = vs3;
vo6 = vo3;
ad6 = ad3;

vs = [vs1; vs2; vs3; vs4; vs5; vs6];
vo = [vo1; vo2; vo3; vo4; vo5; vo6];
ad = [ad1; ad2; ad3; ad4; ad5; ad6];

% Current to torque
% I=tau/Km
% Gear ratios: axis 1: 94.14706, axis 2: -103.23529, axis 3: 51.44118
%              axis 4: 175,       axis5: 150,        axis6:  131.64395
% cfr. robot.par file in Comrade path
% Torque constants: MO 800 axes 1-3: 29.8 Ncm/A
%                   MO 80/8 axes 4-6: 7.1 Ncm/A
% cfr Handbuch KUKA Motoreinheiten p4^M
%
% Don't remember where this 5.7 came from... However. Jan Koch
% says that below are actual Km's
%Km1 = 0.2781*5.77*94.14706; 
%Km2 = 0.2863*5.85*103.23529;
%Km3 = 0.2887*5.78*51.44118;
%Km4 = 0.07*5.7*175;
%Km5 = 0.07*5.7*150; 
%Km6 = 0.07*5.7*131.64395;
Km1 = 0.2781*94.14706; 
Km2 = 0.2863*103.23529;
Km3 = 0.2887*51.44118;
Km4 = 0.07*175;
Km5 = 0.07*150; 
Km6 = 0.07*131.64395;

Km = [Km1; Km2; Km3; Km4; Km5; Km6];

% Position controller parameters
Kpc1 = 20;
Kpc2 = 20;
Kpc3 = 20;
Kpc4 = 20;
Kpc5 = 20;
Kpc6 = 20;
Kpc = [Kpc1; Kpc2; Kpc3; Kpc4; Kpc5; Kpc6];

% Maximum torque
% Quite arbitrarily chosen.
% Based on described measurements found in the Ph.D. of W.Verdonck
% However KUKA info says that 10 A effective current is possible
% Based on values gathered by Jan Koch
taum1 = Km1*15;
taum2 = Km2*15;
taum3 = Km3*15;
taum4 = Km4*12;
taum5 = Km5*12;
taum6 = Km6*12;
taumax = [taum1; taum2; taum3; taum4; taum5; taum6];

% Current controller bandwidth
% Arbitrary choice: 1500 Hz
currc1 = tf(1,conv([1/1000 1], [1/1000 1]));
currc2 = tf(1,conv([1/1000 1], [1/1000 1]));
currc3 = tf(1,conv([1/1000 1], [1/1000 1]));
currc4 = tf(1,conv([1/1000 1], [1/1000 1]));
currc5 = tf(1,conv([1/1000 1], [1/1000 1]));
currc6 = tf(1,conv([1/1000 1], [1/1000 1]));

%                       as 1     as 2      as 3    as 4    as 5     as 6
% as min. [rad]       -3.3336  -2.0508   -2.3911 -4.6688 -3.0980  -4.6688
% as max. [rad]        3.3336   2.0508    2.3911  4.6688  3.0980   4.6688
% vel. max. [rad/s]      10        3         3       3       3        3
% acc. max. [rad/s2]      3       30        30      30      30       30
% overbrengings-     94.14706 103.23529 51.44118   175     150   131.64395
% verhouding
% These values are probably not correct!
qmin1 = -3.3336;
qmin2 = -2.0508;
qmin3 = -2.3911;
qmin4 = -4.6688;
qmin5 = -3.0980;
qmin6 = -4.6688;
qmin = [qmin1 qmin2 qmin3 qmin4 qmin5 qmin6];

qmax1 = 3.3336;
qmax2 = 2.0508;
qmax3 = 2.3911;
qmax4 = 4.6688;
qmax5 = 3.0980;
qmax6 = 4.6688;
qmax = [qmax1 qmax2 qmax3 qmax4 qmax5 qmax6];

% Maximum joint acceleration
% Probably not correct
dqmax1 = 10;
dqmax2 = 3;
dqmax3 = 3;
dqmax4 = 3;
dqmax5 = 3;
dqmax6 = 3;
% Found in Kuka361Constants.hpp
dqmax1 = 1;
dqmax2 = 1;
dqmax3 = 1;
dqmax4 = 2;
dqmax5 = 2;
dqmax6 = 2;

dqmax = [dqmax1;dqmax2;dqmax3;dqmax4;dqmax5;dqmax6];


% Maximum joint acceleration
% Probably not correct
ddqmax1 = 3;
ddqmax2 = 30;
ddqmax3 = 30;
ddqmax4 = 30;
ddqmax5 = 30;
ddqmax6 = 30;

% Angular resolution
% Found in Kuka361Constants.hpp
q1res = 2*pi/(94.14706*4096);
q2res = 2*pi/(103.23529*4096);
q3res = 2*pi/(51.44118*4096);
q4res = 2*pi/(175*4096);
q5res = 2*pi/(150*4096);
q6res = 2*pi/(131.64395*4096);

qres = [q1res;q2res;q3res;q4res;q5res;q6res];

%schaal = [1.0165    1.0178    0.9968];
% schaal = [1.0194    1.0299    1.0229];
%schaal = [1.0198    1.0276    1.0163];
% na herkalibratie motorconstanten e.d.
% tor(1).meas = result(set,9)*0.2781*5.77*94.14706*2/2048*schaal(1);  %measured in range +/- 2 Volt
% tor(2).meas = result(set,10)*0.2863*5.85*103.23529*2/2048*schaal(2); %measured in range +/- 2 Volt
% tor(3).meas = result(set,11)*0.2887*5.78*51.44118*1/2048*schaal(3);  %measured in range +/- 1 Volt

