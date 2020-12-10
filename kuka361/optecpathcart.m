function [xees,yees,zees,s] = optecpathcart()
%***************************************************************************
%  tag: Diederik Verscheure  wo jul  4 10:10:22 CEST 2007  optecpathcart.m
%
%                           optecpathcart.m -  description
%                           ----------------------------
%    begin                : wo juli 04 2007
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
% Generates the optec trajectory in operational space coordinates.
% 
% ***************************************************************************

if exist('../images/optec2.svg') == 0
	d = loadsvg('images/optec2.svg',0.06);
else
	d = loadsvg('../images/optec2.svg',0.06);
end

t = [1:length(d)];
yee = d(:,1)';
xee = d(:,2)';

% Rotate 90 degrees
yee = d(:,2)';
xee = -d(:,1)';

% To origin
yee = yee - yee(1);
xee = xee - xee(1);

t = (t - t(1))./(t(end)-t(1));

s = linspace(t(1),t(end),length(t));

xeei = 0.0;

xeei = -0.1;
yeei = -0.7;
zeei = 1.2;

xeei = -0.2;
yeei = -0.5;
zeei = 1.0;

% Scaling factor
wscale = 0.8/500;

% Write on horizontal plane
xees = xeei - wscale*interp1(t,xee,s,'linear');
yees = yeei - wscale*interp1(t,yee,s,'linear');
zees = ones(size(s))*zeei;

%% Write on vertical plane
%zeei = 0.8;
%xees = xeei - wscale*interp1(t,xee,s,'linear');
%yees = ones(size(s))*yeei;
%zees = zeei + wscale*interp1(t,yee,s,'linear');

