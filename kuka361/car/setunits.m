function setunits(xu, yu)

%***************************************************************************
%  tag: Diederik Verscheure  ma jan  9 00:41:59 CET 2006  setunits.m
%
%                           setunits.m -  description
%                           ----------------------------
%    begin                : ma januari 09 2006
%    copyright            : (C) 2006 K.U.Leuven
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
% Function to set xlabel and ylabel si units with correct names.
% 
% ***************************************************************************

if strcmp(xu, 's') == 1
	xlabel('time (s)');
elseif strcmp(xu, 'kg') == 1
	xlabel('mass (kg)');
elseif (strcmp(xu, 'mp') == 1) | (strcmp(xu, 'm') == 1)
	xlabel('position (m)');
elseif (strcmp(xu, 'md') == 1)
	xlabel('deformation (m)');
elseif strcmp(xu, 'ml') == 1
	xlabel('length (m)');
elseif strcmp(xu, 'st') == 1
	xlabel('stiffness (N/m)');
elseif strcmp(xu, 'co') == 1
	xlabel('compliance (m/N)');
elseif strcmp(xu, 'da') == 1
	xlabel('damping coefficient (N.s/m)');
elseif strcmp(xu, 'n') == 1
	xlabel('force (N)');
elseif strcmp(xu, 'nm') == 1
	xlabel('torque (N.m)');
elseif strcmp(xu, 'nmm') == 1
	xlabel('moment (N.m)');	
elseif strcmp(xu, 'r') == 1
	xlabel('angle (rad)');
elseif strcmp(xu, 'rs') == 1
	xlabel('angular velocity (rad/s)');
elseif strcmp(xu, 'rs2') == 1
	xlabel('angular acceleration (rad/s^2)');
elseif strcmp(xu, 'ms') == 1
	xlabel('velocity (m/s)');
elseif strcmp(xu, 'ms2') == 1
	xlabel('acceleration (m/s^2)');
elseif strcmp(xu, 'dp') == 1
	xlabel('data point (-)');
elseif strcmp(xu, '-') == 1
	xlabel('(-)');
elseif strcmp(xu, 'ts') == 1
	xlabel('time-step (-)');
end

if strcmp(yu, 's') == 1
	ylabel('time (s)');
elseif strcmp(yu, 'kg') == 1
	ylabel('mass (kg)');
elseif (strcmp(yu, 'mp') == 1) | (strcmp(yu, 'm') == 1)
	ylabel('position (m)');
elseif (strcmp(yu, 'md') == 1)
	ylabel('deformation (m)');
elseif strcmp(yu, 'ml') == 1
	ylabel('length (m)');
elseif strcmp(yu, 'st') == 1
	ylabel('stiffness (N/m)');
elseif strcmp(yu, 'co') == 1
	ylabel('compliance (m/N)');
elseif strcmp(yu, 'da') == 1
	ylabel('damping coefficient (N.s/m)');
elseif strcmp(yu, 'n') == 1
	ylabel('force (N)');
elseif strcmp(yu, 'nm') == 1
	ylabel('torque (N.m)');
elseif strcmp(yu, 'nmm') == 1
	ylabel('moment (N.m)');	
elseif strcmp(yu, 'r') == 1
	ylabel('angle (rad)');
elseif strcmp(yu, 'rs') == 1
	ylabel('angular velocity (rad/s)');
elseif strcmp(yu, 'rs2') == 1
	ylabel('angular acceleration (rad/s^2)');
elseif strcmp(yu, 'ms') == 1
	ylabel('velocity (m/s)');
elseif strcmp(yu, 'ms2') == 1
	ylabel('acceleration (m/s^2)');
elseif strcmp(yu, 'dp') == 1
	ylabel('data point (-)');
elseif strcmp(yu, '-') == 1
	ylabel('(-)');
elseif strcmp(yu, 'ts') == 1
	xlabel('time-step (-)');
end


