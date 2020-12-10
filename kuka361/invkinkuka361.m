function [q1,q2,q3,q4,q5,q6] = invkinkuka361(xees,yees,zees,nsol3)
%***************************************************************************
%  tag: Diederik Verscheure  wo jul  4 09:51:30 CEST 2007  invkinkuka361.m
%
%                           invkinkuka361.m -  description
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
% Inverse kinematics.
% 04/07/07 - Robot model
% Zero position is with link 2 and 3 in parallel to Z-X plane
% Angles q2 and q3 are positive in downward direction (negative Y).
% 
% ***************************************************************************

q1 = zeros(size(xees));
q2 = zeros(size(xees));
q3 = zeros(size(xees));
q4 = zeros(size(xees));
q5 = zeros(size(xees));
q6 = zeros(size(xees));

for k = 1:length(xees)
	% Write on horizontal plane
	Tee = htranslate([xees(k);yees(k);zees(k);1])*rotx(pi);
	% Write on vertical plane
	%Tee = htranslate([xees(k);yees(k);zees(k);1])*rotx(pi/2);
	q = invkin361corr(Tee,3);
	q1(k) = q(1);
	q2(k) = q(2);
	q3(k) = q(3);
	q4(k) = q(4);
	q5(k) = q(5);
	q6(k) = q(6);
end
	


