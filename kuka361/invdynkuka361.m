function [M,C,g] = invdynkuka361(q,qdot,coulomb)
%***************************************************************************
%  tag: Diederik Verscheure  di jun 12 17:39:03 CEST 2007  invdynkuka361.m
%
%                           invdynkuka361.m -  description
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
% Inverse dynamic model the KUKA 361 robot.
% Note: qdot could be q'. This poses no significant problem however.
% 
% ***************************************************************************

M = zeros(6,6,size(q,2));
C = zeros(6,6,size(q,2));
g = zeros(6,size(q,2));

fprintf('Generating dynamic model matrices...\n');
for k = 1:size(q,2)
	q1 = q(1,k);
	q2 = q(2,k);
	q3 = q(3,k);
	q4 = q(4,k);
	q5 = q(5,k);
	q6 = q(6,k);
	qs = [q1 q2 q3 q4 q5 q6];

	qdot1 = qdot(1,k);
	qdot2 = qdot(2,k);
	qdot3 = qdot(3,k);
	qdot4 = qdot(4,k);
	qdot5 = qdot(5,k);
	qdot6 = qdot(6,k);
	dqs = [qdot1 qdot2 qdot3 qdot4 qdot5 qdot6];

	M(:,:,k) = mass361(qs);
	C(:,:,k) = pseudocoriolis361(qs,dqs);
	% with Coulomb friction
	if coulomb == 1
		g(:,k) = pseudogravity361(qs,dqs);
	else
	% no Coulomb friction
		g(:,k) = pseudogravity361(qs,[0 0 0 0 0 0]);
	end
end
fprintf('Done generating dynamic model matrices...\n');

