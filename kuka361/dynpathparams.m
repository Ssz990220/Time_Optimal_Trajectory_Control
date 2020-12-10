function [s, sm, ds, q, qp, qpp, Mv, Cv, gv] = dynpathparams(invdynfun, pathfun, sgrid, varargin);
%***************************************************************************
%  tag: Diederik Verscheure  di jun 12 15:28:22 CEST 2007  dynpathparams.m
%
%                           dynpathparams.m -  description
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
% Loads parameters for a time-optimal path-constrained optimal control problem.
% 
% ***************************************************************************
s = [];

% Check if functions exist
if exist(func2str(invdynfun)) ~= 2
	fprintf('Error: function %s does not exist...\n', func2str(invdynfun));
	return
end
if exist(func2str(pathfun)) ~= 2
	fprintf('Error: function %s does not exist...\n', func2str(pathfun));
	return
end

% Check sgrid
if ((length(sgrid) > 1) & (sgrid<=1) & (sgrid>=0))
	fprintf('Error: illegal sgrid specification...\n');
	return
elseif (length(sgrid) > 1)
	s = sgrid;
elseif ((length(sgrid) == 1) & (sgrid > 0))
	s = linspace(0,1,sgrid);
else
	fprintf('Error: illegal sgrid specification...\n');
	return
end

% s midpoints
sm = (s(1:end-1) + s(2:end))/2;
ds = s(2:end)-s(1:end-1);

% Evaluate path and dynamic model
% -------------------------------
% Evaluate q, q' and q'' at the midpoints!
[q,qp,qpp] = feval(pathfun,sm);

% Evaluate the dynamic model
[Mm,Cm,gm] = feval(invdynfun,q,qp,varargin{:});
Mv = zeros(size(Mm,1),length(sm));
Cv = zeros(size(Cm,1),length(sm));
for k = 1:length(sm)
	Mv(:,k) = Mm(:,:,k)*qp(:,k);
	Cv(:,k) = Mm(:,:,k)*qpp(:,k) + Cm(:,:,k)*qp(:,k);
end
gv = gm;

