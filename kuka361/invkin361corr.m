function q = invkin361(Tee,confr)
%***************************************************************************
%  tag: Diederik Verscheure  zo aug  5 19:56:56 CEST 2007  invkin361corr.m
%
%                           invkin361corr.m -  description
%                           ----------------------------
%    begin                : zo augustus 05 2007
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
% Corrected version of invkin361.
% 
% ***************************************************************************

% New to old frame
Tno = [0  1  0  0;
       -1 0  0  0;
       0  0  1  0;
       0  0  0  1];
q = invkin361(inversT(Tno)*Tee*Tno,confr);


