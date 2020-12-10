%***************************************************************************
%  tag: Diederik Verscheure  vr aug 31 22:04:44 CEST 2007  makekuka361.m
%
%                           makekuka361.m -  description
%                           ----------------------------
%    begin                : vr augustus 31 2007
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
% Compiles files for the KUKA 361 test problem.
% 
% ***************************************************************************

clear all;
close all;
global savefigs;
savefigs = 0;

mex src/mass361.c
mex src/pseudogravity361.c
mex src/pseudocoriolis361.c
mex src/invkin361.c
mex src/eeframe361.c

