function [xee,yee,zee] = fwkinkuka361(q1,q2,q3,q4,q5,q6)
%***************************************************************************
%  tag: Diederik Verscheure  vr apr 18 11:08:42 CEST 2008  fwkinkuka361.m
%
%                           fwkinkuka361.m -  description
%                           ----------------------------
%    begin                : vr april 18 2008
%    copyright            : (C) 2008 K.U.Leuven
%    email                : diederik <dot> verscheure <at> mech <dot> kuleuven <dot> be
%
% ***************************************************************************
% Purpose
% ---------------------------------------------------------------------------
% Calculates forward kinematics.
% 
% ***************************************************************************

if nargin < 2
	q = q1;
else
	q = [q1;q2;q3;q4;q5;q6];
end

xee = zeros(1,length(q));
yee = zeros(1,length(q));
zee = zeros(1,length(q));

for k = 1:length(q)
        Tee = eeframe361corr(q(:,k)');
        xee(k) = Tee(1,4);
        yee(k) = Tee(2,4);
        zee(k) = Tee(3,4);
end

