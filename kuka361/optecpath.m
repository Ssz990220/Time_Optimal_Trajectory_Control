function [q,qp,qpp]=optecpath(s)
%***************************************************************************
%  tag: Diederik Verscheure  wo jul  4 10:13:23 CEST 2007  optecpath.m
%
%                           optecpath.m -  description
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
% Generates q, q', q'' for the optec path.
% 
% ***************************************************************************

[xees,yees,zees,s2]=optecpathcart;
% s2是追随轨迹的坐标
% s是生成轨迹的坐标
% 两者的分辨率可能不一致，需要进行缩放

fprintf('Generating joint-space trajectory (this may take around 15 seconds)...\n');

[q1,q2,q3,q4,q5,q6] = invkinkuka361(xees,yees,zees);

for m = 1:6
	eval(sprintf('q%dt = zeros(size(s));',m));
	eval(sprintf('qp%dt = zeros(size(s));',m));
	eval(sprintf('qpp%dt = zeros(size(s));',m));
	
	l = 1;
	for k = 1:(length(s2)-1) % 遍历追随轨迹上的每一个点
		wlength = 6;
		done = 0;
		while done == 0 % 找到该点附近是否可以用二次函数形式表示
			eval(sprintf('B = q%d(1,max(1,k-floor(wlength/2)):min(k+floor(wlength/2),length(s2)))'';',m));
            % max和min是为了防止开头和结尾发生索引溢出问题
			sk = s2(1,max(1,k-floor(wlength/2)):min(k+floor(wlength/2),length(s2)))';
			A = [sk.^2 sk ones(size(sk))];
			x = mldivide(A,B);
			if sqrt(mean((A*x-B).^2)) < 1.5e-3 % 终止阈值
				wlength = wlength+2;
			else
				done = 1;
			end
		end
		while ((l <= length(s)) & (s(l) <= s2(k+1)))% 对应填充，但保证s(l) <= s2(k+1)既仅填充对应端上的s列表
			eval(sprintf('q%dt(l) = [s(l)^2 s(l) 1]*x;',m));
			eval(sprintf('qp%dt(l) = [2*s(l) 1]*x(1:2);',m));
			eval(sprintf('qpp%dt(l) = 2*x(1);',m));
			l = l + 1;
			if mod(floor(l/length(s)*100),20) == 0
				clc	
				fprintf('Generating joint-space trajectory (this may take around 15 seconds)...\n');
				fprintf('%.2f %s completed...\n',((m-1)*length(s) + l)/length(s)*100/6,'%');
			end
		end
	end
end
q = [q1t;q2t;q3t;q4t;q5t;q6t];
qp = [qp1t;qp2t;qp3t;qp4t;qp5t;qp6t];
qpp = [qpp1t;qpp2t;qpp3t;qpp4t;qpp5t;qpp6t];
fprintf('Done generating joint-space trajectory...\n');

