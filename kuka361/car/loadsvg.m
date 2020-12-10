function [d] = loadsvg(svgfile, res)
%***************************************************************************
%  tag: Diederik Verscheure  za aug 11 11:16:25 CEST 2007  loadsvg.m
%
%                           loadsvg.m -  description
%                           ----------------------------
%    begin                : za augustus 11 2007
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
% Loads svg data.
% 
% ***************************************************************************

fh = fopen(svgfile,'r');
d = [];
t = [0:res:1];

if fh == -1
	fprintf('Error: cannot open file %s...\n',svgfile);
	return
else
	% Regular expression for interesting header lines
	cexpr = '(?<svgblock>[CM]([\s][.,0-9]+)+)';
	numexpr = '(?<svgnums>[.0-9]+)';
	% While not EOF
	fl = fgetl(fh);
	while ischar(fl)
		cidx = regexp(fl, cexpr,'names');
		if length(cidx) > 0
			d = zeros((length(cidx)-1)*length(t),2);
			% We have a matches
			for k = 1:length(cidx)
				svgstr = strrep(cidx(k).svgblock,',',' ');
				bidx = regexp(svgstr,numexpr,'names');
				if k == 1
					p1 = [str2double(bidx(1).svgnums) str2double(bidx(2).svgnums)];
				else
					p2 = [str2double(bidx(1).svgnums) str2double(bidx(2).svgnums)];
					p3 = [str2double(bidx(3).svgnums) str2double(bidx(4).svgnums)];
					p4 = [str2double(bidx(5).svgnums) str2double(bidx(6).svgnums)];
					for l = 1:length(t)
						d((k-2)*length(t) + l, :) = (1-t(l))^3*p1 + 3*t(l)*(1-t(l))^2*p2 + 3*t(l)^2*(1-t(l))*p3 + t(l)^3*p4;
					end
					p1 = p4;
				end
			end 
		end
		fl = fgetl(fh);
	end
	fclose(fh);
end

d(:,1) = -d(:,1);
d(:,2) = -d(:,2);
