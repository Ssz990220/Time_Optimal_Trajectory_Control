function setfigtempl(varargin)

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

if nargin == 1
	linewidth = varargin{1};
else
	linewidth = 1.5;
end

set(gca, 'Fontsize', 12);
set(get(gca, 'Title'), 'Fontsize', 12, 'Fontweight', 'bold');
set(get(gca, 'xlabel'), 'Fontsize', 12);
set(get(gca, 'ylabel'), 'Fontsize', 12);

grid on;
axis tight;
lineobj = findobj('type','line');
set(lineobj, 'linewidth', linewidth);
%set(lineobj, 'markersize', 10);

