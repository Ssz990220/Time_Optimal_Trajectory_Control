function Tinv=inversT(T)
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

Rinv = T(1:3,1:3)';

Tinv = zeros(4,4);
Tinv(1:3,1:3) = Rinv;
Tinv(1:3,4) = -Rinv*T(1:3,4);
Tinv(4,4) = 1;
