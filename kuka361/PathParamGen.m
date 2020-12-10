function [s, s_phi, d_phi, s_phi, sd_phi, sdd_phi, Mv, Cv, gv] = PathParamGen(invdynfun,pathfun, num_grid_points, varargin)
%PATHPARAMGEN Summary of this function goes here
%   Detailed explanation goes here
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

[s_phi,s_mid,sd_mid,sdd_mid,phi_mid,d_phi] = PathFunction(num_grid_points);

[Mm,Cm,gm] = feval(invdynfun,s_mid,sd_mid,varargin{:});
Mv = zeros(size(Mm,1),length(phi_mid));
Cv = zeros(size(Cm,1),length(phi_mid));
for k = 1:length(sm)
	Mv(:,k) = Mm(:,:,k)*sd_mid(:,k);
	Cv(:,k) = Mm(:,:,k)*sdd_mid(:,k) + Cm(:,:,k)*sd_mid(:,k);
end
gv = gm;
end

