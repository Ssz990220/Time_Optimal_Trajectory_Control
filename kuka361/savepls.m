function savepls(fh, fname, varargin)
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


global savefigs
if savefigs == 1
	screenx = 1272;
	screeny = 665;
	
	if nargin > 2
		opt = varargin{1};
	        if isfield(opt, 'sizex') == 0
	                opt.sizex = 0;
	        end
	        if isfield(opt, 'sizey') == 0
	                opt.sizey = 0;
	        end
	        if isfield(opt, 'adjustsize') == 0
	                opt.adjustsize = 0;
	        end
	        if isfield(opt, 'adjustfonts') == 0
	                opt.adjustfonts = 0;
	        end
	        if isfield(opt, 'axisfont') == 0
	                opt.axisfont = 22;
	        end
	        if isfield(opt, 'titlefont') == 0
	                opt.titlefont = 26;
	        end
		if isfield(opt, 'saveaspectratio') == 0
	                opt.saveaspectratio = 1;
	        end

	else
                opt.sizex = 0;
	        opt.sizey = 0;
	        opt.adjustsize = 0;
	        opt.adjustfonts = 0;
	        opt.axisfont = 22;
	        opt.titlefont = 26;
	        opt.saveaspectratio = 1;
	end

	if opt.sizex == 0
   		opt.sizex = screenx;
	end
	if opt.sizey == 0
   		opt.sizey = screeny;
	end

	if opt.adjustsize == 1
		set(fh, 'Position', [(screenx-opt.sizex)/2+5 (screeny-opt.sizey)/2+51 opt.sizex opt.sizey]);
	end

	if opt.saveaspectratio == 1
		set(gcf,'paperpositionmode','auto');
	end

	if opt.adjustfonts == 1
		set(gcf,'paperpositionmode','auto');
		xs1 = get(get(gca,'xlabel'),'fontsize');
		ys1 = get(get(gca,'ylabel'),'fontsize');
		as1 = get(gca,'fontsize');
		ls1 = get(legend,'fontsize');
		ts1 = get(get(gca,'title'),'fontsize');
		tw1 = get(get(gca,'title'),'fontweight');
	
		axes = findobj(gcf,'type','axes');
		a = findobj(gcf,'type','axes');
		b = findobj(gcf,'tag','legend');
		c = setdiff(a,b);
		for k = 1:length(c)
			set(get(c(k),'xlabel'),'fontsize',opt.axisfont);
			set(get(c(k),'ylabel'),'fontsize',opt.axisfont);
			set(c(k),'fontsize',opt.axisfont);
			set(get(c(k),'title'),'fontsize',opt.titlefont);
			set(get(c(k),'title'),'fontweight','bold');
		end
		for k = 1:length(b)
			set(b(k),'fontsize',opt.axisfont);
		end
	end
	warning off
	figure(fh)
	retval = mkdir('images');
	savefig(['images/' fname]);

	if opt.adjustfonts == 1
		for k = 1:length(c)
			set(get(c(k),'xlabel'),'fontsize',xs1);
			set(get(c(k),'ylabel'),'fontsize',ys1);
			set(c(k),'fontsize',as1);
			set(get(c(k),'title'),'fontsize',ts1);
			set(get(c(k),'title'),'fontweight',tw1);
		end
		for k = 1:length(b)
			set(b(k),'fontsize',ls1);
		end
	end
   warning on
end

