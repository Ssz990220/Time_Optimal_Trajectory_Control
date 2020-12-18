function [phi,s_phi,s_mid,sd_mid,sdd_mid,phi_mid,d_phi,R,M_tilde,C_tilde] = CarDynPathFunction(num_grid_points)
% num_grid_points = 2000;
% m=20;
global m;
% [xees,yees,zees,s2]=optecpathcart;
% num_grid_points=20;
[xees, yees, s2]=pathgenerator();


phi = linspace(0,num_grid_points,num_grid_points+1)/(num_grid_points);
factor = 20;
psu_phi = [0,1:num_grid_points*factor]/num_grid_points/factor;
s_phi = spline(s2, [xees;yees],phi);
psu_s_phi = spline(s2,[xees;yees],psu_phi);
delta_s = psu_s_phi(:,(0:num_grid_points-1)*factor+factor/2+2)-psu_s_phi(:,(0:num_grid_points-1)*factor+factor/2+1);
% delta_s = s_phi(:,2:end)-s_phi(:,1:end-1);
angle = atan2(delta_s(2,:),delta_s(1,:));
% figure;
% plot(1:size(angle,2),angle);
% title('angle');
% figure;
% plot(1:size(xees,2),xees,1:size(xees,2),yees);
% title('orignal path');
figure;
plot(1:size(s_phi(1,:),2),s_phi(1,:),1:size(s_phi(1,:),2),s_phi(2,:));
Trajectory_Visualizer([s_phi(:,1:end-1);angle])

    
phi_mid = (phi(1:end-1) + phi(2:end))/2;
% d_phi = phi(2:end) - phi(1:end-1);
d_phi = 1/num_grid_points;

s_mid = (s_phi(:,1:end-1) + s_phi(:,2:end))/2;
sd_mid = (s_phi(:,2:end)-s_phi(:,1:end-1))/d_phi;
% figure;
% plot([1:size(sd_mid,2)],sd_mid(1,:),[1:size(sd_mid,2)],sd_mid(2,:));
% title('sd mid');
sdd_mid = zeros(size(s_mid));
sdd_mid(:,1) = 2*(1/2*s_phi(:,1) - s_phi(:,2) + 1/2*s_phi(:,3))/(d_phi)^2;
sdd_mid(:,2) = (s_phi(:,1)-s_phi(:,2)-s_phi(:,3)+s_phi(:,4))/(2*d_phi^2);
sdd_mid(:,end) = 2*(1/2*s_phi(:,end-2)-s_phi(:,end-1)+1/2*s_phi(:,end))/d_phi^2;
sdd_mid(:,end-1) = (s_phi(:,end-3)-s_phi(:,end-2)-s_phi(:,end-1)+s_phi(:,end))/(2*d_phi^2);
sdd_mid(:,3:end-2) = (-5/48*s_phi(:,1:end-5) + 13/16*s_phi(:,2:end-4) - 17/24*s_phi(:,3:end-3)...
                    -17/24*s_phi(:,4:end-2) + 13/16*s_phi(:,5:end-1) -5/48*s_phi(:,6:end))/(d_phi^2);
% figure;
% plot([1:size(sdd_mid,2)],sdd_mid(1,:),[1:size(sdd_mid,2)],sdd_mid(2,:));
% title('sdd_mid');
s = phi_mid;
q1=xees;
q2=yees;
for m = 1:2
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
				fprintf('%.2f %s completed...\n',((m-1)*length(s) + l)/length(s)*100/2,'%');
			end
		end
	end
end
q = [q1t;q2t];
qp = [qp1t;qp2t];
qpp = [qpp1t;qpp2t];               
% R, M, C are all evaluated at the midpoint of each segment
R = zeros(2,2,size(phi_mid,2));
M = zeros(2,2,size(phi_mid,2));
C = zeros(2,2,size(phi_mid,2));

% M_tilde and C_tilde is modified M and C with phi extracted
M_tilde = zeros(size(M,1),length(phi_mid));
C_tilde = zeros(size(C,1),length(phi_mid));
for k = 1:size(phi_mid,2)
   M(:,:,k) = eye(2)*m;
   R(:,:,k) = [cos(angle(k)),-sin(angle(k)); sin(angle(k)),cos(angle(k))];
   M_tilde(:,k) = M(:,:,k)*qp(:,k);
   C_tilde(:,k) = M(:,:,k)*qpp(:,k) + C(:,:,k)*qp(:,k).^2;
end
% figure;
% plot(1:size(M_tilde(1,:),2),M_tilde(1,:),1:size(M_tilde(1,:),2),M_tilde(2,:));
% title('M_{tilde}');
% figure;
% plot(C_tilde(1,:),C_tilde(2,:));
% title('C_{tilde}');
end

