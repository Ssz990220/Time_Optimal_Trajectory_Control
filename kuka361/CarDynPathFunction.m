function [phi,s_phi,s_mid,sd_mid,sdd_mid,phi_mid,d_phi,R,M,C] = CarDynPathFunction(num_grid_points, m)
num_grid_points = 2000;
m=20;
[xees,yees,zees,s2]=optecpathcart;

phi = linspace(0,num_grid_points,num_grid_points+1)/(num_grid_points);

s_phi = spline(s2, [xees;yees],phi);

delta_s = s_phi(:,2:end)-s_phi(:,1:end-1);
angle = atan2(delta_s(2,:),delta_s(1,:));


    
phi_mid = (phi(1:end-1) + phi(2:end))/2;
d_phi = phi(2:end) - phi(1:end-1);

s_mid = (s_phi(:,1:end-1) + s_phi(:,2:end))/2;
sd_mid = (s_phi(:,2:end)-s_phi(:,1:end-1))./d_phi;

sdd_mid = zeros(size(s_mid));
sdd_mid(:,1) = (1/2*s_phi(:,1) - s_phi(:,2) + 1/2*s_phi(:,3))/(d_phi(1))^2;
sdd_mid(:,2) = (s_phi(:,1)-s_phi(:,2)-s_phi(:,3)+s_phi(:,4))/(d_phi(2)^2);
sdd_mid(:,end) = (1/2*s_phi(:,end-2)-s_phi(:,end-1)+1/2*s_phi(:,end))/d_phi(end)^2;
sdd_mid(:,end-1) = (s_phi(:,end-3)-s_phi(:,end-2)-s_phi(:,end-1)+s_phi(:,end))/(d_phi(end-1)^2);
sdd_mid(:,3:end-2) = (-5/48*s_phi(:,1:end-5) + 13/16*s_phi(:,2:end-4) - 17/24*s_phi(:,3:end-3)...
                    -17/24*s_phi(:,4:end-2) + 13/16*s_phi(:,5:end-1) -5/48*s_phi(:,6:end));
                
R = zeros(2,2,size(phi_mid,2));
M = zeros(2,2,size(phi_mid,2));
C = zeros(2,2,size(s_phi,2));
for k = 1:size(phi_mid,2)
   M(:,:,k) = eye(2)*m;
   R(:,:,k) = [cos(angle(k)),-sin(angle(k)); sin(angle(k)),cos(angle(k))];
end
R(:,:,end) = R(:,:,end-1);
M(:,:,end) = eye(2)*m;

end

