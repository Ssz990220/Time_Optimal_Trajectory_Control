function Trajectory_Visualizer(Traj_points)
%TRAJECTORY_VISUALIZER This is a 2D visualizer for paper reproduction
%   Traj_points is a n by 3 matrix with each row indicate a point on the
%   trajectory in form [x y theta]
    end_point1 = [cos(Traj_points(:,3)) sin(Traj_points(:,3))];
    end_point2 = [-sin(Traj_points(:,3)) cos(Traj_points(:,3))];
    figure
    hold on
    plot(Traj_points(:,1),Traj_points(:,2))
    quiver(Traj_points(:,1),Traj_points(:,2),end_point1(:,1),end_point1(:,2),0);
    quiver(Traj_points(:,1),Traj_points(:,2),end_point2(:,1),end_point2(:,2),0);
    

end

