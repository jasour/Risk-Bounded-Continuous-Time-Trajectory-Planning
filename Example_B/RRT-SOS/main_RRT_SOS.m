%% Risk bounded continuous-time trajectory planning is dynamic uncertain environment
%% Example B : Risk Bounded Lane Changing for Autonomous Vehicles
clc;display('RRT-SOS')
%%
dm = 2; % dimension of state space
x0 = [0,0]; % start point
xT = [2,1]; % goal point
bd = [0,0;2,1]; % Bounding box that contain the obstacles and trajectory
num_pieces = 3; % number of trajectory pieces 
delta = 0.1; % Risk level
t = msspoly( 't', 1 );
G = {% static 0.1 Risk contours Eq(12): format constraints >=0
    @(x1,x2) t^2 - 2*t*x1 + (4*t)/5 + x1^2 - (4*x1)/5 + x2^2 - 2*x2 + 161/150,...
    @(x1,x2) (t^2 - 2*t*x1 + (4*t)/5 + x1^2 - (4*x1)/5 + x2^2 - 2*x2 + 161/150)^2 - (1-delta) * (t^4 - 4*t^3*x1 + (8*t^3)/5 + 6*t^2*x1^2 - (24*t^2*x1)/5 + 2*t^2*x2^2 - 4*t^2*x2 + (14*t^2)/5 - 4*t*x1^3 + (24*t*x1^2)/5 - 4*t*x1*x2^2 + 8*t*x1*x2 - (28*t*x1)/5 + (8*t*x2^2)/5 - (16*t*x2)/5 + (216*t)/125 + x1^4 - (8*x1^3)/5 + 2*x1^2*x2^2 - 4*x1^2*x2 + (14*x1^2)/5 - (8*x1*x2^2)/5 + (16*x1*x2)/5 - (216*x1)/125 + x2^4 - 4*x2^3 + (461*x2^2)/75 - (322*x2)/75 + 21641/18750),...
    @(x1,x2) 4*t^2 - 4*t*x1 + (12*t)/5 + x1^2 - (6*x1)/5 + x2^2 + 41/150,...
    @(x1,x2) (4*t^2 - 4*t*x1 + (12*t)/5 + x1^2 - (6*x1)/5 + x2^2 + 41/150)^2 - (1-delta) * (16*t^4 - 32*t^3*x1 + (96*t^3)/5 + 24*t^2*x1^2 - (144*t^2*x1)/5 + 8*t^2*x2^2 + 8*t^2 - 8*t*x1^3 + (72*t*x1^2)/5 - 8*t*x1*x2^2 - 8*t*x1 + (24*t*x2^2)/5 + (168*t)/125 + x1^4 - (12*x1^3)/5 + 2*x1^2*x2^2 + 2*x1^2 - (12*x1*x2^2)/5 - (84*x1)/125 + x2^4 + (41*x2^2)/75 + 497/6250),...
};% constraints >=0

% calculate rrt path
[V, E, path, path_points] = RRT_SOS_Dynamic(dm, G, x0, xT, bd, t, num_pieces);

% Obtained RRT path
traj=[];
for i=1:num_pieces
  traj=[traj;  path_points(i,1:2) + (t-(i-1)/num_pieces)*num_pieces*(path_points(i,3:4) -path_points(i,1:2))];
end
       
%plot RRT tree
% for k=2:num_pieces+1
% for i=1:size(E{k},1)
%     e = E{k}(i,:);
%     a = V{k-1}(int64(e(1)),:);
%     b = V{k}(int64(e(2)),:);
%     plot([a(1),b(1)],[a(2),b(2)],'b');hold on
% end
% end

%% Plot RRT Trajectory
fig_num = 1;k=1;
tt=[0:0.1:0.4, 0.6, 0.8:0.1:1];% plot time steps
for i=1:9
t=tt(i);
subplot(3,3,fig_num);
hold on;fig_num=fig_num+1;

% initial and goal points
plot(x0(1),x0(2),'s','LineWidth',2,'MarkerSize',10,'MarkerEdgeColor','k','MarkerFaceColor','b'); 
plot(xT(1),xT(2),'^','LineWidth',2,'MarkerSize',10,'MarkerEdgeColor','k','MarkerFaceColor','b'); 


% plot trajectory
for i=1:num_pieces
    plot([path_points(i,1), path_points(i,3)], [path_points(i,2), path_points(i,4)],'-b','LineWidth',2);
end

% plot trajectroy at time t
for i=1:num_pieces
    if t >= (i-1)/num_pieces && t <= i/num_pieces
        cur_x = path_points(i,1:2) + (t-(i-1)/num_pieces)*num_pieces*(path_points(i,3:4) -path_points(i,1:2));
        plot(cur_x(1),cur_x(2),'o','LineWidth',2,'MarkerSize',5,'MarkerEdgeColor','k','MarkerFaceColor','r'); 
    end
end

end
