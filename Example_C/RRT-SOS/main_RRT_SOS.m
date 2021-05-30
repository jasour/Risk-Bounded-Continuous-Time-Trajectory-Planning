%% Risk bounded continuous-time trajectory planning is dynamic uncertain environment
%% Example B : Risk Bounded Lane Changing for Autonomous Vehicles
clc;display('RRT-SOS')
%%
dm = 2; % dimension of state space
x0 = [0,0]; % start point
xT = [0,4]; % goal point
bd = [-4,0;4,4]; % Bounding box that contain the obstacles and trajectory
num_pieces = 4; % number of trajectory pieces 
delta = 0.1; % Risk level
t = msspoly( 't', 1 );
G = {% static 0.1 Risk contours Eq(12): format constraints >=0
    @(x1,x2) (81*t^2)/25 - (18*t*x1)/5 - (63*t)/25 + x1^2 + (7*x1)/5 + x2^2 - 6*x2 + 28/3,...
    @(x1,x2) ((81*t^2)/25 - (18*t*x1)/5 - (63*t)/25 + x1^2 + (7*x1)/5 + x2^2 - 6*x2 + 28/3)^2 - (1-delta) * ((6561*t^4)/625 - (2916*t^3*x1)/125 - (10206*t^3)/625 + (486*t^2*x1^2)/25 + (3402*t^2*x1)/125 + (162*t^2*x2^2)/25 - (972*t^2*x2)/25 + (41796*t^2)/625 - (36*t*x1^3)/5 - (378*t*x1^2)/25 - (36*t*x1*x2^2)/5 + (216*t*x1*x2)/5 - (9288*t*x1)/125 - (126*t*x2^2)/25 + (756*t*x2)/25 - (29421*t)/625 + x1^4 + (14*x1^3)/5 + 2*x1^2*x2^2 - 12*x1^2*x2 + (516*x1^2)/25 + (14*x1*x2^2)/5 - (84*x1*x2)/5 + (3269*x1)/125 + x2^4 - 12*x2^3 + (164*x2^2)/3 - 112*x2 + 816728/9375),...
    @(x1,x2) 4*t^2 + 4*t*x1 - (16*t)/5 + x1^2 - (8*x1)/5 + x2^2 - 4*x2 + 269/60,...
    @(x1,x2) (4*t^2 + 4*t*x1 - (16*t)/5 + x1^2 - (8*x1)/5 + x2^2 - 4*x2 + 269/60)^2 - (1-delta) * (16*t^4 + 32*t^3*x1 - (128*t^3)/5 + 24*t^2*x1^2 - (192*t^2*x1)/5 + 8*t^2*x2^2 - 32*t^2*x2 + (1154*t^2)/25 + 8*t*x1^3 - (96*t*x1^2)/5 + 8*t*x1*x2^2 - 32*t*x1*x2 + (1154*t*x1)/25 - (32*t*x2^2)/5 + (128*t*x2)/5 - (3592*t)/125 + x1^4 - (16*x1^3)/5 + 2*x1^2*x2^2 - 8*x1^2*x2 + (577*x1^2)/50 - (16*x1*x2^2)/5 + (64*x1*x2)/5 - (1796*x1)/125 + x2^4 - 8*x2^3 + (749*x2^2)/30 - (538*x2)/15 + 1005441/50000),...
    @(x1,x2) t^2 - 2*t*x1 - t + x1^2 + x1 + x2^2 - 2*x2 + 82/75,...
    @(x1,x2) (t^2 - 2*t*x1 - t + x1^2 + x1 + x2^2 - 2*x2 + 82/75)^2 - (1-delta) * (t^4 - 4*t^3*x1 - 2*t^3 + 6*t^2*x1^2 + 6*t^2*x1 + 2*t^2*x2^2 - 4*t^2*x2 + (16*t^2)/5 - 4*t*x1^3 - 6*t*x1^2 - 4*t*x1*x2^2 + 8*t*x1*x2 - (32*t*x1)/5 - 2*t*x2^2 + 4*t*x2 - (11*t)/5 + x1^4 + 2*x1^3 + 2*x1^2*x2^2 - 4*x1^2*x2 + (16*x1^2)/5 + 2*x1*x2^2 - 4*x1*x2 + (11*x1)/5 + x2^4 - 4*x2^3 + (464*x2^2)/75 - (328*x2)/75 + 3746/3125),...
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
tt=[0,0.2,0.4,0.5,0.6,0.7,0.8,0.9,1];% plot time steps
for i=1:9
t=tt(i);
subplot(3,3,fig_num);
hold on;fig_num=fig_num+1;
xlim([-1.5,1.5]); ylim([bd(1,2),bd(2,2)]);

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
