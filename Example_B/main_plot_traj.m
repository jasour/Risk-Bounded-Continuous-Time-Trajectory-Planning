%% Risk bounded continuous-time trajectory planning is dynamic uncertain environment
%% Example B : Risk Bounded Lane Changing for Autonomous Vehicles
%% Time-Varying-SOS
clc; clear all; 

%% start and goal points
x0 = [0,0]; xT = [2,1];
plot(x0(1),x0(2),'s','LineWidth',2,'MarkerSize',10,'MarkerEdgeColor','k','MarkerFaceColor','b'); hold on
plot(xT(1),xT(2),'^','LineWidth',2,'MarkerSize',10,'MarkerEdgeColor','k','MarkerFaceColor','b'); 

%% obtained optimal piece-wise linear trajectories via time-varying-SOS
% traj=[px(t),py(t)]
syms t
traj_1 =[0.7720002773799888*t, 0.025326757967887636*t];
traj_2 =[1.2279997226200112*t + 0.7720002773799888, 0.9746732420321124*t + 0.025326757967887636];

%reparameterization: t in [0,1] to t in [0,0.5] or to t in [0.5,1]
traj_1 = subs(traj_1, t, (t-0)/(0.5-0)); traj_2 = subs(traj_2, t, (t-0.5)/(1-0.5));

% full trajectory
traj=[double(subs(traj_1, t, [0:0.1:0.5]'));
     double(subs(traj_2, t, [0.5:0.1:1]'))];

%% Plots
fig_num = 1;k=1;
tt=[0:0.1:0.4, 0.6, 0.8:0.1:1];% plot time steps
traj_t=[double(subs(traj_1, t, [0:0.1:0.4]')) ; double(subs(traj_2, t, [0.6,0.8:0.1:1]'))];
for i=1:9
t=tt(i);
subplot(3,3,fig_num);
hold on;fig_num=fig_num+1;

% initial and goal points
plot(x0(1),x0(2),'s','LineWidth',2,'MarkerSize',10,'MarkerEdgeColor','k','MarkerFaceColor','b'); 
plot(xT(1),xT(2),'^','LineWidth',2,'MarkerSize',10,'MarkerEdgeColor','k','MarkerFaceColor','b'); 

% full trajectory
plot(traj(:,1)', traj(:,2)','g','LineWidth',2)

% trajectory at time t
plot(traj_t(k,1), traj_t(k,2),'o','LineWidth',2,'MarkerSize',5,'MarkerEdgeColor','k','MarkerFaceColor','r')
k=k+1;

end