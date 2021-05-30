%% Risk bounded continuous-time trajectory planning is dynamic uncertain environment
%% Example B : Risk Bounded Lane Changing for Autonomous Vehicles
%% Time-Varying-SOS
clc; clear all; 

%% start and goal points
x0 = [0,0]; xT = [0,4];
plot(x0(1),x0(2),'s','LineWidth',2,'MarkerSize',10,'MarkerEdgeColor','k','MarkerFaceColor','b'); hold on
plot(xT(1),xT(2),'^','LineWidth',2,'MarkerSize',10,'MarkerEdgeColor','k','MarkerFaceColor','b'); 

%% obtained optimal piece-wise linear trajectories via time-varying-SOS
% traj=[px(t),py(t)]
syms t
 traj_1 =  [0.872658184940773*t, 1.0052099790572706*t];
 traj_2 =  [-0.029794126562200565*t + 0.872658184940773, 0.3154786569826575*t + 1.0052099790572706];
 traj_3 =  [0.07692804550445606*t + 0.84286405847459, 0.2865019757468874*t + 1.3206886360076144];
 traj_4 =  [-1.5583679586709585*t + 0.9197921040414069, 1.6749044593517735*t + 1.6071906117125512];
 traj_5 =  [0.6385758573829112*t - 0.6385758573829112, 0.7179048887945401*t + 3.28209511120546];

%reparameterization: t in [0,1] to t in [0,0.5] or to t in [0.5,1]
traj_1 = subs(traj_1, t, (t-0)/(1/5-0)); 
traj_2 = subs(traj_2, t, (t-1/5)/(2/5-1/5));
traj_3 = subs(traj_3, t, (t-2/5)/(3/5-2/5));
traj_4 = subs(traj_4, t, (t-3/5)/(4/5-3/5));
traj_5 = subs(traj_5, t, (t-4/5)/(1-4/5));

% full trajectory
traj=[double(subs(traj_1, t, [0:0.1:0.2]'));
     double(subs(traj_2, t, [0.2:0.1:0.4]'));
     double(subs(traj_3, t, [0.4:0.1:0.6]'));
     double(subs(traj_4, t, [0.6:0.1:0.8]'));
     double(subs(traj_5, t, [0.8:0.1:1]'))];

%% Plots
fig_num = 1;k=1;
tt=[0:0.2:0.4, 0.5:0.1:1];% plot time steps
traj_t=[double(subs(traj_1, t, [0]')) ; 
        double(subs(traj_2, t, [0.2]'))
        double(subs(traj_3, t, [0.4,0.5]'))
        double(subs(traj_4, t, [0.6,0.7]'))
        double(subs(traj_5, t, [0.8,0.9,1]'))];
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