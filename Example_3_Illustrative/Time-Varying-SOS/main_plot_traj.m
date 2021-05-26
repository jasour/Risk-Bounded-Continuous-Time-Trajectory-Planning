%% Risk bounded continuous-time trajectory planning is static uncertain environment
%% Illustrative Example 3 : Time-Varying-SOS
clc; clear all; close all

%% start and goal points
x0 = [-1,-1]; xT = [1,1];
plot(x0(1),x0(2),'s','LineWidth',2,'MarkerSize',10,'MarkerEdgeColor','k','MarkerFaceColor','b'); hold on
plot(xT(1),xT(2),'^','LineWidth',2,'MarkerSize',10,'MarkerEdgeColor','k','MarkerFaceColor','b'); 

%% obtained optimal piece-wise linear trajectories via time-varying-SOS
% traj=[px(t),py(t)]
syms t
traj_1=[0.2530144721194292*t - 1.0, 1.096638848429912*t - 1.0];
traj_2=[1.7469855278805708*t - 0.7469855278805708, 0.9033611515700878*t + 0.09663884842991222];

% plots
traj_1 = double(subs(traj_1, t, (0:0.01:1.0)')); plot(traj_1(:,1)', traj_1(:,2)','r','LineWidth',2)
hold on
traj_2 = double(subs(traj_2, t, (0:0.01:1.0)')); plot(traj_2(:,1)', traj_2(:,2)','r','LineWidth',2)

%% plot static 0.1-Risk Contour: 
% intersection of polynomials of Eq(10): outer closed curve
syms x1 x2 w
% obstacle g(x1,x2,w)>=0 where w is probabilistic uncertainty
g=w^2-(x1-0)^2-(x2-0)^2; 
dg=polynomialDegree(g); % max degree of polynomial g
% w: uncertain parameter w~Uniform[l,u]
u=0.4;l=0.3;
m_w=[1];for i=1:2*dg ;m_w(i+1,1)=(1/(u-l))*((u^(i+1) - l^(i+1))/(i+1));end %moments of w
% Calculate the first and Second order moments of new random variable z=g(x1,x2,w)
Mg=[]; %list of first and second order moments of z in Eq(21)
for dd=1:2
% Moment of order dd of z
Md=expand(g^dd);
% replace moments of uncertain parameter w
Md1=subs(Md,flip(w.^[1:dd*dg].'),flip(m_w(2:dd*dg+1))) ; 
Mg=[Mg;Md1];
end
% Inner approximation of Static Delta-risk contour in Eq(10)
Cons_1=(Mg(2)-Mg(1)^2)/Mg(2);
Cons_2=Mg(1);

% plot
[x1,x2]=meshgrid([-2:0.01:2],[-2:0.01:2]);
Delta=0.1;

contour(x1,x2,eval(Cons_1),[Delta Delta],'r','Linewidth',1,'ShowText','on','DisplayName','(E[g]<0'); hold on
contour(x1,x2,eval(Cons_2),[0 0],'b','Linewidth',1,'ShowText','on','DisplayName','(E[g^2]-E^2[g])/E[g^2]<Delta'); hold on
