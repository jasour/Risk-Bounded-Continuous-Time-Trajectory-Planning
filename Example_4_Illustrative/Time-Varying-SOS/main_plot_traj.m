%% Risk bounded continuous-time trajectory planning is dynamic uncertain environment
%% Illustrative Example 4 : Time-Varying-SOS
clc; clear all; close all

%% start and goal points
x0 = [1,-2]; xT = [3,2];
plot(x0(1),x0(2),'s','LineWidth',2,'MarkerSize',10,'MarkerEdgeColor','k','MarkerFaceColor','b'); hold on
plot(xT(1),xT(2),'^','LineWidth',2,'MarkerSize',10,'MarkerEdgeColor','k','MarkerFaceColor','b'); 

%% obtained optimal piece-wise linear trajectories via time-varying-SOS
% traj=[px(t),py(t)]
syms t
traj_1=[-0.9774753154069917*t + 1.0, 0.9998482257485701*t - 2.0];
traj_2=[2.9774753154069917*t + 0.02252468459300827, 3.00015177425143*t - 1.0001517742514299];


%reparameterization: t in [0,1] to t in [0,0.5] or to t in [0.5,1]
traj_1 = subs(traj_1, t, (t-0)/(0.5-0)); traj_2 = subs(traj_2, t, (t-0.5)/(1-0.5));

% trajectory: piece 1
traj_1 = double(subs(traj_1, t, (0:0.1:0.5)'));
% trajectory: piece 2
traj_2 = double(subs(traj_2, t, (0.5:0.1:1)'));
% full trajectory
traj=[traj_1(1:end,:);traj_2(2:end,:)];


%% plot static 0.1-Risk Contour: 
syms x1 x2 w1 w2 w3 t
% probabilistic obstacle
px=2-t+t^2+0.2*w2; % uncertain trajectory px
py=-1+4*t-t^2+0.1*w3; % uncertain trajectory py
g=w1^2-(x1-px)^2-(x2-py)^2; % obstacle g(x1,x2,w1)>=0
dg=polynomialDegree(g); % max degree of polynomial g

% w1: uncertain parameter w~Uniform[l,u]
u=0.4;l=0.3; m_w1=[1];for i=1:2*dg ;m_w1(i+1,1)=(1/(u-l))*((u^(i+1) - l^(i+1))/(i+1));end % moments of w1

% w2: normal distribution on [mean,var] : 
mean=0; var=0.01; for k=0:2*dg; m_w2(k+1,1)=sqrt(var)^k*(-j*sqrt(2))^k*kummerU(-k/2, 1/2,-1/2*mean^2/var);end % moments of w2

% w3: beta distribution [a,b] 
a=3;b=3; m_w3=[1];for k=1:2*dg; m_w3=[m_w3;(a+k-1)/(a+b+k-1)*m_w3(end) ]; end; % moments of w3

% Calculate the first and Second order moments of new random variable z=g(x1,x2,w)
Mg=[]; %list of first and second order moments of z in Eq(21)
for dd=1:2
% Moment of order dd of z
Md=expand(g^dd);
% replace moments of uncertain parameter w1
Md1=subs(Md,flip(w1.^[1:dd*dg].'),flip(m_w1(2:dd*dg+1))) ; 
% replace moments of uncertain parameter w2
Md2=subs(Md1,flip(w2.^[1:dd*dg].'),flip(m_w2(2:dd*dg+1))) ; 
% replace moments of uncertain parameter w3
Md3=subs(Md2,flip(w3.^[1:dd*dg].'),flip(m_w3(2:dd*dg+1))) ; 
Mg=[Mg;Md3];
end

% Inner approximation of Dynamic Delta-risk contour in Eq(12)
Cons_1=(Mg(2)-Mg(1)^2)/Mg(2);
Cons_2=Mg(1);
%% Plots
fig_num = 1;k=1;
for t=0:0.2:1

subplot(2,3,fig_num);
axis square;hold on;fig_num=fig_num+1;

% expected value of uncertain trajectory
[x1,x2]=meshgrid([0.8:0.005:3],[-2:0.005:3]);
tt=[0:0.01:1];px=2-tt+tt.^2;py=-1+4*tt-tt.^2;plot(px,py,'k--','Linewidth',3)
tt=0;px=2-tt+tt.^2;py=-1+4*tt-tt.^2;
plot(px,py,'s','LineWidth',2,'MarkerSize',10,'MarkerEdgeColor','k','MarkerFaceColor','r'); 
tt=1;px=2-tt+tt.^2;py=-1+4*tt-tt.^2;
plot(px,py,'^','LineWidth',2,'MarkerSize',10,'MarkerEdgeColor','k','MarkerFaceColor','r'); 

% initial and goal points
plot(x0(1),x0(2),'s','LineWidth',2,'MarkerSize',10,'MarkerEdgeColor','k','MarkerFaceColor','b'); 
plot(xT(1),xT(2),'^','LineWidth',2,'MarkerSize',10,'MarkerEdgeColor','k','MarkerFaceColor','b'); 

% Risk contour, outside of the outer closed curve
Delta=0.1;[C1,h1]=contour(x1,x2,eval(Cons_1),[Delta Delta],'r','ShowText','on','Linewidth',3);

% trajectory
plot(traj_1(:,1)', traj_1(:,2)','b','LineWidth',2);plot(traj_2(:,1)', traj_2(:,2)','b','LineWidth',2)

% trajectory at time t
plot(traj(k,1), traj(k,2),'o','LineWidth',2,'MarkerSize',10,'MarkerEdgeColor','k','MarkerFaceColor','r')
k=k+2;

end
