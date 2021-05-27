%% Moment-SOS Based SDP for Constrained Nonlinear Optimization : min_x p(x) s.t g_i(x)>=0  , i=1,...,m
clc;clear all;close all
% SDP solvers
mset clear; warning('off','YALMIP:strict')
mset('yalmip',true);mset(sdpsettings('solver','mosek')); 
%% Optimization Variables: Coeffs of trajectories
% Trajectory 1: x1= a_x1+b_x1*t,  y1= a_y1+b_y1*t, t in [0,1]
% Trajectory 2: x2= a_x2+b_x2*t,  y2= a_y2+b_y2*t, t in [1,2]
% start:(-1,-1), end:(1,1)
mpol a_x1 b_x1 a_y1 b_y1 a_x2 b_x2 a_y2 b_y2

% objective function = int_[0,1] sqrt( (dx1/dt)^2 + (dy1/dt)^2 )2 dt + int_[1,2] sqrt( (dx2/dt)^2 + (dy2/dt)^2 )2 dt
%int_[0 1] (b_x1)^2+(b_y1)^2 dt + int_[2 1] (b_x2)^2+(b_y2)^2 dt
obj= (b_x1)^2+(b_y1)^2 +(b_x2)^2+(b_y2)^2;

% Constraints obtained by main_1_Coeffs_RiskContour_traj1.m code  t\in[0 1]
Cons1_1=(9*a_x1*b_x1^3)/10 - (111*a_y1*b_y1)/500 - (111*a_x1*b_x1)/500 + (9*a_x1^3*b_x1)/5 + (9*a_y1*b_y1^3)/10 + (9*a_y1^3*b_y1)/5 - (111*a_x1^2)/500 + (9*a_x1^4)/10 - (111*a_y1^2)/500 + (9*a_y1^4)/10 - (37*b_x1^2)/500 + (9*b_x1^4)/50 - (37*b_y1^2)/500 + (9*b_y1^4)/50 - (a_x1^2 + a_x1*b_x1 + a_y1^2 + a_y1*b_y1 + b_x1^2/3 + b_y1^2/3 - 37/300)^2 + (9*a_x1^2*a_y1^2)/5 + (9*a_x1^2*b_x1^2)/5 + (3*a_x1^2*b_y1^2)/5 + (3*a_y1^2*b_x1^2)/5 + (9*a_y1^2*b_y1^2)/5 + (9*b_x1^2*b_y1^2)/25 + (9*a_x1*a_y1^2*b_x1)/5 + (9*a_x1^2*a_y1*b_y1)/5 + (9*a_x1*b_x1*b_y1^2)/10 + (9*a_y1*b_x1^2*b_y1)/10 + (12*a_x1*a_y1*b_x1*b_y1)/5 + 7029/500000;
Cons2_1=-a_x1^2 - a_x1*b_x1 - a_y1^2 - a_y1*b_y1 - b_x1^2/3 - b_y1^2/3 + 37/300;
% Constraints obtained by main_1_Coeffs_RiskContour_traj2.m code t\in[1 2]
Cons1_2=(27*a_x2*b_x2^3)/2 - (333*a_y2*b_y2)/500 - (333*a_x2*b_x2)/500 + (27*a_x2^3*b_x2)/5 + (27*a_y2*b_y2^3)/2 + (27*a_y2^3*b_y2)/5 - (111*a_x2^2)/500 + (9*a_x2^4)/10 - (111*a_y2^2)/500 + (9*a_y2^4)/10 - (259*b_x2^2)/500 + (279*b_x2^4)/50 - (259*b_y2^2)/500 + (279*b_y2^4)/50 - (a_x2^2 + 3*a_x2*b_x2 + a_y2^2 + 3*a_y2*b_y2 + (7*b_x2^2)/3 + (7*b_y2^2)/3 - 37/300)^2 + (9*a_x2^2*a_y2^2)/5 + (63*a_x2^2*b_x2^2)/5 + (21*a_x2^2*b_y2^2)/5 + (21*a_y2^2*b_x2^2)/5 + (63*a_y2^2*b_y2^2)/5 + (279*b_x2^2*b_y2^2)/25 + (27*a_x2*a_y2^2*b_x2)/5 + (27*a_x2^2*a_y2*b_y2)/5 + (27*a_x2*b_x2*b_y2^2)/2 + (27*a_y2*b_x2^2*b_y2)/2 + (84*a_x2*a_y2*b_x2*b_y2)/5 + 7029/500000;
Cons2_2=-a_x2^2 - 3*b_x2*a_x2 - a_y2^2 - 3*b_y2*a_y2 - (7*b_x2^2)/3 - (7*b_y2^2)/3 + 37/300;

% Boundary constraints
gb=[a_x1+b_x1*(0)==-1, a_y1+b_y1*(0)==-1];% inital point
gb=[gb, a_x2+b_x2*(2)==1,a_y2+b_y2*(2)==1]% goal point
gb=[gb, a_x1+b_x1*(1)==a_x2+b_x2*(1),a_y1+b_y1*(1)==a_y2+b_y2*(1)];% mid point

% Generate moment-SOS SDP of order 2d 
d=2;% 2*d is relaxation order of SDP
P = msdp(min(obj),[Cons1_1, Cons2_1]<=0,[Cons1_2, Cons2_2]<=0,gb,d);
% Solve Moment SDP
tic
[status,obj] = msol(P);
toc
 %% Results
 % status==1: the moment SDP is solved successfully and Rank conditions are satisfied. Hence, GloptiPoly can extract the global optimal solutions.
 % status==0: the moment SDP is solved successfully But Rank conditions are Not satisfied. Hence, GloptiPoly can NOT extract the global optimal solutions. Increase the relaxation order d.
 % status==-1:  : moment SDP could NOT be solved (unbounded SDP).
if status==1
    % extracted 2 trajectories   
    A_x1=double([a_x1]);
    B_x1=double([b_x1]);
    A_y1=double([a_y1]);
    B_y1=double([b_y1]);

    A_x2=double([a_x2]);
    B_x2=double([b_x2]);
    A_y2=double([a_y2]);
    B_y2=double([b_y2]);
    hold on
    t=[0:0.01:1]; % trajectory 1
    xt1=A_x1(1)+B_x1(1)*t;yt1=A_y1(1)+B_y1(1)*t; plot(xt1,yt1, 'LineWidth',2,'Color','k');
    xt1=A_x1(2)+B_x1(2)*t;yt1=A_y1(2)+B_y1(2)*t; plot(xt1,yt1, 'LineWidth',2,'Color','k');

    t=[1:0.01:2]; % trajectory 2
    xt2=A_x2(1)+B_x2(1)*t;yt2=A_y2(1)+B_y2(1)*t; plot(xt2,yt2,'LineWidth',2,'Color','k');
    xt2=A_x2(2)+B_x2(2)*t;yt2=A_y2(2)+B_y2(2)*t; plot(xt2,yt2,'LineWidth',2,'Color','k');

    plot(-1,-1,'s','LineWidth',2,'MarkerSize',10,'MarkerEdgeColor','k','MarkerFaceColor','b'); 
    plot(1,1,'^','LineWidth',2,'MarkerSize',10,'MarkerEdgeColor','k','MarkerFaceColor','b'); 
    ylim([-1.1 1.1]); xlim([-1.1 1.1])

else
   disp('No solution was extracted' )
end

%% plot static 0.1-Risk Contour: 
Delta=0.1; % risl level
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
contour(x1,x2,eval(Cons_1),[Delta Delta],'r','Linewidth',1,'ShowText','on','DisplayName','(E[g]<0'); hold on
contour(x1,x2,eval(Cons_2),[0 0],'b','Linewidth',1,'ShowText','on','DisplayName','(E[g^2]-E^2[g])/E[g^2]<Delta'); hold on
