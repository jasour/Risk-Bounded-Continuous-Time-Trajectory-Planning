%% Moment-SOS Based SDP for Constrained Nonlinear Optimization : min_x p(x) s.t g_i(x)>=0  , i=1,...,m
clc;clear all;close all
% SDP solvers
mset clear; warning('off','YALMIP:strict')
mset('yalmip',true);mset(sdpsettings('solver','mosek')); 
%% Optimization Variables: Coeffs of trajectories
% Trajectory 1: px1= a_x1+b_x1*t+c_x1*t^2,  py1= a_y1+b_y1*t+c_y1*t^2, t in [0,1]
% start:(-1,1), end:(1,1)
mpol a_x1 b_x1 c_x1 a_y1 b_y1 c_y1

% objective function = int_[0,1] sqrt( (dx/dt)^2 + (dy/dt)^2 )2 dt
%int_[0 1] (b_x1+2*c_x1*t)^2+ (b_y1+2*c_y1*t)^2 dt
obj= (b_x1^2+b_y1^2)+0.5*(4*b_x1*c_x1+4*b_y1*c_y1)+(1/3)*(4*c_x1^2+4*c_y1^2);

% Constraints obtained by main_1_Coeffs_RiskContour.m code
Cons_1=(9*a_x1*b_x1^3)/10 - (57*a_y1*b_y1)/2000 - (19*a_x1*c_x1)/1000 - (19*a_y1*c_y1)/1000 - (57*b_x1*c_x1)/4000 - (57*b_y1*c_y1)/4000 - (57*a_x1*b_x1)/2000 + (9*a_x1^3*b_x1)/5 + (9*a_y1*b_y1^3)/10 + (9*a_y1^3*b_y1)/5 + (18*a_x1*c_x1^3)/35 + (6*a_x1^3*c_x1)/5 + (18*a_y1*c_y1^3)/35 + (6*a_y1^3*c_y1)/5 + (9*b_x1*c_x1^3)/20 + (3*b_x1^3*c_x1)/5 + (9*b_y1*c_y1^3)/20 + (3*b_y1^3*c_y1)/5 - (a_x1^2 + a_x1*b_x1 + (2*a_x1*c_x1)/3 + a_y1^2 + a_y1*b_y1 + (2*a_y1*c_y1)/3 + b_x1^2/3 + (b_x1*c_x1)/2 + b_y1^2/3 + (b_y1*c_y1)/2 + c_x1^2/5 + c_y1^2/5 - 19/1200)^2 - (57*a_x1^2)/2000 + (9*a_x1^4)/10 - (57*a_y1^2)/2000 + (9*a_y1^4)/10 - (19*b_x1^2)/2000 + (9*b_x1^4)/50 - (19*b_y1^2)/2000 + (9*b_y1^4)/50 - (57*c_x1^2)/10000 + c_x1^4/10 - (57*c_y1^2)/10000 + c_y1^4/10 + (9*a_x1^2*a_y1^2)/5 + (9*a_x1^2*b_x1^2)/5 + (3*a_x1^2*b_y1^2)/5 + (3*a_y1^2*b_x1^2)/5 + (9*a_y1^2*b_y1^2)/5 + (27*a_x1^2*c_x1^2)/25 + (9*a_x1^2*c_y1^2)/25 + (9*a_y1^2*c_x1^2)/25 + (9*b_x1^2*b_y1^2)/25 + (27*a_y1^2*c_y1^2)/25 + (27*b_x1^2*c_x1^2)/35 + (9*b_x1^2*c_y1^2)/35 + (9*b_y1^2*c_x1^2)/35 + (27*b_y1^2*c_y1^2)/35 + (c_x1^2*c_y1^2)/5 + (9*a_x1*a_y1^2*b_x1)/5 + (9*a_x1^2*a_y1*b_y1)/5 + (9*a_x1*b_x1*b_y1^2)/10 + (6*a_x1*a_y1^2*c_x1)/5 + (9*a_y1*b_x1^2*b_y1)/10 + (6*a_x1^2*a_y1*c_y1)/5 + (9*a_x1*b_x1*c_x1^2)/5 + (54*a_x1*b_x1^2*c_x1)/25 + (27*a_x1^2*b_x1*c_x1)/10 + (3*a_x1*b_x1*c_y1^2)/5 + (18*a_x1*b_y1^2*c_x1)/25 + (9*a_y1^2*b_x1*c_x1)/10 + (3*a_y1*b_y1*c_x1^2)/5 + (18*a_y1*b_x1^2*c_y1)/25 + (9*a_x1^2*b_y1*c_y1)/10 + (9*a_y1*b_y1*c_y1^2)/5 + (54*a_y1*b_y1^2*c_y1)/25 + (27*a_y1^2*b_y1*c_y1)/10 + (18*a_x1*c_x1*c_y1^2)/35 + (3*b_x1*b_y1^2*c_x1)/5 + (18*a_y1*c_x1^2*c_y1)/35 + (3*b_x1^2*b_y1*c_y1)/5 + (9*b_x1*c_x1*c_y1^2)/20 + (9*b_y1*c_x1^2*c_y1)/20 + (12*a_x1*a_y1*b_x1*b_y1)/5 + (9*a_x1*a_y1*b_x1*c_y1)/5 + (9*a_x1*a_y1*b_y1*c_x1)/5 + (36*a_x1*a_y1*c_x1*c_y1)/25 + (36*a_x1*b_x1*b_y1*c_y1)/25 + (36*a_y1*b_x1*b_y1*c_x1)/25 + (6*a_x1*b_y1*c_x1*c_y1)/5 + (6*a_y1*b_x1*c_x1*c_y1)/5 + (36*b_x1*b_y1*c_x1*c_y1)/35 + 21893979372484023/92233720368547758080;
Cons_2=-a_x1^2 - a_x1*b_x1 - (2*a_x1*c_x1)/3 - a_y1^2 - a_y1*b_y1-(2*a_y1*c_y1)/3 - b_x1^2/3 - (b_x1*c_x1)/2 - b_y1^2/3 - (b_y1*c_y1)/2 - c_x1^2/5 - c_y1^2/5 + 19/1200;

% Boundary constraints
gb=[a_x1+b_x1*(0)+c_x1*(0)^2==-1, a_y1+b_y1*(0)+c_y1*(0)^2==-1];% inital point
gb=[gb, a_x1+b_x1*(1)+c_x1*(1)^2==1, a_y1+b_y1*(1)+c_y1*(1)^2==1]% goal point

% Generate moment-SOS SDP of order 2d 
d=2;% 2*d is relaxation order of SDP
P = msdp(min(obj),[Cons_1, Cons_2]<=0,gb,d);
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
    A_x1=double([a_x1]); B_x1=double([b_x1]); C_x1=double([c_x1]);
    A_y1=double([a_y1]); B_y1=double([b_y1]); C_y1=double([c_y1]);
    hold on
    t=[0:0.01:1]; 
    % extracted trajectory 1
    xt1=A_x1(1)+B_x1(1)*t+C_x1(1)*t.^2;yt1=A_y1(1)+B_y1(1)*t+C_y1(1)*t.^2; 
    plot(xt1,yt1,'LineWidth',2,'Color','k');
    % extracted trajectory 2
    xt1=A_x1(2)+B_x1(2)*t+C_x1(2)*t.^2;yt1=A_y1(2)+B_y1(2)*t+C_y1(2)*t.^2; 
    plot(xt1,yt1,'LineWidth',2,'Color','k');
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
