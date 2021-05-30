%% Dynamic Delta-risk contour = {x: Prob(x \in Obs(t)) <= Delta}  Eq(11)
%% Inner approximation in Eq(12) 
%% Example B: Risk Bounded Lane Changing for Autonomous Vehicles
clc; clear all; close all

%% Uncertain Obstacle 1: Eq(5)
syms x1 x2 w t
% probabilistic obstacle
px=t-0.5+w; % uncertain trajectory px
g=0.4^2-(-x1+px)^2-(x2-1)^2; % obstacle g(x1,x2,w1)>=0
dg=polynomialDegree(g); % max degree of polynomial g


% w: uncertain parameter w~Uniform[l,u]
u=0.1;l=-0.1; m_w1=[1];for i=1:2*dg ;m_w1(i+1,1)=(1/(u-l))*((u^(i+1) - l^(i+1))/(i+1));end % moments of w1

% Calculate the first and Second order moments of new random variable z=g(x1,x2,w)
Mg=[]; %list of first and second order moments of z in Eq(21)
for dd=1:2
% Moment of order dd of z
Md=expand(g^dd);
% replace moments of uncertain parameter w
Md1=subs(Md,flip(w.^[1:dd*dg].'),flip(m_w1(2:dd*dg+1))) ; 
Mg=[Mg;Md1];
end

% Inner approximation of Dynamic Delta-risk contour in Eq(12)

Cons_1=(Mg(2)-Mg(1)^2)/Mg(2);
Cons_2=Mg(1);

clc;display('Done!');display('Working on plots')

%% Plot 1
% Intersection of polynomials of Eq(12): intersection 
% outside of the outside of the dashed-line blue curve and solid-line red curve

Delta=0.1;
[x1,x2]=meshgrid([-2:0.01:2],[0:0.01:4]);
figure; hold on

tt=[0:0.2:0.4, 0.5:0.1:1]; % plot time steps
for i=1:9;
    figure(1);subplot(3,3,i); hold on
t=tt(i);
[C1,h1]=contour(x1,x2,eval(Cons_1),[Delta Delta],'r','ShowText','off','Linewidth',1); 
contour(x1,x2,eval(Cons_2),[0 0],'--b','ShowText','off','Linewidth',1); 
t=[0:0.01:1];px=t-0.5;py=1*ones(size(t,2));plot(px,py,'k--','Linewidth',3)
end

