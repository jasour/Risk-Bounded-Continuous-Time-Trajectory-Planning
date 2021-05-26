%% Static Delta-risk contour = {x: Prob(x \in Obs) <= Delta}  Eq(9)
%% Inner approximation in Eq(10) and Theorem 1
%% Illustrative Example 1
clc; clear all; close all
%% Uncertain Obstacle : Eq(4)
syms x1 x2 w
% obstacle g(x1,x2,w)>=0 where w is probabilistic uncertainty
g=w^2-(x1-0)^2-(x2-0)^2; 
dg=polynomialDegree(g); % max degree of polynomial g

% w: uncertain parameter w~Uniform[l,u]
u=0.4;l=0.3;
m_w=[1];for i=1:2*dg ;m_w(i+1,1)=(1/(u-l))*((u^(i+1) - l^(i+1))/(i+1));end %moments of w

%% Calculate the first and Second order moments of new random variable z=g(x1,x2,w)
Mg=[]; %list of first and second order moments of z in Eq(21)

for dd=1:2
% Moment of order dd of z
Md=expand(g^dd);
% replace moments of uncertain parameter w
Md1=subs(Md,flip(w.^[1:dd*dg].'),flip(m_w(2:dd*dg+1))) ; 
Mg=[Mg;Md1];
end

%% Inner approximation of Static Delta-risk contour in Eq(10)

Cons_1=(Mg(2)-Mg(1)^2)/Mg(2);
Cons_2=Mg(1);

clc;display('Done!');display('Working on plots')

%% Plot 1
[x1,x2]=meshgrid([-1:0.01:1],[-1:0.01:1]);l=-0.7;u=0.7;Fs1=20;hold on;axis square
contour(x1,x2,eval(Mg(1)),[0 0],'b','Linewidth',3);
Delta0=0.5;
F0=contour(x1,x2,eval(Cons_1),[Delta0 Delta0],'--r','LineWidth',2);xlim([l u]);ylim([l u]);set(gca,'fontsize',Fs1)
Delta1=0.3;
F1=contour(x1,x2,eval(Cons_1),[Delta1 Delta1],'r','LineWidth',2);xlim([l u]);ylim([l u]);set(gca,'fontsize',Fs1)
Delta2=0.2;
F2=contour(x1,x2,eval(Cons_1),[Delta2 Delta2],'--r','LineWidth',2);xlim([l u]);ylim([l u]);set(gca,'fontsize',Fs1)
Delta3=0.1;
F3=contour(x1,x2,eval(Cons_1),[Delta3 Delta3],'r','LineWidth',2);xlim([l u]);ylim([l u]);set(gca,'fontsize',Fs1)
Delta4=0.07;
F4=contour(x1,x2,eval(Cons_1),[Delta4 Delta4],'--r','LineWidth',2);xlim([l u]);ylim([l u]);set(gca,'fontsize',Fs1)
Delta5=0.05;
F5=contour(x1,x2,eval(Cons_1),[Delta5 Delta5],'r','LineWidth',2);xlim([l u]);ylim([l u]);set(gca,'fontsize',Fs1)
Delta6=0.01;
F6=contour(x1,x2,eval(Cons_1),[Delta6 Delta6],'--r','LineWidth',2);xlim([l u]);ylim([l u]);set(gca,'fontsize',Fs1)
Delta7=0.005;
F7=contour(x1,x2,eval(Cons_1),[Delta7 Delta7],'r','LineWidth',2);xlim([l u]);ylim([l u]);set(gca,'fontsize',Fs1)
title('Intersection of polynomials in Eq(10): outside of the blue curve ')
%% Plot 2
figure; hold on
[x1,x2]=meshgrid([-1:0.002:1],[-1:0.002:1]);
surf(x1,x2,eval(Cons_1),'FaceColor','red','EdgeColor','none','FaceAlpha',0.7);
camlight; lighting gouraud; hold on;grid on;camlight(45,-90)
plot3(F0(1,2:302),F0(2,2:302),Delta0*ones(1,size(F0(2,2:302),2)),'y','LineWidth',2)
plot3(F1(1,2:318),F1(2,2:318),Delta1*ones(1,size(F1(2,2:318),2)),'y','LineWidth',2)
plot3(F2(1,2:326),F2(2,2:326),Delta2*ones(1,size(F2(2,2:326),2)),'y','LineWidth',2)
plot3(F3(1,2:342),F3(2,2:342),Delta3*ones(1,size(F3(2,2:342),2)),'y','LineWidth',2)
plot3(F4(1,2:358),F4(2,2:358),Delta4*ones(1,size(F4(2,2:358),2)),'y','LineWidth',2)
plot3(F5(1,2:366),F5(2,2:366),Delta5*ones(1,size(F5(2,2:366),2)),'y','LineWidth',2)
plot3(F6(1,2:end),F6(2,2:end),Delta6*ones(1,size(F6(2,2:end),2)),'y','LineWidth',2)
plot3(F7(1,2:end),F7(2,2:end),Delta7*ones(1,size(F7(2,2:end),2)),'y','LineWidth',2)
xlabel('$x_1$','Interpreter','latex', 'FontSize',31);ylabel('$x_2$','Interpreter','latex', 'FontSize',31)
set(gca,'fontsize',31);axis square

%% Plot3
figure; hold on
[x1,x2]=meshgrid([-1:0.01:1],[-1:0.01:1]);
surf(x1,x2,eval(Cons_2),'FaceColor','red','EdgeColor','none','FaceAlpha',0.8);
F=contour(x1,x2,eval(Cons_2),[0 0],'--r','LineWidth',2);xlim([l u]);ylim([l u]);set(gca,'fontsize',Fs1)
plot3(F(1,2:end),F(2,2:end),0*ones(1,size(F(2,2:end),2)),'y','LineWidth',2)
camlight(0,0); lighting gouraud; hold on;grid on;set(gca,'fontsize',31);axis square;zlim([-1 0.2])
xlabel('$x_1$','Interpreter','latex', 'FontSize',31);ylabel('$x_2$','Interpreter','latex', 'FontSize',31)
