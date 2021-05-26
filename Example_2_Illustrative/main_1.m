%% Dynamic Delta-risk contour = {x: Prob(x \in Obs(t)) <= Delta}  Eq(11)
%% Inner approximation in Eq(12) 
%% Illustrative Example 2
clc; clear all; close all
%% Uncertain Obstacle : Eq(5)
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

%% Calculate the first and Second order moments of new random variable z=g(x1,x2,w)
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

%% Inner approximation of Dynamic Delta-risk contour in Eq(12)

Cons_1=(Mg(2)-Mg(1)^2)/Mg(2);
Cons_2=Mg(1);

clc;display('Done!');display('Working on plots')
%% Plot 1
% Intersection of polynomials of Eq(12): outside of the outer closed curves
Delta=0.1;
figure; hold on
[x1,x2]=meshgrid([0.8:0.002:3],[-2:0.002:3]);
t=0;[C1,h1]=contour(x1,x2,eval(Cons_1),[Delta Delta],'r','ShowText','on','Linewidth',3); %contour(x1,x2,eval(Mg(1)),[0 0],'b','Linewidth',3);
clabel(C1,h1,'FontSize',20)
t=0.5; [C2,h2]=contour(x1,x2,eval(Cons_1),[Delta Delta],'r','ShowText','on','Linewidth',3); %contour(x1,x2,eval(Mg(1)),[0 0],'b','Linewidth',3);
clabel(C2,h2,'FontSize',20)
t=1; [C3,h3]=contour(x1,x2,eval(Cons_1),[Delta Delta],'r','ShowText','on','Linewidth',3); %contour(x1,x2,eval(Mg(1)),[0 0],'b','Linewidth',3);
clabel(C3,h3,'FontSize',20)
t=[0:0.01:1];px=2-t+t.^2;py=-1+4*t-t.^2;plot(px,py,'k--','Linewidth',3)
t=0;px=2-t+t.^2;py=-1+4*t-t.^2;plot(px,py,'s','LineWidth',2,'MarkerSize',10,'MarkerEdgeColor','k','MarkerFaceColor','b'); hold on
t=1;px=2-t+t.^2;py=-1+4*t-t.^2;plot(px,py,'^','LineWidth',2,'MarkerSize',10,'MarkerEdgeColor','k','MarkerFaceColor','b'); hold on
set(gca,'fontsize',31)
xlabel('$x_1$','Interpreter','latex', 'FontSize',31);ylabel('$x_2$','Interpreter','latex', 'FontSize',31)
%% Plot 2
figure; hold on
t=0;surf(x1,x2,eval(Cons_1),'FaceColor','red','EdgeColor','none','FaceAlpha',0.7);
t=0.5;surf(x1,x2,eval(Cons_1),'FaceColor','red','EdgeColor','none','FaceAlpha',0.7);
t=1;surf(x1,x2,eval(Cons_1),'FaceColor','red','EdgeColor','none','FaceAlpha',0.7);
lighting gouraud; hold on;grid on;camlight(45,-90);
t=[0:0.01:1];px=2-t+t.^2;py=-1+4*t-t.^2;plot(px,py,'k--','Linewidth',3)
t=0;px=2-t+t.^2;py=-1+4*t-t.^2;plot(px,py,'s','LineWidth',2,'MarkerSize',10,'MarkerEdgeColor','k','MarkerFaceColor','b'); hold on
t=1;px=2-t+t.^2;py=-1+4*t-t.^2;plot(px,py,'^','LineWidth',2,'MarkerSize',10,'MarkerEdgeColor','k','MarkerFaceColor','b'); hold on
set(gca,'fontsize',31)
xlabel('$x_1$','Interpreter','latex', 'FontSize',31); ylabel('$x_2$','Interpreter','latex', 'FontSize',31)
plot3(C1(1,2:5*360),C1(2,2:5*360),Delta*ones(1,size(C1(2,2:5*360),2)),'y','LineWidth',2)
plot3(C2(1,2:5*360),C2(2,2:5*360),Delta*ones(1,size(C2(2,2:5*360),2)),'y','LineWidth',2)
plot3(C3(1,2:5*360),C3(2,2:5*360),Delta*ones(1,size(C3(2,2:5*360),2)),'y','LineWidth',2)

