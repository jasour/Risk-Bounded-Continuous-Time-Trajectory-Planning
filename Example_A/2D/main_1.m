%% Static Delta-risk contour = {x: Prob(x \in Obs) <= Delta}  Eq(9)
%% Inner approximation in Eq(10) and Theorem 1
%% Illustrative Example A-1
clc; clear all; close all

%% Uncertain Obstacle : Eq(4)
syms x1 x2 w
% obstacle g(x1,x2,w)>=0 where w is probabilistic uncertainty
C=[0.83,-0.21,-0.08,0.06,0.6,-0.41,0.6,1.87,-0.85,-0.07,-0.47,-0.57,0.17,-0.14,-0.06,-0.42,-1.18,0.30,-0.65,0.69,0.01];
d=5;pow=[];for i=0:d; pow=[pow;genpow(2,i)];end
g= sum(1*C'.*(1*x1).^pow(:,1).*(1*x2).^pow(:,2))-(0.76 + 0.1*w); % obstacle g>=0
dg=polynomialDegree(g); % max degree of polynomial g

% w: uncertain parameter w~Beta(a,b)
a=9;b=0.5; m_w=[1];for k=1:2*dg; m_w=[m_w;(a+k-1)/(a+b+k-1)*m_w(end) ]; end;


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

clc;display('Done!');display('Working on plots!')
%% Plot
[x1,x2]=meshgrid([-1:0.005:1],[-1:0.005:1]);

% Polynomials of risk contours Eq(10)
figure; hold on
surf(x1,x2,eval(Cons_1),'FaceColor','red','EdgeColor','none','FaceAlpha',0.7);
camlight; lighting gouraud; hold on;grid on;camlight(45,-90)
figure; hold on
[x1,x2]=meshgrid([-1:0.01:1],[-1:0.01:1]);
surf(x1,x2,eval(Cons_2),'FaceColor','red','EdgeColor','none','FaceAlpha',0.8);
camlight; lighting gouraud; hold on;grid on;

% Risk contours
% Inner approximation of the risk contours in Eq(9)
% = Intersection of polynomials of the risk contours in Eq(10)
% = Intersection of outside of the blue curve and outside of the red and
% black curves
figure; hold on
Delta=0.3;
[C1,h1]=contour(x1,x2,eval(Cons_1),[Delta Delta],'--k','Linewidth',3,'ShowText','off','DisplayName','(E[g^2]-E^2[g])/E[g^2]<Delta'); 
Delta=0.1;
[C2,h2]=contour(x1,x2,eval(Cons_1),[Delta Delta],'r','Linewidth',3,'ShowText','off','DisplayName','(E[g^2]-E^2[g])/E[g^2]<Delta'); 
Delta=0.05;
[C3,h3]=contour(x1,x2,eval(Cons_1),[Delta Delta],'--k','Linewidth',3,'ShowText','off','DisplayName','(E[g^2]-E^2[g])/E[g^2]<Delta'); 
Delta=0.02;
[C4,h4]=contour(x1,x2,eval(Cons_1),[Delta Delta],'r','Linewidth',3,'ShowText','off','DisplayName','(E[g^2]-E^2[g])/E[g^2]<Delta'); 
Delta=0.01;
[C5,h5]=contour(x1,x2,eval(Cons_1),[Delta Delta],'--k','Linewidth',3,'ShowText','off','DisplayName','(E[g^2]-E^2[g])/E[g^2]<Delta'); 
contour(x1,x2,eval(Mg(1)),[0 0],'b','Linewidth',3,'DisplayName','E[g]<0');
set(gca,'fontsize',31);xlabel('$x_1$','Interpreter','latex', 'FontSize',31);ylabel('$x_2$','Interpreter','latex', 'FontSize',31)