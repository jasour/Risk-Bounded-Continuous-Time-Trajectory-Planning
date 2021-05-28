%% Static Delta-risk contour = {x: Prob(x \in Obs) <= Delta}  Eq(9)
%% Inner approximation in Eq(10) and Theorem 1
%% Illustrative Example A-2
clc; clear all; close all

%% Uncertain Obstacle : Eq(4)
syms x1 x2 x3 w
% obstacle g(x1,x2,w)>=0 where w is probabilistic uncertainty
g=0.94-0.002.*x1-0.004.*x2-0.04.*x3-0.38.*x1.^2+0.04.*x1.*x2-0.31.*x2.^2-0.05.*x1.*x3-0.01.*x2.*x3-0.4.*x3.^2-0.1.*x1.^3-0.02.*x1.^2.*x2+0.09.*x1.*x2.^2-0.05.*x2.^3+0.14.*x1.^2.*x3-1.83.*x1.*x2.*x3+0.11.*x2.^2.*x3-0.1.*x1.*x3.^2+0.12.*x2.*x3.^2+0.34.*x3.^3-0.32.*x1.^4-0.13.*x1.^3.*x2+0.48.*x1.^2.*x2.^2+0.11.*x1.*x2.^3-0.34.*x2.^4+0.03.*x1.^3.*x3+0.01.*x1.^2.*x2.*x3-0.005.*x1.*x2.^2.*x3-0.05.*x2.^3.*x3+0.54.*x1.^2.*x3.^2-0.06.*x1.*x2.*x3.^2+0.48.*x2.^2.*x3.^2+0.008.*x1.*x3.^3+0.06.*x2.*x3.^3-0.3.*x3.^4+0.12.*x1.^5+0.005.*x1.^4.*x2-0.1.*x1.^3.*x2.^2+0.007.*x1.^2.*x2.^3+0.005.*x1.*x2.^4+0.071.*x2.^5-0.02.*x1.^4.*x3+0.73.*x1.^3.*x2.*x3-0.07.*x1.^2.*x2.^2.*x3+0.72.*x1.*x2.^3.*x3-0.20.*x2.^4.*x3+0.03.*x1.^3.*x3.^2-0.01.*x1.^2.*x2.*x3.^2+0.02.*x1.*x2.^2.*x3.^2-0.05.*x2.^3.*x3.^2-0.07.*x1.^2.*x3.^3+0.73.*x1.*x2.*x3.^3+0.09.*x2.^2.*x3.^3+0.03.*x1.*x3.^4-0.06.*x2.*x3.^4-0.31.*x3.^5-w-0.84;
dg=polynomialDegree(g);%max degree of polynomial g

% w: uncertain parameter w~Normal(mean,variance)
mean=0.1; var=0.001; for k=0:2*dg; m_w(k+1,1)=sqrt(var)^k*(-j*sqrt(2))^k*kummerU(-k/2, 1/2,-1/2*mean^2/var);end


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

[x1,x2,x3]=meshgrid([-1:0.03:1],[-1:0.03:1],[-1:0.03:1]);
P=eval(Cons_1);

figure; hold on

Delta=0.5; h = patch(isosurface(x1,x2,x3,P,Delta));
isonormals(x1,x2,x3,P,h);set(h,'FaceColor','green','EdgeColor','none','FaceAlpha',1); % red
lighting phong; view(3); axis tight vis3d; camlight ;axis normal

Delta=0.3;h = patch(isosurface(x1,x2,x3,P,Delta));
isonormals(x1,x2,x3,P,h);set(h,'FaceColor','black','EdgeColor','none','FaceAlpha',0.4); % red
lighting phong; view(3); axis tight vis3d; camlight ;axis normal

Delta=0.1;h = patch(isosurface(x1,x2,x3,P,Delta));
isonormals(x1,x2,x3,P,h);set(h,'FaceColor','blue','EdgeColor','none','FaceAlpha',0.3); % red
lighting phong; view(3); axis tight vis3d; camlight ;axis normal


Delta=0.05;h = patch(isosurface(x1,x2,x3,P,Delta));
isonormals(x1,x2,x3,P,h);set(h,'FaceColor','red','EdgeColor','none','FaceAlpha',0.1); % red
lighting phong; view(3); axis tight vis3d; camlight ;axis normal


grid on; axis square; set(gca,'fontsize',25);xlim([-1 1]);ylim([-1 1]);zlim([-1 1])
xlabel('$x_1$','Interpreter','latex', 'FontSize',25); ylabel('$x_2$','Interpreter','latex', 'FontSize',25);view(-72,10)