%% Figure 4 in Ashkan Jasour, Brian C. Williams, "Risk Contours Map for Risk Bounded Motion Planning under Perception Uncertainties", Robotics: Science and System (RSS), Germany, 2019. 
% MIT 16.S498: Risk Aware and Robust Nonlinear Planning, Fall 2019, http://rarnop.mit.edu
% Lecture 11: Risk Aware Planning and Control Of Probabilistic Nonlinear Dynamical Systems
%% Risk Contours Map
% To obtain the risk contours map described in page 52 of Lecture 11, we
% solve dual Optimization of SOS-SDP in page 160, Lecture 7:Nonlinear Chance Constrained and Chance Optimization.
% Dual read as page 107, Lecture 10: Probabilistic Nonlinear Safety Verification.
clc; close all
mset clear; mset('yalmip',true); mset(sdpsettings('solver','mosek'))
% polynomial order 2*d
d=10;
% polynomial power
vpow=[];for k = 0:2*d; vpow = [vpow;genpow(3,k)]; end
%%
% location x and y, uncertainty w
mpol x y w
% assigned measure
mu = meas(x,y,w);
% moments
m = mom(x.^vpow(:,1).*y.^vpow(:,2).*w.^vpow(:,3));

%slack variables
mpol xs ys ws
% assigned measure
mus = meas(xs,ys,ws); 
% moments
ms = mom(xs.^vpow(:,1).*ys.^vpow(:,2).*ws.^vpow(:,3));

% moments of Lebesgue measure over [-1,1]     
m_leb=[2];for i=1:2*d ;m_leb(i+1,1)=(1/1)*(((1)^(i+1) - (-1)^(i+1))/(i+1));end

% moment of uncertainty w \in [0,1]: Beta distribution with parameters alpha and beta
u=0.4;l=0.3;
m_w=[1];for i=1:2*d ;m_w(i+1,1)=(1/(u-l))*((u^(i+1) - l^(i+1))/(i+1));end 

%upper bound moments: moments of (Lebesgue measure on x: [-1 1])*(Lebesgue measure
%on y [-1 1])*(uncertainty w)
m_up=m_leb(vpow(:,1)+1).*m_leb(vpow(:,2)+1).*m_w(vpow(:,3)+1);
     
% Uncertain Obstacle set
 Xobs=w^2-(x-0)^2-(y-0)^2; % obstacle g>=0

 % moment SDP
 P = msdp(max(mass(mu)),m+ms == m_up, Xobs>=0, w*(1-w)>=0, (1-x^2)>=0, (1-y^2)>=0,(1-xs^2)>=0, (1-ys^2)>=0, ws*(1-ws)>=0);
 
 % solve moment SDP
[stat,obj,mm,dual] = msol(P);

%% Reults

if stat >=0
% obtained dual variables: coefficients of polynomial in x,y,w
dual;

% polynomial construction
syms x y
PP=sum(dual.*x.^vpow(:,1).*y.^vpow(:,2).*m_w(vpow(:,3)+1));
[x,y] = meshgrid(-0.9:0.01:0.9,-0.9:0.01:0.9);
PP=eval(PP);
end
%% Plots
figure; hold on
surf(x,y,PP,'FaceColor','red','EdgeColor','none','FaceAlpha',0.7);
camlight; lighting gouraud
xlim([-0.9 0.9]);ylim([-0.9 0.9]);zlim([0 1.5])
hold on;grid on;set(gca,'fontsize',31)
xlabel('$x_1$','Interpreter','latex', 'FontSize',31);ylabel('$x_2$','Interpreter','latex', 'FontSize',31)
zlabel('$\mathcal{P}_{inner}(x)$','Interpreter','latex', 'FontSize',31)
axis square

%%
Delta=[0.3 0.2 0.1 0.09 0.07 0.05];
openfig('Compare_MC'); hold on;
Fs=20; Fs1=18; Fst=30;l=-0.7;u=0.7;
subplot(2,3,1);hold on
        contour(x,y,PP,Delta(1)*[1,1],'r--','LineWidth',2);xlim([l u]);ylim([l u]);set(gca,'fontsize',Fs1)
        axis square
subplot(2,3,2);hold on
        contour(x,y,PP,Delta(2)*[1,1],'r--','LineWidth',2);xlim([l u]);ylim([l u]);set(gca,'fontsize',Fs1)
        axis square
subplot(2,3,3);hold on
        contour(x,y,PP,Delta(3)*[1,1],'r--','LineWidth',2);xlim([l u]);ylim([l u]);set(gca,'fontsize',Fs1)
        axis square
subplot(2,3,4);hold on
        contour(x,y,PP,Delta(4)*[1,1],'r--','LineWidth',2);xlim([l u]);ylim([l u]);set(gca,'fontsize',Fs1)
        axis square
subplot(2,3,5);hold on
        contour(x,y,PP,Delta(5)*[1,1],'r--','LineWidth',2);xlim([l u]);ylim([l u]);set(gca,'fontsize',Fs1)
        axis square
subplot(2,3,6);hold on
        contour(x,y,PP,Delta(6)*[1,1],'r--','LineWidth',2);xlim([l u]);ylim([l u]);set(gca,'fontsize',Fs1)
        axis square
