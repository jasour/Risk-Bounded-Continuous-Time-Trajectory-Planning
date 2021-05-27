%% Risk bounded continuous-time trajectory planning is static uncertain environment
%% Illustrative Example 3 : RRT-SOS
clear;clc;close all;
%%
dm = 2; % dimension of state space
x0 = [-1,-1]; % start point
xT = [1,1]; % goal point
bd = [-1,-1;1,1]; % Bounding box that contain the obstacles and trajectory
Delta = 0.1; % Risk level
G = {  % static 0.1 Risk contours Eq(10): format constraints >=0
    @(x1,x2) x1^2 + x2^2 - 37/300,... % -E[g]>=0
    @(x1,x2) (x1^2 + x2^2 - 37/300)^2 - (1-Delta) * (x1^4 + 2*x1^2*x2^2 - (37*x1^2)/150 + x2^4 - (37*x2^2)/150 + 781/50000),...
    }; % -( (1-Delta)E[g^2]-E^2[g] )>=0

% improve RRT: 0 ==> RRT (feasible path)
% improve RRT: 1 ==>(optiml path)find the shortest path among samples found by RRT (feasible path)
improve_rrt =0;
[V, E, path] = RRT_SOS_Static(dm, G, x0, xT, bd, improve_rrt);


%% plot RRT tree
figure(1)
hold on
for i=1:size(E,1)
    e = E(i,:);
    a = V(int64(e(1)),:);
    b = V(int64(e(2)),:);
    plot([a(1),b(1)],[a(2),b(2)],'b');
end
plot(x0(1),x0(2),'s','LineWidth',2,'MarkerSize',10,'MarkerEdgeColor','k','MarkerFaceColor','b'); 
plot(xT(1),xT(2),'^','LineWidth',2,'MarkerSize',10,'MarkerEdgeColor','k','MarkerFaceColor','b'); 

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
contour(x1,x2,eval(Cons_1),[Delta Delta],'r','Linewidth',1,'ShowText','on','DisplayName','(E[g]<0'); hold on
contour(x1,x2,eval(Cons_2),[0 0],'b','Linewidth',1,'ShowText','on','DisplayName','(E[g^2]-E^2[g])/E[g^2]<Delta'); hold on

%% plot RRT path
figure(1)
hold on
for i=1:length(path)-1
    va = int64(path(i));
    vb = int64(path(i+1));
    plot([V(va,1),V(vb,1)], [V(va,2),V(vb,2)], 'LineWidth',2,'Color','r');
end
