%% Risk bounded continuous-time trajectory planning is dynamic uncertain environment
%% Illustrative Example 4 : RRT-SOS
clear;clc;close all;
%%
dm = 2; % dimension of state space
x0 = [1,-2]; % start point
xT = [3,2]; % goal point
bd = [0,-4;4,4]; % Bounding box that contain the obstacles and trajectory
num_pieces = 2; % number of trajectory pieces 
delta = 0.1; % Risk level
t = msspoly( 't', 1 );
G = { % static 0.1 Risk contours Eq(12): format constraints >=0
    @(x1,x2) -(- 2*t^4 + 10*t^3 + 2*t^2*x1 - 2*t^2*x2 - (229*t^2)/10 - 2*t*x1 + 8*t*x2 + (58*t)/5 - x1^2 + 4*x1 - x2^2 - (19*x2)/10 - 125473/26250),...
    @(x1,x2) (- 2*t^4 + 10*t^3 + 2*t^2*x1 - 2*t^2*x2 - (229*t^2)/10 - 2*t*x1 + 8*t*x2 + (58*t)/5 - x1^2 + 4*x1 - x2^2 - (19*x2)/10 - 125473/26250)^2 - (1-delta) * (4*t^8 - 40*t^7 - 8*t^6*x1 + 8*t^6*x2 + (958*t^6)/5 + 48*t^5*x1 - 72*t^5*x2 - (2522*t^5)/5 + 8*t^4*x1^2 - 8*t^4*x1*x2 - (738*t^4*x1)/5 + 8*t^4*x2^2 + (1296*t^4*x2)/5 + (10178867*t^4)/13125 - 28*t^3*x1^2 + 40*t^3*x1*x2 + 218*t^3*x1 - 52*t^3*x2^2 - (2254*t^3*x2)/5 - (8227972*t^3)/13125 - 4*t^2*x1^3 + 4*t^2*x1^2*x2 + (329*t^2*x1^2)/5 - 4*t^2*x1*x2^2 - (278*t^2*x1*x2)/5 - (3264488*t^2*x1)/13125 + 4*t^2*x2^3 + (587*t^2*x2^2)/5 + (3829121*t^2*x2)/13125 + (185594893*t^2)/525000 + 4*t*x1^3 - 16*t*x1^2*x2 - (196*t*x1^2)/5 + 4*t*x1*x2^2 + (358*t*x1*x2)/5 + (1468988*t*x1)/13125 - 16*t*x2^3 - (268*t*x2^2)/5 - (1582484*t*x2)/13125 - (14557133*t)/131250 + x1^4 - 8*x1^3 + 2*x1^2*x2^2 + (19*x1^2*x2)/5 + (335494*x1^2)/13125 - 8*x1*x2^2 - (76*x1*x2)/5 - (501976*x1)/13125 + x2^4 + (19*x2^3)/5 + (172873*x2^2)/13125 + (9537373*x2)/525000 + 5999639701/262500000)    
};% constraints >=0

% calculate rrt path
[V, E, path, path_points] = RRT_SOS_Dynamic(dm, G, x0, xT, bd, t, num_pieces);


%% plot RRT Path

figure(1)
hold on
for k=2:num_pieces+1
for i=1:size(E{k},1)
    e = E{k}(i,:);
    a = V{k-1}(int64(e(1)),:);
    b = V{k}(int64(e(2)),:);
    plot([a(1),b(1)],[a(2),b(2)],'b');
end
end
plot(x0(1),x0(2),'s','LineWidth',2,'MarkerSize',10,'MarkerEdgeColor','k','MarkerFaceColor','b'); 
plot(xT(1),xT(2),'^','LineWidth',2,'MarkerSize',10,'MarkerEdgeColor','k','MarkerFaceColor','b'); 
for i=1:num_pieces
    va = V{i}(path(i),:);
    vb = V{i+1}(path(i+1),:);
    plot([va(1), vb(1)], [va(2), vb(2)],'-o','LineWidth',2,'color',rand(1,3));
end


%% plot dynamic 0.1-Risk Contour: 
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

%% Plot RRT Trajectory
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

% plot trajectory
for i=1:num_pieces
    plot([path_points(i,1), path_points(i,3)], [path_points(i,2), path_points(i,4)], 'LineWidth',2,'Color','g');
end

% plot trajectroy at time t
for i=1:num_pieces
    if t >= (i-1)/num_pieces && t <= i/num_pieces
        cur_x = path_points(i,1:2) + (t-(i-1)/num_pieces)*num_pieces*(path_points(i,3:4) -path_points(i,1:2));
        plot(cur_x(1),cur_x(2),'o','LineWidth',2,'MarkerSize',10,'MarkerEdgeColor','k','MarkerFaceColor','r'); 
    end
end

end
