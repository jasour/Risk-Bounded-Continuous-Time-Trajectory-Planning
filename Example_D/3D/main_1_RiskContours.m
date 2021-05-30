%clc;close all
%% 
Delta = 0.1;% risk level
C0 = [% initial nominal center of uncertain obstacles
    [0.2      0.2        0.2];
    [-0.3953   -0.7065   -0.8153];
    [-0.6275   -0.3089   -0.2065];
    [0.0776   -0.1616    0.3704];
    [-0.5911    0.7562   -0.9452];
    [0.3409   -0.1654    0.1174];
];
V0 = [% nomminal velocity of uncertain obstacles
    [0.3      0.3        0.3];
    [0.8372   -0.0232    0.2235];
    [0.5318    0.0368   -0.4064];
    [-0.6246   -0.8385    0.4769];
    [-0.1174   -0.6834    0.7599];
    [-0.4518   -0.1715   -0.4078];
];
num_obs = size(C0,1);
%% Risk Contours

% uncertainties: 
%trajectory of obs: u,v,w ~ Normal(0, 0.001)
mu = 0;var = 0.001;
for k=0:10; m_w(k+1,1)=sqrt(var)^k*(-j*sqrt(2))^k*kummerU(-k/2, 1/2,-1/2*mu^2/var);end
% radius of obs: r ~ Unif(0.2,0.4)
u=0.2;l=0.1;
m_r=[1];for i=1:2*dmax ;m_r(i+1,1)=(1/(u-l))*((u^(i+1) - l^(i+1))/(i+1));end % moments of w

syms x1 x2 x3 u v w r t
Cons_1=[]; % list of risk contours
for i=1:num_obs
    x0 = C0(i,:);v0 = V0(i,:);
    g = sum(([x1 x2 x3] - (x0 + v0*t + [u v w])).^2) - r^2; % uncertain obstacles
    dg=polynomialDegree(g);
    
    Mg=[]; %list of moment of z
    for dd=1:2
        Md=expand(g^dd); 
        Md1=subs(Md,flip(w.^[1:dd*dg].'),flip(m_w(2:dd*dg+1))) ; 
        Md2=subs(Md1,flip(v.^[1:dd*dg].'),flip(m_w(2:dd*dg+1))) ; 
        Md3=subs(Md2,flip(u.^[1:dd*dg].'),flip(m_w(2:dd*dg+1))) ; 
        Md4=subs(Md3,flip(r.^[1:dd*dg].'),flip(m_r(2:dd*dg+1))) ; 
        Mg=[Mg;Md4];
    end
    Cons_1=[Cons_1;(Mg(2)-Mg(1)^2)/Mg(2)];
end


%% plot contour for obstacles
x0 = [-1 -1 -1]; % initial point
xT = [1 1 1]; % goal point
bd = [-1 -1 -1; 1 1 1];% bounding box
[x1,x2,x3]=meshgrid([-1:0.05:1],[-1.5:0.05:1],[-1:0.05:1.5]);

fig_num=1;
for t0=[0:0.1:0.7 0.8 0.85 0.9 1]
subplot(3,4,fig_num);fig_num=fig_num+1
hold on;
for i=1:num_obs
    f = eval(subs(Cons_1(i), t, t0));
    h = patch(isosurface(x1,x2,x3,f,Delta));
    isonormals(x1,x2,x3,f,h)
    set(h,'FaceColor','red','EdgeColor','none','FaceAlpha',1); % red
    lighting phong; view(3); axis tight vis3d; camlight ;axis normal
    xlim([bd(1,1) bd(2,1)]);xlabel('x');ylim([bd(1,2) bd(2,2)]);ylabel('y');zlim([bd(1,3) bd(2,3)]);zlabel('z')
end

plot3(x0(1),x0(2),x0(3),'s','LineWidth',2,'MarkerSize',5,'MarkerEdgeColor','b','MarkerFaceColor','b'); 
plot3(xT(1),xT(2),xT(3),'^','LineWidth',2,'MarkerSize',5,'MarkerEdgeColor','b','MarkerFaceColor','b'); 
view(-70,45);grid on;ylim([-1.2 1]);zlim([-1 1.2])
end

