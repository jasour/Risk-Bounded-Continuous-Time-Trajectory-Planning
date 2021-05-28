clc;clear all;close all
%%
[x1,x2,x3]=meshgrid([-1:0.03:1],[-1:0.03:1],[-1:0.03:1]);
ylabel('$x_2$','Interpreter','latex');xlabel('$x_1$','Interpreter','latex');zlabel('$x_3$','Interpreter','latex')
set(gca,'fontsize',30);grid on;xlabel('x1');ylabel('x2');zlabel('x3')

w=0.1;% uncertain parameter
P=0.94-0.002.*x1-0.004.*x2-0.04.*x3-0.38.*x1.^2+0.04.*x1.*x2-0.31.*x2.^2-0.05.*x1.*x3-0.01.*x2.*x3-0.4.*x3.^2-0.1.*x1.^3-0.02.*x1.^2.*x2+0.09.*x1.*x2.^2-0.05.*x2.^3+0.14.*x1.^2.*x3-1.83.*x1.*x2.*x3+0.11.*x2.^2.*x3-0.1.*x1.*x3.^2+0.12.*x2.*x3.^2+0.34.*x3.^3-0.32.*x1.^4-0.13.*x1.^3.*x2+0.48.*x1.^2.*x2.^2+0.11.*x1.*x2.^3-0.34.*x2.^4+0.03.*x1.^3.*x3+0.01.*x1.^2.*x2.*x3-0.005.*x1.*x2.^2.*x3-0.05.*x2.^3.*x3+0.54.*x1.^2.*x3.^2-0.06.*x1.*x2.*x3.^2+0.48.*x2.^2.*x3.^2+0.008.*x1.*x3.^3+0.06.*x2.*x3.^3-0.3.*x3.^4+0.12.*x1.^5+0.005.*x1.^4.*x2-0.1.*x1.^3.*x2.^2+0.007.*x1.^2.*x2.^3+0.005.*x1.*x2.^4+0.071.*x2.^5-0.02.*x1.^4.*x3+0.73.*x1.^3.*x2.*x3-0.07.*x1.^2.*x2.^2.*x3+0.72.*x1.*x2.^3.*x3-0.20.*x2.^4.*x3+0.03.*x1.^3.*x3.^2-0.01.*x1.^2.*x2.*x3.^2+0.02.*x1.*x2.^2.*x3.^2-0.05.*x2.^3.*x3.^2-0.07.*x1.^2.*x3.^3+0.73.*x1.*x2.*x3.^3+0.09.*x2.^2.*x3.^3+0.03.*x1.*x3.^4-0.06.*x2.*x3.^4-0.31.*x3.^5-w;
h = patch(isosurface(x1,x2,x3,P,0.84));isonormals(x1,x2,x3,P,h)
set(h,'FaceColor','black','EdgeColor','none','FaceAlpha',1);
lighting phong; view(3); axis tight vis3d; camlight ;axis normal

w=0.07;% uncertain parameter
P=0.94-0.002.*x1-0.004.*x2-0.04.*x3-0.38.*x1.^2+0.04.*x1.*x2-0.31.*x2.^2-0.05.*x1.*x3-0.01.*x2.*x3-0.4.*x3.^2-0.1.*x1.^3-0.02.*x1.^2.*x2+0.09.*x1.*x2.^2-0.05.*x2.^3+0.14.*x1.^2.*x3-1.83.*x1.*x2.*x3+0.11.*x2.^2.*x3-0.1.*x1.*x3.^2+0.12.*x2.*x3.^2+0.34.*x3.^3-0.32.*x1.^4-0.13.*x1.^3.*x2+0.48.*x1.^2.*x2.^2+0.11.*x1.*x2.^3-0.34.*x2.^4+0.03.*x1.^3.*x3+0.01.*x1.^2.*x2.*x3-0.005.*x1.*x2.^2.*x3-0.05.*x2.^3.*x3+0.54.*x1.^2.*x3.^2-0.06.*x1.*x2.*x3.^2+0.48.*x2.^2.*x3.^2+0.008.*x1.*x3.^3+0.06.*x2.*x3.^3-0.3.*x3.^4+0.12.*x1.^5+0.005.*x1.^4.*x2-0.1.*x1.^3.*x2.^2+0.007.*x1.^2.*x2.^3+0.005.*x1.*x2.^4+0.071.*x2.^5-0.02.*x1.^4.*x3+0.73.*x1.^3.*x2.*x3-0.07.*x1.^2.*x2.^2.*x3+0.72.*x1.*x2.^3.*x3-0.20.*x2.^4.*x3+0.03.*x1.^3.*x3.^2-0.01.*x1.^2.*x2.*x3.^2+0.02.*x1.*x2.^2.*x3.^2-0.05.*x2.^3.*x3.^2-0.07.*x1.^2.*x3.^3+0.73.*x1.*x2.*x3.^3+0.09.*x2.^2.*x3.^3+0.03.*x1.*x3.^4-0.06.*x2.*x3.^4-0.31.*x3.^5-w;
h = patch(isosurface(x1,x2,x3,P,0.84));isonormals(x1,x2,x3,P,h)
set(h,'FaceColor','blue','EdgeColor','none','FaceAlpha',0.4); 
lighting phong; view(3); axis tight vis3d; camlight ;axis normal

w=0; % uncertain parameter
P=0.94-0.002.*x1-0.004.*x2-0.04.*x3-0.38.*x1.^2+0.04.*x1.*x2-0.31.*x2.^2-0.05.*x1.*x3-0.01.*x2.*x3-0.4.*x3.^2-0.1.*x1.^3-0.02.*x1.^2.*x2+0.09.*x1.*x2.^2-0.05.*x2.^3+0.14.*x1.^2.*x3-1.83.*x1.*x2.*x3+0.11.*x2.^2.*x3-0.1.*x1.*x3.^2+0.12.*x2.*x3.^2+0.34.*x3.^3-0.32.*x1.^4-0.13.*x1.^3.*x2+0.48.*x1.^2.*x2.^2+0.11.*x1.*x2.^3-0.34.*x2.^4+0.03.*x1.^3.*x3+0.01.*x1.^2.*x2.*x3-0.005.*x1.*x2.^2.*x3-0.05.*x2.^3.*x3+0.54.*x1.^2.*x3.^2-0.06.*x1.*x2.*x3.^2+0.48.*x2.^2.*x3.^2+0.008.*x1.*x3.^3+0.06.*x2.*x3.^3-0.3.*x3.^4+0.12.*x1.^5+0.005.*x1.^4.*x2-0.1.*x1.^3.*x2.^2+0.007.*x1.^2.*x2.^3+0.005.*x1.*x2.^4+0.071.*x2.^5-0.02.*x1.^4.*x3+0.73.*x1.^3.*x2.*x3-0.07.*x1.^2.*x2.^2.*x3+0.72.*x1.*x2.^3.*x3-0.20.*x2.^4.*x3+0.03.*x1.^3.*x3.^2-0.01.*x1.^2.*x2.*x3.^2+0.02.*x1.*x2.^2.*x3.^2-0.05.*x2.^3.*x3.^2-0.07.*x1.^2.*x3.^3+0.73.*x1.*x2.*x3.^3+0.09.*x2.^2.*x3.^3+0.03.*x1.*x3.^4-0.06.*x2.*x3.^4-0.31.*x3.^5-w;
h = patch(isosurface(x1,x2,x3,P,0.84));isonormals(x1,x2,x3,P,h)
set(h,'FaceColor','red','EdgeColor','none','FaceAlpha',0.4); 
lighting phong; view(3); axis tight vis3d; camlight ;axis normal

axis square; xlim([-1 1]);ylim([-1 1]);zlim([-1 1]);view(-72,10)
