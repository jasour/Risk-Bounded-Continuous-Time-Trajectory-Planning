clc;close all

% Risk levles
Delta=[0.3 0.1 0.05 0.02 0.01];

N=10^5; 
Hob1=[];Hob2=[];Hob3=[];Hob4=[];Hob5=[];Hob6=[];
w=random('beta',9,.5,1,N); % uncertainty samples

for x1=-1:0.01:1
for x2=-1:0.01:1
[x1,x2]
%obstacle g(x1,x2,w)>=0
g=-(21*x1.^5)/50 - (59*x1.^4.*x2)/50 - (47*x1.^4)/100 + (3*x1.^3.*x2.^2)/10 - (57*x1.^3.*x2)/100 + (3*x1.^3)/5 - (13*x1.^2.*x2.^3)/20 + (17*x1.^2.*x2.^2)/100 + (187*x1.^2.*x2)/100 + (3*x1.^2)/50 + (69*x1.*x2.^4)/100 - (7*x1.*x2.^3)/50 - (17*x1.*x2.^2)/20 + (3*x1.*x2)/5 - (21*x1)/100 + x2.^5/100 - (3*x2.^4)/50 - (7*x2.^3)/100 - (41*x2.^2)/100 - (2*x2)/25 - w/10 + 7/100;
Pro=size(find(g>=0),2)/N;
if Pro<=Delta(1);  Hob1=[Hob1;x1,x2]; end
if Pro<=Delta(2);  Hob2=[Hob2;x1,x2]; end
if Pro<=Delta(3);  Hob3=[Hob3;x1,x2]; end
if Pro<=Delta(4);  Hob4=[Hob4;x1,x2]; end
if Pro<=Delta(5);  Hob5=[Hob5;x1,x2]; end
end
end
%% Plot
Fs=20; Fs1=18; Fst=15;l=-0.7;u=0.7;
subplot(1,5,1);hold on
        plot(Hob1(:,1),Hob1(:,2),'g*')
        xlabel('$x_1$','Interpreter','latex', 'FontSize',Fs);ylabel('$x_2$','Interpreter','latex', 'FontSize',Fs)
        title('$\hat{\mathcal{C}}^{\Delta=0.3}_{r}, \ {\mathcal{C}}^{\Delta=0.3}_{r}$','Interpreter','latex', 'FontSize',Fst) 
        axis square
subplot(1,5,2);hold on
        plot(Hob2(:,1),Hob2(:,2),'g*')
        xlabel('$x_1$','Interpreter','latex', 'FontSize',Fs);ylabel('$x_2$','Interpreter','latex', 'FontSize',Fs)
        title('$\hat{\mathcal{C}}^{\Delta=0.1}_{r}, \ {\mathcal{C}}^{\Delta=0.1}_{r}$','Interpreter','latex', 'FontSize',Fst)
        axis square
subplot(1,5,3);hold on
        plot(Hob3(:,1),Hob3(:,2),'g*')
        xlabel('$x_1$','Interpreter','latex', 'FontSize',Fs);ylabel('$x_2$','Interpreter','latex', 'FontSize',Fs)
        title('$\hat{\mathcal{C}}^{\Delta=0.05}_{r}, \ {\mathcal{C}}^{\Delta=0.05}_{r}$','Interpreter','latex', 'FontSize',Fst)
        axis square
subplot(1,5,4);hold on
        plot(Hob4(:,1),Hob4(:,2),'g*')
        xlabel('$x_1$','Interpreter','latex', 'FontSize',Fs);ylabel('$x_2$','Interpreter','latex', 'FontSize',Fs)
        title('$\hat{\mathcal{C}}^{\Delta=0.02}_{r}, \ {\mathcal{C}}^{\Delta=0.02}_{r}$','Interpreter','latex', 'FontSize',Fst)
        axis square
subplot(1,5,5);hold on
        plot(Hob5(:,1),Hob5(:,2),'g*')
        xlabel('$x_1$','Interpreter','latex', 'FontSize',Fs);ylabel('$x_2$','Interpreter','latex', 'FontSize',Fs)
        title('$\hat{\mathcal{C}}^{\Delta=0.01}_{r}, \ {\mathcal{C}}^{\Delta=0.01}_{r}$','Interpreter','latex', 'FontSize',Fst)
        axis square

