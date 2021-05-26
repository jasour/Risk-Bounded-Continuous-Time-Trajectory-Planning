clc;close all

% Risk levles
Delta=[0.3 0.2 0.1 0.09 0.07 0.05];

N=10^6; w=random('Uniform',0.3,.4,1,N);
Hob1=[];Hob2=[];Hob3=[];Hob4=[];Hob5=[];Hob6=[];
for x=-1:0.01:1
for y=-1:0.01:1
[x,y]
g=w.^2-x^2-y^2; %obstacle g(x1,x2,w)>=0
Pro=size(find(g>=0),2)/N;
if Pro<=Delta(1);  Hob1=[Hob1;x,y]; end
if Pro<=Delta(2);  Hob2=[Hob2;x,y]; end
if Pro<=Delta(3);  Hob3=[Hob3;x,y]; end
if Pro<=Delta(4);  Hob4=[Hob4;x,y]; end
if Pro<=Delta(5);  Hob5=[Hob5;x,y]; end
if Pro<=Delta(6);  Hob6=[Hob6;x,y]; end
end
end
%% Plot
figure(10); Fs=20; Fs1=18; Fst=30;l=-0.7;u=0.7;
subplot(2,3,1);hold on
        plot(Hob1(:,1),Hob1(:,2),'g*')
        xlabel('$x_1$','Interpreter','latex', 'FontSize',Fs);ylabel('$x_2$','Interpreter','latex', 'FontSize',Fs)
        title('$\hat{\mathcal{C}}^{\Delta=0.3}_{r}, \ {\mathcal{C}}^{\Delta=0.3}_{r}$','Interpreter','latex', 'FontSize',Fst) 
        axis square
subplot(2,3,2);hold on
        plot(Hob2(:,1),Hob2(:,2),'g*')
        xlabel('$x_1$','Interpreter','latex', 'FontSize',Fs);ylabel('$x_2$','Interpreter','latex', 'FontSize',Fs)
        title('$\hat{\mathcal{C}}^{\Delta=0.2}_{r}, \ {\mathcal{C}}^{\Delta=0.2}_{r}$','Interpreter','latex', 'FontSize',Fst)
        axis square
subplot(2,3,3);hold on
        plot(Hob3(:,1),Hob3(:,2),'g*')
        xlabel('$x_1$','Interpreter','latex', 'FontSize',Fs);ylabel('$x_2$','Interpreter','latex', 'FontSize',Fs)
        title('$\hat{\mathcal{C}}^{\Delta=0.1}_{r}, \ {\mathcal{C}}^{\Delta=0.1}_{r}$','Interpreter','latex', 'FontSize',Fst)
        axis square
subplot(2,3,4);hold on
        plot(Hob4(:,1),Hob4(:,2),'g*')
        xlabel('$x_1$','Interpreter','latex', 'FontSize',Fs);ylabel('$x_2$','Interpreter','latex', 'FontSize',Fs)
        title('$\hat{\mathcal{C}}^{\Delta=0.09}_{r}, \ {\mathcal{C}}^{\Delta=0.09}_{r}$','Interpreter','latex', 'FontSize',Fst)
        axis square
subplot(2,3,5);hold on
        plot(Hob5(:,1),Hob5(:,2),'g*')
        xlabel('$x_1$','Interpreter','latex', 'FontSize',Fs);ylabel('$x_2$','Interpreter','latex', 'FontSize',Fs)
        title('$\hat{\mathcal{C}}^{\Delta=0.07}_{r}, \ {\mathcal{C}}^{\Delta=0.07}_{r}$','Interpreter','latex', 'FontSize',Fst)
        axis square
subplot(2,3,6);hold on
        plot(Hob6(:,1),Hob6(:,2),'g*')
        xlabel('$x_1$','Interpreter','latex', 'FontSize',Fs);ylabel('$x_2$','Interpreter','latex', 'FontSize',Fs)
        title('$\hat{\mathcal{C}}^{\Delta=0.05}_{r}, \ {\mathcal{C}}^{\Delta=0.05}_{r}$','Interpreter','latex', 'FontSize',Fst)
        axis square

