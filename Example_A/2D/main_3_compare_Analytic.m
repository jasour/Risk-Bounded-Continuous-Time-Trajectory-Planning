%% Intersection of polynomials in Eq(10): intersection of outside of the inner and outer solid curves 
% Inner approximation of the risk contours in Eq(9)
% = Intersection of polynomials of the risk contours in Eq(10)
% = Intersection of outside of the dashed-curve and outside of the outer solid curve
clc; Fs1=20;l=-1;u=1;
Delta=[0.3 0.1 0.05 0.02 0.01]; % Risk levles
openfig('Compare_MC'); hold on;

[x1,x2]=meshgrid([-1:0.01:1],[-1:0.01:1]);
subplot(1,5,1);hold on
        contour(x1,x2,eval(Cons_1),[Delta(1) Delta(1)],'r','LineWidth',2);xlim([l u]);ylim([l u]);set(gca,'fontsize',Fs1)
        contour(x1,x2,eval(Cons_2),[0 0],'-.b','LineWidth',0.5);xlim([l u]);ylim([l u]);set(gca,'fontsize',Fs1)
subplot(1,5,2);hold on
        contour(x1,x2,eval(Cons_1),[Delta(2) Delta(2)],'r','LineWidth',2);xlim([l u]);ylim([l u]);set(gca,'fontsize',Fs1)
        contour(x1,x2,eval(Cons_2),[0 0],'-.b','LineWidth',0.5);xlim([l u]);ylim([l u]);set(gca,'fontsize',Fs1)
subplot(1,5,3);hold on
        contour(x1,x2,eval(Cons_1),[Delta(3) Delta(3)],'r','LineWidth',2);xlim([l u]);ylim([l u]);set(gca,'fontsize',Fs1)
        contour(x1,x2,eval(Cons_2),[0 0],'-.b','LineWidth',0.5);xlim([l u]);ylim([l u]);set(gca,'fontsize',Fs1)
subplot(1,5,4);hold on
        contour(x1,x2,eval(Cons_1),[Delta(4) Delta(4)],'r','LineWidth',2);xlim([l u]);ylim([l u]);set(gca,'fontsize',Fs1)
        contour(x1,x2,eval(Cons_2),[0 0],'-.b','LineWidth',0.5);xlim([l u]);ylim([l u]);set(gca,'fontsize',Fs1)
subplot(1,5,5);hold on
        contour(x1,x2,eval(Cons_1),[Delta(5) Delta(5)],'r','LineWidth',2);xlim([l u]);ylim([l u]);set(gca,'fontsize',Fs1)
        contour(x1,x2,eval(Cons_2),[0 0],'-.b','LineWidth',0.5);xlim([l u]);ylim([l u]);set(gca,'fontsize',Fs1)