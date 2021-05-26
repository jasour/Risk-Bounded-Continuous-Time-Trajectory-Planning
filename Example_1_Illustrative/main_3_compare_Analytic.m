clc; 
%% Intersection of polynomials in Eq(10): intersection of outside of the inner and outer solid curves 
openfig('Compare_MC_Opt'); hold on;
Delta=[0.3 0.2 0.1 0.09 0.07 0.05];

[x1,x2]=meshgrid([-1:0.01:1],[-1:0.01:1]);
subplot(2,3,1);hold on
        contour(x1,x2,eval(Cons_1),[Delta(1) Delta(1)],'r','LineWidth',2);xlim([l u]);ylim([l u]);set(gca,'fontsize',Fs1)
subplot(2,3,2);hold on
        contour(x1,x2,eval(Cons_1),[Delta(2) Delta(2)],'r','LineWidth',2);xlim([l u]);ylim([l u]);set(gca,'fontsize',Fs1)
subplot(2,3,3);hold on
        contour(x1,x2,eval(Cons_1),[Delta(3) Delta(3)],'r','LineWidth',2);xlim([l u]);ylim([l u]);set(gca,'fontsize',Fs1)
subplot(2,3,4);hold on
        contour(x1,x2,eval(Cons_1),[Delta(4) Delta(4)],'r','LineWidth',2);xlim([l u]);ylim([l u]);set(gca,'fontsize',Fs1)
subplot(2,3,5);hold on
        contour(x1,x2,eval(Cons_1),[Delta(5) Delta(5)],'r','LineWidth',2);xlim([l u]);ylim([l u]);set(gca,'fontsize',Fs1)
subplot(2,3,6);hold on
        contour(x1,x2,eval(Cons_1),[Delta(6) Delta(6)],'r','LineWidth',2);xlim([l u]);ylim([l u]);set(gca,'fontsize',Fs1)

