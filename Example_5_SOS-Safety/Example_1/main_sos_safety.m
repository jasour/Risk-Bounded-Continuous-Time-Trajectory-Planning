%% SOS based continuous-time safety verification in Eq(15)in Spotles
clc; 

t = msspoly('t',1); % time
Px=t; % trajectory x(t)
Py=((t-5)^4 + 2*(t-5)^3 - 15*(t-5)^2 - 12*(t-5) + 36)/20;
t0=0;tf=9; % start and final time, i.e., t in [t0 tf]
% Safe region: g(x) >=0 
  Safe= @(x1,x2) ((x1-2)/1)^2+((x2-2)/2)^2-1^2;%  Example 1: status 1
% Safe= @(x1,x2) ((x1-3)/1)^2+((x2-2)/2)^2-1^2;%  Example 2: status 0

d=2;
tic; status=func_sos_spot(Safe,Px,Py,t0,tf,d) 
toc

%% visualization
% obs/safe
hold off; fcontour(Safe,'LevelList',[0 0],'LineWidth',3,'LineColor','r');hold on
% trajectory
PPx=[];PPy=[];
for tt=[t0:0.1:tf];  PPx=[PPx,double(subs(Px,t,tt))]; PPy=[PPy,double(subs(Py,t,tt))];end
plot(PPx(1),PPy(1),'s','LineWidth',2,'MarkerSize',10,'MarkerEdgeColor','k','MarkerFaceColor','b'); hold on
plot(PPx(end),PPy(end),'^','LineWidth',2,'MarkerSize',10,'MarkerEdgeColor','k','MarkerFaceColor','b'); 
plot(PPx,PPy,'--','LineWidth',2);grid on; 

