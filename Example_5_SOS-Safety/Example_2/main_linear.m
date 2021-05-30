%% RRT-SOS: 
%% Continuous-Time Safety Verification of linear trajectory between x1 and x2 over t in [0 1] with respect to risk contours (safe regions)
Delta=0.1;
% safe regions g>=0
 G = {  % static 0.1 Risk contours Eq(10): format constraints >=0
     @(x1,x2) x1^2 + x2^2 - 37/300,... 
     @(x1,x2) (x1^2 + x2^2 - 37/300)^2 - (1-Delta) * (x1^4 + 2*x1^2*x2^2 - (37*x1^2)/150 + x2^4 - (37*x2^2)/150 + 781/50000),...
     }; 
x1=[-1 -1]; %start point
x2=[0 2];% end point
tic
status = verify_all_line_segments(G, x1, x2, 2)
toc