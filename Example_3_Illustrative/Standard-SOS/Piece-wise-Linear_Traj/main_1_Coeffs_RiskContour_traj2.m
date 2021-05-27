%% Static Delta-risk contour = {x: Prob(x \in Obs) <= Delta}  Eq(9)
%% Inner approximation in Eq(10) and Theorem 1
%% Illustrative Example 3
clc; clear all; close all
%% Uncertain Obstacle : Eq(4)
Delta=0.1; % risk level
% Piece-wise linear trajectory:  Piece 2
syms a_x2 b_x2 a_y2 b_y2 t w
x2=a_x2+b_x2*t;
y2=a_y2+b_y2*t;
% obstacle g(x1,x2,w)>=0 where w is probabilistic uncertainty
g=w^2-(x2)^2-(y2)^2;
dg=polynomialDegree(g); %max degree of polynomial g

% w: uncertain parameter w~Uniform[l,u]
u=0.4;l=0.3;
m_w=[1];for i=1:2*dg ;m_w(i+1,1)=(1/(u-l))*((u^(i+1) - l^(i+1))/(i+1));end %moments of w

% t: t~Uniform[1,2]
ut=2;lt=1;
m_t=[1];for i=1:2*dg ;m_t(i+1,1)=(1/(ut-lt))*((ut^(i+1) - lt^(i+1))/(i+1));end % moments of t

%% Calculate the first and Second order moments of new random variable z=g(x1,x2,w)
Mg=[]; %list of first and second order moments of z in Eq(21)

for dd=1:2
% Moment of order dd of z
Md=expand(g^dd);
% replace moments of uncertain parameter w
Md1=subs(Md,flip(w.^[1:dd*dg].'),flip(m_w(2:dd*dg+1))) ; 
% replace moments of uncertain parameter t
Md2=subs(Md1,flip(t.^[1:dd*dg].'),flip(m_t(2:dd*dg+1))) ; 
Mg=[Mg;Md2];
end

%% Static Delta-risk contour interms of the coefficients in V-A-2

Cons_1=(1-Delta)*Mg(2)-Mg(1)^2; %format: Cons_1<=0 ===>(E[g^2]-E^2[g])/E[g^2] <=Delta
Cons_2=Mg(1);%format: Cons_2<=0 ===> E[g]<=0
