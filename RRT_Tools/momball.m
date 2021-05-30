function y = momball(a)
% Returns the moment of monomial x^a on the unit ball
% y = \int_{x : \sum_i x^2_i <= 1} x^a dx
% cf. Theorem 3.1 in J. B. Lasserre, E. S. Zeron. Solving a class
% of multivariate integration problems via Laplace techniques.
% Applicationes Mathematicae 28(4):391-405, 2001.

[m,n] = size(a);
y = zeros(m,1);

for k = 1:m

 if all(~rem(a(k,:),2))
  y(k) = prod(gamma((a(k,:)+1)/2))/ gamma(1+(n+sum(a(k,:)))/2);
 end

end


