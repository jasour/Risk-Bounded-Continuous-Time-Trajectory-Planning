function p = iterative_widening_sampling(bd, nV, line, width, number_of_samples_per_stripe)
% bd: [x_l, y_l; x_u, y_u]
% nV: number of vertices
% line: [a,b,c], ax+by+c=0
idx = int64(1+floor(nV/number_of_samples_per_stripe));
if idx <= length(width)
    w = width(idx);
else
    w = 1000000;
end
a = line(1); b = line(2); c = line(3);
while true 
    p = rand(1,2).*(bd(2,:)-bd(1,:)) + bd(1,:);
    dist_to_line = abs(a*p(1) + b*p(2) + c)/sqrt(a^2 + b^2);
    if dist_to_line <= w
        break;
    end
end
