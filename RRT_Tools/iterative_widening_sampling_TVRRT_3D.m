function p = iterative_widening_sampling_TVRRT_3D(bd, nV, line, iter, i, num_pieces, width, number_of_samples_per_stripe, iter_per_stripe)
% bd: [x_l, y_l, z_l; x_u, y_u, z_u]
% nV: number of vertices
% line: [x0;xT]
if i < num_pieces
    idx = int64(1+floor(nV/number_of_samples_per_stripe));
else
    idx = int64(1+floor(iter/iter_per_stripe));
end
if idx <= 21
    w = width(idx);
else
    w = 1000000;
end
x1 = line(1,:);
x2 = line(2,:);
while true
    p = rand(1,3).*(bd(2,:)-bd(1,:)) + bd(1,:);
    t = dot(p - x1, x2 - x1)/dot(x2 - x1, x2 - x1);
    vec = (x2-x1)*t + x1;
    dist_to_line = norm(vec);
    if dist_to_line <= w
        break;
    end
end
