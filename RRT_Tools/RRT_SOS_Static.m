function [V, E, path] = RRT_SOS_Static(dm, G, x0, xT, bd, improve_rrt, ...
                        random_seed, width, number_of_samples_per_stripe)
% dm: problem dimension
% G: List of Risk contours
% x0: initial point
% xT: goal point
% bd: boundary of state space
% improve_rrt: binary variable. true if want to optimize the solution among
% the found vertices.
% Assume the total planning time T = 1.
% random_seed: random seed
% width and number_of_samples_per_stripe are parameters for heuristic
% sampling
%% 
E = [];
V = [x0];
nV = 1; % number of vertices, including x0, excluding xT
rlim = 1;
done = 0;
d = 4;
%% RRT
if nargin <= 6
    random_seed = 1;
end
rng(random_seed);
if nargin <= 7
    width = 0.1:0.1:1;
    number_of_samples_per_stripe = 10;
end
parent = [1];

if dm==2
    iterative_widening_sampling_func = @iterative_widening_sampling;
    verify_all_line_segments_func = @verify_all_line_segments;
    line = [xT(2)-x0(2), x0(1)-xT(1), x0(2)*xT(1)-x0(1)*xT(2)];
else
    iterative_widening_sampling_func = @iterative_widening_sampling_3D;
    verify_all_line_segments_func = @verify_all_line_segments_3D;
    line = [x0;xT];
end


t0=tic;
while ~done
    % randomly generate a point and trim it
    % new_point = rand(1,2)*5; %naive random sampling
    new_point = iterative_widening_sampling_func(bd, nV, line, width, number_of_samples_per_stripe);
    [nearest_tree_point, nearest_tree_point_idx, new_point] = find_nearest_tree_point(V, new_point, rlim);
    % SOS verify the line segment is obstacle free
    if verify_all_line_segments_func(G, nearest_tree_point, new_point, d)
        % add the point to vertex set
        V = [V; new_point];
        nV = nV + 1;
        E = [E; [nearest_tree_point_idx, nV]];
        parent = [parent, nearest_tree_point_idx];
        if verify_all_line_segments_func(G, new_point, xT, d)
            E = [E; [nV, nV+1]];
            parent = [parent, nV];
            done = 1;
        end
    end
end
toc(t0)
nV = nV + 1;
V = [V;xT];
%calculate path
cur = nV;
path = [cur];
while cur ~= 1
    cur = parent(cur);
    path = [cur, path];
end
%% (optional) improve RRT solutions
if improve_rrt

t1=tic;
E2 = {};
for i=1:nV,E2{i} = [];end
for i=1:nV-1
    for j=i+1:nV
        va = V(i,:);
        vb = V(j,:);
        if verify_all_line_segments_func(G, va, vb, d)
            dist = norm(va-vb);
            E2{i} = [E2{i};[dist, j]];
        end
    end
end
MAX_DIST = 1000000;
shortest_dist = ones(1,nV)*MAX_DIST;
shortest_dist(1) = 0;
parent = zeros(1,nV);
parent(1) = 1;
q = PriorityQueue(1);
q.insert([0, 1]); % [shortest dist from 1 to v, index of v]
while q.size() ~= 0
    e = q.peek();
    q.remove();
    dist_a = e(1);
    va = int64(e(2));
    if shortest_dist(va) < dist_a, continue; end
    for i=1:size(E2{va},1)
        dist = E2{va}(i,1);
        vb = E2{va}(i,2);
        if shortest_dist(vb) > shortest_dist(va) + dist
            shortest_dist(vb) = shortest_dist(va) + dist;
            parent(vb) = va;
            q.insert([shortest_dist(vb), vb]);
        end
    end
end
cur = nV;
path = [cur];
while cur ~= 1
    cur = parent(cur);
    path = [cur, path];
end
toc(t1)

end