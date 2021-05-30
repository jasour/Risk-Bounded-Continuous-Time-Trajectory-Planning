function [V, E, path, path_points] = RRT_SOS_Dynamic(dm, G, x0, xT, bd, t, num_pieces,...
                                        random_seed,...
                                        width,...
                                        number_of_samples_per_stripe,...
                                        iter_per_stripe)
% dm: problem dimension (2 or 3)
% G: List of Risk contours
% x0: initial point
% xT: goal point
% bd: boundary of state space
% t: spotless variable msspoly( 't', 1 )
% num_pieces: number of linear pieces of the trajectory
% Assume the total planning time T = 1.
% width, number_of_samples_per_stripe, iter_per_stripe are parameters for
% heuristic sampling
V = {};
V{1} = [x0];
E = {};
parent = {};
nV = zeros(1,num_pieces); % number of vertices, including x0, excluding xT
nV(1) = 1;
rlim = 1;
nV_MAX = 20;
MAX_ITER = 1000;
d = 8;
if nargin <= 7
    random_seed = 1;
end
rng(random_seed);
done = 0;
for i=2:num_pieces+1
    V{i} = [];
    E{i} = [];
    nV(i) = 0;
    parent{i} = {};
end
V{num_pieces+1} = [xT];

if nargin <= 8
    width = 0.1:0.1:5;
    width(1) = 0.1;
    number_of_samples_per_stripe = 10;
    iter_per_stripe = 50;
end

if dm==2
    iterative_widening_sampling_func = @iterative_widening_sampling_TVRRT;
    verify_all_line_segments_func = @verify_all_line_segments_TVRRT;
    line = [xT(2)-x0(2), x0(1)-xT(1), x0(2)*xT(1)-x0(1)*xT(2)];
else
    iterative_widening_sampling_func = @iterative_widening_sampling_TVRRT_3D;
    verify_all_line_segments_func = @verify_all_line_segments_TVRRT_3D;
    line = [x0;xT];
end

t0 = tic;
for i=2:num_pieces
for iter=1:MAX_ITER
    % randomly generate a point and trim it
    % new_point = rand(1,2)*5; %naive random sampling
    new_point = iterative_widening_sampling_func(bd, nV(i), line, iter, i, num_pieces, width, number_of_samples_per_stripe, iter_per_stripe);
    [nearest_tree_point, nearest_tree_point_idx, new_point] = find_nearest_tree_point(V{i-1}, new_point, rlim);
    % SOS verify the line segment is obstacle free
    if verify_all_line_segments_func(G, nearest_tree_point, new_point, d, t, i-1, num_pieces)
        if i~=num_pieces
            % add the point to vertex set
            V{i} = [V{i}; new_point];
            nV(i) = nV(i) + 1;
            E{i} = [E{i}; [nearest_tree_point_idx, nV(i)]];
            parent{i}{nV(i)} = nearest_tree_point_idx;
        elseif verify_all_line_segments_func(G, new_point, xT, d, t, i, num_pieces)
            % add the point to vertex set
            V{i} = [V{i}; new_point];
            nV(i) = nV(i) + 1;
            E{i} = [E{i}; [nearest_tree_point_idx, nV(i)]];
            parent{i}{nV(i)} = nearest_tree_point_idx;
            % add the edge between the point and  the target
            E{i+1} = [E{i+1}; [nV(i), 1]];
            parent{i+1}{1} = nV(i);
            done = 1;
        end
    end
    % for all but the last layer, sample certain number of points
    if (i~=num_pieces && nV(i) >= nV_MAX) || done
        break
    end
end
end
toc(t0)

% find path
path = [1];
path_points = [];
cur = 1;
for i=num_pieces+1:-1:2
    cur = parent{i}{cur};
    path = [cur, path];
end
for i=1:num_pieces
    va = V{i}(path(i),:);
    vb = V{i+1}(path(i+1),:);
    path_points = [path_points;[va,vb]];
end
