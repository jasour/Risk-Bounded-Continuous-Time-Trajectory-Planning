function [nearest_tree_point, nearest_tree_point_idx, new_point] = find_nearest_tree_point(V, point, rlim)

nearest_tree_point = V(1,:);
nearest_tree_point_idx = 1;
min_dist = norm(nearest_tree_point-point);
for i = 2:size(V,1)
    dist = norm(V(i,:) - point);
    if dist < min_dist
        min_dist = dist;
        nearest_tree_point = V(i,:);
        nearest_tree_point_idx = i;
    end
end

new_point = point;
if min_dist > rlim
    new_point = (point - nearest_tree_point)*rlim/min_dist + nearest_tree_point;
end