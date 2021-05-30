function status = verify_all_line_segments_3D(G, a, b, d)

status = 1;
for i=1:length(G)
    f = G{i};
    s = verify_line_segment_3D(f, a, b, d);
    if s == 0
        status = 0;
        break
    end
end
