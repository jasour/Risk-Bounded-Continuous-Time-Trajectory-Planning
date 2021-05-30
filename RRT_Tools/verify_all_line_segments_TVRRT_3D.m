function status = verify_all_line_segments_TVRRT_3D(G, a, b, d, t, piece_idx, num_pieces)

status = 1;
for i=1:length(G)
    f = G{i};
    s = verify_line_segment_TVRRT_3D(f, a, b, d, t, piece_idx, num_pieces);
    if s == 0
        status = 0;
        break
    end
end
