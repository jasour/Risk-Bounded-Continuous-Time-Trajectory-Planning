function status = verify_all_line_segments(G, a, b, d)

status = 1;
tic
for i=1:length(G)
    f = G{i};
      s = verify_line_segment(f, a, b, d);
   
    if s == 0
        status = 0;
        break
    end
end
 toc
clc