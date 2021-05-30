function status = verify_line_segment(f, a, b, d)
% verify if line segment (a,b) satisfy f >= 0 (safe region)
%        a,b are two endpoints of the line segment
%        d is degree
% output: status = 1 if line segment is safe, 0 otherwise

t = msspoly( 't', 1 );
x = a + (b-a)*t;
x1 = x(1); x2 = x(2);
g = f(x1,x2); g01 = t*(1-t);

prog = spotsosprog;
prog = prog.withIndeterminate( t );
prog = sosOnK( prog, g, t, g01, d);

spot_options = spot_sdp_default_options();
spot_options.verbose = 0;
sol = prog.minimize( 0, @spot_mosek, spot_options );

%out.time = toc;
if sol.status == 'STATUS_PRIMAL_AND_DUAL_FEASIBLE'; status = 1; else; status = 0; end

