function status = verify_line_segment_TVRRT_3D(f, a, b, d, t, piece_idx, num_pieces)
% verify if line segment (a,b) satisfy f >= 0
% input: f <= 0 is obstacle. e.g. f = @(x1,x2) x1^2 + x2^2 - 1^2;
%        a,b are two endpoints of the line segment
%        d is degree
% output: status = 1 if line segment is obs free, 0 otherwise

x = a + num_pieces*(b-a)*(t-(piece_idx-1)/num_pieces);
x1 = x(1);
x2 = x(2);
x3 = x(3);
g = f(x1,x2,x3);
g01 = -(t-(piece_idx-1)/num_pieces)*(t-piece_idx/num_pieces);

prog = spotsosprog;
prog = prog.withIndeterminate( t );
[prog,gamma] = prog.newFree(1);

prog = sosOnK( prog, g-gamma, t, g01, d);

% Objective
obj = -gamma;; 


spot_options = spot_sdp_default_options();
spot_options.verbose = 0;

% Solve
tic;
[sol] = prog.minimize( obj, @spot_mosek, spot_options );
out.time = toc;
if sol.status == 'STATUS_PRIMAL_AND_DUAL_FEASIBLE'
    status = 1;
else
    status = 0;
end