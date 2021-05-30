%% SOS Decomposition 
% Obs(x) <=0 ---> Safe(x)>=0 --->Safe(Px(t))>=0 for all 0=<t=<1
%%
function status=func_sos_spot(Safe,Px,Py,t0,tf,d)


t = msspoly('t',1); % time

prog = spotsosprog;
prog = prog.withIndeterminate( t );
bases = monomials( t, 0:d );

[ prog, s1 ] = prog.newFreePoly( bases ); 
prog = prog.withSOS(s1);

prog = prog.withSOS( Safe(Px,Py)-s1*(t-t0)*(tf-t)); 

spot_options = spot_sdp_default_options();
spot_options.verbose = 0; %printing information
sol = prog.minimize(0, @spot_mosek,spot_options);

if sol.status == 'STATUS_PRIMAL_AND_DUAL_FEASIBLE'
    status = 1; % SOS Decomposition  Exists. Hence trajectory is safe.
else
    status = 0;
end
end