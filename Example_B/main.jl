# Convex Risk Bounded Continuous-Time Trajectory Planning in Uncertain Nonconvex Environments
# RSS 2021


########## Parameters %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
n = 2             # dimension of the space
max_deg_uv = 4    # degree of moment relaxation
num_pieces = 2    # number of linear pieces
num_iterations=20 # number of iterations of the heuristic
x0 = [0, 0]      # starting point
xT = [2, 1]        # destination
edge_size = 3   # edgesize of the bounding box where the trajectory lives

########### Generated Dynamic 0.1-Risk Contours in Eq(12) and Bounding Box %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Delta=0.1

constraint = [# 0.1-Risk Contour Cons 1 and 2
    (t,x) -> t^2 - 2*t*x[1] + (4*t)/5 + x[1]^2 - (4*x[1])/5 + x[2]^2 - 2*x[2] + 161/150    
    (t,x) -> (t^2 - 2*t*x[1] + (4*t)/5 + x[1]^2 - (4*x[1])/5 + x[2]^2 - 2*x[2] + 161/150)^2 - 
             (1-Delta)*(t^4 - 4*t^3*x[1] + (8*t^3)/5 + 6*t^2*x[1]^2 - (24*t^2*x[1])/5 + 2*t^2*x[2]^2 - 4*t^2*x[2] + (14*t^2)/5 - 4*t*x[1]^3 + (24*t*x[1]^2)/5 - 4*t*x[1]*x[2]^2 + 8*t*x[1]*x[2] - (28*t*x[1])/5 + (8*t*x[2]^2)/5 - (16*t*x[2])/5 + (216*t)/125 + x[1]^4 - (8*x[1]^3)/5 + 2*x[1]^2*x[2]^2 - 4*x[1]^2*x[2] + (14*x[1]^2)/5 - (8*x[1]*x[2]^2)/5 + (16*x[1]*x[2])/5 - (216*x[1])/125 + x[2]^4 - 4*x[2]^3 + (461*x[2]^2)/75 - (322*x[2])/75 + 21641/18750)
    (t,x) -> x[1]*(2-x[1]) # Bounding Box
    (t,x) -> x[2]*(1-x[2]) # Bounding Box
]

########### Run Time-Varying-SOS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
solver = optimizer_with_attributes(Mosek.Optimizer, "QUIET" => true)

@time begin
opt_trajectory=TimeVaryingSOS(n, constraint, edge_size, x0, xT, max_deg_uv, num_pieces, num_iterations)
end