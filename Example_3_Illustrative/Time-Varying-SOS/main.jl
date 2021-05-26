# Convex Risk Bounded Continuous-Time Trajectory Planning in Uncertain Nonconvex Environments
# RSS 2021


########## Parameters %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
n = 2             # dimension of the space
max_deg_uv = 4    # degree of moment relaxation
num_pieces = 2    # number of linear pieces
num_iterations=40 # number of iterations of the heuristic
x0 = [-1, -1]      # starting point
xT = [1, 1]        # destination
edge_size = 2   # edgesize of the bounding box where the trajectory lives

########### Generated 0.1-Risk Contours in Eq(10) and Bounding Box %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Delta=0.1
constraint = [
    (t,x) -> (x[1]^2 + x[2]^2 - 37/300)^2 - (1-Delta)*
        (x[1]^4 + 2*x[1]^2*x[2]^2 - (37*x[1]^2)/150 + x[2]^4 - (37*x[2]^2)/150 + 781/50000) # 0.1-Risk Contour Cons 1
    (t,x) -> x[1]^2 + x[2]^2 - 37/300    # 0.1-Risk Contour Cons 2
    (t,x) -> (x[1]+1)*(1-x[1]) # Bounding Box
    (t,x) -> (x[2]+1)*(1-x[2]) # Bounding Box
]

########### Run Time-Varying-SOS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
@time begin
opt_trajectory=TimeVaryingSOS(n, constraint, edge_size, x0, xT, max_deg_uv, num_pieces, num_iterations)
end