# Convex Risk Bounded Continuous-Time Trajectory Planning in Uncertain Nonconvex Environments
# RSS 2021


########## Parameters %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
n = 2             # dimension of the space
max_deg_uv = 4    # degree of moment relaxation
num_pieces = 5    # number of linear pieces
num_iterations=20 # number of iterations of the heuristic
random_seed = 3 # random seed used to initialize the heuristic                        
x0 = [0, 0]      # starting point
xT = [0, 4]        # destination
edge_size = 5   # edgesize of the bounding box where the trajectory lives

########### Generated Dynamic 0.1-Risk Contours in Eq(12) and Bounding Box %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Delta=0.1

constraint = [# 0.1-Risk Contour Cons 1 and 2
    (t,x) -> t^2 - 2*t*x[1] - t + x[1]^2 + x[1] + x[2]^2 - 2*x[2] + 82/75  
    (t,x) -> (t^2 - 2*t*x[1] - t + x[1]^2 + x[1] + x[2]^2 - 2*x[2] + 82/75)^2 - 
             (1-Delta)*(t^4 - 4*t^3*x[1] - 2*t^3 + 6*t^2*x[1]^2 + 6*t^2*x[1] + 2*t^2*x[2]^2 - 4*t^2*x[2] + (16*t^2)/5 - 4*t*x[1]^3 - 6*t*x[1]^2 - 4*t*x[1]*x[2]^2 + 8*t*x[1]*x[2] - (32*t*x[1])/5 - 2*t*x[2]^2 + 4*t*x[2] - (11*t)/5 + x[1]^4 + 2*x[1]^3 + 2*x[1]^2*x[2]^2 - 4*x[1]^2*x[2] + (16*x[1]^2)/5 + 2*x[1]*x[2]^2 - 4*x[1]*x[2] + (11*x[1])/5 + x[2]^4 - 4*x[2]^3 + (464*x[2]^2)/75 - (328*x[2])/75 + 3746/3125)
    (t,x) -> 4*t^2 + 4*t*x[1] - (16*t)/5 + x[1]^2 - (8*x[1])/5 + x[2]^2 - 4*x[2] + 269/60
    (t,x) -> (4*t^2 + 4*t*x[1] - (16*t)/5 + x[1]^2 - (8*x[1])/5 + x[2]^2 - 4*x[2] + 269/60)^2 - 
             (1-Delta)*(16*t^4 + 32*t^3*x[1] - (128*t^3)/5 + 24*t^2*x[1]^2 - (192*t^2*x[1])/5 + 8*t^2*x[2]^2 - 32*t^2*x[2] + (1154*t^2)/25 + 8*t*x[1]^3 - (96*t*x[1]^2)/5 + 8*t*x[1]*x[2]^2 - 32*t*x[1]*x[2] + (1154*t*x[1])/25 - (32*t*x[2]^2)/5 + (128*t*x[2])/5 - (3592*t)/125 + x[1]^4 - (16*x[1]^3)/5 + 2*x[1]^2*x[2]^2 - 8*x[1]^2*x[2] + (577*x[1]^2)/50 - (16*x[1]*x[2]^2)/5 + (64*x[1]*x[2])/5 - (1796*x[1])/125 + x[2]^4 - 8*x[2]^3 + (749*x[2]^2)/30 - (538*x[2])/15 + 1005441/50000)  
    (t,x) -> (81*t^2)/25 - (18*t*x[1])/5 - (63*t)/25 + x[1]^2 + (7*x[1])/5 + x[2]^2 - 6*x[2] + 28/3
    (t,x) -> ((81*t^2)/25 - (18*t*x[1])/5 - (63*t)/25 + x[1]^2 + (7*x[1])/5 + x[2]^2 - 6*x[2] + 28/3)^2 -
            (1-Delta) * ((6561*t^4)/625 - (2916*t^3*x[1])/125 - (10206*t^3)/625 + (486*t^2*x[1]^2)/25 + (3402*t^2*x[1])/125 + (162*t^2*x[2]^2)/25 - (972*t^2*x[2])/25 + (41796*t^2)/625 - (36*t*x[1]^3)/5 - (378*t*x[1]^2)/25 - (36*t*x[1]*x[2]^2)/5 + (216*t*x[1]*x[2])/5 - (9288*t*x[1])/125 - (126*t*x[2]^2)/25 + (756*t*x[2])/25 - (29421*t)/625 + x[1]^4 + (14*x[1]^3)/5 + 2*x[1]^2*x[2]^2 - 12*x[1]^2*x[2] + (516*x[1]^2)/25 + (14*x[1]*x[2]^2)/5 - (84*x[1]*x[2])/5 + (3269*x[1])/125 + x[2]^4 - 12*x[2]^3 + (164*x[2]^2)/3 - 112*x[2] + 816728/9375)
    (t,x) -> (x[1]+2)*(2-x[1])  # Bounding Box
    (t,x) -> x[2]*(5-x[2])      # Bounding Box
]
########### Run Time-Varying-SOS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
solver = optimizer_with_attributes(Mosek.Optimizer, "QUIET" => true)

@time begin
opt_trajectory=TimeVaryingSOS(n, constraint, edge_size, x0, xT, max_deg_uv, num_pieces, num_iterations,random_seed)
end

