# Convex Risk Bounded Continuous-Time Trajectory Planning in Uncertain Nonconvex Environments
# RSS 2021


########## Parameters %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
n = 2             # dimension of the space
max_deg_uv = 4    # degree of moment relaxation
num_pieces = 2    # number of linear pieces
num_iterations=20 # number of iterations of the heuristic
x0 = [1, -2]      # starting point
xT = [3, 2]        # destination
edge_size = 4   # edgesize of the bounding box where the trajectory lives

########### Generated Dynamic 0.1-Risk Contours in Eq(12) and Bounding Box %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Delta=0.1

constraint = [# 0.1-Risk Contour Cons 1 and 2
    (t,x) -> -(- 2*t^4 + 10*t^3 + 2*t^2*x[1] - 2*t^2*x[2] - (229*t^2)/10 - 2*t*x[1] + 8*t*x[2] + (58*t)/5 - x[1]^2 + 4*x[1] - x[2]^2 - (19*x[2])/10 - 125473/26250)
    (t,x) -> (- 2*t^4 + 10*t^3 + 2*t^2*x[1] - 2*t^2*x[2] - (229*t^2)/10 - 2*t*x[1] + 8*t*x[2] + (58*t)/5 - x[1]^2 + 4*x[1] - x[2]^2 - (19*x[2])/10 - 125473/26250)^2 - (1-Delta)*
             (4*t^8 - 40*t^7 - 8*t^6*x[1] + 8*t^6*x[2] + (958*t^6)/5 + 48*t^5*x[1] - 72*t^5*x[2] - (2522*t^5)/5 + 8*t^4*x[1]^2 - 8*t^4*x[1]*x[2] - (738*t^4*x[1])/5 + 8*t^4*x[2]^2 + (1296*t^4*x[2])/5 + (10178867*t^4)/13125 - 28*t^3*x[1]^2 + 40*t^3*x[1]*x[2] + 218*t^3*x[1] - 52*t^3*x[2]^2 - (2254*t^3*x[2])/5 - (8227972*t^3)/13125 - 4*t^2*x[1]^3 + 4*t^2*x[1]^2*x[2] + (329*t^2*x[1]^2)/5 - 4*t^2*x[1]*x[2]^2 - (278*t^2*x[1]*x[2])/5 - (3264488*t^2*x[1])/13125 + 4*t^2*x[2]^3 + (587*t^2*x[2]^2)/5 + (3829121*t^2*x[2])/13125 + (185594893*t^2)/525000 + 4*t*x[1]^3 - 16*t*x[1]^2*x[2] - (196*t*x[1]^2)/5 + 4*t*x[1]*x[2]^2 + (358*t*x[1]*x[2])/5 + (1468988*t*x[1])/13125 - 16*t*x[2]^3 - (268*t*x[2]^2)/5 - (1582484*t*x[2])/13125 - (14557133*t)/131250 + x[1]^4 - 8*x[1]^3 + 2*x[1]^2*x[2]^2 + (19*x[1]^2*x[2])/5 + (335494*x[1]^2)/13125 - 8*x[1]*x[2]^2 - (76*x[1]*x[2])/5 - (501976*x[1])/13125 + x[2]^4 + (19*x[2]^3)/5 + (172873*x[2]^2)/13125 + (9537373*x[2])/525000 + 5999639701/262500000)
    (t,x) -> (x[1])*(4-x[1])   # Bounding Box
    (t,x) -> (x[2]+2)*(2-x[2]) # Bounding Box
]


########### Run Time-Varying-SOS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
@time begin
opt_trajectory=TimeVaryingSOS(n, constraint, edge_size, x0, xT, max_deg_uv, num_pieces, num_iterations)
end