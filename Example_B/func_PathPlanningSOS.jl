# For more information:
# https://github.com/bachirelkhadir/PathPlanningSOS.jl
# B. E. Khadir, J. B. Lasserre, and V. Sindhwani, “Piecewise-linear motion planning amidst static, moving, or morphing obstacles,” IEEE International Conference on Robotics and Automation (ICRA), 2021

using MosekTools
using JuMP
using Base64
using DynamicPolynomials
using LinearAlgebra
using MultivariateMoments
using Plots
using ProgressMeter
using PyCall
using PyPlot
using Random
using SumOfSquares
using Test



## Helper functions
dist_squared(a, b) = sum((a .- b).^2)


"""
Returns the localization matrix (g * mi * mj)_ij,
where mi and mj are monomials in `vars` up to degree `max_deg`.
"""

function loc_matrix(vars, t, g, max_deg)
    half_deg = (max_deg - maxdegree(subs(g, t=>1))) ÷ 2
    half_mons = monomials(vars, 0:half_deg)
    M = g .* (half_mons * half_mons')
    M
end


"""
Returns a newly created measure `ϕ` in variables `vars`
"""

function measure_in_vars(model, vars, t, max_deg, name)
    mons = monomials(vars, 0:max_deg)
    moments = @variable(model, [i=1:size(mons,1)], base_name=name)
    μ = MultivariateMoments.measure(moments, mons)
    E_μ = p -> MultivariateMoments.expectation(μ, p)
    # make E_μ handle polynomials that depend on t
    E_μ_tv = p -> begin
        mons_p = monomials(p)
        coeff_p = coefficients(p)
        r = 0
        for (m, c)=zip(mons_p, coeff_p)
            deg_t = degree(m, t)
            m_without_t = subs(m, t=>1)
            r += c * E_μ(m_without_t) * t^deg_t
        end
        r
    end

    return μ, E_μ_tv
end

"""
Make the matrix M(t) sos on [0, 1].
"""
function make_sos_on_0_1(model, t, M)
    # polynomial variables inside M
    var_M = hcat(map(Mij -> Mij.x, M)...)

    # if M does not depent on `t`,
    # there is no need to invoke the SOS machinery
    if t in var_M
        y = [similarvariable(eltype(M), gensym()) for i in 1:size(M, 1)]
        p = dot(y, M * y)
        # todo: adapt multipliers to degree of t in M
        domain_t = @set t*(1-t) >= 0
        @constraint(model, p >= 0, domain=domain_t )
    else
        # hack to convert a constant polynomial to a constant
        M = map(Mij -> Mij.a[1], M)
        if size(M,1) == 1
            @constraint model M[1] >= 0
        else
            @constraint model M in JuMP.PSDCone()
        end
    end
end



function find_path_using_rigorous_approach(n::Int, contraint_fcts, edge_size::Float64, 
    a::Array{Float64, 1}, b::Array{Float64, 1},
    max_deg_uv::Int, max_deg_z::Int, num_pieces::Int, solver,
    ;scale_init=1, reg=0, seed=0)
    # Paste here
end



"""
Convert linear pieces to a single trajectory.
"""

function pieces_to_trajectory(trajectory_pieces)
     t -> begin
        num_pieces = size(trajectory_pieces, 1)
        idx_piece = Int(fld(t * num_pieces, 1))
        idx_piece = min(idx_piece, num_pieces-1)
        piece = trajectory_pieces[idx_piece+1]
        # time shift
        map(xi -> xi(t*num_pieces-idx_piece), piece)
    end
end

# Animation helpers

function plot_levelset(g)
    """
    Plot the set {x | g(x) = 0}
    """
    min_x, max_x = PyPlot.xlim()
    min_y, max_y = PyPlot.ylim()
    xs = collect(min_x:.03:max_x)
    ys = collect(min_y:.03:max_y)
    zs = g.(xs', ys)
    PyPlot.contour(xs, ys, zs, levels=[0])
end


function plot_at_time(t, edge_size, a, b, eq_obstacles, opt_trajectory)
    PyPlot.xlim(-edge_size*1.1, edge_size*1.1)
    PyPlot.ylim(-edge_size*1.1, edge_size*1.1)

    ts = collect(0:0.01:1)
    xts = hcat(opt_trajectory.(ts)...)
    xt = opt_trajectory(t)
    PyPlot.plot(xts[1, :], xts[2, :])
    PyPlot.scatter([xt[1]], [xt[2]], s=100, c="g")


    for g=eq_obstacles
        plot_levelset((x, y) -> g(t, [x, y]))
    end

    PyPlot.scatter([a[1],b[1]], [a[2], b[2]],  s=100, c="r")
end

function find_path_using_heuristic(n, contraint_fcts, edge_size, a, b,
    max_deg_uv, num_pieces, solver,
    weight_lenght,
    num_iterations,
    ;scale_init=1, reg=0, seed=0)


    @polyvar u[1:n]
    @polyvar v[1:n]
    @polyvar t
    @polyvar x[1:n]
    xt = u .+ t .* v
    measure_vars = [u..., v...]
    contraint_polys = [
        f(t, x) for f in contraint_fcts
    ]


    @info "Definiting the model and the decision vars..."
    model = SOSModel(solver)

    @variable model γ # γ = ||decision_vars||
    @variable model α[1:num_pieces] # α = ||E(v)||

    μ_uvs, Eμ_uvs = zip([
        measure_in_vars(model, measure_vars, t, max_deg_uv, "uv_$i")
        for i=1:num_pieces
            ]...)

    decision_vars = cat([μ.a for μ=μ_uvs]..., dims=1)

    ## Constraints
    @info "Constraints definition..."


    # total mass is one
    for Eμ=Eμ_uvs
        @constraint model Eμ(0*t+1).a[1] == 1
    end


    # obstacle constraints
    # localization of (u,v)

    loc_polynomials = [
        0*u[1] + 1,
        contraint_polys...
    ]

    for (i, E_uv)=enumerate(Eμ_uvs)

        for g=loc_polynomials
            g_time_corrected = subs(g, t=>(t+i-1)/num_pieces,
                                [xj => xtj for (xj, xtj)=zip(x, xt)]...)


            Mi_g = loc_matrix(measure_vars, t, g_time_corrected, max_deg_uv)
            M = E_uv.(Mi_g)
            # @show g
            # @show M
            # make sos on [0, 1]
            make_sos_on_0_1(model, t, M)

        end
    end


    # continuity constraints
    # x(t=1+) = x(t=0-)

    for (E_uv, E_uv_plus)=zip(Eμ_uvs, Eμ_uvs[2:end])
        c = @constraint(model, E_uv.(subs(xt, t=>1)) .== E_uv_plus.(subs(xt, t=>0)))
    end



    # x(0) ~ a, x(1) ~ b
    mons = monomials([u..., v...], 0:max_deg_uv-1)
    for m=mons
        @constraint model (Eμ_uvs[1]).(m .* (subs(xt, t=>0) .- a)) .== 0
        @constraint model (Eμ_uvs[end]).(m .* (subs(xt, t=>1) .- b)) .== 0
    end

    # regularization

    @constraint model [γ; decision_vars] in SecondOrderCone()

    for i=1:num_pieces
        piece_i = [m.a[1] for m=Eμ_uvs[i].(v)]
        @constraint model [α[i]; piece_i...] in SecondOrderCone()
    end

    ## Random initialization


    Random.seed!(seed)
    uv_k = []

    push!(uv_k, scale_init .* (2 .* Random.rand(Float64, (num_pieces, n)) .- 1))

    @info "Starting iterative heuristic..."
    @showprogress for k=1:num_iterations
        # objective = sum([ Eμ(ui^2) - 2*Eμ(ui)*old_ui
        #         for (Eμ,old_u)=zip(Eμ_uvs, uv_k[end])
        #         for (old_ui,ui)=zip(old_u, measure_vars)])
        d = max_deg_uv
        objective = sum([ Eμ(ui^d) - d*Eμ(ui)*old_ui^(d-1)
                for (Eμ,old_u)=zip(Eμ_uvs, uv_k[end])
                for (old_ui,ui)=zip(old_u, measure_vars)])

        objective = objective.a[1] + reg * γ + weight_lenght * sum(α)
        @objective model Min objective

        optimize!(model)
        # @info termination_status(model), objective_value(model), value(objective)
        # @info value.(α)
        st = termination_status(model)
        st_opt = objective_value(model)
        opt_trajectory = [value.(Euv.(xt)) for Euv in Eμ_uvs]

        push!(uv_k,  [ [value(Eμ(ui))
                        for ui=measure_vars] for Eμ=Eμ_uvs])

    end

    rank_one_dist = sum([value(Eμ(ui^2)) - value(Eμ(ui))^2
                for (Eμ,old_u)=zip(Eμ_uvs, uv_k[end])
                for (old_ui,ui)=zip(old_u, measure_vars)])
    @show rank_one_dist

    rank_one_dist1 = sum([value(Eμ(ui^max_deg_uv)) - value(Eμ(ui))^max_deg_uv
                for (Eμ,old_u)=zip(Eμ_uvs, uv_k[end])
                for (old_ui,ui)=zip(old_u, measure_vars)])
    @show rank_one_dist1
        opt_trajectory_pieces = [value.(Euv.(xt)) for Euv in Eμ_uvs]
        #pieces_to_trajectory(opt_trajectory_pieces)
end

function TimeVaryingSOS(n, constraint, edge_size, a, b, max_deg_uv, num_pieces, num_iterations)
    """
    n # dimension of the space
    constraint # constraint g>=0
    edge_size # edgesize of the bounding box where the trajectory lives
    a # starting point
    b # destination
    max_deg_uv # degree of moment relaxation
    num_pieces # number of linear pieces
    num_iterations # number of iterations of the heuristic
    """
    solver = optimizer_with_attributes(Mosek.Optimizer, "QUIET" => true)
    weight_lenght= .1 # trade off between minimizing length and rank
    random_seed = 6 # random seed used to initialize the heuristic
    # compute optimal piece-wise linear trajectory
    opt_trajectory = find_path_using_heuristic(n, constraint, edge_size, a, b,
        max_deg_uv, num_pieces, solver,
        weight_lenght,
        num_iterations,
        seed=random_seed)

    # note that for every piece, the range for t is [0,1]
    @show opt_trajectory
end

