using TrajectoryOptimization

function ADMM(params)
    num_iter = params["num_iter"]
    N = params["N"]
    n = params["n"]
    m = params["m"]
    # History
    cost_history = zeros(num_iter, 3)
    constraint_violation = zeros(num_iter)
    # Model initialization
    model = TrajectoryOptimization.Model(cw_dynamics!, n, m)
    # TVcost initialization
    Q = params["Q"]
    ρ = params["ρ"]
    R = ρ * Matrix{Float64}(I, m, m)
    H = params["H"]
    q = params["q"]
    ν = params["ν0"]
    Y = params["Y0"]
    TVr = ν - ρ * Y
    c = params["c"]
    Qf = params["Qf"]
    qf = params["qf"]
    cf = params["cf"]
    TVcost = [TrajectoryOptimization.QuadraticCost(Q,R,H,q,TVr[:,k],c,Qf,qf,cf) for k=1:N-1]
    Q0 = params["Q"]
    R0 = params["R"]
    H0 = params["H"]
    q0 = params["q"]
    r0 = params["r"]
    c0 = params["c"]
    Qf0 = params["Qf"]
    qf0 = params["qf"]
    cf0 = params["cf"]
    final_cost = QuadraticCost(Q0,R0,H0,q0,r0,c0,Qf0,qf0,cf0)
    objective = TrajectoryOptimization.Objective([TVcost...,final_cost])
    # Problem initialization
    x0 = params["x0"]
    xf = params["xf"]
    tf = params["tf"]
    bound_constraints = bound_constraint(n, m, u_min=params["u_min"], u_max=params["u_max"])
    goal_constraints = goal_constraint(xf)
    problem_constraints = TrajectoryOptimization.ProblemConstraints([bound_constraints, goal_constraints], N)
    problem = Problem(model, objective, constraints=problem_constraints, x0=x0, N=N, tf=tf)
    initial_controls!(problem, params["U0"])
    # Solver initialization
    # solver = AugmentedLagrangianSolver(problem)
    solver = iLQRSolver(problem)
    solver.opts.iterations = 1
    # solver = iLQRSolver(problem)

    X = params["X0"]
    U = params["U0"]
    Y = params["Y0"]
    ν = params["ν0"]
    for i=1:num_iter
        # Updates
        params["ρ"] *= 1.01
        X, U = dynamics_update(X, U, Y, ν, params, problem, solver)
        Y = soft_threshold_update(U, Y, ν, params)
        ν = dual_update(U, Y, ν, params)
        # Costs computations
        cost = cost_function(X, U, params)
        lqr_cost = lqr_cost_function(X, U, Y, ν, params)
        augmented_cost = augmented_lagrangian(X, U, Y, ν, params)
        cost_history[i, :] = [cost, lqr_cost, augmented_cost]
        #Constraint violation
        uy = maximum(abs.(U .- Y))
        constraint_violation[i] = log(10, maximum(uy))
        println("uy, C, lqr C, augmented C = ", [uy, cost, lqr_cost, augmented_cost])
        println("ρ = ", params["ρ"])
        println("i = ", i)
    end
    return X, U, Y, ν, cost_history, constraint_violation
end



function dynamics_update(X, U, Y, ν, params, problem, solver)
    N = params["N"]
    m = params["m"]
    ρ = params["ρ"]
    # TVcost update
    TVr = ν - ρ * Y
    for  k=1:N-1
        problem.obj.cost[k].R = ρ * Matrix{Float64}(I, m, m)
        problem.obj.cost[k].r = TVr[:,k]
    end
    # Initial control update
    initial_controls!(problem, U)
    # Solving the problem
    solve!(problem, solver)
    X = to_array(problem.X)
    U = to_array(problem.U)
    return X, U
end

function soft_threshold_update(U, Y, ν, params)
    N = params["N"]
    α = params["α"]
    ρ = params["ρ"]
    # println("α/ρ", α/ρ)
    for i=1:N-1
        Y[:,i] = soft_threshold(α/ρ, U[:,i] + ν[:,i]/ρ)
    end
    return Y
end

function soft_threshold(τ, y)
    out = sign.(y) .* max.(abs.(y) .- τ, zeros(size(y)))
    return out
end

function dual_update(U, Y, ν, params)
    ρ = params["ρ"]
    ν += ρ * (U .- Y)
    return ν
end
