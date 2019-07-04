using TrajectoryOptimization

function l1_solver(parameters)
    num_iter = parameters["num_iter"]
    N = parameters["N"]
    n = parameters["n"]
    m = parameters["m"]
    # History
    cost_history = zeros(num_iter, 3)
    constraint_violation = zeros(num_iter)
    # Model initialization
    if parameters["linearity"]
        model = TrajectoryOptimization.Model(scaled_cw_dynamics!, n, m)
    elseif !parameters["linearity"]
        model = TrajectoryOptimization.Model(scaled_non_linear_dynamics!, n, m)
    end
    # TVcost initialization
    ρ = parameters["ρ"]
    R = ρ * Matrix{Float64}(I, m, m)
    ν = parameters["ν0"]
    Y = parameters["Y0"]
    TVr = ν - ρ * Y
    TVcost = [TrajectoryOptimization.QuadraticCost(
                                        parameters["Q"],
                                        R,
                                        parameters["H"],
                                        parameters["q"],
                                        TVr[:,k],
                                        parameters["c"],
                                        parameters["Qf"],
                                        parameters["qf"],
                                        parameters["cf"]) for k=1:N-1]
    final_cost = QuadraticCost(
                    parameters["Q"],
                    parameters["R"],
                    parameters["H"],
                    parameters["q"],
                    parameters["r"],
                    parameters["c"],
                    parameters["Qf"],
                    parameters["qf"],
                    parameters["cf"])
    objective = TrajectoryOptimization.Objective([TVcost...,final_cost])
    # Problem initialization
    x0 = parameters["x0"]
    xf = parameters["xf"]
    tf = parameters["tf"]
    # u_min = parameters["u_min"]*[1.0, 0.2, 1.0] ###
    # u_max = parameters["u_max"]*[1.0, 0.2, 1.0] ###
    # bound_constraints = bound_constraint(n, m, u_min=u_min, u_max=u_max)
    bound_constraints = bound_constraint(n, m, u_min=parameters["u_min"], u_max=parameters["u_max"])
    # if parameters["linearity"] && parameters["using_final_constraints"]
    if parameters["using_final_constraints"]
        goal_constraints = goal_constraint(xf)
        problem_constraints = TrajectoryOptimization.ProblemConstraints([bound_constraints, goal_constraints], N)
    else
        problem_constraints = TrajectoryOptimization.ProblemConstraints([bound_constraints], N)
    end

    # println("x0 = ", x0)
    problem = Problem(model, objective, constraints=problem_constraints, x0=x0, N=N, tf=tf)

    initial_controls!(problem, parameters["U0"])
    # Solver initialization
    if parameters["using_constraints"]
        solver = AugmentedLagrangianSolver(problem)
        solver.opts.iterations = parameters["al_solver_iter"]
    else
        solver = iLQRSolver(problem)
        solver.opts.iterations = parameters["ilqr_solver_iter"]
    end

    X = parameters["X0"]
    U = parameters["U0"]
    Y = parameters["Y0"]
    ν = parameters["ν0"]
    for i=1:num_iter
        filename = "inter_" * "iter_" * string(i) * "_rho_" * string(parameters["ρ"]) * "_iter_" * string(parameters["num_iter"]) * "_Qf_" * string(parameters["Qf"][1,1]) * ".png"
        # Updates
        # parameters["ρ"] *= 1.05
        # ν .*= 1.05
        X, U = dynamics_update(X, U, Y, ν, parameters, problem, solver)
        Y = soft_threshold_update(U, Y, ν, parameters)
        ν = dual_update(U, Y, ν, parameters)
        # Costs computations
        cost = cost_function(X, U, parameters)
        lqr_cost = lqr_cost_function(X, U, Y, ν, parameters)
        augmented_cost = augmented_lagrangian(X, U, Y, ν, parameters)
        cost_history[i, :] = [cost, lqr_cost, augmented_cost]
        #Constraint violation
        uy = maximum(abs.(U .- Y))
        constraint_violation[i] = log(10, maximum(uy))
        println("uy, C, lqr C, augmented C = ", [uy, cost, lqr_cost, augmented_cost])
        println("i = ", i)
        if parameters["stage_plot"] && (i-1)%parameters["stage_plot_freq"] == 0
            filename = "lin_uncons_" * "rho_" * string(parameters["ρ"]) * "_iter_" * string(parameters["num_iter"]) * "_Qf_" * string(parameters["Qf"][1,1]) * "_stage_" * string(i) * ".png"
            save_results(X, U, Y, ν, cost_history, filename, parameters)
        end
    end
    return X, U, Y, ν, cost_history, constraint_violation
end



function dynamics_update(X, U, Y, ν, parameters, problem, solver)
    N = parameters["N"]
    m = parameters["m"]
    ρ = parameters["ρ"]
    # TVcost update
    TVr = ν - ρ * Y

    for k=1:N-1
        problem.obj.cost[k].R = ρ * Matrix{Float64}(I, m, m)
        problem.obj.cost[k].r = TVr[:,k]
    end

    # println("cost = ", cost(problem.obj, problem.X, problem.U))

    # Initial control update
    initial_controls!(problem, U)
    # Solving the problem
    rollout!(problem)
    J_prev = cost(problem.obj, problem.X, problem.U)
    # println("problem.X = ", problem.X)
    println("J_prev = ", J_prev)
    solve!(problem, solver)

    X = to_array(problem.X)
    U = to_array(problem.U)
    return X, U
end

function soft_threshold_update(U, Y, ν, parameters)
    N = parameters["N"]
    α = parameters["α"]
    ρ = parameters["ρ"]
    # println("α/ρ", α/ρ)
    for i=1:N-1
        Y[:,i] = soft_threshold(α/ρ, U[:,i] + ν[:,i]/ρ) ###
        # x = U[:,i] + ν[:,i]/ρ
        # Y[:,i] = prox_op_l2(2*α/ρ, U[:,i] + ν[:,i]/ρ)
        # Y[:,i] = prox_op_l1(α/ρ, U[:,i] + ν[:,i]/ρ)
    end
    return Y
end

function prox_op_l2(β, x)
    out = x * (1 - β / max(norm(x, 2), β))
    return out
end

function soft_threshold(τ, y)
    out = sign.(y) .* max.(abs.(y) .- τ, zeros(size(y)))
    return out
end

function dual_update(U, Y, ν, parameters)
    ρ = parameters["ρ"]
    ν += ρ * (U .- Y)
    return ν
end
