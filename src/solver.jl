function l1_solver(parameters)
    # Solves the trjaectory optimization problem for which the cost function
    # contains a L1-norm cost term on the control.
    num_iter = parameters["num_iter"]
    N = parameters["N"]
    n = parameters["n"]
    m = parameters["m"]
    # History
    cost_history = zeros(num_iter)
    constraint_violation = zeros(num_iter, 6)
    optimality_criterion = zeros(num_iter)

    # Dynamics model initialization
    if parameters["linearity"]
        function scaled_cw_dynamics!(ẋ,x,u)
            scaled_cw_dynamics(ẋ, x, u, parameters)
        end
        model = TrajectoryOptimization.Model(scaled_cw_dynamics!, n, m)
        discretized_model = midpoint(model)
    elseif !parameters["linearity"]
        function scaled_non_linear_dynamics!(ẋ,x,u)
            scaled_non_linear_dynamics(ẋ, x, u, parameters)
        end
        model = TrajectoryOptimization.Model(scaled_non_linear_dynamics!, n, m)
        discretized_model = rk3(model)
    end

    # Time Varying cost initialization
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
                                        parameters["c"]) for k=1:N-1]
    final_cost = QuadraticCost(
                    parameters["Qf"],
                    parameters["qf"],
                    parameters["cf"])
    objective = TrajectoryOptimization.Objective([TVcost...,final_cost])

    # Problem initialization
    x0 = parameters["x0"]
    xf = parameters["xf"]
    tf = parameters["tf"]
    bound_constraints = BoundConstraint(n, m,
        u_min=parameters["u_min"], u_max=parameters["u_max"])
    if parameters["using_final_constraints"]
        goal_constraints = goal_constraint(xf)
        problem_constraints = TrajectoryOptimization.Constraints(
            [bound_constraints, goal_constraints], N)
    else
        problem_constraints = TrajectoryOptimization.Constraints(
            [bound_constraints], N)
    end
    problem = Problem(discretized_model, objective,
        constraints=problem_constraints, x0=x0, N=N, tf=tf)
    initial_controls!(problem, parameters["U0"])

    # Solver initialization
    opts_ilqr = iLQRSolverOptions{Float64}(gradient_type=:ℓ2)
    if parameters["using_constraints"]
        opts_al = AugmentedLagrangianSolverOptions{Float64}(opts_uncon=opts_ilqr)
        solver = AugmentedLagrangianSolver(problem, opts_al)
        solver.opts.iterations = parameters["al_solver_iter"]
    else
        solver = iLQRSolver(problem, opts_ilqr)
        solver.opts.iterations = parameters["ilqr_solver_iter"]
    end

    X = parameters["X0"]
    U = parameters["U0"]
    Y = parameters["Y0"]
    ν = parameters["ν0"]
    i = 1 # ADMM iteration counter
    num_lqr = 0 # Counter for the total number of LQR passes
    while i <= num_iter
        parameters["ρ"] = min(parameters["ρ"]*1.00, 1e5) ###
        # State, Control, Dual and Dummy variables updates
        X, U = dynamics_update(X, U, Y, ν, parameters, problem, solver)
        Y = soft_threshold_update(U, Y, ν, parameters)
        ν = dual_update(U, Y, ν, parameters)
        # At the first time step we rescale the dual variables so that they reach
        # a maximum absolute value of `α`.
        # Indeed, the algorithm enforces |dual variable| <= α, while α is not
        # reached the algorithm is not converging towards a bang-off-bang behavior.
        # Using this rescaling cuts this first (useless) phase so the convergence
        # start after the first iteration.
        if i == 1
            scale_ν = parameters["α"] ./ [norm(ν[i, :], Inf) for i=1:m]
            ν = ν .* scale_ν
        end
        num_lqr += solver.stats[:iterations] # adds the number of LQR passes
        optimality_criterion[i] = compute_optimality_criterion(U, Y)
        if !parameters["timing"]
            # Costs computations
            cost_history[i] = cost_function(X, U, parameters)
            #Constraint violation
            println("opt. crit.,  cost = ", [optimality_criterion[i], cost_history[i]])
            if parameters["stage_plot"] && (i-1)%parameters["stage_plot_freq"] == 0
                filename = "inter_" * "rho_" * string(parameters["ρ"]) * "_iter_"
                filename *= string(parameters["num_iter"]) * "_Qf_"
                filename *= string(parameters["Qf"][1,1]) * "_stage_" * string(i)
                save_results(X, U, Y, ν, cost_history, optimality_criterion,
                    filename, num_iter, parameters)
                save_results_pyplot(X, U, Y, ν, cost_history, optimality_criterion,
                    filename, num_iter, parameters)
            end
        end
        if optimality_criterion[i] <= parameters["stopping_criterion"]
            break
        end
        i += 1
    end
    println("Number of LQR passes = ", num_lqr)
    return X, U, Y, ν, cost_history, optimality_criterion, i-1
end

function compute_optimality_criterion(U, Y)
    # Compute the optimality criterion ||U-Y||_2^2/dim(U)
    return norm(U .- Y) / length(U)
end

function dynamics_update(X, U, Y, ν, parameters, problem, solver)
    # We solve a trajectory optimization problem with smooth cost function.
    N = parameters["N"]
    m = parameters["m"]
    ρ = parameters["ρ"]
    # TVcost update
    TVr = ν - ρ * Y

    for k=1:N-1
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

function soft_threshold_update(U, Y, ν, parameters)
    N = parameters["N"]
    α = parameters["α"]
    ρ = parameters["ρ"]
    for i=1:N-1
        m = parameters["m"]
        Y[:,i] = soft_threshold(α/ρ, U[:,i] + ν[:,i]/ρ) ###
    end
    return Y
end

function prox_op_l2(β, x)
    # Prox operator for L2-norm
    out = x * (1 - β / max(norm(x, 2), β))
    return out
end

function soft_threshold(τ, y)
    # Soft threshold operator
    out = sign.(y) .* max.(abs.(y) .- τ, zeros(size(y)))
    return out
end

function dual_update(U, Y, ν, parameters)
    ρ = parameters["ρ"]
    ν += ρ * (U .- Y)
    return ν
end
