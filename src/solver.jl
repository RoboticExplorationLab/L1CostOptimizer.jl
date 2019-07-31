using TrajectoryOptimization

function l1_solver(parameters)
    num_iter = parameters["num_iter"]
    N = parameters["N"]
    n = parameters["n"]
    m = parameters["m"]
    # History
    cost_history = zeros(num_iter, 3)
    constraint_violation = zeros(num_iter, 6)
    optimality_criterion = zeros(num_iter)

    # Model initialization
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
    bound_constraints = BoundConstraint(n, m, u_min=parameters["u_min"], u_max=parameters["u_max"])
    if parameters["using_final_constraints"]
        goal_constraints = goal_constraint(xf)
        problem_constraints = TrajectoryOptimization.Constraints([bound_constraints, goal_constraints], N)
    else
        problem_constraints = TrajectoryOptimization.Constraints([bound_constraints], N)
    end
    problem = Problem(discretized_model, objective, constraints=problem_constraints, x0=x0, N=N, tf=tf)
    initial_controls!(problem, parameters["U0"])

    # Solver initialization
    opts_ilqr = iLQRSolverOptions{Float64}(gradient_type=:ℓ2)
    if parameters["using_constraints"]
        opts_al = AugmentedLagrangianSolverOptions{Float64}(opts_uncon=opts_ilqr)
        solver = AugmentedLagrangianSolver(problem, opts_al)
        # solver = AugmentedLagrangianSolver(problem)
        solver.opts.iterations = parameters["al_solver_iter"]
    else
        solver = iLQRSolver(problem, opts_ilqr)
        # solver = iLQRSolver(problem)
        solver.opts.iterations = parameters["ilqr_solver_iter"]
    end

    X = parameters["X0"]
    U = parameters["U0"]
    Y = parameters["Y0"]
    ν = parameters["ν0"]
    i = 1
    num_lqr = 0
    while i <= num_iter
        parameters["ρ"] = min(parameters["ρ"]*1.00, 1e5) ###
        # Updates
        # parameters["scale_y"] = 1 + 0.90*(parameters["scale_y"] - 1)
        X, U = dynamics_update(X, U, Y, ν, parameters, problem, solver)
        Y = soft_threshold_update(U, Y, ν, parameters)
        ν = dual_update(U, Y, ν, parameters) ###
        if i == 1
            scale_ν = parameters["α"] ./ [norm(ν[i, :], Inf) for i=1:m]
            ν = ν .* scale_ν
        # elseif i <= 30
        #     scale_ν = parameters["α"] / norm(ν, Inf)
        #     ν = ν .* scale_ν
        end
        num_lqr += solver.stats[:iterations]
        optimality_criterion[i] = compute_optimality_criterion(U, Y)
        if !parameters["timing"]
            constraint_violation[i,:] = compute_constraint_violation(U, Y, parameters)
            # Costs computations
            cost = cost_function(X, U, parameters)
            lqr_cost = lqr_cost_function(X, U, Y, ν, parameters)
            augmented_cost = augmented_lagrangian(X, U, Y, ν, parameters)
            cost_history[i, :] = [cost, lqr_cost, augmented_cost]
            #Constraint violation
            uy = maximum(abs.(U .- Y))
            println("uy, C, lqr C, augmented C = ", [uy, cost, lqr_cost, augmented_cost])
            if parameters["stage_plot"] && (i-1)%parameters["stage_plot_freq"] == 0
                filename = "inter_" * "rho_" * string(parameters["ρ"]) * "_iter_" * string(parameters["num_iter"]) * "_Qf_" * string(parameters["Qf"][1,1]) * "_stage_" * string(i)
                save_results(X, U, Y, ν, cost_history, constraint_violation, optimality_criterion, filename, num_iter, parameters)
                save_results_pyplot(X, U, Y, ν, cost_history, constraint_violation, optimality_criterion, filename, num_iter, parameters)
            end
        end
        i += 1
        if optimality_criterion[i-1] <= parameters["stopping_criterion"]
            println("break at ", i-1)
            break
        end
    end
    println("Number of LQR passes = ", num_lqr)
    return X, U, Y, ν, cost_history, constraint_violation, optimality_criterion, i-1
end

function compute_optimality_criterion(U, Y)
    return norm(U .- Y) / length(U)
end

function compute_lagrangian_gradient(X, U, Y, ν, parameters)
    # We compute the gradient of the scaled version of the Lagrangian
    N = parameters["N"]
    n = parameters["n"]
    m = parameters["m"]
    ρ = parameters["ρ"]
    gradient = zeros(N*n+(N-1)*2*m) #[x1, u1, y1, x2, ...., xn]
    # Objective function gradient
    # Stage cost
    α = parameters["α"]
    ρ = parameters["ρ"]
    R = ρ * Matrix{Float64}(I, m, m)
    TVr = ν - ρ * Y
    Q = parameters["Q"]
    H = parameters["H"]
    q = parameters["q"]
    for k=1:N-1
        # gradient wrt xk
        gradient[(k-1)*(n+2*m)+1:(k-1)*(n+2*m)+n] += q + (X[:,k]'*(Q + Q')./2 + Y[:,k]'*H)[1,:]
        # gradient wrt yk
        gradient[k*(n+2*m)-m+1:k*(n+2*m)] += TVr[:,k] + (Y[:,k]'*(R + R')./2 + X[:,k]'*H')[1,:]
        # L1 cost gradient wrt yk
        # we approximate the derivative of the L1 norm using sign(max(|x|-ϵ, 0))*sign(x)
        # we get                -------- =1
        #                       |
        #            -ϵ ------- ϵ
        #             |
        # -1 = --------
        ϵ = 1e-3
        gradient[k*(n+2*m)-m+1:k*(n+2*m)] += α*sign.(max.(abs.(Y[:,k]).-ϵ, 0)).*sign.(Y[:,k])
    end
    # Final cost
    qf = parameters["qf"]
    Qf = parameters["Qf"]
    gradient[end-n+1:end] += qf + (X[:,end]'*(Qf + Qf')./2)[1,:]

    # Gradient of terms in (U-Y)
    for k=1:N-1
        # gradient wrt uk
        gradient[k*(n+2*m)-2*m+1:k*(n+2*m)-m] += ν[:,k] + ρ*(U[:,k] .- Y[:,k])
        # gradient wrt yk
        gradient[k*(n+2*m)-m+1:k*(n+2*m)] += -ν[:,k] + ρ*(U[:,k] .- Y[:,k])
    end
    # println(norm(gradient, 2)/length(gradient))
    return norm(gradient, 2)/length(gradient)
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
        # Y[:,i] = parameters["scale_y"]*soft_threshold(α/ρ, U[:,i] + ν[:,i]/ρ) ###
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
    # ν += ρ * (U .- 1.0/parameters["scale_y"]*Y) ###
    ν += ρ * (U .- Y) ###
    return ν
end


#
# function accelerated_l1_solver(parameters)
#     num_iter = parameters["num_iter"]
#     N = parameters["N"]
#     n = parameters["n"]
#     m = parameters["m"]
#     # History
#     cost_history = zeros(num_iter, 3)
#     constraint_violation = zeros(num_iter, 3)
#     # Model initialization
#     if parameters["linearity"]
#         model = TrajectoryOptimization.Model(scaled_cw_dynamics!, n, m)
#     elseif !parameters["linearity"]
#         model = TrajectoryOptimization.Model(scaled_non_linear_dynamics!, n, m)
#     end
#     # TVcost initialization
#     ρ = parameters["ρ"]
#     R = ρ * Matrix{Float64}(I, m, m)
#     ν = parameters["ν0"]
#     Y = parameters["Y0"]
#     ###
#     A = zeros(num_iter+2)
#     ν_hat = copy(ν)
#     Y_hat = copy(Y)
#     ###
#     TVr = ν_hat - ρ * Y_hat
#     TVcost = [TrajectoryOptimization.QuadraticCost(
#                                         parameters["Q"],
#                                         R,
#                                         parameters["H"],
#                                         parameters["q"],
#                                         TVr[:,k],
#                                         parameters["c"],
#                                         parameters["Qf"],
#                                         parameters["qf"],
#                                         parameters["cf"]) for k=1:N-1]
#     final_cost = QuadraticCost(
#                     parameters["Q"],
#                     parameters["R"],
#                     parameters["H"],
#                     parameters["q"],
#                     parameters["r"],
#                     parameters["c"],
#                     parameters["Qf"],
#                     parameters["qf"],
#                     parameters["cf"])
#     objective = TrajectoryOptimization.Objective([TVcost...,final_cost])
#     # Problem initialization
#     x0 = parameters["x0"]
#     xf = parameters["xf"]
#     tf = parameters["tf"]
#     # u_min = parameters["u_min"]*[1.0, 0.2, 1.0] ###
#     # u_max = parameters["u_max"]*[1.0, 0.2, 1.0] ###
#     # bound_constraints = bound_constraint(n, m, u_min=u_min, u_max=u_max)
#     bound_constraints = bound_constraint(n, m, u_min=parameters["u_min"], u_max=parameters["u_max"])
#     # if parameters["linearity"] && parameters["using_final_constraints"]
#     if parameters["using_final_constraints"]
#         goal_constraints = goal_constraint(xf)
#         problem_constraints = TrajectoryOptimization.ProblemConstraints([bound_constraints, goal_constraints], N)
#     else
#         problem_constraints = TrajectoryOptimization.ProblemConstraints([bound_constraints], N)
#     end
#
#     # println("x0 = ", x0)
#     problem = Problem(model, objective, constraints=problem_constraints, x0=x0, N=N, tf=tf)
#
#     initial_controls!(problem, parameters["U0"])
#     # Solver initialization
#     if parameters["using_constraints"]
#         solver = AugmentedLagrangianSolver(problem)
#         solver.opts.iterations = parameters["al_solver_iter"]
#     else
#         solver = iLQRSolver(problem)
#         solver.opts.iterations = parameters["ilqr_solver_iter"]
#     end
#
#     X = parameters["X0"]
#     X_hat = copy(X)
#     U = parameters["U0"]
#     U_hat = copy(U)
#     Y = parameters["Y0"]
#     Y_hat = copy(Y)
#     ν = parameters["ν0"]
#     ν_hat = copy(ν)
#     for i=1:num_iter
#         # println("i = ", i)
#         # Updates
#         X, U = dynamics_update(X, U, Y_hat, ν_hat, parameters, problem, solver)
#         Y = soft_threshold_update(U, Y, ν_hat, parameters)
#         ν = dual_update(U, Y, ν_hat, parameters) ###
#         A[i+2] = (1+sqrt(1+4*A[i+1])) / 2
#         ν_hat = dual_hat_update(ν, A, parameters)
#         Y_hat = Y_hat_update(Y, ν_hat, parameters)
#         if !parameters["timing"]
#             # Costs computations
#             cost = cost_function(X, U, parameters)
#             lqr_cost = lqr_cost_function(X, U, Y, ν, parameters)
#             augmented_cost = augmented_lagrangian(X, U, Y, ν, parameters)
#             cost_history[i, :] = [cost, lqr_cost, augmented_cost]
#             #Constraint violation
#             uy = maximum(abs.(U .- Y))
#             constraint_violation[i] = log(10, maximum(uy))
#             println("uy, C, lqr C, augmented C = ", [uy, cost, lqr_cost, augmented_cost])
#             if parameters["stage_plot"] && (i-1)%parameters["stage_plot_freq"] == 0
#                 filename = "inter_" * "rho_" * string(parameters["ρ"]) * "_iter_" * string(parameters["num_iter"]) * "_Qf_" * string(parameters["Qf"][1,1]) * "_stage_" * string(i)
#                 save_results(X, U, Y, ν, cost_history, filename, parameters)
#             end
#         end
#     end
#     return X, U, Y, ν, cost_history, constraint_violation
# end
