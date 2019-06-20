using LinearAlgebra
using PyPlot
using TrajectoryOptimization
using BenchmarkTools
# using constrinat or not
# using non linear dynamics or not
# timing the code

N = 100
n = 6
m = 3
tf = 50.0
Δt = tf/N
x0 = [2000.0, 1500.0, 1000.0, 0.0, 0.0, 0.0]
xf = zeros(n)

params = Dict("num_iter" => 400, # number of iteration of ADMM
              "N" => N, # horizon
              "n"=> n, # state size
              "m"=> m, # control size
              "Δt" => Δt, # time step
              "tf" => tf, # final time
              "ρ" => 1.0, # weighting parameter for augmented lagrangian
              "α" => 100.0, # weighting parameter
              "X0" => zeros(n, N),#reshape(repeat(x0, outer = [N, 1]), (n, N)), # initial state trajectory
              "U0" => zeros(m, N-1), # initial control trajectory
              "ν0" => ones(m, N-1), # initial dual
              "Y0" => zeros(m, N-1), # initial dummy
              "x0" => x0, # initial state
              "xf" => xf, # final state
              "using_constraints" => true, # using constraints on control
              "u_min" => -10.0, # lower bound on u
              "u_max" => 10.0, # upper bound on u
              "Q" => 0.0 * Matrix{Float64}(I, n, n), # Quadratic stage cost for states (n,n)
              "R" => 1e-20 * Matrix{Float64}(I, m, m), # Quadratic stage cost for controls (m,m)
              "H" => zeros(m, n), # Quadratic Cross-coupling for state and controls (m,n)
              "q" => zeros(n), # Linear term on states (n,)
              "r" => zeros(m), # Linear term on controls (m,)
              "c" => 0.0, # constant term
              "Qf" => 10.0 * Matrix{Float64}(I, n, n), # Quadratic final cost for terminal state (n,n)
              "qf" => zeros(n), # Linear term on terminal state (n,)
              "cf" => 0.0, # constant term (terminal)
              "μ" => 3.99*10^14, # Standard gravitational parameter m^3 s^-2
              "a" => 6731.0*10^3, # semi major axis m
               )

function cw_dynamics!(ẋ, x, u, params)
   n_cst = sqrt(params["μ"]/params["a"]^3)
   ẋ[1] = x[4]
   ẋ[2] = x[5]
   ẋ[3] = x[6]
   ẋ[4] = 3*n_cst^2*x[1] + 2*n_cst*x[5] + u[1]
   ẋ[5] = -2*n_cst*x[4] + u[2]
   ẋ[6] = -n_cst^2*x[3] + u[3]
end

function cw_dynamics!(ẋ,x,u)
   cw_dynamics!(ẋ, x, u, params)
end

function non_linear_dynamics!(ẋ, x, u, params)
   n_cst = sqrt(params["μ"]/params["a"]^3)
   ẋ[1] = x[4]
   ẋ[2] = x[5]
   ẋ[3] = x[6]
   ẋ[4] = 3*n_cst^2*x[1] + 2*n_cst*x[5] + u[1]
   ẋ[5] = -2*n_cst*x[4] + u[2]
   ẋ[6] = -n_cst^2*x[3] + u[3]
end

function non_linear_dynamics!(ẋ,x,u)
   non_linear_dynamics!(ẋ, x, u, params)
end

function cost_function(X, U, params)
    N = params["N"]
    Qf = params["Qf"]
    Q = params["Q"]
    α = params["α"]
    cost = 0
    for i=1:N-1
        cost += 1/2 * X[:,i]'*Q*X[:,i] + α * norm(U[:,i], 1)
    end
    cost += 1/2 * X[:,N]'*Qf*X[:,N]
    return cost
end

function lqr_cost_function(X, U, Y, ν, params)
    N = params["N"]
    Qf = params["Qf"]
    Q = params["Q"]
    α = params["α"]
    ρ = params["ρ"]
    cost = 0
    for i=1:N-1
        cost += 1/2 * X[:,i]'*Q*X[:,i] + ρ * U[:,i]'*U[:,i] + U[:,i]'*(ν[:,i] - ρ * Y[:,i])
    end
    cost += 1/2 * X[:,N]'*Qf*X[:,N]
    return cost
end

function augmented_lagrangian(X, U, Y, ν, params)
    N = params["N"]
    ρ = params["ρ"]
    α = params["α"]
    lagrangian = cost_function(X, Y, params)
    for i=1:N-1
        lagrangian += ν[:,i]' * (U[:,i] - Y[:,i])
        lagrangian += ρ / 2 * norm(U[:,i] - Y[:,i], 2)^2
    end
    return lagrangian
end

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
    objective = TrajectoryOptimization.ObjectiveNew([TVcost...,final_cost])
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
    solver = AugmentedLagrangianSolver(problem)
    # solver = iLQRSolver(problem)
    solver.opts.iterations = 10
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
        println("uy, C, lqr C, augmented C", [uy, cost, lqr_cost, augmented_cost])
        println("ρ", params["ρ"])
        println("i", i)
    end
    return X, U, Y, ν, cost_history, constraint_violation
end



function dynamics_update(X, U, Y, ν, params, problem, solver)
    N = params["N"]
    n = params["n"]
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
    N = params["N"]
    m = params["m"]
    ρ = params["ρ"]
    ν += ρ * (U .- Y)
    return ν
end

function save_results(X, U, Y, filename)
    num_iter = params["num_iter"]
    N = params["N"]
    n = params["n"]
    Δt = params["Δt"]
    T = [Δt * i for i=1:N]

    fig = figure(figsize=(9,3))
    subplot(1, 3, 1)
    step(T, X[1,:], color="blue", linewidth=1.0, linestyle="-", label=L"$x_1$")
    step(T, X[2,:], color="red", linewidth=1.0, linestyle="-", label=L"$x_2$")
    step(T, X[3,:], color="green", linewidth=1.0, linestyle="-", label=L"$x_3$")
    title(L"Positions.")
    grid("on")
    xlabel(L"Time in $s$")
    ylabel(L"Position in $m$")
    legend()

    subplot(1, 3, 2)
    step(T, X[4,:], color="blue", linewidth=1.0, linestyle="-", label=L"$\dot{x}_1$")
    step(T, X[5,:], color="red", linewidth=1.0, linestyle="-", label=L"$\dot{x}_2$")
    step(T, X[6,:], color="green", linewidth=1.0, linestyle="-", label=L"$\dot{x}_3$")
    title(L"Velocities.")
    grid("on")
    xlabel(L"Time in $s$")
    ylabel(L"Velocity in $m/s$")
    legend()

    subplot(1, 3, 3)
    step(T[1:end-1], U[1,:], color="blue", linewidth=1.0, linestyle="-", label=L"$u_1$")
    step(T[1:end-1], U[2,:], color="red", linewidth=1.0, linestyle="-", label=L"$u_2$")
    step(T[1:end-1], U[3,:], color="green", linewidth=1.0, linestyle="-", label=L"$u_3$")
    title(L"Controls.")
    grid("on")
    xlabel(L"Time in $s$")
    ylabel(L"Controls in $N$")
    legend()

    tight_layout()
    savefig("results/" * filename, format="png", dpi=1000)
    # fig.close()
    return
end

function run_experiments(params)
    N = params["N"]
    n = params["n"]
    m = params["m"]
    P = [params["ρ"]*10.0^i for i=-1:-1]
    logs = Dict("X_log" => zeros(length(P), n, N),
        "U_log" => zeros(length(P), m, N-1),
        "Y_log" => zeros(length(P), m, N-1),
        "ν_log" => zeros(length(P), m, N-1))
    for k=1:length(P)
        params["ρ"] = P[k]
        X, U, Y, ν, cost_history, constraint_violation = ADMM(params)
        logs["X_log"][k,:,:] = X
        logs["U_log"][k,:,:] = U
        logs["Y_log"][k,:,:] = Y
        logs["ν_log"][k,:,:] = ν
        filename = "cw_constrained_" * string(P[k]) * ".png"
        save_results(X, U, Y, filename)
    end
    return logs
end

# @profiler logs = run_experiments()
logs = run_experiments(params)
