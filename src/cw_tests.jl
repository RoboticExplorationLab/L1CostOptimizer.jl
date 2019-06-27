using LinearAlgebra
using PyPlot
using TrajectoryOptimization
using BenchmarkTools

function define_parameters()
    N = 100
    n = 6
    m = 3
    tf = 50.0
    Δt = tf/N
    # x0 = [2000.0, 1500.0, 1000.0, 0.0, 0.0, 0.0]
    x_ref = [6378136.3 + 500*10^3, 0.0, 0.0, 0.0, 0.0, 0.0]
    x0 = [2000.0, 1500.0, 1000.0, 0.0, 0.0, 0.0]
    xf = zeros(n)

    params = Dict("num_iter" => 3000, # number of iteration of ADMM
                  "dynamics_update_iter" => 10, # number of iter in the dyanmics update solver
                  "N" => N, # horizon
                  "n"=> n, # state size
                  "m"=> m, # control size
                  "Δt" => Δt, # time step
                  "tf" => tf, # final time
                  "ρ" => 10^-7, # weighting parameter for augmented lagrangian
                  "penalty_rate" => 1.002, # rate at which we crank up the penalty term ρ
                  "α" => 10.0^-3, # weighting parameter
                  "X0" => zeros(n, N),#reshape(repeat(x0, outer = [N, 1]), (n, N)), # initial state trajectory
                  "U0" => zeros(m, N-1), # initial control trajectory
                  "ν0" => ones(m, N-1), # initial dual
                  "Y0" => zeros(m, N-1), # initial dummy
                  "x0" => x0, # initial state
                  "xf" => xf, # final state
                  "x_ref" => x_ref, # final state
                  "using_constraints" => true, # using constraints on control
                  "using_linear_dynamics" => true, # using constraints on control
                  "u_min" => -12.0, # lower bound on u
                  "u_max" => 12.0, # upper bound on u
                  "Q" => 0.0 * Matrix{Float64}(I, n, n), # Quadratic stage cost for states (n,n)
                  "R" => 1e-20 * Matrix{Float64}(I, m, m), # Quadratic stage cost for controls (m,m)
                  "H" => zeros(m, n), # Quadratic Cross-coupling for state and controls (m,n)
                  "q" => zeros(n), # Linear term on states (n,)
                  "r" => zeros(m), # Linear term on controls (m,)
                  "c" => 0.0, # constant term
                  "Qf" => 10.0^-4 * Matrix{Float64}(I, n, n), # Quadratic final cost for terminal state (n,n)
                  "qf" => zeros(n), # Linear term on terminal state (n,)
                  "cf" => 0.0, # constant term (terminal)
                  "μ" => 3986004.415*10^8, # Standard gravitational parameter m^3 s^-2
                  "a" => 6378136.3, # semi major axis m
                  "J2" => 1.08263e-3, # spherical harmonic coefficient
                  "Cd" => 2.2, # Drag coefficient
                  "A" => 0.04^2, # ChipSat board area m^2
                  "mass" => 2.5, # Mass of the vehicle kg
                  "ω" => [0, 0, 2*pi/((23*60 + 56)*60 + 4)], # Earth's rotational speed vector rad s^-1
                   )
    return params
end

function cw_dynamics!(ẋ, x, u, params::Dict{String,Any})
    n_cst = sqrt(params["μ"]/params["a"]^3)
    ẋ[1] = x[4]
    ẋ[2] = x[5]
    ẋ[3] = x[6]
    ẋ[4] = 3*n_cst^2*x[1] + 2*n_cst*x[5] + u[1]
    ẋ[5] = -2*n_cst*x[4] + u[2]
    ẋ[6] = -n_cst^2*x[3] + u[3]
end

function non_linear_dynamics!(ẋ, x, u, params::Dict{String,Any})
    # This function gives the dynamics of a chipsat including the J2
    # gravitational term and atmospheric drag.

    # x = [position; velocity] is the spacecraft state vector in MKS units
    # u is a scalar control input that is equal to the cosine of the angle
    # between the spacecraft normal vector and the velocity vector. It should
    # be between 0 and 1, and modulates the drag on the spacecraft.

    # Constants
    μ = params["μ"]
    a = params["a"]
    J2 = params["J2"]
    A = params["A"]
    ω = params["ω"]
    Cd = params["Cd"]
    x_ref = params["x_ref"]

    r = x[1:3] + x_ref[1:3] # position in meters
    v = x[4:6] # velocity in m/s
    rmag = norm(r)
    vmag = norm(v)

    alt = rmag - a
    density = atmospheric_density(alt)

    a_spherical = -r * (μ/rmag^3)
    J2_term = [(1-5*(r[3]/rmag)^2)*r[1], (1-5*(r[3]/rmag)^2)*r[2], (3-5*(r[3]/rmag)^2)*r[3]]
    a_J2 = -(3/2) * J2 * (μ/rmag^5) *a^2 * J2_term
    a_grav = a_spherical + a_J2;

    v_rel = v + cross(ω, r)
    a_drag = -0.5 * density * A * Cd * norm(v_rel) * v_rel
    # a = a_grav + a_drag + u; # full drag (max drag: satellite facing air flow)
    # a = a_grav + u*a_drag;
    ẋ[1:3] = x[4:6]
    ẋ[4:6] = a_grav + a_drag + u
    return ẋ
end

function atmospheric_drag(x, params)
    # Constants
    a = params["a"]
    A = params["A"]
    mass = params["mass"]
    ω = params["ω"]
    Cd = params["Cd"]
    # area = (.6)*.1*.1 + (1.2)*.35*.1; %Average surface area assuming tumbling
    # Calculate Altitude
    r = x[1:3]
    v = x[4:6]
    alt = norm(r) - a
    v_rel = v + cross(ω, r)
    # Calculate Atmospheric Density
    density = atmospheric_density(alt)
    # Calculate Acceleration
    a = -0.5 * (A/mass) * density * Cd * norm(v_rel) * v_rel
    # Return state derivative
    x_dot = vcat(v, a)
    return x_dot
end

function atmospheric_density(alt)
    # Data is from SMAD tables for density vs. altitude
    # Density is kg/m^3

    #  h = [100 150 200 250 300 350 400 450 500]'; %Altitude in km.
    #  rho_min = [4.61e-7 1.65e-9 1.78e-10 3.35e-11 8.19e-12 2.34e-12 7.32e-13 2.47e-13 8.98e-14]';
    #  rho_avg = [4.79e-7 1.81e-9 2.53e-10 6.24e-11 1.95e-11 6.98e-12 2.72e-12 1.13e-12 4.89e-13]';
    #  rho_max = [5.10e-7 2.04e-9 3.52e-10 1.06e-10 3.96e-11 1.68e-11 7.55e-12 3.61e-12 1.80e-12]';

    # Fits to SMAD data (for h in meters)
    density = 9.201e-05*exp(-5.301e-05*alt) # min
    # rho = 5.987e-05*exp(-4.836e-05*h); # avg
    # rho = 4.393e-05*exp(-4.467e-05*h); # max
    return density
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
    if params["using_linear_dynamics"]
        function _cw_dynamics!(ẋ,x,u)
           cw_dynamics!(ẋ, x, u, params)
        end
        model = TrajectoryOptimization.Model(_cw_dynamics!, n, m)
    else
        function _non_linear_dynamics!(ẋ,x,u)
           non_linear_dynamics!(ẋ, x, u, params)
        end
        model = TrajectoryOptimization.Model(_non_linear_dynamics!, n, m)
    end
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
    if params["using_constraints"]
        solver = AugmentedLagrangianSolver(problem)
    else
        solver = iLQRSolver(problem)
    end
    solver.opts.iterations = params["dynamics_update_iter"]
    # solver = iLQRSolver(problem)

    X = params["X0"]
    U = params["U0"]
    Y = params["Y0"]
    ν = params["ν0"]
    for i=1:num_iter
        # Updates
        params["ρ"] *= params["penalty_rate"]
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

function save_results(X, U, Y, filename, params)
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
    params = define_parameters()
    constraints_params = [false]
    dynamics_params = [false]
    N = params["N"]
    n = params["n"]
    m = params["m"]
    ρ_start = params["ρ"]
    logs = Dict("X_log" => zeros(4, n, N),
        "U_log" => zeros(4, m, N-1),
        "Y_log" => zeros(4, m, N-1),
        "ν_log" => zeros(4, m, N-1))
    for constraints_param in constraints_params
        for dynamics_param in dynamics_params
            params["using_constraints"] = constraints_param
            params["using_linear_dynamics"] = dynamics_param
            X, U, Y, ν, cost_history, constraint_violation = ADMM(params)
            filename = "rdv_const_" * string(constraints_param) * "_linear_dyn_" * string(dynamics_param)
            filename *= "_rho_start_" * string(ρ_start) * "_rho_rate_" * string(params["penalty_rate"])
            filename *= "_dynamics_iter_" * string(params["dynamics_update_iter"]) * "_ADMM_iter_" * string(params["num_iter"]) * ".png"
            save_results(X, U, Y, filename, params)
            k = parse(Int, string(Int(constraints_param)) * string(Int(dynamics_param)), base=2) + 1
            println(k, "**********************")
            logs["X_log"][k,:,:] = X
            logs["U_log"][k,:,:] = U
            logs["Y_log"][k,:,:] = Y
            logs["ν_log"][k,:,:] = ν
        end
    end
    return logs
end

# @time logs = run_experiments() # basic
# @btime logs = run_experiments() # multiple sampling
# @profiler logs = run_experiments() # depth analysis
logs = run_experiments(params)
