using BenchmarkTools
using LinearAlgebra
using TrajectoryOptimization
using PyPlot

include("cost.jl")
include("dynamics.jl")
include("experiments.jl")
include("experiment_parameters.jl")
include("parameters.jl")
include("solver.jl")
include("utils.jl")

function lin_cons_example(ρ)
    lin_cons_parameters = define_lin_constrained_parameters()
    non_lin_parameters = define_non_lin_parameters()
    lin_cons_parameters["ρ"] = ρ

    # Define initial state
    non_lin_parameters["x0"] = lin_cons_parameters["x0_drift"]
    println("x0 = ", non_lin_parameters["x0"])

    # Propagate the non linear dynamics forward without control
    tf_drift = lin_cons_parameters["tf_drift"]
    N_drift = lin_cons_parameters["N_drift"]
    X_history = drifting_simulation(N_drift, tf_drift, non_lin_parameters)
    U_history = zeros(3, N_drift)
    T_history = [k*tf_drift/(N_drift-1) for k=0:N_drift-1]

    filename = "history_after_drift_" * "tf_" * string(tf_drift) * "_N_" * string(N_drift) * ".png"
    save_history(T_history, X_history, U_history, filename, non_lin_parameters)

    x0_full = X_history[:,N_drift]
    # We recover from the initial drift using a open loop controller reyling on the linear dynamics model.
    lin_cons_parameters["orbit_radius"] = norm(x0_full[4:6]) # we set the orbit radius of the cw model.
    x0_cw = full_to_cw(x0_full)
    lin_cons_parameters["x0"] = x0_cw
    println("x0_full = ", x0_full)
    println("x0_cw = ", x0_cw)

    # Scale the parameters of the linear model
    scale_lin_parameters(lin_cons_parameters)
    println("lin_cons_parameters[x0] = ", lin_cons_parameters["x0"])

    # Compute the control sequence
    X, U, Y, ν, cost_history, constraint_violation = l1_solver(lin_cons_parameters)
    filename = "lin_uncons_" * "_rho_" * string(lin_cons_parameters["ρ"])
    filename *= "_iter_" * string(lin_cons_parameters["num_iter"]) * "_Qf_" * string(lin_cons_parameters["Qf"][1,1]) * ".png"
    save_results(X, U, Y, ν, cost_history, filename, lin_cons_parameters)
end

function lin_uncons_example(ρ)
    lin_uncons_parameters = define_lin_unconstrained_parameters()
    non_lin_parameters = define_non_lin_parameters()
    lin_uncons_parameters["ρ"] = ρ

    # Define initial state
    non_lin_parameters["x0"] = lin_uncons_parameters["x0_drift"]
    println("x0 = ", non_lin_parameters["x0"])

    # Propagate the non linear dynamics forward without control
    tf_drift = lin_uncons_parameters["tf_drift"]
    N_drift = lin_uncons_parameters["N_drift"]
    X_history = drifting_simulation(N_drift, tf_drift, non_lin_parameters)
    U_history = zeros(3, N_drift)
    T_history = [k*tf_drift/(N_drift-1) for k=0:N_drift-1]

    filename = "history_after_drift_" * "tf_" * string(tf_drift) * "_N_" * string(N_drift) * ".png"
    save_history(T_history, X_history, U_history, filename, non_lin_parameters)

    x0_full = X_history[:,N_drift]
    # We recover from the initial drift using a open loop controller reyling on the linear dynamics model.
    lin_uncons_parameters["orbit_radius"] = norm(x0_full[4:6]) # we set the orbit radius of the cw model.
    x0_cw = full_to_cw(x0_full)
    lin_uncons_parameters["x0"] = x0_cw
    println("x0_full = ", x0_full)
    println("x0_cw = ", x0_cw)

    # Scale the parameters of the linear model
    scale_lin_parameters(lin_uncons_parameters)
    println("lin_uncons_parameters[x0] = ", lin_uncons_parameters["x0"])

    # Compute the control sequence
    X, U, Y, ν, cost_history, constraint_violation = l1_solver(lin_uncons_parameters)
    filename = "lin_uncons_" * "_rho_" * string(lin_uncons_parameters["ρ"])
    filename *= "_iter_" * string(lin_uncons_parameters["num_iter"]) * "_Qf_" * string(lin_uncons_parameters["Qf"][1,1]) * ".png"
    save_results(X, U, Y, ν, cost_history, filename, lin_uncons_parameters)
end

function non_lin_uncons_example(ρ)
    non_lin_uncons_parameters = define_non_lin_unconstrained_parameters()
    non_lin_uncons_parameters["ρ"] = ρ

    # Define initial state
    println("x0 = ", non_lin_uncons_parameters["x0_drift"])
    non_lin_uncons_parameters["x0"] = non_lin_uncons_parameters["x0_drift"]
    # Propagate the non linear dynamics forward without control
    tf_drift = non_lin_uncons_parameters["tf_drift"]
    N_drift = non_lin_uncons_parameters["N_drift"]
    X_history = drifting_simulation(N_drift, tf_drift, non_lin_uncons_parameters)
    U_history = zeros(3, N_drift)
    T_history = [k*tf_drift/(N_drift-1) for k=0:N_drift-1]
    filename = "history_after_drift_" * "tf_" * string(tf_drift) * "_N_" * string(N_drift) * ".png"
    save_history(T_history, X_history, U_history, filename, non_lin_uncons_parameters)

    x0_full = X_history[:,N_drift]
    # We recover from the initial drift using a open loop controller relying on the linear dynamics model.
    non_lin_uncons_parameters["orbit_radius"] = norm(x0_full[4:6]) # we set the orbit radius of the cw model.
    non_lin_uncons_parameters["x0"] = x0_full
    println("x0_full = ", x0_full)

    # Scale the parameters of the linear model
    scale_non_lin_parameters(non_lin_uncons_parameters)
    println("non_lin_uncons_parameters[x0] = ", non_lin_uncons_parameters["x0"])

    # Compute the control sequence
    X, U, Y, ν, cost_history, constraint_violation = l1_solver(non_lin_uncons_parameters)
    filename = "lin_uncons_" * "_rho_" * string(non_lin_uncons_parameters["ρ"])
    filename *= "_iter_" * string(non_lin_uncons_parameters["num_iter"]) * "_Qf_" * string(non_lin_uncons_parameters["Qf"][1,1]) * ".png"
    save_results(X, U, Y, ν, cost_history, filename, non_lin_uncons_parameters)
end

function non_lin_cons_example(ρ)
    non_lin_cons_parameters = define_non_lin_constrained_parameters()
    non_lin_cons_parameters["ρ"] = ρ

    # Define initial state
    println("x0 = ", non_lin_cons_parameters["x0_drift"])
    non_lin_cons_parameters["x0"] = non_lin_cons_parameters["x0_drift"]
    # non_lin_cons_parameters["xf"] =
    # Propagate the non linear dynamics forward without control
    tf_drift = non_lin_cons_parameters["tf_drift"]
    N_drift = non_lin_cons_parameters["N_drift"]
    X_history = drifting_simulation(N_drift, tf_drift, non_lin_cons_parameters)
    U_history = zeros(3, N_drift)
    T_history = [k*tf_drift/(N_drift-1) for k=0:N_drift-1]
    filename = "history_after_drift_" * "tf_" * string(tf_drift) * "_N_" * string(N_drift) * ".png"
    save_history(T_history, X_history, U_history, filename, non_lin_cons_parameters)

    x0_full = X_history[:,N_drift]
    # We recover from the initial drift using a open loop controller relying on the linear dynamics model.
    non_lin_cons_parameters["orbit_radius"] = norm(x0_full[4:6]) # we set the orbit radius of the cw model.
    non_lin_cons_parameters["x0"] = x0_full
    println("x0_full = ", x0_full)

    # Scale the parameters of the linear model
    scale_non_lin_parameters(non_lin_cons_parameters)
    println("non_lin_cons_parameters[x0] = ", non_lin_cons_parameters["x0"])

    # Compute the control sequence
    X, U, Y, ν, cost_history, constraint_violation = l1_solver(non_lin_cons_parameters)
    filename = "lin_uncons_" * "_rho_" * string(non_lin_cons_parameters["ρ"])
    filename *= "_iter_" * string(non_lin_cons_parameters["num_iter"]) * "_Qf_" * string(non_lin_cons_parameters["Qf"][1,1]) * ".png"
    save_results(X, U, Y, ν, cost_history, filename, non_lin_cons_parameters)
    # println("X_final = ", X[:,end])
end


include("parameters.jl")
include("solver.jl")
include("utils.jl")

non_lin_parameters = define_non_lin_parameters()
scale_non_lin_parameters(non_lin_parameters)
lin_parameters = define_lin_parameters()
scale_lin_parameters(lin_parameters)


P = [10.0^i for i=0:0]
for ρ in P
    non_lin_cons_example(ρ)
end

#
# P = [10.0^i for i=-1:-1]
# for ρ in P
#     non_lin_uncons_example(ρ)
# end

#
#
# P = [10.0^i for i=-1:-1]
# for ρ in P
#     lin_uncons_example(ρ)
# end


# P = [10.0^i for i=0:0]
# for ρ in P
#     lin_cons_example(ρ)
# end
#



#
# include("parameters.jl")
# non_lin_parameters = define_non_lin_parameters()
# lin_parameters = define_lin_parameters()
#
# # mpc_example()

# include("parameters.jl")
# lin_parameters = define_lin_parameters()
# non_lin_parameters = define_non_lin_parameters()
#
# # @profiler logs = run_experiments()
# # logs = run_experiments(lin_parameters)
# logs = run_experiments(lin_parameters)
