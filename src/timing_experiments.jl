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

# non_lin_parameters = define_non_lin_parameters()
# scale_non_lin_parameters(non_lin_parameters)
# lin_parameters = define_lin_parameters()
# scale_lin_parameters(lin_parameters)
#
# lin_cons_parameters = define_lin_constrained_parameters()
# non_lin_parameters = define_non_lin_parameters()
#
# # Define initial state
# non_lin_parameters["x0"] = lin_cons_parameters["x0_drift"]
# println("x0 = ", non_lin_parameters["x0"])
#
# # Propagate the non linear dynamics forward without control
# tf_drift = lin_cons_parameters["tf_drift"]
# N_drift = lin_cons_parameters["N_drift"]
# X_history = drifting_simulation(N_drift, tf_drift, non_lin_parameters)
# U_history = zeros(3, N_drift)
# T_history = [k*tf_drift/(N_drift-1) for k=0:N_drift-1]
#
# filename = "history_after_drift_" * "tf_" * string(tf_drift) * "_N_" * string(N_drift) * ".png"
# save_history(T_history, X_history, U_history, filename, non_lin_parameters)
#
# x0_full = X_history[:,N_drift]
# # We recover from the initial drift using a open loop controller reyling on the linear dynamics model.
# lin_cons_parameters["orbit_radius"] = norm(x0_full[4:6]) # we set the orbit radius of the cw model.
# x0_cw = full_to_cw(x0_full)
# lin_cons_parameters["x0"] = x0_cw
# println("x0_full = ", x0_full)
# println("x0_cw = ", x0_cw)
#
# # Scale the parameters of the linear model
# scale_lin_parameters(lin_cons_parameters)
# println("lin_cons_parameters[x0] = ", lin_cons_parameters["x0"])
#
# # Compute the control sequence
# function lin_cons_timing_example()
#     x0_full = [31350.9, -73076.3, -29.3723, 6.67083e6, 2.7906e6, 0.0, 75.8617, 32.5897, -0.0958906, -2866.72, 6852.81, 0.0]
#     x0_cw = [-79514.2, -720.471, 29.3723, 0.788372, -82.5619, 0.0958906]
#     lin_cons_parameters = define_lin_constrained_parameters()
#
#     lin_cons_parameters["orbit_radius"] = norm(x0_full[4:6]) # we set the orbit radius of the cw model.
#     lin_cons_parameters["x0"] = x0_cw
#     scale_lin_parameters(lin_cons_parameters)
#
#     X, U, Y, ν, cost_history, constraint_violation = l1_solver(lin_cons_parameters)
#     if !lin_cons_parameters["timing"]
#         filename = "lin_cons_" * "_rho_" * string(lin_cons_parameters["ρ"])
#         filename *= "_iter_" * string(lin_cons_parameters["num_iter"]) * "_Qf_" * string(lin_cons_parameters["Qf"][1,1])
#         save_results(X, U, Y, ν, cost_history, filename, lin_cons_parameters)
#     end
#     return
# end
#
# for i=1:10
#     benchmark = @time lin_cons_timing_example()
# end



#
# lin_uncons_parameters = define_lin_unconstrained_parameters()
# non_lin_parameters = define_non_lin_parameters()
#
# # Define initial state
# non_lin_parameters["x0"] = lin_uncons_parameters["x0_drift"]
# println("x0 = ", non_lin_parameters["x0"])
#
# # Propagate the non linear dynamics forward without control
# tf_drift = lin_uncons_parameters["tf_drift"]
# N_drift = lin_uncons_parameters["N_drift"]
# X_history = drifting_simulation(N_drift, tf_drift, non_lin_parameters)
# U_history = zeros(3, N_drift)
# T_history = [k*tf_drift/(N_drift-1) for k=0:N_drift-1]
#
# filename = "history_after_drift_" * "tf_" * string(tf_drift) * "_N_" * string(N_drift) * ".png"
# save_history(T_history, X_history, U_history, filename, non_lin_parameters)
#
# x0_full = X_history[:,N_drift]
# # We recover from the initial drift using a open loop controller reyling on the linear dynamics model.
# lin_uncons_parameters["orbit_radius"] = norm(x0_full[4:6]) # we set the orbit radius of the cw model.
# x0_cw = full_to_cw(x0_full)
# lin_uncons_parameters["x0"] = x0_cw
# println("x0_full = ", x0_full)
# println("x0_cw = ", x0_cw)
#
# # Scale the parameters of the linear model
# scale_lin_parameters(lin_uncons_parameters)
# println("lin_uncons_parameters[x0] = ", lin_uncons_parameters["x0"])
#
# # Compute the control sequence
# function lin_uncons_timing_example()
#     x0_full = [31350.9, -73076.3, -29.3723, 6.67083e6, 2.7906e6, 0.0, 75.8617, 32.5897, -0.0958906, -2866.72, 6852.81, 0.0]
#     x0_cw = [-79514.2, -720.471, 29.3723, 0.788372, -82.5619, 0.0958906]
#     lin_uncons_parameters = define_lin_unconstrained_parameters()
#
#     lin_uncons_parameters["orbit_radius"] = norm(x0_full[4:6]) # we set the orbit radius of the cw model.
#     lin_uncons_parameters["x0"] = x0_cw
#     scale_lin_parameters(lin_uncons_parameters)
#
#     X, U, Y, ν, cost_history, constraint_violation = l1_solver(lin_uncons_parameters)
#     if !lin_uncons_parameters["timing"]
#         filename = "lin_uncons_" * "_rho_" * string(lin_uncons_parameters["ρ"])
#         filename *= "_iter_" * string(lin_uncons_parameters["num_iter"]) * "_Qf_" * string(lin_uncons_parameters["Qf"][1,1])
#         save_results(X, U, Y, ν, cost_history, filename, lin_uncons_parameters)
#     end
#     return
# end
#
# for i=1:10
#     benchmark = @time lin_uncons_timing_example()
# end




non_lin_uncons_parameters = define_non_lin_unconstrained_parameters()

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
function non_lin_uncons_timing_example()
    x0_full = [31350.9, -73076.3, -29.3723, 6.67083e6, 2.7906e6, 0.0, 75.8617, 32.5897, -0.0958906, -2866.72, 6852.81, 0.0]
    x0_cw = [-79514.2, -720.471, 29.3723, 0.788372, -82.5619, 0.0958906]
    non_lin_uncons_parameters = define_non_lin_unconstrained_parameters()

    non_lin_uncons_parameters["orbit_radius"] = norm(x0_full[4:6]) # we set the orbit radius of the cw model.
    non_lin_uncons_parameters["x0"] = x0_full
    scale_non_lin_parameters(non_lin_uncons_parameters)

    X, U, Y, ν, cost_history, constraint_violation = l1_solver(non_lin_uncons_parameters)
    if !non_lin_uncons_parameters["timing"]
        filename = "non_lin_uncons_" * "_rho_" * string(non_lin_uncons_parameters["ρ"])
        filename *= "_iter_" * string(non_lin_uncons_parameters["num_iter"]) * "_Qf_" * string(non_lin_uncons_parameters["Qf"][1,1])
        save_results(X, U, Y, ν, cost_history, filename, non_lin_uncons_parameters)
    end
    return
end

for i=1:10
    benchmark = @time non_lin_uncons_timing_example()
end
