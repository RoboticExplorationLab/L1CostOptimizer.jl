using BenchmarkTools
using LinearAlgebra
using TrajectoryOptimization

include("cost.jl")
include("dynamics.jl")
include("parameters.jl")
include("solver.jl")
include("utils.jl")

function run_experiments(parameters)
    N = parameters["N"]
    n = parameters["n"]
    m = parameters["m"]
    if parameters["linearity"]
        QF = [10.0^i .* Diagonal(ones(6)) for i=0:0]
    elseif !parameters["linearity"]
        QF = [10.0^i .* Diagonal([ones(3); zeros(3); ones(3); zeros(3)]) for i=0:0]
    end
    P = [10.0^i for i=0:0]
    logs = Dict("X_log" => zeros(length(P), n, N),
        "U_log" => zeros(length(P), m, N-1),
        "Y_log" => zeros(length(P), m, N-1),
        "ν_log" => zeros(length(P), m, N-1))
    for j=1:length(QF)
        for k=1:length(P)
            parameters = define_lin_parameters()
            parameters["Qf"] = QF[j]
            parameters["ρ"] = P[k]
            X, U, Y, ν, cost_history, constraint_violation = ADMM(parameters)
            logs["X_log"][k,:,:] = X
            logs["U_log"][k,:,:] = U
            logs["Y_log"][k,:,:] = Y
            logs["ν_log"][k,:,:] = ν
            filename = "lin_uncons_" * "rho_" * string(P[k]) * "_iter_" * string(parameters["num_iter"]) * "_Qf_" * string(parameters["Qf"][1,1]) * ".png"
            save_results(X, U, Y, ν, cost_history, filename, parameters)
        end
    end
    return logs
end


include("parameters.jl")
lin_parameters = define_lin_parameters()
non_lin_parameters = define_non_lin_parameters()

# @profiler logs = run_experiments()
# logs = run_experiments(lin_parameters)
logs = run_experiments(lin_parameters)


a = 1
a = 1
a = 1
a = 1
a = 1
a = 1
a = 1
a = 1
a = 1
a = 1

#
# μ = 3.99*10^14 # Standard gravitational parameter m^3 s^-2
# a = 6731.0*10^3 # semi major axis m
# r0 = [100000.0 + a, 0.0, 0.0]
# v0 = [0.0, sqrt(μ/a), 0.0]
# x0 = [zeros(3); r0; zeros(3); v0]
# x0_scaled = zeros(12)
# tf = 4.0
# # Scale
# t_ref = tf
# # l_ref = norm(x0[1:3] - xf[1:3]) # maybe to small
# l_ref = norm(x0[4:6])
#
# # Scaling
# x0_scaled[1:6] = x0[1:6] / l_ref
# x0_scaled[7:12] = x0[7:12] * t_ref / l_ref
# T = 4
# X = zeros(12, T)
# X_scaled = zeros(12, T)
# X[:,1] = x0
# X_scaled[:,1] = x0_scaled
# dt = 1
# for t=2:T
#     ẋ = zeros(12)
#     u = zeros(3)
#     non_linear_dynamics!(ẋ,X[:,t-1],u)
#     X[:,t] = X[:,t-1] + dt * ẋ
#
#     ẋ_scaled = zeros(12)
#     u_scaled = zeros(3)
#     scaled_non_linear_dynamics!(ẋ_scaled,X_scaled[:,t-1],u_scaled)
#     X_scaled[:,t] = X_scaled[:,t-1] + dt/T * ẋ_scaled
# end
# println(X)
# X_rescaled = zeros(12,T)
# for t=1:T
#     X_rescaled[1:6,t] = X_scaled[1:6,t]*l_ref
#     X_rescaled[7:12,t] = X_scaled[7:12,t]*l_ref/t_ref
# end
# fig = figure(figsize=(9,3))
# subplot(1, 3, 1)
# plot(X[1,:], X[2,:], color="blue", linewidth=1.0, linestyle="-", label=L"traj ego")
# plot(X[4,:], X[5,:], color="red", linewidth=1.0, linestyle="-", label=L"traj target")
# plot(X[7,:], X[8,:], color="blue", linewidth=2.0, linestyle="--", label=L"speed ego")
# plot(X[10,:], X[11,:], color="red", linewidth=2.0, linestyle="--", label=L"speed target")
# title(L"Traj and Speed of ego and target")
# grid("on")
# xlabel(L"X")
# ylabel(L"Y")
# legend()
# axis("equal")
#
# subplot(1, 3, 2)
# plot([norm(X[1:2,t]) for t=1:T], color="blue", linewidth=1.0, linestyle="-", label=L"dist ego")
# plot([norm(X[4:5,t]) for t=1:T], color="red", linewidth=1.0, linestyle="-", label=L"dist target")
# plot([norm(X[7:8,t]) for t=1:T], color="blue", linewidth=2.0, linestyle="--", label=L"speed ego")
# plot([norm(X[10:11,t]) for t=1:T], color="red", linewidth=2.0, linestyle="--", label=L"speed target")
# title(L"speed and dist")
# grid("on")
# xlabel(L"X")
# ylabel(L"Y")
# legend()
# axis("equal")
#
#
# filename = "traj.png"
# tight_layout()
# savefig("result/" * filename, format="png", dpi=300)
# close()
