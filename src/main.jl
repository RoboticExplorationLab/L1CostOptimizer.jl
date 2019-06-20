using BenchmarkTools
using LinearAlgebra

include("cost.jl")
include("dynamics.jl")
include("parameters.jl")
include("solver.jl")
include("utils.jl")

function run_experiments()
    N = params["N"]
    n = params["n"]
    m = params["m"]
    P = [10.0^i for i=-4:-4]
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

params = define_parameters()
# @profiler logs = run_experiments()
logs = run_experiments()
