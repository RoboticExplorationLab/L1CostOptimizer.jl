using BenchmarkTools
using JLD
using LinearAlgebra
using TrajectoryOptimization
using PartedArrays
using PyPlot

include("cost.jl")
include("dynamics.jl")
include("experiment.jl")
include("experiment_parameters.jl")
include("parameter_scaling.jl")
include("solver.jl")
include("test.jl")
include("utils.jl")
include("visualization.jl")




P = [10.0^i for i=2:2]
stopping_criterion = 1e-6
for ρ in P
    non_lin_cons_example(ρ, stopping_criterion)
end


P = [10.0^i for i=2:2]
stopping_criterion = 5e-7
for ρ in P
    non_lin_uncons_example(ρ, stopping_criterion)
end


P = [10.0^i for i=-0:-0]
stopping_criterion = 5e-5
for ρ in P
    lin_uncons_example(ρ, stopping_criterion)
end


P = [10.0^i for i=-0:-0]
stopping_criterion = 1e-4
for ρ in P
    lin_cons_example(ρ, stopping_criterion)
end


# test_linear_dynamics_scaling()
# test_non_linear_dynamics_scaling()
test_dynamics_consistency()
# 199 92 108 258

# function gradient_todorov(prob::Problem,solver::iLQRSolver)
#
# function gradient_feedforward(solver::iLQRSolver)
#
# gradient saved in
# @logmsg InnerLoop :grad value=solver.stats[:gradient][end]
#
# can specify todorov or feedforward for the grad computation
#     todoriv is approx of L2 norm of the grad
#
#
# for ilqr use the const convergence and 1e-3 for the value
# for aug lag solver use 1e-3 on constraint violation
#
#



# # @profiler logs = run_experiments()
# # logs = run_experiments(lin_parameters)
# logs = run_experiments(lin_parameters)
#
# solver.stats["iteration"]
