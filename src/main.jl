using BenchmarkTools
using JLD
using LinearAlgebra
using TrajectoryOptimization
using PartedArrays
using PyPlot

include("cost.jl")
include("dynamics.jl")
include("experiments.jl")
include("experiment_parameters.jl")
include("parameters.jl")
include("solver.jl")
include("test.jl")
include("utils.jl")




# P = [10.0^i for i=-0:-0]
# stopping_criterion = 1e-4
# for ρ in P
#     non_lin_cons_example(ρ, stopping_criterion)
# end
#
#
# P = [10.0^i for i=-0:-0]
# stopping_criterion = 1e-5
# for ρ in P
#     non_lin_uncons_example(ρ, stopping_criterion)
# end


# P = [10.0^i for i=-1:-1]
# stopping_criterion = 1e-4
# for ρ in P
#     lin_uncons_example(ρ, stopping_criterion)
# end


P = [10.0^i for i=-0:-0]
stopping_criterion = 1e-4
for ρ in P
    lin_cons_example(ρ, stopping_criterion)
end


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
