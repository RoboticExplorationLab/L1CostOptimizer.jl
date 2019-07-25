using BenchmarkTools
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
include("utils.jl")

non_lin_parameters = define_non_lin_parameters()
scale_non_lin_parameters(non_lin_parameters)
lin_parameters = define_lin_parameters()
scale_lin_parameters(lin_parameters)

# P = [10.0^i for i=0:0]
# for ρ in P
#     non_lin_cons_example(ρ)
# end
#
P = [10.0^i for i=-1:-1]
for ρ in P
    non_lin_uncons_example(ρ)
end
# # @btime non_lin_uncons_example(0.1)




P = [10.0^i for i=-1:-1]
# P = [10.0^i for i=-0:-0]
for ρ in P
    lin_uncons_example(ρ)
end
a=1



#
# P = [10.0^i for i=0:0]
# for ρ in P
#     lin_cons_example(ρ)
# end


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
#





# n = 5
# m = 3
# Q = ones(n,n)
# R = Array(Diagonal(ones(m)))
# H = ones(m,n)
# q = ones(n)
# r = ones(m)
# c = 1
# Qf = ones(n,n)
# qf = ones(n)
# cf = 1
# QuadraticCost(Q, R, H, q, r, c, Qf, qf, cf)
