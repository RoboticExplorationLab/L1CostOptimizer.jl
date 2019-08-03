using BenchmarkTools
using JLD
using LinearAlgebra
using TrajectoryOptimization
using PartedArrays
using PGFPlots
using PyPlot

include("cost_function.jl")
include("dynamics.jl")
include("experiment.jl")
include("experiment_parameters.jl")
include("parameter_scaling.jl")
include("solver.jl")
include("test.jl")
include("utils.jl")
include("visualization.jl")
include("visualization_latex.jl")
#
#
P = [10.0^i for i=-2:-2]
stopping_criterion = 4.5e-3
for ρ in P
    lin_uncons_example(ρ, stopping_criterion)
end #86

P = [10.0^i for i=-1:-1]
stopping_criterion = 4e-4
for ρ in P
    lin_cons_example(ρ, stopping_criterion)
end #404

P = [10.0^i for i=-1:-1]
stopping_criterion = 6e-5
for ρ in P
    non_lin_uncons_example(ρ, stopping_criterion)
end #120

P = [10.0^i for i=1:1]
stopping_criterion = 5e-6
for ρ in P
    non_lin_cons_example(ρ, stopping_criterion)
end #206

test_linear_dynamics_scaling()
test_non_linear_dynamics_scaling()
test_dynamics_consistency()

# # @profiler logs = run_experiments()
# # logs = run_experiments(lin_parameters)
# logs = run_experiments(lin_parameters)
#
# solver.stats["iteration"]
