using BenchmarkTools
using JLD
using LinearAlgebra
using TrajectoryOptimization
using PartedArrays
using PGFPlots
using PyPlot

include("animation.jl")
include("cost_function.jl")
include("dynamics.jl")
include("experiment.jl")
include("experiment_parameters.jl")
include("parameter_scaling.jl")
include("solver.jl")
include("test.jl")
include("utils.jl")
include("visualization_image.jl")
include("visualization_latex.jl")
#
# # Running Experiments
ρ = 1e-2
stopping_criterion = 4.5e-3
lin_uncons_example(ρ, stopping_criterion)
#86

ρ = 1e-1
stopping_criterion = 4e-4
lin_cons_example(ρ, stopping_criterion)
#404

ρ = 1e-1
stopping_criterion = 6e-5
non_lin_uncons_example(ρ, stopping_criterion)
#120

ρ = 1e1
stopping_criterion = 5e-6
non_lin_cons_example(ρ, stopping_criterion)
#206

# # Running Tests
# test_linear_dynamics_scaling()
# test_non_linear_dynamics_scaling()
# test_dynamics_consistency()

# # Running visualization
# parameters = define_animation_parameters()
# # parameters["filename"] = "unconstrained_nonlinear_dynamics"
# parameters["filename"] = "constrained_nonlinear_dynamics"
# # parameters["filename"] = "quadratic_unconstrained_nonlinear_dynamics"
# visualizer(parameters)
