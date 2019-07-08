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
