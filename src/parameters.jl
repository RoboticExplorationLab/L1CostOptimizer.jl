function define_lin_parameters()
    μ = 3.99*10^14 # Standard gravitational parameter m^3 s^-2
    a = 6731.0*10^3 # semi major axis m
    alt = 4e5 # altitude of the target satellite in m
    N = 100
    n = 6
    m = 3
    tf = 1e5 # 5 revolutions
    Δt = tf/(N-1)
    x0 = [2e5, 1.5e5, 1.0e5, 0.0, 0.0, 0.0]
    xf = zeros(n)

    # mass
    m_ego = 4.6

    # Scale
    t_ref = tf
    l_ref = norm(x0[1:3] - xf[1:3])
    v_ref = l_ref / t_ref
    a_ref = l_ref / (t_ref*Δt)
    # a_ref = l_ref / (t_ref)
    u_ref = a_ref * m_ego
    println("u_ref = ", u_ref)
    # Scaling
    tf /= t_ref
    x0[1:3] /= l_ref
    x0[4:6] /= v_ref
    xf[1:3] /= l_ref
    xf[4:6] /= v_ref

    println("x0 = ", x0)
    println("xf = ", xf)

    println("t_ref = ", t_ref)
    println("l_ref = ", l_ref)
    println("v_ref = ", v_ref)
    println("a_ref = ", a_ref)
    println("u_ref = ", u_ref)

    parameters = Dict("num_iter" => 100, # number of iteration of ADMM
                  "ilqr_solver_iter" => 1, # number of iterations of the inner solver (ilqr = unconstrained)
                  "al_solver_iter" => 10, # number of iterations of the inner solver (al = constrained)
                  "N" => N, # horizon
                  "n"=> n, # state size
                  "m"=> m, # control size
                  "Δt" => Δt, # time step
                  "tf" => tf, # final time
                  "ρ" => 1e-1, # weighting parameter for augmented lagrangian
                  "α" => 1.0, # weighting parameter
                  "X0" => zeros(n, N),#reshape(repeat(x0, outer = [N, 1]), (n, N)), # initial state trajectory
                  "U0" => zeros(m, N-1), # initial control trajectory
                  "ν0" => zeros(m, N-1), # initial dual
                  "Y0" => zeros(m, N-1), # initial dummy
                  "x0" => x0, # initial state
                  "xf" => xf, # final state
                  "t_ref" => t_ref, # reference duration
                  "l_ref" => l_ref, # reference length for delta ego target
                  "v_ref" => v_ref, # reference velocity for delta ego target
                  "a_ref" => a_ref, # reference acceleration for delta ego target
                  "u_ref" => u_ref, # reference control for ego
                  "using_constraints" => true, # using constraints on control
                  "complete_results" => true, # using constraints on control
                  "stage_plot" => true, # using constraints on control
                  "stage_plot_freq" => 25, # using constraints on control
                  "linearity" => true, # using linear dynamics
                  "u_min" => -10.0^0, # lower bound on u
                  "u_max" => 10.0^0, # upper bound on u
                  "Q" => 0.0 * Matrix{Float64}(I, n, n), # Quadratic stage cost for states (n,n)
                  "R" => 1e-20 * Matrix{Float64}(I, m, m), # Quadratic stage cost for controls (m,m)
                  "H" => zeros(m, n), # Quadratic Cross-coupling for state and controls (m,n)
                  "q" => zeros(n), # Linear term on states (n,)
                  "r" => zeros(m), # Linear term on controls (m,)
                  "c" => 0.0, # constant term
                  "Qf" => 10.0 * Matrix{Float64}(I, n, n), # Quadratic final cost for terminal state (n,n)
                  "qf" => zeros(n), # Linear term on terminal state (n,)
                  "cf" => 0.0, # constant term (terminal)
                  "μ" => μ, # Standard gravitational parameter m^3 s^-2
                  "a" => a, # semi major axis m
                  "radius" => a+alt, # semi major axis m
                  "m_ego" => m_ego, # mass of the ego satellite kg
                   )
    return parameters
end

function define_non_lin_parameters()
    μ = 3.99*10^14 # Standard gravitational parameter m^3 s^-2
    a = 6731.0*10^3 # semi major axis m
    N = 100
    n = 12
    m = 3
    tf = 1000.0
    Δt = tf/(N-1)
    r0 = [4*10.0^5 + a, 0.0, 0.0]
    v0 = [0.0, sqrt(μ/norm(r0)), 0.0]

    x0 = [[0, -100, 0]; r0; zeros(3); v0]
    xf = zeros(n) # ### we need to get rid of penalty on x4,5,6 and x10,11,12
    Qf = 10 .* Diagonal([ones(3); zeros(3); ones(3); zeros(3)])

    # mass
    m_ego = 1.0

    # Scale
    t_ref = tf
    l_ego_ref = norm(x0[1:3]) ### make sure this is not zero
    l_target_ref =  norm(x0[4:6])
    v_ego_ref = l_ego_ref / t_ref
    v_target_ref = sqrt(μ/l_target_ref)
    a_ego_ref = l_ego_ref / (t_ref*Δt)
    a_target_ref = μ/l_target_ref^2
    u_ref = a_ego_ref * m_ego

    # Scaling
    tf /= t_ref
    x0[1:3] /= l_ego_ref
    x0[4:6] /= l_target_ref
    x0[7:9] /= v_ego_ref
    x0[10:12] /= v_target_ref
    xf[1:3] /= l_ego_ref
    xf[4:6] /= l_target_ref
    xf[7:9] /= v_ego_ref
    xf[10:12] /= v_target_ref

    parameters = Dict("num_iter" => 300, # number of iteration of ADMM
                  "ilqr_solver_iter" => 1, # number of iterations of the inner solver (ilqr = unconstrained)
                  "al_solver_iter" => 10, # number of iterations of the inner solver (al = constrained)
                  "N" => N, # horizon
                  "n"=> n, # state size
                  "m"=> m, # control size
                  "Δt" => Δt, # time step
                  "tf" => tf, # final time
                  "ρ" => 1e-4, # weighting parameter for augmented lagrangian
                  "α" => 1.0, # weighting parameter
                  "X0" => zeros(n, N), # reshape(repeat(x0, outer = [N, 1]), (n, N)), # initial state trajectory
                  "U0" => zeros(m, N-1), # initial control trajectory
                  "ν0" => zeros(m, N-1), # initial dual
                  "Y0" => zeros(m, N-1), # initial dummy
                  "x0" => x0, # initial state
                  "xf" => xf, # final state
                  "t_ref" => t_ref, # reference duration
                  "l_ego_ref" => l_ego_ref, # reference length for delta ego target
                  "l_target_ref" => l_target_ref, # reference length for target
                  "v_ego_ref" => v_ego_ref, # reference velocity for delta ego target
                  "v_target_ref" => v_target_ref, # reference velocity for target
                  "a_ego_ref" => a_ego_ref, # reference acceleration for delta ego target
                  "a_target_ref" => a_target_ref, # reference acceleration for target
                  "u_ref" => u_ref, # reference control for ego
                  "using_constraints" => false, # using constraints on control
                  "complete_results" => true, # using constraints on control
                  "stage_plot" => true, # using constraints on control
                  "stage_plot_freq" => 5, # using constraints on control
                  "linearity" => false, # using non linear dynamics
                  "u_min" => -40.0, # lower bound on u
                  "u_max" => 40.0, # upper bound on u
                  "Q" => 0.0 * Matrix{Float64}(I, n, n), # Quadratic stage cost for states (n,n)
                  "R" => 1e-20 * Matrix{Float64}(I, m, m), # Quadratic stage cost for controls (m,m)
                  "H" => zeros(m, n), # Quadratic Cross-coupling for state and controls (m,n)
                  "q" => zeros(n), # Linear term on states (n,)
                  "r" => zeros(m), # Linear term on controls (m,)
                  "c" => 0.0, # constant term
                  "Qf" => Qf, # Quadratic final cost for terminal state (n,n)
                  "qf" => zeros(n), # Linear term on terminal state (n,)
                  "cf" => 0.0, # constant term (terminal)
                  "μ" => μ, # Standard gravitational parameter m^3 s^-2
                  "μs" => μ*t_ref^2/l_target_ref^3, # Standard gravitational parameter m^3 s^-2
                  "a" => a, # semi major axis m
                  "as" => a/l_target_ref, # semi major axis m
                  "J2" => 1.75553*10^25, # J2 factor in m^5.s^-2
                  "ω" => [0, 0, 2*pi/((23*60 + 56)*60 + 4)], # Earth's rotational speed vector rad s^-1
                  "m_ego" => m_ego, # mass of the ego satellite kg
                  "m_target" => 8.0, # mass of the target satellite kg
                  "A_ego" => 1.0, # section area of the ego satellite m^2
                  "A_target" => 4.0, # section area of the target satellite m^2
                  "Cd_ego" => 1.0, # Drag coefficient of the ego satellite s.u
                  "Cd_target" => 1.0, # Drag coefficient of the target satellite s.u
                   )
    return parameters
end

# linear unconstrained
# parameters = Dict("num_iter" => 100, # number of iteration of ADMM
#               "ilqr_solver_iter" => 10, # number of iterations of the inner solver (ilqr = unconstrained)
#               "al_solver_iter" => 10, # number of iterations of the inner solver (al = constrained)
#               "N" => N, # horizon
#               "n"=> n, # state size
#               "m"=> m, # control size
#               "Δt" => Δt, # time step
#               "tf" => tf, # final time
#               "ρ" => 1e-1, # weighting parameter for augmented lagrangian
#               "α" => 10.0, # weighting parameter
#               "X0" => zeros(n, N),#reshape(repeat(x0, outer = [N, 1]), (n, N)), # initial state trajectory
#               "U0" => zeros(m, N-1), # initial control trajectory
#               "ν0" => ones(m, N-1), # initial dual
#               "Y0" => zeros(m, N-1), # initial dummy
#               "x0" => x0, # initial state
#               "xf" => xf, # final state
#               "t_ref" => t_ref, # reference duration
#               "l_ref" => l_ref, # reference length
#               "using_constraints" => false, # using constraints on control
#               "linearity" => true, # using linear dynamics
#               "u_min" => -10.0, # lower bound on u
#               "u_max" => 10.0, # upper bound on u
#               "Q" => 0.0 * Matrix{Float64}(I, n, n), # Quadratic stage cost for states (n,n)
#               "R" => 1e-20 * Matrix{Float64}(I, m, m), # Quadratic stage cost for controls (m,m)
#               "H" => zeros(m, n), # Quadratic Cross-coupling for state and controls (m,n)
#               "q" => zeros(n), # Linear term on states (n,)
#               "r" => zeros(m), # Linear term on controls (m,)
#               "c" => 0.0, # constant term
#               "Qf" => 100.0 * Matrix{Float64}(I, n, n), # Quadratic final cost for terminal state (n,n)
#               "qf" => zeros(n), # Linear term on terminal state (n,)
#               "cf" => 0.0, # constant term (terminal)
#               "μ" => μ, # Standard gravitational parameter m^3 s^-2
#               "̂μ" => μ*t_ref^2/l_ref^3, # Standard gravitational parameter m^3 s^-2
#               "a" => a, # semi major axis m
#               "̂a" => a/l_ref, # semi major axis m
#               "m_ego" => 1.0, # mass of the ego satellite kg
#                )
#

# linear constrained
# parameters = Dict("num_iter" => 300, # number of iteration of ADMM
#               "ilqr_solver_iter" => 1, # number of iterations of the inner solver (ilqr = unconstrained)
#               "al_solver_iter" => 10, # number of iterations of the inner solver (al = constrained)
#               "N" => N, # horizon
#               "n"=> n, # state size
#               "m"=> m, # control size
#               "Δt" => Δt, # time step
#               "tf" => tf, # final time
#               "ρ" => 1e0, # weighting parameter for augmented lagrangian
#               "α" => 1.0, # weighting parameter
#               "X0" => zeros(n, N),#reshape(repeat(x0, outer = [N, 1]), (n, N)), # initial state trajectory
#               "U0" => zeros(m, N-1), # initial control trajectory
#               "ν0" => zeros(m, N-1), # initial dual
#               "Y0" => zeros(m, N-1), # initial dummy
#               "x0" => x0, # initial state
#               "xf" => xf, # final state
#               "t_ref" => t_ref, # reference duration
#               "l_ref" => l_ref, # reference length for delta ego target
#               "v_ref" => v_ref, # reference velocity for delta ego target
#               "a_ref" => a_ref, # reference acceleration for delta ego target
#               "u_ref" => u_ref, # reference control for ego
#               "using_constraints" => true, # using constraints on control
#               "complete_results" => true, # using constraints on control
#               "stage_plot" => true, # using constraints on control
#               "stage_plot_freq" => 25, # using constraints on control
#               "linearity" => true, # using linear dynamics
#               "u_min" => -500.0, # lower bound on u
#               "u_max" => 500.0, # upper bound on u
#               "Q" => 0.0 * Matrix{Float64}(I, n, n), # Quadratic stage cost for states (n,n)
#               "R" => 1e-20 * Matrix{Float64}(I, m, m), # Quadratic stage cost for controls (m,m)
#               "H" => zeros(m, n), # Quadratic Cross-coupling for state and controls (m,n)
#               "q" => zeros(n), # Linear term on states (n,)
#               "r" => zeros(m), # Linear term on controls (m,)
#               "c" => 0.0, # constant term
#               "Qf" => 1.0 * Matrix{Float64}(I, n, n), # Quadratic final cost for terminal state (n,n)
#               "qf" => zeros(n), # Linear term on terminal state (n,)
#               "cf" => 0.0, # constant term (terminal)
#               "μ" => μ, # Standard gravitational parameter m^3 s^-2
#               "a" => a, # semi major axis m
#               "m_ego" => m_ego, # mass of the ego satellite kg
#                )
# return parameters
# end


# non linear unconstrained
# function define_non_lin_parameters()
#     μ = 3.99*10^14 # Standard gravitational parameter m^3 s^-2
#     a = 6731.0*10^3 # semi major axis m
#     N = 100
#     n = 12
#     m = 3
#     tf = 1000.0
#     Δt = tf/(N-1)
#     r0 = [100000.0 + a, 0.0, 0.0]
#     v0 = [0.0, sqrt(μ/norm(r0)), 0.0]
#
#     x0 = [[0, -100, 0]; r0; zeros(3); v0]
#     xf = zeros(n) # ### we need to get rid of penalty on x4,5,6 and x10,11,12
#     Qf = 10 .* Diagonal([ones(3); zeros(3); ones(3); zeros(3)])
#
#     # mass
#     m_ego = 1.0
#
#     # Scale
#     t_ref = tf
#     l_ego_ref = norm(x0[1:3]) ### make sure this is not zero
#     l_target_ref =  norm(x0[4:6])
#     v_ego_ref = l_ego_ref / t_ref
#     v_target_ref = sqrt(μ/l_target_ref)
#     a_ego_ref = l_ego_ref / (t_ref*Δt)
#     a_target_ref = μ/l_target_ref^2
#     u_ref = a_ego_ref * m_ego
#
#     # Scaling
#     tf /= t_ref
#     x0[1:3] /= l_ego_ref
#     x0[4:6] /= l_target_ref
#     x0[7:9] /= v_ego_ref
#     x0[10:12] /= v_target_ref
#     xf[1:3] /= l_ego_ref
#     xf[4:6] /= l_target_ref
#     xf[7:9] /= v_ego_ref
#     xf[10:12] /= v_target_ref
#     println("x0 = ", x0)
#     println("xf = ", xf)
#     parameters = Dict("num_iter" => 100, # number of iteration of ADMM
#                   "ilqr_solver_iter" => 1, # number of iterations of the inner solver (ilqr = unconstrained)
#                   "al_solver_iter" => 10, # number of iterations of the inner solver (al = constrained)
#                   "N" => N, # horizon
#                   "n"=> n, # state size
#                   "m"=> m, # control size
#                   "Δt" => Δt, # time step
#                   "tf" => tf, # final time
#                   "ρ" => 1e-7, # weighting parameter for augmented lagrangian
#                   "α" => 1.0, # weighting parameter
#                   "X0" => zeros(n, N), # reshape(repeat(x0, outer = [N, 1]), (n, N)), # initial state trajectory
#                   "U0" => zeros(m, N-1), # initial control trajectory
#                   "ν0" => ones(m, N-1), # initial dual
#                   "Y0" => zeros(m, N-1), # initial dummy
#                   "x0" => x0, # initial state
#                   "xf" => xf, # final state
#                   "t_ref" => t_ref, # reference duration
#                   "l_ego_ref" => l_ego_ref, # reference length for delta ego target
#                   "l_target_ref" => l_target_ref, # reference length for target
#                   "v_ego_ref" => v_ego_ref, # reference velocity for delta ego target
#                   "v_target_ref" => v_target_ref, # reference velocity for target
#                   "a_ego_ref" => a_ego_ref, # reference acceleration for delta ego target
#                   "a_target_ref" => a_target_ref, # reference acceleration for target
#                   "u_ref" => u_ref, # reference control for ego
#                   "using_constraints" => false, # using constraints on control
#                   "linearity" => false, # using non linear dynamics
#                   "u_min" => -10.0, # lower bound on u
#                   "u_max" => 10.0, # upper bound on u
#                   "Q" => 0.0 * Matrix{Float64}(I, n, n), # Quadratic stage cost for states (n,n)
#                   "R" => 1e-20 * Matrix{Float64}(I, m, m), # Quadratic stage cost for controls (m,m)
#                   "H" => zeros(m, n), # Quadratic Cross-coupling for state and controls (m,n)
#                   "q" => zeros(n), # Linear term on states (n,)
#                   "r" => zeros(m), # Linear term on controls (m,)
#                   "c" => 0.0, # constant term
#                   "Qf" => Qf, # Quadratic final cost for terminal state (n,n)
#                   "qf" => zeros(n), # Linear term on terminal state (n,)
#                   "cf" => 0.0, # constant term (terminal)
#                   "μ" => μ, # Standard gravitational parameter m^3 s^-2
#                   "μs" => μ*t_ref^2/l_target_ref^3, # Standard gravitational parameter m^3 s^-2
#                   "a" => a, # semi major axis m
#                   "as" => a/l_target_ref, # semi major axis m
#                   "J2" => 1.75553*10^25, # J2 factor in m^5.s^-2
#                   "ω" => [0, 0, 2*pi/((23*60 + 56)*60 + 4)], # Earth's rotational speed vector rad s^-1
#                   "m_ego" => m_ego, # mass of the ego satellite kg
#                   "m_target" => 8.0, # mass of the target satellite kg
#                   "A_ego" => 1.0, # section area of the ego satellite m^2
#                   "A_target" => 4.0, # section area of the target satellite m^2
#                   "Cd_ego" => 1.0, # Drag coefficient of the ego satellite s.u
#                   "Cd_target" => 1.0, # Drag coefficient of the target satellite s.u
#                    )
#     return parameters
# end


# non linear constrained
# function define_non_lin_parameters()
#     μ = 3.99*10^14 # Standard gravitational parameter m^3 s^-2
#     a = 6731.0*10^3 # semi major axis m
#     N = 100
#     n = 12
#     m = 3
#     tf = 1000.0
#     Δt = tf/(N-1)
#     r0 = [4*10.0^5 + a, 0.0, 0.0]
#     v0 = [0.0, sqrt(μ/norm(r0)), 0.0]
#
#     x0 = [[0, -100, 0]; r0; zeros(3); v0]
#     xf = zeros(n) # ### we need to get rid of penalty on x4,5,6 and x10,11,12
#     Qf = 10 .* Diagonal([ones(3); zeros(3); ones(3); zeros(3)])
#
#     # mass
#     m_ego = 1.0
#
#     # Scale
#     t_ref = tf
#     l_ego_ref = norm(x0[1:3]) ### make sure this is not zero
#     l_target_ref =  norm(x0[4:6])
#     v_ego_ref = l_ego_ref / t_ref
#     v_target_ref = sqrt(μ/l_target_ref)
#     a_ego_ref = l_ego_ref / (t_ref*Δt)
#     a_target_ref = μ/l_target_ref^2
#     u_ref = a_ego_ref * m_ego
#
#     # Scaling
#     tf /= t_ref
#     x0[1:3] /= l_ego_ref
#     x0[4:6] /= l_target_ref
#     x0[7:9] /= v_ego_ref
#     x0[10:12] /= v_target_ref
#     xf[1:3] /= l_ego_ref
#     xf[4:6] /= l_target_ref
#     xf[7:9] /= v_ego_ref
#     xf[10:12] /= v_target_ref
#     println("x0 = ", x0)
#     println("xf = ", xf)
#     parameters = Dict("num_iter" => 300, # number of iteration of ADMM
#                   "ilqr_solver_iter" => 1, # number of iterations of the inner solver (ilqr = unconstrained)
#                   "al_solver_iter" => 10, # number of iterations of the inner solver (al = constrained)
#                   "N" => N, # horizon
#                   "n"=> n, # state size
#                   "m"=> m, # control size
#                   "Δt" => Δt, # time step
#                   "tf" => tf, # final time
#                   "ρ" => 1e-4, # weighting parameter for augmented lagrangian
#                   "α" => 1.0, # weighting parameter
#                   "X0" => zeros(n, N), # reshape(repeat(x0, outer = [N, 1]), (n, N)), # initial state trajectory
#                   "U0" => zeros(m, N-1), # initial control trajectory
#                   "ν0" => zeros(m, N-1), # initial dual
#                   "Y0" => zeros(m, N-1), # initial dummy
#                   "x0" => x0, # initial state
#                   "xf" => xf, # final state
#                   "t_ref" => t_ref, # reference duration
#                   "l_ego_ref" => l_ego_ref, # reference length for delta ego target
#                   "l_target_ref" => l_target_ref, # reference length for target
#                   "v_ego_ref" => v_ego_ref, # reference velocity for delta ego target
#                   "v_target_ref" => v_target_ref, # reference velocity for target
#                   "a_ego_ref" => a_ego_ref, # reference acceleration for delta ego target
#                   "a_target_ref" => a_target_ref, # reference acceleration for target
#                   "u_ref" => u_ref, # reference control for ego
#                   "using_constraints" => true, # using constraints on control
#                   "complete_results" => true, # using constraints on control
#                   "stage_plot" => true, # using constraints on control
#                   "stage_plot_freq" => 5, # using constraints on control
#                   "linearity" => false, # using non linear dynamics
#                   "u_min" => -40.0, # lower bound on u
#                   "u_max" => 40.0, # upper bound on u
#                   "Q" => 0.0 * Matrix{Float64}(I, n, n), # Quadratic stage cost for states (n,n)
#                   "R" => 1e-20 * Matrix{Float64}(I, m, m), # Quadratic stage cost for controls (m,m)
#                   "H" => zeros(m, n), # Quadratic Cross-coupling for state and controls (m,n)
#                   "q" => zeros(n), # Linear term on states (n,)
#                   "r" => zeros(m), # Linear term on controls (m,)
#                   "c" => 0.0, # constant term
#                   "Qf" => Qf, # Quadratic final cost for terminal state (n,n)
#                   "qf" => zeros(n), # Linear term on terminal state (n,)
#                   "cf" => 0.0, # constant term (terminal)
#                   "μ" => μ, # Standard gravitational parameter m^3 s^-2
#                   "μs" => μ*t_ref^2/l_target_ref^3, # Standard gravitational parameter m^3 s^-2
#                   "a" => a, # semi major axis m
#                   "as" => a/l_target_ref, # semi major axis m
#                   "J2" => 1.75553*10^25, # J2 factor in m^5.s^-2
#                   "ω" => [0, 0, 2*pi/((23*60 + 56)*60 + 4)], # Earth's rotational speed vector rad s^-1
#                   "m_ego" => m_ego, # mass of the ego satellite kg
#                   "m_target" => 8.0, # mass of the target satellite kg
#                   "A_ego" => 1.0, # section area of the ego satellite m^2
#                   "A_target" => 4.0, # section area of the target satellite m^2
#                   "Cd_ego" => 1.0, # Drag coefficient of the ego satellite s.u
#                   "Cd_target" => 1.0, # Drag coefficient of the target satellite s.u
#                    )
#     return parameters
# end
