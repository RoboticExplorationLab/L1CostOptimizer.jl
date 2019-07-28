# function define_lin_parameters()
#     μ = 3.99*10^14 # Standard gravitational parameter m^3 s^-2
#     a = 6731.0*10^3 # semi major axis m
#     alt = 5e5 # altitude of the target satellite in m
#     N = 100
#     n = 6
#     m = 3
#     tf = 1e4 # 5 revolutions
#     Δt = tf/(N-1)
#     x0 = [1.0, 1.0, 1.0, 0.3, 0.2, 0.0]
#     xf = zeros(n)
#     # mass
#     m_ego = 4.6
#
#     parameters = Dict("num_iter" => 50, # number of iteration of ADMM
#                   "ilqr_solver_iter" => 1, # number of iterations of the inner solver (ilqr = unconstrained)
#                   "al_solver_iter" => 100, # number of iterations of the inner solver (al = constrained)
#                   "N" => N, # horizon
#                   "n"=> n, # state size
#                   "m"=> m, # control size
#                   "Δt" => Δt, # time step
#                   "tf" => tf, # final time
#                   "ρ" => 1e-1, # weighting parameter for augmented lagrangian
#                   "α" => 1.0, # weighting parameter
#                   "X0" => zeros(n, N),#reshape(repeat(x0, outer = [N, 1]), (n, N)), # initial state trajectory
#                   "U0" => zeros(m, N-1), # initial control trajectory
#                   "ν0" => zeros(m, N-1), # initial dual
#                   "Y0" => zeros(m, N-1), # initial dummy
#                   "x0" => x0, # initial state
#                   "xf" => xf, # final state
#                   "t_ref" => 1.0, # reference duration
#                   "l_ref" => 1.0, # reference length for delta ego target
#                   "v_ref" => 1.0, # reference velocity for delta ego target
#                   "a_ref" => 1.0, # reference acceleration for delta ego target
#                   "u_ref" => 1.0, # reference control for ego
#                   "using_constraints" => true, # using constraints on control
#                   "using_final_constraints" => true, # using constraints on control
#                   "complete_results" => true, # using constraints on control
#                   "stage_plot" => false, # using constraints on control
#                   "stage_plot_freq" => 10, # using constraints on control
#                   "linearity" => true, # using linear dynamics
#                   "u_min" => -2e-1, # lower bound on u
#                   "u_max" => 2e-1, # upper bound on u
#                   "Q" => 0.0 * Matrix{Float64}(I, n, n), # Quadratic stage cost for states (n,n)
#                   "R" => 1e-20 * Matrix{Float64}(I, m, m), # Quadratic stage cost for controls (m,m)
#                   "H" => zeros(m, n), # Quadratic Cross-coupling for state and controls (m,n)
#                   "q" => zeros(n), # Linear term on states (n,)
#                   "r" => zeros(m), # Linear term on controls (m,)
#                   "c" => 0.0, # constant term
#                   "Qf" => 10.0 * Matrix{Float64}(I, n, n), # Quadratic final cost for terminal state (n,n)
#                   "qf" => zeros(n), # Linear term on terminal state (n,)
#                   "cf" => 0.0, # constant term (terminal)
#                   "μ" => μ, # Standard gravitational parameter m^3 s^-2
#                   "a" => a, # semi major axis m
#                   "orbit_radius" => a+alt, # semi major axis m
#                   "m_ego" => m_ego, # mass of the ego satellite kg
#                    )
#     return parameters
# end

function scale_lin_parameters(parameters)
    x0 = parameters["x0"]
    xf = parameters["xf"]
    tf = parameters["tf"]
    Δt = parameters["Δt"]
    m_ego = parameters["m_ego"]

    # Scale
    l_ref = max(norm(x0[1:3] - xf[1:3]), 1)
    v_ref = max(norm(x0[4:6] - xf[4:6]), 1)
    t_ref = l_ref / v_ref
    # a_ref = l_ref / (t_ref*Δt)
    a_ref = l_ref / (t_ref^2)
    u_ref = a_ref * m_ego
    # Scaling
    tf /= t_ref
    Δt /= t_ref
    x0[1:3] /= l_ref
    x0[4:6] /= v_ref
    xf[1:3] /= l_ref
    xf[4:6] /= v_ref
    # println("t_ref = ", t_ref)
    # println("l_ref = ", l_ref)
    # println("v_ref = ", v_ref)
    # println("a_ref = ", a_ref)
    # println("u_ref = ", u_ref)

    parameters["Δt"] = Δt
    parameters["tf"] = tf
    parameters["x0"] = x0
    parameters["xf"] = xf
    parameters["t_ref"] = t_ref
    parameters["l_ref"] = l_ref
    parameters["v_ref"] = v_ref
    parameters["a_ref"] = a_ref
    parameters["u_ref"] = u_ref
    parameters["u_min"] /= u_ref
    parameters["u_max"] /= u_ref
end

function update_scaling_lin_parameters(x0, parameters)
    t_ref = parameters["t_ref"]
    l_ref = parameters["l_ref"]
    v_ref = parameters["v_ref"]
    u_ref = parameters["u_ref"]

    # rescale everything to the origin
    parameters["Δt"] *= t_ref
    parameters["tf"] *= t_ref
    parameters["xf"][1:3] *= l_ref
    parameters["xf"][4:6] *= v_ref
    parameters["u_min"] *= u_ref
    parameters["u_max"] *= u_ref

    parameters["x0"] = x0
    scale_lin_parameters(parameters)
end

function define_non_lin_parameters()
    μ = 3.99*10^14 # Standard gravitational parameter m^3 s^-2
    a = 6731.0*10^3 # semi major axis m
    alt = 5e5 # altitude
    N = 100
    n = 12
    m = 3
    tf = 1000.0
    Δt = tf/(N-1)
    r0 = [a + alt, 0.0, 0.0]
    v0 = [0.0, sqrt(μ/norm(r0)), 0.0]

    x0 = [[0, -100, 0]; r0; [1.0, 2.0, 3.0]; v0]
    xf = zeros(n) # ### we need to get rid of penalty on x4,5,6 and x10,11,12
    Qf = 100 .* Diagonal([ones(3); zeros(3); ones(3); zeros(3)])

    # mass
    m_ego = 4.6
    m_target = 4.6

    parameters = Dict("num_iter" => 30, # number of iteration of ADMM
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
                  "t_ref" => 1.0, # reference duration
                  "l_ego_ref" => 1.0, # reference length for delta ego target
                  "l_target_ref" => 1.0, # reference length for target
                  "v_ego_ref" => 1.0, # reference velocity for delta ego target
                  "v_target_ref" => 1.0, # reference velocity for target
                  "a_ego_ref" => 1.0, # reference acceleration for delta ego target
                  "a_target_ref" => 1.0, # reference acceleration for target
                  "u_ref" => 1.0, # reference control for ego
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
                  "a" => a, # semi major axis m
                  "orbit_radius" => a+alt, # semi major axis m
                  "J2" => 1.75553*10^25, # J2 factor in m^5.s^-2
                  "ω" => [0, 0, 2*pi/((23*60 + 56)*60 + 4)], # Earth's rotational speed vector rad s^-1
                  "m_ego" => m_ego, # mass of the ego satellite kg
                  "m_target" => m_target, # mass of the target satellite kg
                  "A_ego" => 0.01, # section area of the ego satellite m^2
                  "A_target" => 0.01, # section area of the target satellite m^2
                  "Cd_ego" => 1.0, # Drag coefficient of the ego satellite s.u
                  "Cd_target" => 1.0, # Drag coefficient of the target satellite s.u
                   )
    return parameters
end

function scale_non_lin_parameters(parameters)
    μ = parameters["μ"]
    x0 = parameters["x0"]
    xf = parameters["xf"]
    tf = parameters["tf"]
    Δt = parameters["Δt"]
    m_ego = parameters["m_ego"]

    # Scale
    l_ego_ref = max(norm(x0[1:3] - xf[1:3]), 1)
    l_target_ref =  norm(x0[4:6])
    v_ego_ref = max(norm(x0[7:9] - xf[7:9]), 1)
    v_target_ref = sqrt(μ/l_target_ref)
    t_ref = l_ego_ref / v_ego_ref
    a_ego_ref = l_ego_ref / (t_ref^2)
    a_target_ref = μ/l_target_ref^2
    u_ref = a_ego_ref * m_ego

    # println("t_ref = ", t_ref)
    # println("l_ego_ref = ", l_ego_ref)
    # println("l_target_ref = ", l_target_ref)
    # println("v_ego_ref = ", v_ego_ref)
    # println("v_target_ref = ", v_target_ref)
    # println("a_ego_ref = ", a_ego_ref)
    # println("a_target_ref = ", a_target_ref)
    # println("u_ref = ", u_ref)
    # Scaling
    tf /= t_ref
    Δt /= t_ref
    x0[1:3] /= l_ego_ref
    x0[4:6] /= l_target_ref
    x0[7:9] /= v_ego_ref
    x0[10:12] /= v_target_ref
    xf[1:3] /= l_ego_ref
    xf[4:6] /= l_target_ref
    xf[7:9] /= v_ego_ref
    xf[10:12] /= v_target_ref

    parameters["Δt"] = Δt
    parameters["tf"] = tf
    parameters["x0"] = x0
    parameters["xf"] = xf
    parameters["t_ref"] = t_ref
    parameters["l_ego_ref"] = l_ego_ref
    parameters["l_target_ref"] = l_target_ref
    parameters["v_ego_ref"] = v_ego_ref
    parameters["v_target_ref"] = v_target_ref
    parameters["a_ego_ref"] = a_ego_ref
    parameters["a_target_ref"] = a_target_ref
    parameters["u_ref"] = u_ref
    parameters["u_min"] /= u_ref
    parameters["u_max"] /= u_ref
end
