function define_lin_constrained_parameters()
    # Defines the necessary parameters for testing the solver
    # in the linear dynamics with constraints case.

    μ = 3.99*10^14 # Standard gravitational parameter m^3 s^-2
    a = 6731.0*10^3 # Earth radius in m
    alt = 5e5 # Altitude of the target satellite in m
    orbit_radius = a+alt
    N = 100 # Number of node points
    n = 6 # State vector dimension
    m = 3 # Control vector dimension
    tf = 6000 # Trajectory duration in s (approx. = 1 revolutions)
    Δt = tf/(N-1)

    x_target = circular_orbit(orbit_radius, 0, μ) # inital target state
    Δr0 = [20.0, 30.0, 10.0] # initial position delta
    Δrd0 = [0.4, 0.6, -0.1] # intial velocity delta
    x0_drift = [Δr0; x_target[1:3]; Δrd0; x_target[4:6]] # initial state

    # Initial state obtained using drift
    x0 = zeros(n)
    xf = zeros(n)
    m_ego = 4.6

    parameters = Dict("num_iter" => 300, # number of iteration of ADMM
                  "ilqr_solver_iter" => 1, # max number of iterations of the inner solver (ilqr = unconstrained)
                  "al_solver_iter" => 5, # max number of iterations of the inner solver (al = constrained)
                  "stopping_criterion" => 1e-3, # stops the algo when ||U-Y||_2/dim(U) <= stopping_criterion
                  "N" => N, # node points
                  "n"=> n, # state size
                  "m"=> m, # control size
                  "Δt" => Δt, # time step
                  "tf" => tf, # final time
                  "N_drift" => 10000, # drifting nodes
                  "tf_drift" => 60*60, # drifting time
                  "x0_drift" => x0_drift, # intial state before drift
                  "ρ" => 1e0, # weighting parameter for augmented lagrangian
                  "α" => 1.0, # L1 cost parameter
                  "X0" => zeros(n, N), # initial state trajectory
                  "U0" => zeros(m, N-1), # initial control trajectory
                  "ν0" => zeros(m, N-1), # initial dual
                  "Y0" => zeros(m, N-1), # initial dummy
                  "x0" => x0, # initial state
                  "xf" => xf, # final state
                  "t_ref" => 1.0, # reference duration
                  "l_ref" => 1.0, # reference length
                  "v_ref" => 1.0, # reference velocity
                  "a_ref" => 1.0, # reference acceleration
                  "u_ref" => 1.0, # reference control for ego
                  "using_constraints" => true, # using constraints on control
                  "using_final_constraints" => true, # using constraints on final state
                  "complete_results" => false, # saving more plots
                  "stage_plot" => false, # saving stage plots
                  "stage_plot_freq" => 1, # stage plot frequency
                  "plot_format" => "png", # saving format
                  "show_result" => false, # show result in notebook
                  "verbose" => true, # show the optimaility criterion and cost during solve.
                  "timing" => false, # timing mode
                  "linearity" => true, # using linear dynamics
                  "u_min" => -5e-3, # lower bound on u
                  "u_max" => 5e-3, # upper bound on u
                  "Q" => 0.0 * Matrix{Float64}(I, n, n), # Quadratic stage cost for states (n,n)
                  "R" => 1e-40 * Matrix{Float64}(I, m, m), # Quadratic stage cost for controls (m,m)
                  "H" => zeros(m, n), # Quadratic Cross-coupling for state and controls (m,n)
                  "q" => zeros(n), # Linear term on states (n,)
                  "r" => zeros(m), # Linear term on controls (m,)
                  "c" => 0.0, # constant term
                  "Qf" => 1000.0 * Matrix{Float64}(I, n, n), # Quadratic final cost for terminal state (n,n)
                  "qf" => zeros(n), # Linear term on terminal state (n,)
                  "cf" => 0.0, # constant term (terminal)
                  "μ" => μ, # Standard gravitational parameter m^3 s^-2
                  "a" => a, # Earth radius m
                  "orbit_radius" => orbit_radius, # orbit radius of target m
                  "m_ego" => m_ego, # mass of the ego satellite kg
                   )
    return parameters
end

function define_lin_unconstrained_parameters()
    # Defines the necessary parameters for testing the solver
    # in the linear dyanmics without constraints case.

    μ = 3.99*10^14 # Standard gravitational parameter m^3 s^-2
    a = 6731.0*10^3 # Earth radius in m
    alt = 5e5 # Altitude of the target satellite in m
    orbit_radius = a+alt
    N = 100 # Number of node points
    n = 6 # State vector dimension
    m = 3 # Control vector dimension
    tf = 6000 # Trajectory duration in s (approx. = 1 revolutions)
    Δt = tf/(N-1)

    x_target = circular_orbit(orbit_radius, 0, μ) # Initial target state
    Δr0 = [20.0, 30.0, 10.0] # Initial position delta
    Δrd0 = [0.4, 0.6, -0.1] # Intial velocity delta
    x0_drift = [Δr0; x_target[1:3]; Δrd0; x_target[4:6]] # Initial state

    # Initial state obtained using drift
    x0 = zeros(n)
    xf = zeros(n)
    m_ego = 4.6

    parameters = Dict("num_iter" => 300, # number of iteration of ADMM
                  "ilqr_solver_iter" => 1, # number of iterations of the inner solver (ilqr = unconstrained)
                  "al_solver_iter" => 2, # number of iterations of the inner solver (al = constrained)
                  "scale_y" => 1.1, # helpful parameter for fast convergence
                  "stopping_criterion" => 1e-4, # stops the algo when ||U-Y||_2/dim(U) <= stopping_criterion
                  "N" => N, # node points
                  "n"=> n, # state size
                  "m"=> m, # control size
                  "Δt" => Δt, # time step
                  "tf" => tf, # final time
                  "N_drift" => 10000, # drifting nodes
                  "tf_drift" => 60*60, # drifting time
                  "x0_drift" => x0_drift, # initial state before drift
                  "ρ" => 1e-1, # weighting parameter for augmented lagrangian
                  "α" => 1.0, # L1 cost parameter
                  "X0" => zeros(n, N), # initial state trajectory
                  "U0" => zeros(m, N-1), # initial control trajectory
                  "ν0" => zeros(m, N-1), # initial dual
                  "Y0" => zeros(m, N-1), # initial dummy
                  "x0" => x0, # initial state
                  "xf" => xf, # final state
                  "t_ref" => 1.0, # reference duration
                  "l_ref" => 1.0, # reference length
                  "v_ref" => 1.0, # reference velocity
                  "a_ref" => 1.0, # reference acceleration
                  "u_ref" => 1.0, # reference control for ego
                  "using_constraints" => false, # using constraints on control
                  "using_final_constraints" => false, # using constraints on final state /!\ cannot be enforced using ILQR
                  "complete_results" => false, # saving more plots
                  "stage_plot" => false, # saving stage plots
                  "stage_plot_freq" => 1, # stage plot frequency
                  "plot_format" => "png", # saving format
                  "show_result" => false, # show result in notebook
                  "verbose" => true, # show the optimaility criterion and cost during solve.
                  "timing" => false, # timing mode
                  "linearity" => true, # using linear dynamics
                  "u_min" => -5e-3, # lower bound on u
                  "u_max" => 5e-3, # upper bound on u
                  "Q" => 0.0 * Matrix{Float64}(I, n, n), # Quadratic stage cost for states (n,n)
                  "R" => 1e-40 * Matrix{Float64}(I, m, m), # Quadratic stage cost for controls (m,m)
                  "H" => zeros(m, n), # Quadratic Cross-coupling for state and controls (m,n)
                  "q" => zeros(n), # Linear term on states (n,)
                  "r" => zeros(m), # Linear term on controls (m,)
                  "c" => 0.0, # constant term
                  "Qf" => 1000.0 * Matrix{Float64}(I, n, n), # Quadratic final cost for terminal state (n,n)
                  "qf" => zeros(n), # Linear term on terminal state (n,)
                  "cf" => 0.0, # constant term (terminal)
                  "μ" => μ, # Standard gravitational parameter m^3 s^-2
                  "a" => a, # Earth radius m
                  "orbit_radius" => orbit_radius, # orbit radius of target m
                  "m_ego" => m_ego, # mass of the ego satellite kg
                   )
    return parameters
end

function define_non_lin_unconstrained_parameters()
    # Defines the necessary parameters for testing the solver
    # in the nonlinear dyanmics without constraints case.

    μ = 3.99*10^14 # Standard gravitational parameter m^3 s^-2
    a = 6731.0*10^3 # Earth radius in m
    alt = 5e5 # Altitude of the target satellite in m
    orbit_radius = a+alt
    N = 100 # Number of node points
    n = 12 # State vector dimension
    m = 3 # Control vector dimension
    tf = 6000 # Trajectory duration in s (approx. = 1 revolutions)
    Δt = tf/(N-1)
    r0 = [a + alt, 0.0, 0.0]
    v0 = [0.0, sqrt(μ/norm(r0)), 0.0]

    x_target = circular_orbit(orbit_radius, 0, μ)
    Δr0 = [20.0, 30.0, 10.0] # initial position delta
    Δrd0 = [0.4, 0.6, -0.1] # intial velocity delta
    x0_drift = [Δr0; x_target[1:3]; Δrd0; x_target[4:6]] # initial state before drift

    x0 = [[0, -100, 0]; r0; zeros(3); v0]
    xf = zeros(n)
    Qf = 1000 .* Array(Diagonal([ones(3); zeros(3); ones(3); zeros(3)]))

    m_ego = 4.6 # mass of the ego satellite in kg
    m_target = m_ego # mass of the target satellite in kg

    parameters = Dict("num_iter" => 300, # number of iteration of ADMM
                  "ilqr_solver_iter" => 1, # number of iterations of the inner solver (ilqr = unconstrained)
                  "al_solver_iter" => 2, # number of iterations of the inner solver (al = constrained
                  "scale_y" => 1.5, # helpful parameter for fast convergence
                  "stopping_criterion" => 1e-3, # stops the algo when ||U-Y||_2/dim(U) <= stopping_criterion
                  "N" => N, # horizon
                  "n"=> n, # state size
                  "m"=> m, # control size
                  "Δt" => Δt, # time step
                  "tf" => tf, # final time
                  "N_drift" => 10000, # drifting nodes
                  "tf_drift" => 60*60, # drifting time
                  "x0_drift" => x0_drift, # drifting time
                  "ρ" => 1e-1, # weighting parameter for augmented lagrangian
                  "α" => 1.0, # L1 cost parameter
                  "X0" => zeros(n, N), # initial state trajectory
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
                  "using_final_constraints" => false, # using constraints on final state
                  "complete_results" => false, # saving more plots
                  "stage_plot" => false, # saving stage plots
                  "stage_plot_freq" => 1, # stage plot frequency
                  "plot_format" => "png", # saving format
                  "show_result" => false, # show result in notebook
                  "verbose" => true, # show the optimaility criterion and cost during solve.
                  "timing" => false, # timing mode
                  "linearity" => false, # using non linear dynamics
                  "u_min" => -5e-3, # lower bound on u
                  "u_max" => 5e-3, # upper bound on u
                  "Q" => 0.0 * Matrix{Float64}(I, n, n), # Quadratic stage cost for states (n,n)
                  "R" => 1e-40 * Matrix{Float64}(I, m, m), # Quadratic stage cost for controls (m,m)
                  "H" => zeros(m, n), # Quadratic Cross-coupling for state and controls (m,n)
                  "q" => zeros(n), # Linear term on states (n,)
                  "r" => zeros(m), # Linear term on controls (m,)
                  "c" => 0.0, # constant term
                  "Qf" => Qf, # Quadratic final cost for terminal state (n,n)
                  "qf" => zeros(n), # Linear term on terminal state (n,)
                  "cf" => 0.0, # constant term (terminal)
                  "μ" => μ, # Standard gravitational parameter m^3 s^-2
                  "a" => a, # Earth radius m
                  "orbit_radius" => orbit_radius, # semi major axis m
                  "ω" => [0, 0, 2*pi/((23*60 + 56)*60 + 4)], # Earth's rotational speed vector rad s^-1
                  "m_ego" => m_ego, # mass of the ego satellite kg
                  "m_target" => m_target, # mass of the target satellite kg
                  "A_ego" => 0.01, # section area of the ego satellite m^2
                  "A_target" => 0.01, # section area of the target satellite m^2
                  "Cd_ego" => 1.0, # Drag coefficient of the ego satellite n.u
                  "Cd_target" => 1.0, # Drag coefficient of the target satellite n.u
                   )
    return parameters
end

function define_non_lin_constrained_parameters()
    # Defines the necessary parameters for testing the solver
    # in the nonlinear dyanmics with constraints case.

    μ = 3.99*10^14 # Standard gravitational parameter m^3 s^-2
    a = 6731.0*10^3 # Earth radius in m
    alt = 5e5 # Altitude of the target satellite in m
    orbit_radius = a+alt
    N = 100 # Number of node points
    n = 12 # State vector dimension
    m = 3 # Control vector dimension
    tf = 6000 # Trajectory duration in s (approx. = 1 revolutions)
    Δt = tf/(N-1)
    r0 = [a + alt, 0.0, 0.0]
    v0 = [0.0, sqrt(μ/norm(r0)), 0.0]

    x_target = circular_orbit(orbit_radius, 0, μ)
    Δr0 = [20.0, 30.0, 10.0] # initial position delta
    Δrd0 = [0.4, 0.6, -0.1] # intial velocity delta
    x0_drift = [Δr0; x_target[1:3]; Δrd0; x_target[4:6]] # initial state

    x0 = [[0, -100, 0]; r0; zeros(3); v0]
    xf = zeros(n)

    Qf = 1000 .* Array(Diagonal([ones(3); zeros(3); ones(3); zeros(3)]))

    m_ego = 4.6 # mass of the ego satellite in kg
    m_target = m_ego # mass of the target satellite in kg

    parameters = Dict("num_iter" => 300, # number of iteration of ADMM
                  "ilqr_solver_iter" => 1, # number of iterations of the inner solver (ilqr = unconstrained)
                  "al_solver_iter" => 5, # number of iterations of the inner solver (al = constrained)
                  "stopping_criterion" => 1e-3, # stops the algo when ||U-Y||_2/dim(U) <= stopping_criterion
                  "N" => N, # horizon
                  "n"=> n, # state size
                  "m"=> m, # control size
                  "Δt" => Δt, # time step
                  "tf" => tf, # final time
                  "N_drift" => 10000, # drifting nodes
                  "tf_drift" => 60*60, # drifting time
                  "x0_drift" => x0_drift, # intial state before drift
                  "ρ" => 1e0, # weighting parameter for augmented lagrangian
                  "α" => 1.0, # L1 cost parameter
                  "X0" => zeros(n, N), # initial state trajectory
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
                  "using_constraints" => true, # using constraints on control
                  "using_final_constraints" => false, # using constraints on final state
                  "complete_results" => false, # saving more plots
                  "stage_plot" => false, # saving stage plots
                  "stage_plot_freq" => 1, # stage plot frequency
                  "plot_format" => "png", # saving format
                  "show_result" => false, # show result in notebook
                  "verbose" => true, # show the optimaility criterion and cost during solve.
                  "timing" => false, # timing mode
                  "linearity" => false, # using non linear dynamics
                  "u_min" => -5e-3, # lower bound on u
                  "u_max" => 5e-3, # upper bound on u
                  "Q" => 0.0 * Matrix{Float64}(I, n, n), # Quadratic stage cost for states (n,n)
                  "R" => 1e-40 * Matrix{Float64}(I, m, m), # Quadratic stage cost for controls (m,m)
                  "H" => zeros(m, n), # Quadratic Cross-coupling for state and controls (m,n)
                  "q" => zeros(n), # Linear term on states (n,)
                  "r" => zeros(m), # Linear term on controls (m,)
                  "c" => 0.0, # constant term
                  "Qf" => Qf, # Quadratic final cost for terminal state (n,n)
                  "qf" => zeros(n), # Linear term on terminal state (n,)
                  "cf" => 0.0, # constant term (terminal)
                  "μ" => μ, # Standard gravitational parameter m^3 s^-2
                  "a" => a, # Earth radius m
                  "orbit_radius" => orbit_radius, # orbit radius of target m
                  "ω" => [0, 0, 2*pi/((23*60 + 56)*60 + 4)], # Earth's rotational speed vector rad s^-1
                  "m_ego" => m_ego, # mass of the ego satellite kg
                  "m_target" => m_target, # mass of the target satellite kg
                  "A_ego" => 0.01, # section area of the ego satellite m^2
                  "A_target" => 0.01, # section area of the target satellite m^2
                  "Cd_ego" => 1.0, # Drag coefficient of the ego satellite n.u
                  "Cd_target" => 1.0, # Drag coefficient of the target satellite n.u
                   )
    return parameters
end
