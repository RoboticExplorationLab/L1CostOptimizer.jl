function define_lin_constrained_parameters()
    μ = 3.99*10^14 # Standard gravitational parameter m^3 s^-2
    a = 6731.0*10^3 # semi major axis m
    alt = 5e5 # altitude of the target satellite in m
    N = 100
    n = 6
    m = 3
    tf = 6000 # 1 revolutions
    Δt = tf/(N-1)

    x_target = circular_orbit(lin_parameters["orbit_radius"], 0, lin_parameters["μ"])
    Δr0 = [20.0, 30.0, 10.0] # initial position delta
    Δrd0 = [0.4, 0.6, -0.1] # intial velocity delta
    x0_drift = [Δr0; x_target[1:3]; Δrd0; x_target[4:6]] # initial state

    # Initial state obtained using drift
    x0 = [-79514.2, -720.471, 29.3723, 0.788372, -82.5619, 0.0958906]
    xf = zeros(n)
    # mass
    m_ego = 4.6

    parameters = Dict("num_iter" => 30,# number of iteration of ADMM
                  "ilqr_solver_iter" => 1, # number of iterations of the inner solver (ilqr = unconstrained)
                  "al_solver_iter" => 100, # number of iterations of the inner solver (al = constrained)
                  "scale_y" => 1.1, # helpful parameter for fast convergence
                  "N" => N, # horizon
                  "n"=> n, # state size
                  "m"=> m, # control size
                  "Δt" => Δt, # time step
                  "tf" => tf, # final time
                  "N_drift" => 10000, # drifting nodes
                  "tf_drift" => 12*60*60, # drifting time
                  "x0_drift" => x0_drift, # drifting time
                  "ρ" => 1e0, # weighting parameter for augmented lagrangian
                  "α" => 1.0, # weighting parameter
                  "X0" => zeros(n, N),#reshape(repeat(x0, outer = [N, 1]), (n, N)), # initial state trajectory
                  "U0" => zeros(m, N-1), # initial control trajectory
                  "ν0" => zeros(m, N-1), # initial dual
                  "Y0" => zeros(m, N-1), # initial dummy
                  "x0" => x0, # initial state
                  "xf" => xf, # final state
                  "t_ref" => 1.0, # reference duration
                  "l_ref" => 1.0, # reference length for delta ego target
                  "v_ref" => 1.0, # reference velocity for delta ego target
                  "a_ref" => 1.0, # reference acceleration for delta ego target
                  "u_ref" => 1.0, # reference control for ego
                  "using_constraints" => true, # using constraints on control
                  "using_final_constraints" => true, # using constraints on control
                  "complete_results" => false, # using constraints on control
                  "stage_plot" => false, # using constraints on control
                  "stage_plot_freq" => 5, # using constraints on control
                  "plot_format" => "eps", # using constraints on control
                  "timing" => true, # timing the function
                  "linearity" => true, # using linear dynamics
                  "u_min" => -2e-1, # lower bound on u
                  "u_max" => 2e-1, # upper bound on u
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
                  "orbit_radius" => a+alt, # semi major axis m
                  "m_ego" => m_ego, # mass of the ego satellite kg
                   )
    return parameters
end

function define_lin_unconstrained_parameters()
    μ = 3.99*10^14 # Standard gravitational parameter m^3 s^-2
    a = 6731.0*10^3 # semi major axis m
    alt = 5e5 # altitude of the target satellite in m
    N = 100
    n = 6
    m = 3
    tf = 6000 # 1 revolutions
    Δt = tf/(N-1)

    x_target = circular_orbit(lin_parameters["orbit_radius"], 0, lin_parameters["μ"])
    Δr0 = [20.0, 30.0, 10.0] # initial position delta
    Δrd0 = [0.4, 0.6, -0.1] # intial velocity delta
    x0_drift = [Δr0; x_target[1:3]; Δrd0; x_target[4:6]] # initial state

    # Initial state obtained using drift
    x0 = [-79514.2, -720.471, 29.3723, 0.788372, -82.5619, 0.0958906]
    xf = zeros(n)
    # mass
    m_ego = 4.6

    parameters = Dict("num_iter" => 120, # number of iteration of ADMM
                  "ilqr_solver_iter" => 1, # number of iterations of the inner solver (ilqr = unconstrained)
                  "al_solver_iter" => 100, # number of iterations of the inner solver (al = constrained)
                  "scale_y" => 1.1, # helpful parameter for fast convergence
                  "N" => N, # horizon
                  "n"=> n, # state size
                  "m"=> m, # control size
                  "Δt" => Δt, # time step
                  "tf" => tf, # final time
                  "N_drift" => 10000, # drifting nodes
                  "tf_drift" => 12*60*60, # drifting time
                  "x0_drift" => x0_drift, # drifting time
                  "ρ" => 1e-1, # weighting parameter for augmented lagrangian
                  "α" => 1.0, # weighting parameter
                  "X0" => zeros(n, N),#reshape(repeat(x0, outer = [N, 1]), (n, N)), # initial state trajectory
                  "U0" => zeros(m, N-1), # initial control trajectory
                  "ν0" => zeros(m, N-1), # initial dual
                  "Y0" => zeros(m, N-1), # initial dummy
                  "x0" => x0, # initial state
                  "xf" => xf, # final state
                  "t_ref" => 1.0, # reference duration
                  "l_ref" => 1.0, # reference length for delta ego target
                  "v_ref" => 1.0, # reference velocity for delta ego target
                  "a_ref" => 1.0, # reference acceleration for delta ego target
                  "u_ref" => 1.0, # reference control for ego
                  "using_constraints" => false, # using constraints on control
                  "using_final_constraints" => false, # using constraints on control
                  "complete_results" => false, # using constraints on control
                  "stage_plot" => false, # using constraints on control
                  "stage_plot_freq" => 5, # using constraints on control
                  "plot_format" => "eps", # using constraints on control
                  "timing" => false, # timing the function
                  "linearity" => true, # using linear dynamics
                  "u_min" => -2e-1, # lower bound on u
                  "u_max" => 2e-1, # upper bound on u
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
                  "orbit_radius" => a+alt, # semi major axis m
                  "m_ego" => m_ego, # mass of the ego satellite kg
                   )
    return parameters
end

function define_non_lin_unconstrained_parameters()
    μ = 3.99*10^14 # Standard gravitational parameter m^3 s^-2
    a = 6731.0*10^3 # semi major axis m
    alt = 5e5 # altitude
    N = 100
    n = 12
    m = 3
    tf = 6000.0
    Δt = tf/(N-1)
    r0 = [a + alt, 0.0, 0.0]
    v0 = [0.0, sqrt(μ/norm(r0)), 0.0]

    x_target = circular_orbit(lin_parameters["orbit_radius"], 0, lin_parameters["μ"])
    Δr0 = [20.0, 30.0, 10.0] # initial position delta
    Δrd0 = [0.4, 0.6, -0.1] # intial velocity delta
    x0_drift = [Δr0; x_target[1:3]; Δrd0; x_target[4:6]] # initial state

    x0 = [[0, -100, 0]; r0; zeros(3); v0]
    xf = zeros(n) # ### we need to get rid of penalty on x4,5,6 and x10,11,12
    Qf = 100 .* Diagonal([ones(3); zeros(3); ones(3); zeros(3)])

    # mass
    m_ego = 4.6 # kg
    m_target = m_ego

    parameters = Dict("num_iter" => 170, # number of iteration of ADMM
                  "ilqr_solver_iter" => 1, # number of iterations of the inner solver (ilqr = unconstrained)
                  "al_solver_iter" => 100, # number of iterations of the inner solver (al = constrained
                  "scale_y" => 1.1, # helpful parameter for fast convergence
                  "N" => N, # horizon
                  "n"=> n, # state size
                  "m"=> m, # control size
                  "Δt" => Δt, # time step
                  "tf" => tf, # final time
                  "N_drift" => 10000, # drifting nodes
                  "tf_drift" => 12*60*60, # drifting time
                  "x0_drift" => x0_drift, # drifting time
                  "ρ" => 1e-1, # weighting parameter for augmented lagrangian
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
                  "using_final_constraints" => false, # using constraints on control
                  "complete_results" => false, # using constraints on control
                  "stage_plot" => false, # using constraints on control
                  "stage_plot_freq" => 5, # using constraints on control
                  "plot_format" => "eps", # using constraints on control
                  "timing" => false, # timing the function
                  "linearity" => false, # using non linear dynamics
                  "u_min" => -2e-1, # lower bound on u
                  "u_max" => 2e-1, # upper bound on u
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

function define_non_lin_constrained_parameters()
    μ = 3.99*10^14 # Standard gravitational parameter m^3 s^-2
    a = 6731.0*10^3 # semi major axis m
    alt = 5e5 # altitude
    N = 100
    n = 12
    m = 3
    tf = 6000.0
    Δt = tf/(N-1)
    r0 = [a + alt, 0.0, 0.0]
    v0 = [0.0, sqrt(μ/norm(r0)), 0.0]

    x_target = circular_orbit(lin_parameters["orbit_radius"], 0, lin_parameters["μ"])
    Δr0 = [20.0, 30.0, 10.0] # initial position delta
    Δrd0 = [0.4, 0.6, -0.1] # intial velocity delta
    x0_drift = [Δr0; x_target[1:3]; Δrd0; x_target[4:6]] # initial state

    x0 = [[0, -100, 0]; r0; zeros(3); v0]
    xf = [0.0, 0.0, 0.0, 6.80966e6, 2.43227e6, 0.0, 0.0, 0.0, 0.0, -2498.63, 6995.42, 0.0] #6000 sec
    # xf = [0.0, 0.0, 0.0, -1.54994e6, -7.06293e6, 0.0, 0.0, 0.0, 0.0, 7255.61, -1592.23, 0.0] #10000 sec
    # xf = [0.0, 0.0, 0.0, -6.74266e6, -2.61225e6, 0.0, 0.0, 0.0, 0.0, 2683.52, -6926.6, 0.0] #3000 sec

    # Qf = 1 .* Diagonal([ones(3); zeros(3); ones(3); zeros(3)])
    Qf = 0.1 .* Diagonal(ones(12))

    # mass
    m_ego = 4.6 # kg
    m_target = m_ego

    # ν0 = zeros(m, N-1)
    # ν0[2,:] = 0.96*ones(N-1)
    parameters = Dict("num_iter" => 200, # number of iteration of ADMM
                  "ilqr_solver_iter" => 1, # number of iterations of the inner solver (ilqr = unconstrained)
                  "al_solver_iter" => 10, # number of iterations of the inner solver (al = constrained)
                  "scale_y" => 1.5, # helpful parameter for fast convergence
                  "N" => N, # horizon
                  "n"=> n, # state size
                  "m"=> m, # control size
                  "Δt" => Δt, # time step
                  "tf" => tf, # final time
                  "N_drift" => 10000, # drifting nodes
                  "tf_drift" => 12*60*60, # drifting time
                  "x0_drift" => x0_drift, # drifting time
                  "ρ" => 1e0, # weighting parameter for augmented lagrangian
                  "α" => 1e0, # weighting parameter
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
                  "using_constraints" => true, # using constraints on control
                  "using_final_constraints" => true, # using constraints on control
                  "complete_results" => true, # using constraints on control
                  "stage_plot" => true, # using constraints on control
                  "stage_plot_freq" => 5, # using constraints on control
                  "plot_format" => "eps", # using constraints on control
                  "timing" => false, # timing the function
                  "linearity" => false, # using non linear dynamics
                  "u_min" => -4e-1, # lower bound on u
                  "u_max" => 4e-1, # upper bound on u
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

function lin_cons_example(ρ)
    lin_cons_parameters = define_lin_constrained_parameters()
    non_lin_parameters = define_non_lin_parameters()
    lin_cons_parameters["ρ"] = ρ

    # Define initial state
    non_lin_parameters["x0"] = lin_cons_parameters["x0_drift"]
    println("x0 = ", non_lin_parameters["x0"])

    # Propagate the non linear dynamics forward without control
    tf_drift = lin_cons_parameters["tf_drift"]
    N_drift = lin_cons_parameters["N_drift"]
    X_history = drifting_simulation(N_drift, tf_drift, non_lin_parameters)
    U_history = zeros(3, N_drift)
    T_history = [k*tf_drift/(N_drift-1) for k=0:N_drift-1]

    filename = "history_after_drift_" * "tf_" * string(tf_drift) * "_N_" * string(N_drift) * ".png"
    save_history(T_history, X_history, U_history, filename, non_lin_parameters)

    x0_full = X_history[:,N_drift]
    # We recover from the initial drift using a open loop controller reyling on the linear dynamics model.
    lin_cons_parameters["orbit_radius"] = norm(x0_full[4:6]) # we set the orbit radius of the cw model.
    x0_cw = full_to_cw(x0_full)
    lin_cons_parameters["x0"] = x0_cw
    println("x0_full = ", x0_full)
    println("x0_cw = ", x0_cw)

    # Scale the parameters of the linear model
    scale_lin_parameters(lin_cons_parameters)
    println("lin_cons_parameters[x0] = ", lin_cons_parameters["x0"])

    # Compute the control sequence
    X, U, Y, ν, cost_history, constraint_violation = l1_solver(lin_cons_parameters)
    filename = "lin_cons_" * "_rho_" * string(lin_cons_parameters["ρ"])
    filename *= "_iter_" * string(lin_cons_parameters["num_iter"]) * "_Qf_" * string(lin_cons_parameters["Qf"][1,1])
    save_results(X, U, Y, ν, cost_history, filename, lin_cons_parameters)
end

function lin_uncons_example(ρ)
    lin_uncons_parameters = define_lin_unconstrained_parameters()
    non_lin_parameters = define_non_lin_parameters()
    lin_uncons_parameters["ρ"] = ρ

    # Define initial state
    non_lin_parameters["x0"] = lin_uncons_parameters["x0_drift"]
    println("x0 = ", non_lin_parameters["x0"])

    # Propagate the non linear dynamics forward without control
    tf_drift = lin_uncons_parameters["tf_drift"]
    N_drift = lin_uncons_parameters["N_drift"]
    X_history = drifting_simulation(N_drift, tf_drift, non_lin_parameters)
    U_history = zeros(3, N_drift)
    T_history = [k*tf_drift/(N_drift-1) for k=0:N_drift-1]

    filename = "history_after_drift_" * "tf_" * string(tf_drift) * "_N_" * string(N_drift) * ".png"
    save_history(T_history, X_history, U_history, filename, non_lin_parameters)

    x0_full = X_history[:,N_drift]
    # We recover from the initial drift using a open loop controller reyling on the linear dynamics model.
    lin_uncons_parameters["orbit_radius"] = norm(x0_full[4:6]) # we set the orbit radius of the cw model.
    x0_cw = full_to_cw(x0_full)
    lin_uncons_parameters["x0"] = x0_cw
    println("x0_full = ", x0_full)
    println("x0_cw = ", x0_cw)

    # Scale the parameters of the linear model
    scale_lin_parameters(lin_uncons_parameters)
    println("lin_uncons_parameters[x0] = ", lin_uncons_parameters["x0"])

    # Compute the control sequence
    X, U, Y, ν, cost_history, constraint_violation = l1_solver(lin_uncons_parameters)
    filename = "lin_uncons_" * "_rho_" * string(lin_uncons_parameters["ρ"])
    filename *= "_iter_" * string(lin_uncons_parameters["num_iter"]) * "_Qf_" * string(lin_uncons_parameters["Qf"][1,1])
    save_results(X, U, Y, ν, cost_history, filename, lin_uncons_parameters)
end

function non_lin_uncons_example(ρ)
    non_lin_uncons_parameters = define_non_lin_unconstrained_parameters()
    non_lin_uncons_parameters["ρ"] = ρ

    # Define initial state
    println("x0 = ", non_lin_uncons_parameters["x0_drift"])
    non_lin_uncons_parameters["x0"] = non_lin_uncons_parameters["x0_drift"]
    # Propagate the non linear dynamics forward without control
    tf_drift = non_lin_uncons_parameters["tf_drift"]
    N_drift = non_lin_uncons_parameters["N_drift"]
    X_history = drifting_simulation(N_drift, tf_drift, non_lin_uncons_parameters)
    U_history = zeros(3, N_drift)
    T_history = [k*tf_drift/(N_drift-1) for k=0:N_drift-1]
    filename = "history_after_drift_" * "tf_" * string(tf_drift) * "_N_" * string(N_drift) * ".png"
    save_history(T_history, X_history, U_history, filename, non_lin_uncons_parameters)

    x0_full = X_history[:,N_drift]
    # We recover from the initial drift using a open loop controller relying on the linear dynamics model.
    non_lin_uncons_parameters["orbit_radius"] = norm(x0_full[4:6]) # we set the orbit radius of the cw model.
    non_lin_uncons_parameters["x0"] = x0_full
    println("x0_full = ", x0_full)

    # Scale the parameters of the linear model
    scale_non_lin_parameters(non_lin_uncons_parameters)
    println("non_lin_uncons_parameters[x0] = ", non_lin_uncons_parameters["x0"])

    # Compute the control sequence
    X, U, Y, ν, cost_history, constraint_violation = l1_solver(non_lin_uncons_parameters)
    filename = "non_lin_uncons" * "_rho_" * string(non_lin_uncons_parameters["ρ"])
    filename *= "_iter_" * string(non_lin_uncons_parameters["num_iter"]) * "_Qf_" * string(non_lin_uncons_parameters["Qf"][1,1])
    save_results(X, U, Y, ν, cost_history, filename, non_lin_uncons_parameters)
end

function non_lin_cons_example(ρ)
    non_lin_cons_parameters = define_non_lin_constrained_parameters()
    non_lin_cons_parameters["ρ"] = ρ

    # Define initial state
    println("x0 = ", non_lin_cons_parameters["x0_drift"])
    non_lin_cons_parameters["x0"] = non_lin_cons_parameters["x0_drift"]
    # non_lin_cons_parameters["xf"] =
    # Propagate the non linear dynamics forward without control
    tf_drift = non_lin_cons_parameters["tf_drift"]
    N_drift = non_lin_cons_parameters["N_drift"]
    X_history = drifting_simulation(N_drift, tf_drift, non_lin_cons_parameters)
    U_history = zeros(3, N_drift)
    T_history = [k*tf_drift/(N_drift-1) for k=0:N_drift-1]
    filename = "history_after_drift_" * "tf_" * string(tf_drift) * "_N_" * string(N_drift) * ".png"
    save_history(T_history, X_history, U_history, filename, non_lin_cons_parameters)

    x0_full = X_history[:,N_drift]
    # We recover from the initial drift using a open loop controller relying on the linear dynamics model.
    non_lin_cons_parameters["orbit_radius"] = norm(x0_full[4:6]) # we set the orbit radius of the cw model.
    non_lin_cons_parameters["x0"] = x0_full
    println("x0_full = ", x0_full)

    # Scale the parameters of the linear model
    scale_non_lin_parameters(non_lin_cons_parameters)
    println("non_lin_cons_parameters[x0] = ", non_lin_cons_parameters["x0"])

    # Compute the control sequence
    X, U, Y, ν, cost_history, constraint_violation = l1_solver(non_lin_cons_parameters)
    filename = "non_lin_cons" * "_rho_" * string(non_lin_cons_parameters["ρ"])
    filename *= "_iter_" * string(non_lin_cons_parameters["num_iter"]) * "_Qf_" * string(non_lin_cons_parameters["Qf"][1,1])
    save_results(X, U, Y, ν, cost_history, filename, non_lin_cons_parameters)
    # println("X_final = ", X[:,end])
end
