function circular_orbit(orbit_radius, θ, μ)
    # Compute the state [x, y, z, xd, yd, zd]
    # for a circular orbit in the XY plane.
    state = zeros(6)
    state[1:3] = orbit_radius.*[cos(θ), sin(θ), 0]
    velocity = sqrt(μ/orbit_radius)
    state[4:6] = velocity.*[-sin(θ), cos(θ), 0]
    return state
end

function full_to_reduced_state(x_full)
    # Convert a full state ∈ R^12 to a reduced state ∈ R^6.
    # The full state is the pne corresponding to the nonlinear dynamics formulation
    # and the reduced state corresponds to the linear dynamics.
    μ = 3.99*10^14 # Standard gravitational parameter m^3 s^-2
    r_target = x_full[4:6]
    rd_target = x_full[10:12]
    r_ego = r_target - x_full[1:3]
    rd_ego = rd_target - x_full[7:9]

    # define the axis of the C-W frame as deined in
    # R. Wiltshire and W. Clohessy,
    # “Terminal guidance system for satellite rendezvous”.
    ey_cw = r_target / norm(r_target)
    ez_cw = cross(ey_cw, rd_target)
    ez_cw = ez_cw / norm(ez_cw)
    ex_cw = cross(ey_cw, ez_cw)
    R_wc = [ex_cw ey_cw ez_cw]
    r = r_ego
    v = rd_ego
    ω = cross(r,v) / norm(cross(r,v))
    orbit_radius = norm(r_target)
    ω *= sqrt(μ * orbit_radius) / norm(r_target)^2 ###
    ω_hat = [0 -ω[3] ω[2] ; ω[3] 0 -ω[1] ; -ω[2] ω[1] 0]
    x_cw = zeros(6)
    x_cw[1:3] = R_wc'*(r_ego .- r_target)
    x_cw[4:6] = R_wc'*(rd_ego .- rd_target) - ω_hat*R_wc'*(r_ego .- r_target)
    return x_cw
end

function rk4_step(f!, x, u, Δt)
    # Runge-Kutta 4
    k1 = k2 = k3 = k4 = zero(x)
    f!(k1, x, u);        k1 *= Δt;
    f!(k2, x + k1/2, u); k2 *= Δt;
    f!(k3, x + k2/2, u); k3 *= Δt;
    f!(k4, x + k3, u);   k4 *= Δt;
    x + (k1 + 2*k2 + 2*k3 + k4)/6
end

function drifting_simulation(n, m, N, tf, x0, dynamics)
    # Let the two satellites drift apart from each other starting in state x0
    # for a duration `tf` with the dynamics `dynamics`.
    X = zeros(n, N)
    X[:,1] = x0
    for k=2:N
        u = zeros(m)
        Δt = tf / (N-1)
        X[:,k] = rk4_step(dynamics, X[:,k-1], u, Δt)
    end
    return X
end

function rollout_dynamics(n, m, N, tf, x0, U, dynamics)
    # Let the two satellites evolve under control `U` starting in state x0
    # for a duration `tf` with the dynamics `dynamics`.
    X = zeros(n, N)
    X[:,1] = x0
    for k=2:N
        Δt = tf / (N-1)
        X[:,k] = rk4_step(dynamics, X[:,k-1], U[:,k-1], Δt)
    end
    return X
end

function initial_drift(parameters)
    # Let the two satellites drift apart from each other starting
    # according to the defined parameters.
    non_lin_uncons_parameters = define_non_lin_unconstrained_parameters()
    tf_drift = parameters["tf_drift"]
    N_drift = parameters["N_drift"]
    x0_drift = parameters["x0_drift"]
    function non_linear_dynamics!(ẋ,x,u)
        non_linear_dynamics(ẋ, x, u, non_lin_uncons_parameters)
    end
    n = non_lin_uncons_parameters["n"]
    m = non_lin_uncons_parameters["m"]
    X_history = drifting_simulation(n, m, N_drift, tf_drift, x0_drift,
        non_linear_dynamics!)
    x0_full = X_history[:,N_drift]
    return x0_full
end

function process_results(X_, U_, Y_, ν, cost_history, optimality_criterion,
    filename, iter, parameters)
    # Process the results of the solver to return data easy to plot.
    num_iter = parameters["num_iter"]
    N = parameters["N"]
    n = parameters["n"]
    Δt_rescaled = parameters["Δt"] * parameters["t_ref"]

    t_ref = parameters["t_ref"]
    T = [Δt_rescaled * i for i=0:N-1]

    # Rescaling
    if parameters["linearity"]
        l_ref = parameters["l_ref"]
        v_ref = parameters["v_ref"]
        u_ref = parameters["u_ref"]
        X = zero(X_)
        X[1:3,:] = X_[1:3,:] * l_ref
        X[4:6,:] = X_[4:6,:] * v_ref
        U = U_ * u_ref
        Y = Y_ * u_ref
        return T, X, U, Y
    elseif !parameters["linearity"]
        l_ego_ref = parameters["l_ego_ref"]
        l_target_ref = parameters["l_target_ref"]
        v_ego_ref = parameters["v_ego_ref"]
        v_target_ref = parameters["v_target_ref"]
        a_ego_ref = parameters["a_ego_ref"]
        a_target_ref = parameters["a_target_ref"]
        u_ref = parameters["u_ref"]
        X = zero(X_)
        X[1:3,:] = X_[1:3,:] * l_ego_ref
        X[4:6,:] = X_[4:6,:] * l_target_ref
        X[7:9,:] = X_[7:9,:] * v_ego_ref
        X[10:12,:] = X_[10:12,:] * v_target_ref
        U = U_ * u_ref
        Y = Y_ * u_ref
        X_cw = zeros(6,N)
        for k = 1:N
            X_cw[:,k] = full_to_reduced_state(X[:,k])
        end
        return T, X, U, Y, X_cw
    end
end

function save_results_file(X_, U_, Y_, ν, cost_history, optimality_criterion,
    filename, iter, parameters)
    # Saves files of the state, control trajectories as well as cost, optimality
    # across iterations.
    if parameters["linearity"]
        T, X, U, Y = process_results(X_, U_, Y_, ν, cost_history,
            optimality_criterion, filename, iter, parameters)
    elseif !parameters["linearity"]
        T, X, U, Y, X_cw = process_results(X_, U_, Y_, ν, cost_history,
            optimality_criterion, filename, iter, parameters)
    end
    JLD.save("animation/trajectory/" * filename * ".jld",
        "T", T, "X", X, "U", U, "Y", Y)
end
