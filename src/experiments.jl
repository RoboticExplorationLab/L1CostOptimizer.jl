function run_experiments(parameters)
    N = parameters["N"]
    n = parameters["n"]
    m = parameters["m"]
    if parameters["linearity"]
        QF = [10.0^i .* Diagonal(ones(6)) for i=0:0]
    elseif !parameters["linearity"]
        QF = [10.0^i .* Diagonal([ones(3); zeros(3); ones(3); zeros(3)]) for i=0:0]
    end
    P = [10.0^i for i=0:0]
    logs = Dict("X_log" => zeros(length(P), n, N),
        "U_log" => zeros(length(P), m, N-1),
        "Y_log" => zeros(length(P), m, N-1),
        "ν_log" => zeros(length(P), m, N-1))
    for j=1:length(QF)
        for k=1:length(P)
            parameters = define_lin_parameters()
            parameters["Qf"] = QF[j]
            parameters["ρ"] = P[k]
            X, U, Y, ν, cost_history, constraint_violation = ADMM(parameters)
            logs["X_log"][k,:,:] = X
            logs["U_log"][k,:,:] = U
            logs["Y_log"][k,:,:] = Y
            logs["ν_log"][k,:,:] = ν
            filename = "lin_uncons_" * "rho_" * string(P[k]) * "_iter_" * string(parameters["num_iter"]) * "_Qf_" * string(parameters["Qf"][1,1]) * ".png"
            save_results(X, U, Y, ν, cost_history, filename, parameters)
        end
    end
    return logs
end

function circular_orbit(orbit_radius, θ, μ)
    # Compute the state [x, y, z, xd, yd, zd]
    # for a circular orbit in the XY plane.
    state = zeros(6)
    state[1:3] = orbit_radius.*[cos(θ), sin(θ), 0]
    velocity = sqrt(μ/orbit_radius)
    state[4:6] = velocity.*[-sin(θ), cos(θ), 0]
    return state
end

function full_to_cw(x_full)
    μ = 3.99*10^14 # Standard gravitational parameter m^3 s^-2
    # Convert a full state ∈ R^12 to a cw state ∈ R^6.
    r_target = x_full[4:6]
    rd_target = x_full[10:12]
    # Δr = -x_full[1:3] # r_ego - r_target
    # Δrd = -x_full[7:9] # rd_ego - rd_target
    r_ego = r_target - x_full[1:3]
    rd_ego = rd_target - x_full[7:9]

    # define the axis of the cw frame
    ey_cw = r_target / norm(r_target)
    ez_cw = cross(ey_cw, rd_target)
    ez_cw = ez_cw / norm(ez_cw)
    ex_cw = cross(ey_cw, ez_cw)
    R_wc = [ex_cw ey_cw ez_cw]
    r = r_ego
    v = rd_ego
    ω = cross(r,v) / norm(cross(r,v))
    # ω *= v'*(cross(cross(r,v),r)) / norm(r)
    orbit_radius = norm(r_target)
    ω *= sqrt(μ) / orbit_radius^2
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


# function execute_control(x0, U, Δt, parameters)
#     n = parameters["n"]
#     N = size(U)[2]+1
#     X = zeros(n, N)
#     X[:,1] = x0
#     for k=2:N
#         X[:,k] = rk4_step(non_linear_dynamics!, X[:,k-1], U[:,k-1], Δt)
#     end
#     return X
# end

# function control_cw_to_full(x0_full, X, U, Δt_mpc, lin_parameters, non_lin_parameters)
#     n = lin_parameters["n"]
#     m = lin_parameters["m"]
#     N = size(U)[2] + 1
#     U_full = zero(U)
#     X_full = zeros(12, N)
#     X_full[:,1] = x0_full
#     for k=2:N
#         u = zeros(m)
#         X_full[:,k] = rk4_step(circular_dynamics!, X_full[:,k-1], u, Δt_mpc)
#     end
#     for k=1:N-1
#         r_target = X_full[4:6,k]
#         rd_target = X_full[10:12,k]
#         ey_cw = r_target ./ norm(r_target)
#         ex_cw = -rd_target ./ norm(rd_target)
#         ez_cw = cross(ex_cw, ey_cw)
#         U_full[:,k] = U[1,k]*ex_cw + U[2,k]*ey_cw + U[3,k]*ez_cw
#     end
#     return U_full
# end

# function mpc_example()
#     lin_parameters = define_lin_parameters()
#     non_lin_parameters = define_non_lin_parameters()
#
#     # Define initial state
#     x_target = circular_orbit(lin_parameters["orbit_radius"], 0, lin_parameters["μ"])
#     Δr0 = [1.0, 2.0, 3.0] # position delta
#     ### Δrd0 = [0.2, 0.3, 0.0] # velocity delta
#     Δrd0 = [50.2, 60.3, 0.0] # velocity delta
#     x0 = [Δr0; x_target[1:3]; Δrd0; x_target[4:6]] # initial state
#     non_lin_parameters["x0"] = x0
#     println("x0 = ", x0)
#
#     # Propagate the non linear dynamics forward without control
#     tf = 1e2
#     N = 100 # N = 1e4 sufficient for tf = 1e6 seconds
#     X_full_history = drifting_simulation(N, tf, non_lin_parameters)
#     U_full_history = zeros(3, N)
#     T_full_history = [k*tf/(N-1) for k=0:N-1]
#
#     filename = "history_after_drift_" * "tf_" * string(tf) * "_N_" * string(N) * ".png"
#     save_history(T_full_history, X_full_history, U_full_history, filename, non_lin_parameters)
#
#     x0_full = X_full_history[:,N]
#     # We recover from the initial drift using a MPC style controller reyling on the linear dynamics model.
#     lin_parameters["orbit_radius"] = norm(x0_full[4:6]) # we set the orbit radius of the cw model.
#     x0_cw = full_to_cw(x0_full)
#     lin_parameters["x0"] = x0_cw
#     println("x0_full = ", x0_full)
#     println("x0_cw = ", x0_cw)
#
#     lin_parameters["using_final_constraints"] = false
#     tf = 2000
#     num_exec = 20 # number of time the mpc is executed
#     t_exec = tf / num_exec # duration of the executed control sequence
#     M_horizon = 10 # mpc horizon duration / executed sequence duration
#     t_horizon = M_horizon*t_exec # mpc horizon duration
#     N_exec = 10 # number of timesteps in the executed sequence
#     N_horizon = N_exec * M_horizon + 1 # number of nodes in the horizon sequence
#     Δt_mpc = t_exec / N_exec # duration of each timestep in the horizon or executed sequence
#     lin_parameters["tf"] = t_horizon
#     lin_parameters["N"] = N_horizon
#     lin_parameters["Δt"] = Δt_mpc
#     # Scale the parameters of the linear model
#     scale_lin_parameters(lin_parameters)
#     println("lin_parameters[x0] = ", lin_parameters["x0"])
#
#     for k=1:num_exec
#         t = (k-1)*t_exec
#         effective_t_horizon = min(t_horizon, tf-t)
#         N_horizon = Int(effective_t_horizon / Δt_mpc) + 1
#         n = lin_parameters["n"]
#         m = lin_parameters["m"]
#         lin_parameters["X0"] = zeros(n, N_horizon)
#         lin_parameters["U0"] = zeros(m, N_horizon-1)
#         lin_parameters["ν0"] = zeros(m, N_horizon-1)
#         lin_parameters["Y0"] = zeros(m, N_horizon-1)
#         lin_parameters["N"] = N_horizon
#         lin_parameters["tf"] = effective_t_horizon / lin_parameters["t_ref"]
#         println("effective_t_horizon = ", effective_t_horizon)
#
#         # Compute the control sequence
#         X, U, Y, ν, cost_history, constraint_violation = l1_solver(lin_parameters)
#         filename = "lin_uncons_" * "mpc_iter_" * string(k) * "_rho_" * string(lin_parameters["ρ"]) * "_iter_" * string(lin_parameters["num_iter"]) * "_Qf_" * string(lin_parameters["Qf"][1,1]) * ".png"
#         save_results(X, U, Y, ν, cost_history, filename, lin_parameters)
#
#         # Rescale X, U
#         X[1:3,:] *= lin_parameters["l_ref"]
#         X[4:6,:] *= lin_parameters["v_ref"]
#         U *= lin_parameters["u_ref"]
#
#         U = control_cw_to_full(x0_full, X, U, Δt_mpc, lin_parameters, non_lin_parameters)
#         # Execute the first part
#         T_executed = [T_full_history[end]+k*Δt_mpc for k=1:N_exec]
#         U_executed = U[:,1:N_exec]
#         X_executed = execute_control(x0_full, U_executed, Δt_mpc, non_lin_parameters)
#
#         x0_full = X_executed[:,end]
#         X_full_history = [X_full_history X_executed]
#         U_full_history = [U_full_history U_executed]
#         T_full_history = [T_full_history; T_executed]
#
#         filename = "history_after_tmstp_" * string(k) * "_tf_" * string(tf) * "_N_" * string(N) * ".png"
#         save_history(T_full_history, X_full_history, U_full_history, filename, non_lin_parameters)
#
#         lin_parameters["orbit_radius"] = norm(x0_full[4:6]) # we set the orbit radius of the cw model.
#         x0_cw = full_to_cw(x0_full)
#         println("x0_cw = ", x0_cw)
#         update_scaling_lin_parameters(x0_cw, lin_parameters)
#     end

    # # Recovery
    # P = [10.0^i for i=1:2]
    # for k=1:length(P)
    #     lin_parameters["ρ"] = P[k]
    #     X, U, Y, ν, cost_history, constraint_violation = l1_solver(lin_parameters)
    #     filename = "lin_uncons_" * "rho_" * string(P[k]) * "_iter_" * string(lin_parameters["num_iter"]) * "_Qf_" * string(lin_parameters["Qf"][1,1]) * ".png"
    #     save_results(X, U, Y, ν, cost_history, filename, lin_parameters)
    # end
# end
