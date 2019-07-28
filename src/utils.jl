function save_history(T_full_history, X_full_history, U_full_history, filename, parameters)
    num_iter = parameters["num_iter"]
    N = parameters["N"]
    n = parameters["n"]

    XT = X_full_history[4,:]
    YT = X_full_history[5,:]
    XE = XT - X_full_history[1,:]
    YE = YT - X_full_history[2,:]
    plot_X = 2
    plot_Y = 1
    fig = figure(figsize=(3*plot_X,3*plot_Y))
    subplot(plot_Y, plot_X, 1)
    plot(XT, YT, color="darkorange", linewidth=1.0, linestyle="-", label=L"target")
    plot(XE, YE, color="cornflowerblue", linewidth=1.0, linestyle="-", label=L"ego")
    title("Trajectories")
    #grid("on")
    xlabel(L"X")
    ylabel(L"Y")
    # axis("equal")
    legend()

    subplot(plot_Y, plot_X, 2)
    plot(T_full_history, U_full_history[1,:], color="cornflowerblue", linewidth=1.0, linestyle="-", label=L"u1")
    plot(T_full_history, U_full_history[2,:], color="darkorange", linewidth=1.0, linestyle="-", label=L"u2")
    plot(T_full_history, U_full_history[3,:], color="forestgreen", linewidth=1.0, linestyle="-", label=L"u3")
    title("Trajectories")
    #grid("on")
    xlabel(L"Time in $s$")
    ylabel(L"U")
    legend()

    tight_layout()
    savefig("result/history/" * filename, format="png", dpi=300)
    close()
    return
end

function control_constraint_violation(U, parameters)
    u_min = parameters["u_min"]
    u_max = parameters["u_max"]
    out = maximum(abs.(U .- u_max) + (U .- u_max) + abs.(-U .+ u_min) + (-U .+ u_min), dims=2) * parameters["u_ref"] ./ 2
    out = out[:,1]
end

function compute_constraint_violation(U, Y, parameters)
    constraint_violation_U = control_constraint_violation(U, parameters)
    constraint_violation_Y = control_constraint_violation(Y, parameters)
    out = [constraint_violation_U; constraint_violation_Y]
    return out
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
