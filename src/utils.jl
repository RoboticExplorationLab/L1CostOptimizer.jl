using PyPlot

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

function save_results(X_, U_, Y_, ν, cost_history, constraint_violation, optimality_criterion, filename, iter, parameters)
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
            X_cw[:,k] = full_to_cw(X[:,k])
        end
    end

    plot_X = 3
    if parameters["complete_results"]
        plot_Y = 4
    else
        plot_Y = 1
    end
    fig = figure(figsize=(3*plot_X,3*plot_Y))
    subplot(plot_Y, plot_X, 1)
    if parameters["linearity"]
        step(T, X[1,:], color="cornflowerblue", linewidth=1.0, linestyle="-", label=L"$x_1$ ego")
        step(T, X[2,:], color="darkorange", linewidth=1.0, linestyle="-", label=L"$x_2$ ego")
        step(T, X[3,:], color="forestgreen", linewidth=1.0, linestyle="-", label=L"$x_3$ ego")
    elseif !parameters["linearity"]
        step(T, X_cw[1,:], color="cornflowerblue", linewidth=1.0, linestyle="-", label=L"$x_1$ ego")
        step(T, X_cw[2,:], color="darkorange", linewidth=1.0, linestyle="-", label=L"$x_2$ ego")
        step(T, X_cw[3,:], color="forestgreen", linewidth=1.0, linestyle="-", label=L"$x_3$ ego")
        # step(T, X[1,:], color="cornflowerblue", linewidth=2.0, linestyle="--", label=L"$x_1$ ego")
        # step(T, X[2,:], color="darkorange", linewidth=2.0, linestyle="--", label=L"$x_2$ ego")
        # step(T, X[3,:], color="forestgreen", linewidth=2.0, linestyle="--", label=L"$x_3$ ego")
    end
    title("Positions")
    #grid("on")
    ticklabel_format(axis="both", style="sci", scilimits=(0,0))
    xlabel(L"Time in $s$")
    ylabel(L"Position in $m$")
    # legend()

    subplot(plot_Y, plot_X, 2)
    if parameters["linearity"]
        step(T, X[4,:], color="cornflowerblue", linewidth=1.0, linestyle="-", label=L"$\dot{x}_1$")
        step(T, X[5,:], color="darkorange", linewidth=1.0, linestyle="-", label=L"$\dot{x}_2$")
        step(T, X[6,:], color="forestgreen", linewidth=1.0, linestyle="-", label=L"$\dot{x}_3$")
    elseif !parameters["linearity"]
        step(T, X_cw[4,:], color="cornflowerblue", linewidth=1.0, linestyle="-", label=L"$\dot{x}_1$ ego")
        step(T, X_cw[5,:], color="darkorange", linewidth=1.0, linestyle="-", label=L"$\dot{x}_2$ ego")
        step(T, X_cw[6,:], color="forestgreen", linewidth=1.0, linestyle="-", label=L"$\dot{x}_3$ ego")
        # step(T, X[7,:], color="cornflowerblue", linewidth=2.0, linestyle="--", label=L"$\dot{x}_1$ ego")
        # step(T, X[8,:], color="darkorange", linewidth=2.0, linestyle="--", label=L"$\dot{x}_2$ ego")
        # step(T, X[9,:], color="forestgreen", linewidth=2.0, linestyle="--", label=L"$\dot{x}_3$ ego")
    end
    title("Velocities")
    #grid("on")
    ticklabel_format(axis="both", style="sci", scilimits=(0,0))
    xlabel(L"Time in $s$")
    ylabel(L"Velocity in $m/s$")
    # legend()

    subplot(plot_Y, plot_X, 3)
    step(T[1:end-1], U[1,:], color="cornflowerblue", linewidth=1.0, linestyle="-", label=L"$u_1$")
    step(T[1:end-1], U[2,:], color="darkorange", linewidth=1.0, linestyle="-", label=L"$u_2$")
    step(T[1:end-1], U[3,:], color="forestgreen", linewidth=1.0, linestyle="-", label=L"$u_3$")
    title("Control")
    #grid("on")
    ticklabel_format(axis="both", style="sci", scilimits=(0,0))
    xlabel(L"Time in $s$")
    ylabel(L"Controls in $N$")
    yscale("linear")
    # yscale("log")
    legend()

    if parameters["complete_results"]
        subplot(plot_Y, plot_X, 4)
        step(T[1:end-1], parameters["ρ"]*(U[1,:] - Y[1,:]), color="cornflowerblue", linewidth=1.0, linestyle="-", label=L"$y_1$")
        step(T[1:end-1], parameters["ρ"]*(U[2,:] - Y[2,:]), color="darkorange", linewidth=1.0, linestyle="-", label=L"$y_2$")
        step(T[1:end-1], parameters["ρ"]*(U[3,:] - Y[3,:]), color="forestgreen", linewidth=1.0, linestyle="-", label=L"$y_3$")
        title(L"$\rho (U - Y)$")
        #grid("on")
        xlabel(L"Iterations")
        # ylabel(L" ")
        legend()

        subplot(plot_Y, plot_X, 5)
        step(T[1:end-1], ν[1,:], color="cornflowerblue", linewidth=1.0, linestyle="-", label=L"$ν_1$")
        step(T[1:end-1], ν[2,:], color="darkorange", linewidth=1.0, linestyle="-", label=L"$ν_2$")
        step(T[1:end-1], ν[3,:], color="forestgreen", linewidth=1.0, linestyle="-", label=L"$ν_3$")
        title(L"\nu")
        #grid("on")
        xlabel(L"Time in $s$")
        # ylabel(L" ")
        legend()

        subplot(plot_Y, plot_X, 6)
        step(T[1:end-1], U[1,:], color="cornflowerblue", linewidth=1.0, linestyle="-", label=L"$u_1$")
        step(T[1:end-1], U[2,:], color="darkorange", linewidth=1.0, linestyle="--", label=L"$u_2$")
        step(T[1:end-1], U[3,:], color="forestgreen", linewidth=1.0, linestyle="--", label=L"$u_3$")
        step(T[1:end-1], Y[1,:], color="cornflowerblue", linewidth=2.0, linestyle="-", label=L"$y_1$")
        step(T[1:end-1], Y[2,:], color="darkorange", linewidth=2.0, linestyle="-", label=L"$y_2$")
        step(T[1:end-1], Y[3,:], color="forestgreen", linewidth=2.0, linestyle="-", label=L"$y_3$")
        title("Constraint Violations")
        #grid("on")
        xlabel(L"Time in $s$")
        ylabel(L"Controls in $N$")
        legend()

        subplot(plot_Y, plot_X, 7)
        # plot(log.(10, cost_history[:,1]), color="cornflowerblue", linewidth=1.0, linestyle="-", label=L"$cost$")
        plot(cost_history[1:iter,1], color="cornflowerblue", linewidth=1.0, linestyle="-", label=L"$cost$")
        title(L"Cost")
        #grid("on")
        xlabel("L1 Solver Iterations")
        ylabel(L"Cost")
        yscale("log")
        xlim(1,iter)
        legend()

        if parameters["linearity"]
            subplot(plot_Y, plot_X, 8)
            θ_dot = sqrt(parameters["μ"]/parameters["orbit_radius"]^3)
            orbit_radius = parameters["orbit_radius"]
            N = parameters["N"]
            Δt_rescaled = parameters["Δt"] * parameters["t_ref"]
            XT = [orbit_radius*cos(k*Δt_rescaled*θ_dot) for k=0:N-1]
            YT = [orbit_radius*sin(k*Δt_rescaled*θ_dot) for k=0:N-1]
            XE = XT .+ [X[2,k+1]*cos(k*Δt_rescaled*θ_dot) + X[1,k+1]*sin(k*Δt_rescaled*θ_dot) for k=0:N-1]
            YE = YT .+ [X[2,k+1]*sin(k*Δt_rescaled*θ_dot) - X[1,k+1]*cos(k*Δt_rescaled*θ_dot) for k=0:N-1]
            plot(XT, YT, color="cornflowerblue", linewidth=1.0, linestyle="-", label=L"target")
            plot(XE, YE, color="darkorange", linewidth=1.0, linestyle="-", label=L"ego")
            title(L"Trajectory X-Y")
            #grid("on")
            xlabel(L"X")
            ylabel(L"Y")
            axis("equal")
            legend()
        end

        subplot(plot_Y, plot_X, 9)
        step(T[1:end-1], Y[1,:], color="cornflowerblue", linewidth=1.0, linestyle="-", label=L"$y_1$")
        step(T[1:end-1], Y[2,:], color="darkorange", linewidth=1.0, linestyle="-", label=L"$y_2$")
        step(T[1:end-1], Y[3,:], color="forestgreen", linewidth=1.0, linestyle="-", label=L"$y_3$")
        title(L"$Y$")
        #grid("on")
        xlabel(L"Iterations")
        # ylabel(L" ")
        legend()

        if parameters["using_constraints"]
            subplot(plot_Y, plot_X, 10)
            plot(constraint_violation[:,1], color="cornflowerblue", linewidth=1.0, linestyle="-", label=L"$U_{cv1}$")
            plot(constraint_violation[:,2], color="darkorange", linewidth=1.0, linestyle="-", label=L"$U_{cv2}$")
            plot(constraint_violation[:,3], color="forestgreen", linewidth=1.0, linestyle="-", label=L"$U_{cv3}$")
            plot(constraint_violation[:,4], color="cornflowerblue", linewidth=1.0, linestyle="--", label=L"$Y_{cv1}$")
            plot(constraint_violation[:,5], color="darkorange", linewidth=1.0, linestyle="--", label=L"$Y_{cv2}$")
            plot(constraint_violation[:,6], color="forestgreen", linewidth=1.0, linestyle="--", label=L"$Y_{cv3}$")
            title(L"Constraint Violation")
            #grid("on")
            xlabel("L1 Solver Iterations")
            ylabel(L"Constraint Violation")
            legend()
        end

        subplot(plot_Y, plot_X, 11)
        plot(optimality_criterion[1:iter], color="forestgreen", linewidth=1.0, linestyle="-", label=L"$||\nabla_{X,U} L||_{\infty}$")
        title(L"Optimality Criterion")
        #grid("on")
        xlabel("L1 Solver Iterations")
        ylabel(L"Optimality Criterion")
        xlim(1,iter)
        legend()
        yscale("log")
    end

    tight_layout()
    savefig("result/" * filename*"."*parameters["plot_format"], format=parameters["plot_format"], dpi=300)
    close()
    save("result/control/" * filename*".jld", "U", U, "x0", X[:,1])
    return
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
