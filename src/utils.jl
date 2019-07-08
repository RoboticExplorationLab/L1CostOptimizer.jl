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

function save_results(X_, U_, Y_, ν, cost_history, filename, parameters)
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
    end
    println("**********X_final = ", X[:,end])

    plot_X = 3
    if parameters["complete_results"]
        plot_Y = 3
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
        step(T, X[1,:], color="cornflowerblue", linewidth=1.0, linestyle="-", label=L"$x_1$ ego")
        step(T, X[2,:], color="darkorange", linewidth=1.0, linestyle="-", label=L"$x_2$ ego")
        step(T, X[3,:], color="forestgreen", linewidth=1.0, linestyle="-", label=L"$x_3$ ego")
    end
    title("Positions")
    #grid("on")
    ticklabel_format(axis="both", style="sci", scilimits=(0,0))
    xlabel(L"Time in $s$")
    ylabel(L"Position in $m$")
    legend()

    subplot(plot_Y, plot_X, 2)
    if parameters["linearity"]
        step(T, X[4,:], color="cornflowerblue", linewidth=1.0, linestyle="-", label=L"$\dot{x}_1$")
        step(T, X[5,:], color="darkorange", linewidth=1.0, linestyle="-", label=L"$\dot{x}_2$")
        step(T, X[6,:], color="forestgreen", linewidth=1.0, linestyle="-", label=L"$\dot{x}_3$")
    elseif !parameters["linearity"]
        step(T, X[7,:], color="cornflowerblue", linewidth=1.0, linestyle="-", label=L"$\dot{x}_1$ ego")
        step(T, X[8,:], color="darkorange", linewidth=1.0, linestyle="-", label=L"$\dot{x}_2$ ego")
        step(T, X[9,:], color="forestgreen", linewidth=1.0, linestyle="-", label=L"$\dot{x}_3$ ego")
    end
    title("Velocities")
    #grid("on")
    ticklabel_format(axis="both", style="sci", scilimits=(0,0))
    xlabel(L"Time in $s$")
    ylabel(L"Velocity in $m/s$")
    legend()

    subplot(plot_Y, plot_X, 3)
    step(T[1:end-1], U[1,:], color="cornflowerblue", linewidth=1.0, linestyle="-", label=L"$u_1$")
    step(T[1:end-1], U[2,:], color="darkorange", linewidth=1.0, linestyle="-", label=L"$u_2$")
    step(T[1:end-1], U[3,:], color="forestgreen", linewidth=1.0, linestyle="-", label=L"$u_3$")
    title("Control")
    #grid("on")
    ticklabel_format(axis="both", style="sci", scilimits=(0,0))
    xlabel(L"Time in $s$")
    ylabel(L"Controls in $N$")
    legend()

    if parameters["complete_results"]
        subplot(3, 3, 4)
        step(T[1:end-1], parameters["ρ"]*(U[1,:] - Y[1,:]), color="cornflowerblue", linewidth=1.0, linestyle="-", label=L"$y_1$")
        step(T[1:end-1], parameters["ρ"]*(U[2,:] - Y[2,:]), color="darkorange", linewidth=1.0, linestyle="-", label=L"$y_2$")
        step(T[1:end-1], parameters["ρ"]*(U[3,:] - Y[3,:]), color="forestgreen", linewidth=1.0, linestyle="-", label=L"$y_3$")
        title(L"$\rho (U - Y)$")
        #grid("on")
        xlabel(L"Iterations")
        # ylabel(L" ")
        legend()

        subplot(3, 3, 5)
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
        plot(log.(cost_history[:,1]), color="cornflowerblue", linewidth=1.0, linestyle="-", label=L"$cost$")
        # plot(log.(cost_history[:,2]), color="darkorange", linewidth=1.0, linestyle="-", label=L"$lqr_cost$")
        # plot(log.(cost_history[:,3]), color="forestgreen", linewidth=1.0, linestyle="-", label=L"$augmented_cost$")
        title(L"Cost")
        #grid("on")
        xlabel(L"Time in $s$")
        ylabel(L"Cost")
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

        subplot(3, 3, 9)
        step(T[1:end-1], Y[1,:], color="cornflowerblue", linewidth=1.0, linestyle="-", label=L"$y_1$")
        step(T[1:end-1], Y[2,:], color="darkorange", linewidth=1.0, linestyle="-", label=L"$y_2$")
        step(T[1:end-1], Y[3,:], color="forestgreen", linewidth=1.0, linestyle="-", label=L"$y_3$")
        title(L"$Y$")
        #grid("on")
        xlabel(L"Iterations")
        # ylabel(L" ")
        legend()

    end

    tight_layout()
    savefig("result/" * filename*"."*parameters["plot_format"], format=parameters["plot_format"], dpi=300)
    close()
    return
end
