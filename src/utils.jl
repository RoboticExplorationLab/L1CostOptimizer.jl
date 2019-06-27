using PyPlot

function save_results(X_, U_, Y_, ν, cost_history, filename, parameters)
    num_iter = parameters["num_iter"]
    N = parameters["N"]
    n = parameters["n"]
    Δt = parameters["Δt"]

    t_ref = parameters["t_ref"]
    T = [Δt * i / t_ref for i=0:N-1]

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
        T *= t_ref
    elseif !parameters["linearity"]
        l_ego_ref = parameters["l_ego_ref"]
        l_target_ref = parameters["l_target_ref"]
        v_ego_ref = parameters["v_ego_ref"]
        v_target_ref = parameters["v_target_ref"]
        a_ego_ref = parameters["a_ego_ref"]
        a_target_ref = parameters["a_target_ref"]
        u_ego_ref = parameters["u_ego_ref"]
        X = zero(X_)
        X[1:3,:] = X_[1:3,:] * l_ego_ref
        X[4:6,:] = X_[4:6,:] * l_target_ref
        X[7:9,:] = X_[7:9,:] * v_ego_ref
        X[10:12,:] = X_[10:12,:] * v_target_ref
        U = U_ * u_ref
        Y = Y_ * u_ref
        T *= t_ref
    end

    plot_X = 3
    if parameters["complete_results"]
        plot_Y = 3
    else
        plot_Y = 1
    end
    fig = figure(figsize=(3*plot_X,3*plot_Y))
    subplot(plot_Y, plot_X, 1)
    if parameters["linearity"]
        step(T, X[1,:], color="blue", linewidth=1.0, linestyle="-", label=L"$x_1$ ego")
        step(T, X[2,:], color="red", linewidth=1.0, linestyle="-", label=L"$x_2$ ego")
        step(T, X[3,:], color="green", linewidth=1.0, linestyle="-", label=L"$x_3$ ego")
    elseif !parameters["linearity"]
        step(T, X[1,:], color="blue", linewidth=1.0, linestyle="-", label=L"$x_1$ ego")
        step(T, X[2,:], color="red", linewidth=1.0, linestyle="-", label=L"$x_2$ ego")
        step(T, X[3,:], color="green", linewidth=1.0, linestyle="-", label=L"$x_3$ ego")
        # step(T, X[4,:]-X[1,:], color="blue", linewidth=1.0, linestyle="-", label=L"$x_1$ ego")
        # step(T, X[5,:]-X[2,:], color="red", linewidth=1.0, linestyle="-", label=L"$x_2$ ego")
        # step(T, X[6,:]-X[3,:], color="green", linewidth=1.0, linestyle="-", label=L"$x_3$ ego")
        # step(T, X[4,:], color="blue", linewidth=2.0, linestyle="--", label=L"$x_1$ target")
        # step(T, X[5,:], color="red", linewidth=2.0, linestyle="--", label=L"$x_2$ target")
        # step(T, X[6,:], color="green", linewidth=2.0, linestyle="--", label=L"$x_3$ target")
    end
    title(L"Positions")
    grid("on")
    xlabel(L"Time in $s$")
    ylabel(L"Position in $m$")
    legend()

    subplot(plot_Y, plot_X, 2)
    if parameters["linearity"]
        step(T, X[4,:], color="blue", linewidth=1.0, linestyle="-", label=L"$\dot{x}_1$")
        step(T, X[5,:], color="red", linewidth=1.0, linestyle="-", label=L"$\dot{x}_2$")
        step(T, X[6,:], color="green", linewidth=1.0, linestyle="-", label=L"$\dot{x}_3$")
    elseif !parameters["linearity"]
        step(T, X[7,:], color="blue", linewidth=1.0, linestyle="-", label=L"$\dot{x}_1$ ego")
        step(T, X[8,:], color="red", linewidth=1.0, linestyle="-", label=L"$\dot{x}_2$ ego")
        step(T, X[9,:], color="green", linewidth=1.0, linestyle="-", label=L"$\dot{x}_3$ ego")
        # step(T, X[10,:]-X[7,:], color="blue", linewidth=1.0, linestyle="-", label=L"$\dot{x}_1$ ego")
        # step(T, X[11,:]-X[8,:], color="red", linewidth=1.0, linestyle="-", label=L"$\dot{x}_2$ ego")
        # step(T, X[12,:]-X[9,:], color="green", linewidth=1.0, linestyle="-", label=L"$\dot{x}_3$ ego")
        # step(T, X[10,:], color="blue", linewidth=2.0, linestyle="--", label=L"$\dot{x}_1$ target")
        # step(T, X[11,:], color="red", linewidth=2.0, linestyle="--", label=L"$\dot{x}_2$ target")
        # step(T, X[12,:], color="green", linewidth=2.0, linestyle="--", label=L"$\dot{x}_3$ target")
    end
    title(L"Velocities")
    grid("on")
    xlabel(L"Time in $s$")
    ylabel(L"Velocity in $m/s$")
    legend()

    subplot(plot_Y, plot_X, 3)
    step(T[1:end-1], U[1,:], color="blue", linewidth=1.0, linestyle="-", label=L"$u_1$")
    step(T[1:end-1], U[2,:], color="red", linewidth=1.0, linestyle="-", label=L"$u_2$")
    step(T[1:end-1], U[3,:], color="green", linewidth=1.0, linestyle="-", label=L"$u_3$")
    title(L"Control")
    grid("on")
    xlabel(L"Time in $s$")
    ylabel(L"Controls in $N$")
    legend()

    if parameters["complete_results"]
        subplot(3, 3, 4)
        step(T[1:end-1], parameters["ρ"]*(U[1,:] - Y[1,:]), color="blue", linewidth=1.0, linestyle="-", label=L"$y_1$")
        step(T[1:end-1], parameters["ρ"]*(U[2,:] - Y[2,:]), color="red", linewidth=1.0, linestyle="-", label=L"$y_2$")
        step(T[1:end-1], parameters["ρ"]*(U[3,:] - Y[3,:]), color="green", linewidth=1.0, linestyle="-", label=L"$y_3$")
        title(L"$\rho (U - Y)$")
        grid("on")
        xlabel(L"Time in $s$")
        # ylabel(L" ")
        legend()

        subplot(3, 3, 5)
        step(T[1:end-1], ν[1,:], color="blue", linewidth=1.0, linestyle="-", label=L"$ν_1$")
        step(T[1:end-1], ν[2,:], color="red", linewidth=1.0, linestyle="-", label=L"$ν_2$")
        step(T[1:end-1], ν[3,:], color="green", linewidth=1.0, linestyle="-", label=L"$ν_3$")
        title(L"\nu")
        grid("on")
        xlabel(L"Time in $s$")
        # ylabel(L" ")
        legend()

        subplot(plot_Y, plot_X, 6)
        step(T[1:end-1], U[1,:], color="blue", linewidth=1.0, linestyle="-", label=L"$u_1$")
        step(T[1:end-1], U[2,:], color="red", linewidth=1.0, linestyle="--", label=L"$u_2$")
        step(T[1:end-1], U[3,:], color="green", linewidth=1.0, linestyle="--", label=L"$u_3$")
        step(T[1:end-1], Y[1,:], color="blue", linewidth=2.0, linestyle="-", label=L"$y_1$")
        step(T[1:end-1], Y[2,:], color="red", linewidth=2.0, linestyle="-", label=L"$y_2$")
        step(T[1:end-1], Y[3,:], color="green", linewidth=2.0, linestyle="-", label=L"$y_3$")
        title("Constraint Violations")
        grid("on")
        xlabel(L"Time in $s$")
        ylabel(L"Controls in $N$")
        legend()

        subplot(plot_Y, plot_X, 7)
        plot(log.(cost_history[:,1]), color="blue", linewidth=1.0, linestyle="-", label=L"$cost$")
        # plot(log.(cost_history[:,2]), color="red", linewidth=1.0, linestyle="-", label=L"$lqr_cost$")
        # plot(log.(cost_history[:,3]), color="green", linewidth=1.0, linestyle="-", label=L"$augmented_cost$")
        title(L"Cost")
        grid("on")
        xlabel(L"Time in $s$")
        ylabel(L"Cost")
        legend()
    end

    tight_layout()
    savefig("result/" * filename, format="png", dpi=300)
    close()
    return
end
