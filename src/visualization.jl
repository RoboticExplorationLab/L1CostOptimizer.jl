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
        plot_Y = 3
    else
        plot_Y = 2
    end
    fig = figure(figsize=(3*plot_X,3*plot_Y))
    subplot(plot_Y, plot_X, 1)
    if parameters["linearity"]
        plot(T, X[1,:], color="cornflowerblue", linewidth=1.0, linestyle="-", label=L"$x_1$ ego")
        plot(T, X[2,:], color="darkorange", linewidth=1.0, linestyle="-", label=L"$x_2$ ego")
        plot(T, X[3,:], color="forestgreen", linewidth=1.0, linestyle="-", label=L"$x_3$ ego")
    elseif !parameters["linearity"]
        plot(T, X_cw[1,:], color="cornflowerblue", linewidth=1.0, linestyle="-", label=L"$x_1$ ego")
        plot(T, X_cw[2,:], color="darkorange", linewidth=1.0, linestyle="-", label=L"$x_2$ ego")
        plot(T, X_cw[3,:], color="forestgreen", linewidth=1.0, linestyle="-", label=L"$x_3$ ego")
    end
    title("Positions")
    ticklabel_format(axis="both", style="sci", scilimits=(0,0))
    xlabel(L"Time in $s$")
    ylabel(L"Position in $m$")
    legend()

    subplot(plot_Y, plot_X, 2)
    if parameters["linearity"]
        plot(T, X[4,:], color="cornflowerblue", linewidth=1.0, linestyle="-", label=L"$\dot{x}_1$")
        plot(T, X[5,:], color="darkorange", linewidth=1.0, linestyle="-", label=L"$\dot{x}_2$")
        plot(T, X[6,:], color="forestgreen", linewidth=1.0, linestyle="-", label=L"$\dot{x}_3$")
    elseif !parameters["linearity"]
        plot(T, X_cw[4,:], color="cornflowerblue", linewidth=1.0, linestyle="-", label=L"$\dot{x}_1$ ego")
        plot(T, X_cw[5,:], color="darkorange", linewidth=1.0, linestyle="-", label=L"$\dot{x}_2$ ego")
        plot(T, X_cw[6,:], color="forestgreen", linewidth=1.0, linestyle="-", label=L"$\dot{x}_3$ ego")
    end
    title("Velocities")
    ticklabel_format(axis="both", style="sci", scilimits=(0,0))
    xlabel(L"Time in $s$")
    ylabel(L"Velocity in $m/s$")
    legend()

    subplot(plot_Y, plot_X, 3)
    step(T[1:end-1], U[1,:], color="cornflowerblue", linewidth=1.0, linestyle="-", label=L"$u_1$")
    step(T[1:end-1], U[2,:], color="darkorange", linewidth=1.0, linestyle="-", label=L"$u_2$")
    step(T[1:end-1], U[3,:], color="forestgreen", linewidth=1.0, linestyle="-", label=L"$u_3$")
    title("Control")
    ticklabel_format(axis="both", style="sci", scilimits=(0,0))
    xlabel(L"Time in $s$")
    ylabel(L"Controls in $N$")
    yscale("linear")
    # yscale("log")
    legend()

    subplot(plot_Y, plot_X, 4)
    # plot(log.(10, cost_history[:,1]), color="cornflowerblue", linewidth=1.0, linestyle="-", label=L"$cost$")
    plot(cost_history[1:iter,1], color="cornflowerblue", linewidth=1.0, linestyle="-", label=L"$cost$")
    title(L"Cost")
    xlabel("L1 Solver Iterations")
    ylabel(L"Cost")
    yscale("log")
    xlim(1,iter)
    legend()

    subplot(plot_Y, plot_X, 5)
    plot(optimality_criterion[1:iter], color="forestgreen", linewidth=1.0, linestyle="-", label=L"$||\nabla_{X,U} L||_{\infty}$")
    title(L"Optimality Criterion")
    xlabel("L1 Solver Iterations")
    ylabel(L"Optimality Criterion")
    xlim(1,iter)
    legend()
    yscale("log")

    if parameters["complete_results"]
        subplot(plot_Y, plot_X, 6)
        step(T[1:end-1], parameters["ρ"]*(U[1,:] - Y[1,:]), color="cornflowerblue", linewidth=1.0, linestyle="-", label=L"$y_1$")
        step(T[1:end-1], parameters["ρ"]*(U[2,:] - Y[2,:]), color="darkorange", linewidth=1.0, linestyle="-", label=L"$y_2$")
        step(T[1:end-1], parameters["ρ"]*(U[3,:] - Y[3,:]), color="forestgreen", linewidth=1.0, linestyle="-", label=L"$y_3$")
        title(L"$\rho (U - Y)$")
        xlabel(L"Iterations")
        # ylabel(L" ")
        legend()

        subplot(plot_Y, plot_X, 7)
        step(T[1:end-1], ν[1,:], color="cornflowerblue", linewidth=1.0, linestyle="-", label=L"$ν_1$")
        step(T[1:end-1], ν[2,:], color="darkorange", linewidth=1.0, linestyle="-", label=L"$ν_2$")
        step(T[1:end-1], ν[3,:], color="forestgreen", linewidth=1.0, linestyle="-", label=L"$ν_3$")
        title(L"\nu")
        xlabel(L"Time in $s$")
        # ylabel(L" ")
        legend()


        # subplot(plot_Y, plot_X, 6)
        # step(T[1:end-1], U[1,:], color="cornflowerblue", linewidth=1.0, linestyle="-", label=L"$u_1$")
        # step(T[1:end-1], U[2,:], color="darkorange", linewidth=1.0, linestyle="--", label=L"$u_2$")
        # step(T[1:end-1], U[3,:], color="forestgreen", linewidth=1.0, linestyle="--", label=L"$u_3$")
        # step(T[1:end-1], Y[1,:], color="cornflowerblue", linewidth=2.0, linestyle="-", label=L"$y_1$")
        # step(T[1:end-1], Y[2,:], color="darkorange", linewidth=2.0, linestyle="-", label=L"$y_2$")
        # step(T[1:end-1], Y[3,:], color="forestgreen", linewidth=2.0, linestyle="-", label=L"$y_3$")
        # xlabel(L"Time in $s$")
        # ylabel(L"Controls in $N$")
        # legend()



        # if parameters["linearity"]
        #     subplot(plot_Y, plot_X, 8)
        #     θ_dot = sqrt(parameters["μ"]/parameters["orbit_radius"]^3)
        #     orbit_radius = parameters["orbit_radius"]
        #     N = parameters["N"]
        #     Δt_rescaled = parameters["Δt"] * parameters["t_ref"]
        #     XT = [orbit_radius*cos(k*Δt_rescaled*θ_dot) for k=0:N-1]
        #     YT = [orbit_radius*sin(k*Δt_rescaled*θ_dot) for k=0:N-1]
        #     XE = XT .+ [X[2,k+1]*cos(k*Δt_rescaled*θ_dot) + X[1,k+1]*sin(k*Δt_rescaled*θ_dot) for k=0:N-1]
        #     YE = YT .+ [X[2,k+1]*sin(k*Δt_rescaled*θ_dot) - X[1,k+1]*cos(k*Δt_rescaled*θ_dot) for k=0:N-1]
        #     plot(XT, YT, color="cornflowerblue", linewidth=1.0, linestyle="-", label=L"target")
        #     plot(XE, YE, color="darkorange", linewidth=1.0, linestyle="-", label=L"ego")
        #     title(L"Trajectory X-Y")
        #     xlabel(L"X")
        #     ylabel(L"Y")
        #     axis("equal")
        #     legend()
        # end

        # subplot(plot_Y, plot_X, 9)
        # step(T[1:end-1], Y[1,:], color="cornflowerblue", linewidth=1.0, linestyle="-", label=L"$y_1$")
        # step(T[1:end-1], Y[2,:], color="darkorange", linewidth=1.0, linestyle="-", label=L"$y_2$")
        # step(T[1:end-1], Y[3,:], color="forestgreen", linewidth=1.0, linestyle="-", label=L"$y_3$")
        # title(L"$Y$")
        # xlabel(L"Iterations")
        # # ylabel(L" ")
        # legend()

        # if parameters["using_constraints"]
        #     subplot(plot_Y, plot_X, 10)
        #     plot(constraint_violation[:,1], color="cornflowerblue", linewidth=1.0, linestyle="-", label=L"$U_{cv1}$")
        #     plot(constraint_violation[:,2], color="darkorange", linewidth=1.0, linestyle="-", label=L"$U_{cv2}$")
        #     plot(constraint_violation[:,3], color="forestgreen", linewidth=1.0, linestyle="-", label=L"$U_{cv3}$")
        #     plot(constraint_violation[:,4], color="cornflowerblue", linewidth=1.0, linestyle="--", label=L"$Y_{cv1}$")
        #     plot(constraint_violation[:,5], color="darkorange", linewidth=1.0, linestyle="--", label=L"$Y_{cv2}$")
        #     plot(constraint_violation[:,6], color="forestgreen", linewidth=1.0, linestyle="--", label=L"$Y_{cv3}$")
        #     title(L"Constraint Violation")
        #     xlabel("L1 Solver Iterations")
        #     ylabel(L"Constraint Violation")
        #     legend()
        # end
    end

    tight_layout()
    savefig("result/" * filename*"."*parameters["plot_format"], format=parameters["plot_format"], dpi=1000)
    close()
    save("result/control/" * filename*".jld", "U", U, "x0", X[:,1])
    return
end
