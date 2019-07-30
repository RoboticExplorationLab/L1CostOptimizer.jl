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

    plot_X = 2
    if parameters["complete_results"]
        plot_Y = 3
    else
        plot_Y = 2
    end
    group_plot = GroupPlot(plot_X, plot_Y, groupStyle = "horizontal sep = 2.0cm, vertical sep = 2.0cm")
    if parameters["linearity"]
        p1 = Plots.Linear(T./3600, X[1,:]./1000, style="no marks, cyan, very thick", legendentry=L"x_1")
        p2 = Plots.Linear(T./3600, X[2,:]./1000, style="no marks, orange, very thick", legendentry=L"x_2")
        p3 = Plots.Linear(T./3600, X[3,:]./1000, style="no marks, green, very thick", legendentry=L"x_3")
    elseif !parameters["linearity"]
        p1 = Plots.Linear(T./3600, X_cw[1,:]./1000, style="no marks, cyan, very thick", legendentry=L"x_1")
        p2 = Plots.Linear(T./3600, X_cw[2,:]./1000, style="no marks, orange, very thick", legendentry=L"x_2")
        p3 = Plots.Linear(T./3600, X_cw[3,:]./1000, style="no marks, green, very thick", legendentry=L"x_3")
    end
    axis = Axis([p1, p2, p3],
        # xmin = T[1]/3600,
        # xmax = T[end]/3600,
        xlabel="Time " * L"$[h]$",
        ylabel="Positions " * L"[km]",
        title="Positions",
        legendPos="south east",
        width="6cm",
        height="4cm")
    push!(group_plot, axis)

    if parameters["linearity"]
        p1 = Plots.Linear(T./3600, X[4,:].*3.6, style="no marks, cyan, very thick", legendentry=L"\dot{x}_1")
        p2 = Plots.Linear(T./3600, X[5,:].*3.6, style="no marks, orange, very thick", legendentry=L"\dot{x}_2")
        p3 = Plots.Linear(T./3600, X[6,:].*3.6, style="no marks, green, very thick", legendentry=L"\dot{x}_3")
    elseif !parameters["linearity"]
        p1 = Plots.Linear(T./3600, X_cw[4,:].*3.6, style="no marks, cyan, very thick", mark="triangle", legendentry=L"\dot{x}_1")
        p2 = Plots.Linear(T./3600, X_cw[5,:].*3.6, style="no marks, orange, very thick", legendentry=L"\dot{x}_2")
        p3 = Plots.Linear(T./3600, X_cw[6,:].*3.6, style="no marks, green, very thick", legendentry=L"\dot{x}_3")
    end
    axis = Axis([p1, p2, p3],
        xlabel="Time " * L"$[h]$",
        ylabel="Velocities " * L"[km/h]",
        title="Velocities",
        legendPos="south east",
        width="6cm",
        height="4cm")
    push!(group_plot, axis)

    p1 = Plots.Linear(T[1:end-1]./3600, U[1,:]*1000, style="const plot, no marks, cyan, very thick", legendentry=L"u_1")
    p2 = Plots.Linear(T[1:end-1]./3600, U[2,:]*1000, style="const plot, no marks, orange, very thick", legendentry=L"u_2")
    p3 = Plots.Linear(T[1:end-1]./3600, U[3,:]*1000, style="const plot, no marks, green, very thick", legendentry=L"u_3")
    axis = Axis([p1, p2, p3],
        xlabel="Time " * L"$[h]$",
        ylabel="Controls " * L"[mN]",
        title="Controls",
        legendPos="south east",
        width="6cm",
        height="4cm")
    push!(group_plot, axis)

    #Preprocessing
    y_minimum_cost = minimum(cost_history[1:iter,1])
    y_maximum_cost = maximum(cost_history[1:iter,1])
    y_min_cost = y_minimum_cost - (y_maximum_cost - y_minimum_cost) / 10
    y_max_cost = y_maximum_cost + (y_maximum_cost - y_minimum_cost) / 10

    log_optimality_criterion = log.(10, optimality_criterion)
    y_minimum_opt = minimum(log_optimality_criterion[1:iter])
    y_maximum_opt = maximum(log_optimality_criterion[1:iter])
    y_min_opt = y_minimum_opt - (y_maximum_opt - y_minimum_opt) / 10
    y_max_opt = y_maximum_opt + (y_maximum_opt - y_minimum_opt) / 10

    p1 = Plots.Linear([i for i=1:iter], cost_history[1:iter,1], style="no marks, cyan, very thick", legendentry="Cost")
    # p2 = Plots.Linear([i for i=1:iter], log_optimality_criterion[1:iter], style="no marks, orange, very thick", legendentry=L"$||U - Y||_{2}$")
    axis = Axis([p1, p2],
        xlabel="L1 Solver Iterations",
        ylabel="Cost",
        ymin=y_min_cost,
        ymax=y_max_cost,
        # ylabel="Optimality Criterion",
        title="Convergence",
        legendPos="north east",
        width="6cm",
        height="4cm")
    push!(group_plot, axis)


        # subplot(plot_Y, plot_X, 4)
        # plot([i for i=1:iter], cost_history[1:iter,1], color="cornflowerblue", linewidth=1.0, linestyle="-", label=L"$cost$")
        # title("Cost")
        # xlabel("L1 Solver Iterations")
        # yscale("linear")
        # xlim(1,iter)
        #
        # subplot(plot_Y, plot_X, 5)
        # plot([i for i=1:iter], optimality_criterion[1:iter], color="forestgreen", linewidth=1.0, linestyle="-", label=L"$||\nabla_{X,U} L||_{\infty}$")
        # title("Optimality Criterion")
        # xlabel("L1 Solver Iterations")
        # ylabel(L"$||U - Y||_{2} / dim(U)$")
        # xlim(1,iter)
        # yscale("log")
        #
    PGFPlots.save("result/pgf_plot/" * filename*".tikz", include_preamble=false, group_plot)
    PGFPlots.save("result/pgf_plot/" * filename*".tex", group_plot)


    # if parameters["complete_results"]
    #     subplot(plot_Y, plot_X, 6)
    #     step(T[1:end-1], parameters["ρ"]*(U[1,:] - Y[1,:]), color="cornflowerblue", linewidth=1.0, linestyle="-", label=L"$y_1$")
    #     step(T[1:end-1], parameters["ρ"]*(U[2,:] - Y[2,:]), color="darkorange", linewidth=1.0, linestyle="-", label=L"$y_2$")
    #     step(T[1:end-1], parameters["ρ"]*(U[3,:] - Y[3,:]), color="forestgreen", linewidth=1.0, linestyle="-", label=L"$y_3$")
    #     title(L"$\rho (U - Y)$")
    #     xlabel(L"Iterations")
    #     legend()
    #
    #     subplot(plot_Y, plot_X, 7)
    #     step(T[1:end-1], ν[1,:], color="cornflowerblue", linewidth=1.0, linestyle="-", label=L"$ν_1$")
    #     step(T[1:end-1], ν[2,:], color="darkorange", linewidth=1.0, linestyle="-", label=L"$ν_2$")
    #     step(T[1:end-1], ν[3,:], color="forestgreen", linewidth=1.0, linestyle="-", label=L"$ν_3$")
    #     title(L"\nu")
    #     xlabel(L"Time in $s$")
    #     legend()
    # end
    #
    # tight_layout()
    # savefig("result/" * filename*"."*parameters["plot_format"], format=parameters["plot_format"], dpi=1000)
    # close()
    # save("result/control/" * filename*".jld", "U", U, "x0", X[:,1])
    return
end
