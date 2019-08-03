function save_results(X_, U_, Y_, ν, cost_history, optimality_criterion,
        filename, iter, parameters)
    if parameters["linearity"]
        T, X, U, Y = process_results(X_, U_, Y_, ν, cost_history,
            optimality_criterion, filename, iter, parameters)
    elseif !parameters["linearity"]
        T, X, U, Y, X_cw = process_results(X_, U_, Y_, ν, cost_history,
            optimality_criterion, filename, iter, parameters)
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
        xlabel="Time " * L"$[h]$",
        ylabel="Positions " * L"[km]",
        title="Positions",
        legendPos="south east"
        )
    PGFPlots.save("result/pgf_plot/" * filename * "_positions" * ".tikz", include_preamble=false, axis)
    push!(group_plot, axis)

    if parameters["linearity"]
        p1 = Plots.Linear(T./3600, X[4,:].*3.6, style="no marks, cyan, very thick", legendentry=L"\dot{x}_1")
        p2 = Plots.Linear(T./3600, X[5,:].*3.6, style="no marks, orange, very thick", legendentry=L"\dot{x}_2")
        p3 = Plots.Linear(T./3600, X[6,:].*3.6, style="no marks, green, very thick", legendentry=L"\dot{x}_3")
    elseif !parameters["linearity"]
        p1 = Plots.Linear(T./3600, X_cw[4,:].*3.6, style="no marks, cyan, very thick", legendentry=L"\dot{x}_1")
        p2 = Plots.Linear(T./3600, X_cw[5,:].*3.6, style="no marks, orange, very thick", legendentry=L"\dot{x}_2")
        p3 = Plots.Linear(T./3600, X_cw[6,:].*3.6, style="no marks, green, very thick", legendentry=L"\dot{x}_3")
    end
    axis = Axis([p1, p2, p3],
        xlabel="Time " * L"$[h]$",
        ylabel="Velocities " * L"[km/h]",
        title="Velocities",
        legendPos="north east",
        )
    PGFPlots.save("result/pgf_plot/" * filename * "_velocities" * ".tikz", include_preamble=false, axis)
    push!(group_plot, axis)

    p1 = Plots.Linear(T[1:end-1]./3600, U[1,:]*1000, style="const plot, no marks, cyan, very thick", legendentry=L"u_1")
    p2 = Plots.Linear(T[1:end-1]./3600, U[2,:]*1000, style="const plot, no marks, orange, very thick", legendentry=L"u_2")
    p3 = Plots.Linear(T[1:end-1]./3600, U[3,:]*1000, style="const plot, no marks, green, very thick", legendentry=L"u_3")
    axis = Axis([p1, p2, p3],
        xlabel="Time " * L"$[h]$",
        ylabel="Controls " * L"[mN]",
        title="Controls",
        legendPos="south east",
        )
    PGFPlots.save("result/pgf_plot/" * filename * "_controls" * ".tikz", include_preamble=false, axis)
    push!(group_plot, axis)

    #Preprocessing
    log_optimality_criterion = log.(10, optimality_criterion)
    p1 = Plots.Linear([i for i=1:iter], cost_history[1:iter],
        style="no marks, cyan, very thick", legendentry="Cost")
    p2 = Plots.Linear([i for i=1:iter], log_optimality_criterion[1:iter],
        style="no marks, orange, very thick", legendentry="Opt. Crit.")
    axis = Axis([p1, p2],
        xlabel="L1 Solver Iterations",
        ylabel="Cost",
        title="Convergence",
        legendPos="north east",
        )
    PGFPlots.save("result/pgf_plot/" * filename * "_convergence" * ".tikz", include_preamble=false, axis)
    push!(group_plot, axis)

    PGFPlots.save("result/pgf_plot/" * filename*".tikz", include_preamble=false, group_plot)
    PGFPlots.save("result/pgf_plot/" * filename*".tex", group_plot)
    return
end
