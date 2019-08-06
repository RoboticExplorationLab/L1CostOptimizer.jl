function save_results_image(X_, U_, Y_, ν, cost_history, optimality_criterion,
        filename, iter, parameters)
    # Saves plots of the state, control trajectories as well as cost, optimality
    # across iterations.
    if parameters["linearity"]
        T, X, U, Y = process_results(X_, U_, Y_, ν, cost_history,
            optimality_criterion, filename, iter, parameters)
    elseif !parameters["linearity"]
        T, X, U, Y, X_cw = process_results(X_, U_, Y_, ν, cost_history,
            optimality_criterion, filename, iter, parameters)
    end

    # Define figure
    plot_X = 3
    if parameters["complete_results"]
        plot_Y = 3
    else
        plot_Y = 2
    end
    fig = PyPlot.figure(figsize=(3*plot_X,3*plot_Y))

    PyPlot.subplot(plot_Y, plot_X, 1)
    if parameters["linearity"]
        PyPlot.plot(T, X[1,:], color="cornflowerblue", linewidth=1.0, linestyle="-", label=L"$x_1$ ego")
        PyPlot.plot(T, X[2,:], color="darkorange", linewidth=1.0, linestyle="-", label=L"$x_2$ ego")
        PyPlot.plot(T, X[3,:], color="forestgreen", linewidth=1.0, linestyle="-", label=L"$x_3$ ego")
    elseif !parameters["linearity"]
        PyPlot.plot(T, X_cw[1,:], color="cornflowerblue", linewidth=1.0, linestyle="-", label=L"$x_1$ ego")
        PyPlot.plot(T, X_cw[2,:], color="darkorange", linewidth=1.0, linestyle="-", label=L"$x_2$ ego")
        PyPlot.plot(T, X_cw[3,:], color="forestgreen", linewidth=1.0, linestyle="-", label=L"$x_3$ ego")
    end
    PyPlot.title("Positions")
    PyPlot.ticklabel_format(axis="both", style="sci", scilimits=(0,0))
    PyPlot.xlabel(L"Time in $s$")
    PyPlot.ylabel(L"Position in $m$")
    PyPlot.legend()

    PyPlot.subplot(plot_Y, plot_X, 2)
    if parameters["linearity"]
        PyPlot.plot(T, X[4,:], color="cornflowerblue", linewidth=1.0, linestyle="-", label=L"$\dot{x}_1$")
        PyPlot.plot(T, X[5,:], color="darkorange", linewidth=1.0, linestyle="-", label=L"$\dot{x}_2$")
        PyPlot.plot(T, X[6,:], color="forestgreen", linewidth=1.0, linestyle="-", label=L"$\dot{x}_3$")
    elseif !parameters["linearity"]
        PyPlot.plot(T, X_cw[4,:], color="cornflowerblue", linewidth=1.0, linestyle="-", label=L"$\dot{x}_1$ ego")
        PyPlot.plot(T, X_cw[5,:], color="darkorange", linewidth=1.0, linestyle="-", label=L"$\dot{x}_2$ ego")
        PyPlot.plot(T, X_cw[6,:], color="forestgreen", linewidth=1.0, linestyle="-", label=L"$\dot{x}_3$ ego")
    end
    PyPlot.title("Velocities")
    PyPlot.ticklabel_format(axis="both", style="sci", scilimits=(0,0))
    PyPlot.xlabel(L"Time in $s$")
    PyPlot.ylabel(L"Velocity in $m/s$")
    PyPlot.legend()

    PyPlot.subplot(plot_Y, plot_X, 3)
    PyPlot.step(T[1:end-1], U[1,:], color="cornflowerblue", linewidth=1.0, linestyle="-", label=L"$u_1$")
    PyPlot.step(T[1:end-1], U[2,:], color="darkorange", linewidth=1.0, linestyle="-", label=L"$u_2$")
    PyPlot.step(T[1:end-1], U[3,:], color="forestgreen", linewidth=1.0, linestyle="-", label=L"$u_3$")
    PyPlot.title("Controls")
    PyPlot.ticklabel_format(axis="both", style="sci", scilimits=(0,0))
    PyPlot.xlabel(L"Time in $s$")
    PyPlot.ylabel(L"Controls in $N$")
    PyPlot.yscale("linear")
    PyPlot.legend()

    PyPlot.subplot(plot_Y, plot_X, 4)
    PyPlot.plot([i for i=1:iter], cost_history[1:iter], color="cornflowerblue", linewidth=1.0, linestyle="-", label=L"$cost$")
    PyPlot.title("Cost")
    PyPlot.xlabel("L1 Solver Iterations")
    PyPlot.yscale("linear")
    PyPlot.xlim(1,iter)

    PyPlot.subplot(plot_Y, plot_X, 5)
    PyPlot.plot([i for i=1:iter], optimality_criterion[1:iter], color="forestgreen", linewidth=1.0, linestyle="-", label=L"$||\nabla_{X,U} L||_{\infty}$")
    PyPlot.title("Optimality Criterion")
    PyPlot.xlabel("L1 Solver Iterations")
    PyPlot.ylabel(L"$||U - Y||_{2} / dim(U)$")
    PyPlot.xlim(1,iter)
    PyPlot.yscale("log")

    if parameters["complete_results"]
        PyPlot.subplot(plot_Y, plot_X, 6)
        PyPlot.step(T[1:end-1], parameters["ρ"]*(U[1,:] - Y[1,:]), color="cornflowerblue", linewidth=1.0, linestyle="-", label=L"$y_1$")
        PyPlot.step(T[1:end-1], parameters["ρ"]*(U[2,:] - Y[2,:]), color="darkorange", linewidth=1.0, linestyle="-", label=L"$y_2$")
        PyPlot.step(T[1:end-1], parameters["ρ"]*(U[3,:] - Y[3,:]), color="forestgreen", linewidth=1.0, linestyle="-", label=L"$y_3$")
        PyPlot.title(L"$\rho (U - Y)$")
        PyPlot.xlabel(L"Iterations")
        PyPlot.legend()

        PyPlot.subplot(plot_Y, plot_X, 7)
        PyPlot.step(T[1:end-1], ν[1,:], color="cornflowerblue", linewidth=1.0, linestyle="-", label=L"$ν_1$")
        PyPlot.step(T[1:end-1], ν[2,:], color="darkorange", linewidth=1.0, linestyle="-", label=L"$ν_2$")
        PyPlot.step(T[1:end-1], ν[3,:], color="forestgreen", linewidth=1.0, linestyle="-", label=L"$ν_3$")
        PyPlot.title(L"\nu")
        PyPlot.xlabel(L"Time in $s$")
        PyPlot.legend()
    end
    PyPlot.tight_layout()
    if parameters["show_result"]
        return fig
    end
    PyPlot.savefig("visualization/image_plot/" * filename*"."*parameters["plot_format"], format=parameters["plot_format"], dpi=1000)
    PyPlot.close()
    return
end
