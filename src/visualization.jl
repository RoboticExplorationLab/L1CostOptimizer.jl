using PyPlot
function save_results_pyplot(X_, U_, Y_, ν, cost_history, constraint_violation, optimality_criterion, filename, iter, parameters)
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
    # yscale("log")
    PyPlot.legend()

    PyPlot.subplot(plot_Y, plot_X, 4)
    # PyPlot.plot(log.(10, cost_history[:,1]), color="cornflowerblue", linewidth=1.0, linestyle="-", label=L"$cost$")
    PyPlot.plot([i for i=1:iter], cost_history[1:iter,1], color="cornflowerblue", linewidth=1.0, linestyle="-", label=L"$cost$")
    PyPlot.title("Cost")
    PyPlot.xlabel("L1 Solver Iterations")
    # ylabel("")
    PyPlot.yscale("linear")
    PyPlot.xlim(1,iter)
    # legend()

    PyPlot.subplot(plot_Y, plot_X, 5)
    PyPlot.plot([i for i=1:iter], optimality_criterion[1:iter], color="forestgreen", linewidth=1.0, linestyle="-", label=L"$||\nabla_{X,U} L||_{\infty}$")
    PyPlot.title("Optimality Criterion")
    PyPlot.xlabel("L1 Solver Iterations")
    PyPlot.ylabel(L"$||U - Y||_{2} / dim(U)$")
    PyPlot.xlim(1,iter)
    # legend()
    PyPlot.yscale("log")

    if parameters["complete_results"]
        PyPlot.subplot(plot_Y, plot_X, 6)
        PyPlot.step(T[1:end-1], parameters["ρ"]*(U[1,:] - Y[1,:]), color="cornflowerblue", linewidth=1.0, linestyle="-", label=L"$y_1$")
        PyPlot.step(T[1:end-1], parameters["ρ"]*(U[2,:] - Y[2,:]), color="darkorange", linewidth=1.0, linestyle="-", label=L"$y_2$")
        PyPlot.step(T[1:end-1], parameters["ρ"]*(U[3,:] - Y[3,:]), color="forestgreen", linewidth=1.0, linestyle="-", label=L"$y_3$")
        PyPlot.title(L"$\rho (U - Y)$")
        PyPlot.xlabel(L"Iterations")
        # ylabel(L" ")
        PyPlot.legend()

        PyPlot.subplot(plot_Y, plot_X, 7)
        PyPlot.step(T[1:end-1], ν[1,:], color="cornflowerblue", linewidth=1.0, linestyle="-", label=L"$ν_1$")
        PyPlot.step(T[1:end-1], ν[2,:], color="darkorange", linewidth=1.0, linestyle="-", label=L"$ν_2$")
        PyPlot.step(T[1:end-1], ν[3,:], color="forestgreen", linewidth=1.0, linestyle="-", label=L"$ν_3$")
        PyPlot.title(L"\nu")
        PyPlot.xlabel(L"Time in $s$")
        # ylabel(L" ")
        PyPlot.legend()
    end
    PyPlot.tight_layout()
    PyPlot.savefig("result/" * filename*"."*parameters["plot_format"], format=parameters["plot_format"], dpi=1000)
    PyPlot.close()
    JLD.save("result/control/" * filename*".jld", "U", U, "x0", X[:,1])
    return
end
