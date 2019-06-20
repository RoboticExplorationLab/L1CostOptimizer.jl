using PyPlot

function save_results(X, U, Y, filename)
    num_iter = params["num_iter"]
    N = params["N"]
    n = params["n"]
    Δt = params["Δt"]
    T = [Δt * i for i=1:N]

    fig = figure(figsize=(9,3))
    subplot(1, 3, 1)
    step(T, X[1,:], color="blue", linewidth=1.0, linestyle="-", label=L"$x_1$")
    step(T, X[2,:], color="red", linewidth=1.0, linestyle="-", label=L"$x_2$")
    step(T, X[3,:], color="green", linewidth=1.0, linestyle="-", label=L"$x_3$")
    title(L"Positions")
    grid("on")
    xlabel(L"Time in $s$")
    ylabel(L"Position in $m$")
    legend()

    subplot(1, 3, 2)
    step(T, X[4,:], color="blue", linewidth=1.0, linestyle="-", label=L"$\dot{x}_1$")
    step(T, X[5,:], color="red", linewidth=1.0, linestyle="-", label=L"$\dot{x}_2$")
    step(T, X[6,:], color="green", linewidth=1.0, linestyle="-", label=L"$\dot{x}_3$")
    title(L"Velocities")
    grid("on")
    xlabel(L"Time in $s$")
    ylabel(L"Velocity in $m/s$")
    legend()

    subplot(1, 3, 3)
    step(T[1:end-1], U[1,:], color="blue", linewidth=1.0, linestyle="-", label=L"$u_1$")
    step(T[1:end-1], U[2,:], color="red", linewidth=1.0, linestyle="-", label=L"$u_2$")
    step(T[1:end-1], U[3,:], color="green", linewidth=1.0, linestyle="-", label=L"$u_3$")
    title(L"Controls")
    grid("on")
    xlabel(L"Time in $s$")
    ylabel(L"Controls in $N$")
    legend()

    tight_layout()
    savefig("result/" * filename, format="png", dpi=1000)
    close()
    return
end
