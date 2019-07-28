include("cost.jl")
include("dynamics.jl")
include("experiment.jl")
include("parameter_scaling.jl")
include("solver.jl")
include("utils.jl")

function test_linear_dynamics_scaling()
    lin_uncons_parameters = define_lin_unconstrained_parameters()
    scale_lin_parameters(lin_parameters)

    ẋ1 = zeros(lin_uncons_parameters["n"])
    x1 = ones(lin_uncons_parameters["n"])
    u1 = ones(lin_uncons_parameters["m"])
    cw_dynamics(ẋ1, x1, u1, lin_uncons_parameters)
    Δt_rescaled = lin_uncons_parameters["Δt"] * lin_uncons_parameters["t_ref"]
    x1_next = x1 + Δt_rescaled * ẋ1

    ẋ2 = zeros(lin_uncons_parameters["n"])
    x2 = ones(lin_uncons_parameters["n"])
    u2 = ones(lin_uncons_parameters["m"])
    x2[1:3] /= lin_uncons_parameters["l_ref"]
    x2[4:6] /= lin_uncons_parameters["v_ref"]
    u2 /= lin_uncons_parameters["u_ref"]
    scaled_cw_dynamics(ẋ2, x2, u2, lin_uncons_parameters)
    Δt = lin_uncons_parameters["Δt"]
    x2_next = x2 + Δt * ẋ2

    ẋ2_rescaled = zeros(lin_uncons_parameters["n"])
    ẋ2_rescaled[1:3] = ẋ2[1:3] * lin_uncons_parameters["v_ref"]
    ẋ2_rescaled[4:6] = ẋ2[4:6] * lin_uncons_parameters["a_ref"]

    x2_next_rescaled = zeros(lin_uncons_parameters["n"])
    x2_next_rescaled[1:3] = x2_next[1:3] * lin_uncons_parameters["l_ref"]
    x2_next_rescaled[4:6] = x2_next[4:6] * lin_uncons_parameters["v_ref"]

    println("**TEST 1** ", maximum((ẋ1 - ẋ2_rescaled) ./ (ẋ1 .+ 1e-5)))
    println("**TEST 2** ", maximum((x1_next - x2_next_rescaled) ./ (x1_next .+ 1e-5)))
end

function test_non_linear_dynamics_scaling()
    non_lin_uncons_parameters = define_non_lin_unconstrained_parameters()
    scale_non_lin_parameters(non_lin_uncons_parameters)

    ẋ1 = zeros(non_lin_uncons_parameters["n"])
    μ = non_lin_uncons_parameters["μ"]
    orbit_radius = non_lin_uncons_parameters["orbit_radius"]
    x1 = [1.0, 2.0, 3.0, orbit_radius, 0.0, 0.0, 10.0, 20.0, 30.0, 0.0, sqrt(μ/orbit_radius), 0.0]
    u1 = ones(non_lin_uncons_parameters["m"])
    non_linear_dynamics(ẋ1, x1, u1, non_lin_uncons_parameters)
    Δt_rescaled = non_lin_uncons_parameters["Δt"] * non_lin_uncons_parameters["t_ref"]
    x1_next = x1 + Δt_rescaled * ẋ1

    ẋ2 = zeros(non_lin_uncons_parameters["n"])
    x2 = [1.0, 2.0, 3.0, orbit_radius, 0.0, 0.0, 10.0, 20.0, 30.0, 0.0, sqrt(μ/orbit_radius), 0.0]
    u2 = ones(non_lin_uncons_parameters["m"])
    x2[1:3] /= non_lin_uncons_parameters["l_ego_ref"]
    x2[4:6] /= non_lin_uncons_parameters["l_target_ref"]
    x2[7:9] /= non_lin_uncons_parameters["v_ego_ref"]
    x2[10:12] /= non_lin_uncons_parameters["v_target_ref"]
    u2 /= non_lin_uncons_parameters["u_ref"]
    scaled_non_linear_dynamics(ẋ2, x2, u2, non_lin_uncons_parameters)
    Δt = non_lin_uncons_parameters["Δt"]
    x2_next = x2 + Δt * ẋ2

    ẋ2_rescaled = zeros(non_lin_uncons_parameters["n"])
    ẋ2_rescaled[1:3] = ẋ2[1:3] * non_lin_uncons_parameters["v_ego_ref"]
    ẋ2_rescaled[4:6] = ẋ2[4:6] * non_lin_uncons_parameters["v_target_ref"]
    ẋ2_rescaled[7:9] = ẋ2[7:9] * non_lin_uncons_parameters["a_ego_ref"]
    ẋ2_rescaled[10:12] = ẋ2[10:12] * non_lin_uncons_parameters["a_target_ref"]

    x2_next_rescaled = zeros(non_lin_uncons_parameters["n"])
    x2_next_rescaled[1:3] = x2_next[1:3] * non_lin_uncons_parameters["l_ego_ref"]
    x2_next_rescaled[4:6] = x2_next[4:6] * non_lin_uncons_parameters["l_target_ref"]
    x2_next_rescaled[7:9] = x2_next[7:9] * non_lin_uncons_parameters["v_ego_ref"]
    x2_next_rescaled[10:12] = x2_next[10:12] * non_lin_uncons_parameters["v_target_ref"]
    println("**TEST 1** ", maximum((ẋ1 - ẋ2_rescaled) ./ (ẋ1 .+ 1e-5)))
    println("**TEST 2** ", maximum((x1_next - x2_next_rescaled) ./ (x1_next .+ 1e-5)))
end

function test_dynamics_consistency()
    lin_uncons_parameters = define_lin_unconstrained_parameters()
    non_lin_uncons_parameters = define_non_lin_unconstrained_parameters()
    x0_full = initial_drift(lin_uncons_parameters)
    lin_uncons_parameters["x0"] = full_to_cw(x0_full)
    non_lin_uncons_parameters["x0"] = x0_full
    N = lin_uncons_parameters["N"]
    tf = lin_uncons_parameters["tf"]
    x0_cw =  full_to_cw(x0_full)
    println("x0_full = ", x0_full)
    println("x0_cw = ", x0_cw)
    U_cw = load("result/control/lin_uncons.jld", "U")
    U_full = load("result/control/non_lin_uncons.jld", "U")

    function cw_dynamics!(ẋ,x,u)
        cw_dynamics(ẋ, x, u, lin_uncons_parameters)
    end
    function non_linear_dynamics!(ẋ,x,u)
        non_linear_dynamics(ẋ, x, u, non_lin_uncons_parameters)
    end
    function scaled_cw_dynamics!(ẋ,x,u)
        scaled_cw_dynamics(ẋ, x, u, lin_uncons_parameters)
    end
    function scaled_non_linear_dynamics!(ẋ,x,u)
        scaled_non_linear_dynamics(ẋ, x, u, non_lin_uncons_parameters)
    end

    n = 12
    m = 3
    X_full = rollout_dynamics(n, m, N, tf, x0_full, U_full, non_linear_dynamics!)
    X_full_scaled = rollout_dynamics(n, m, N, tf, x0_full, U_full, scaled_non_linear_dynamics!)
    n = 6
    X_cw = rollout_dynamics(n, m, N, tf, x0_cw, U_cw, cw_dynamics!)
    X_cw_scaled = rollout_dynamics(n, m, N, tf, x0_cw, U_cw, scaled_cw_dynamics!)

    X_converted = zeros(lin_uncons_parameters["n"], N)
    X_converted_scaled = zeros(lin_uncons_parameters["n"], N)
    println(x0_cw)
    for k=1:N
        X_converted[:,k] = full_to_cw(X_full[:,k])
        X_converted_scaled[:,k] = full_to_cw(X_full_scaled[:,k])
    end
    # println("delta = ", (X_converted .- X_cw)./ (1e-5 .+ X_cw))
    # println("delta = ", (U_full .- U_cw)./ (1e-5 .+ U_cw))

    compare_plot(N, tf, X_cw, X_cw_scaled, X_converted, X_converted_scaled, U_cw, U_full)
end

function compare_plot(N, tf, X_cw, X_cw_scaled, X_converted, X_converted_scaled, U_cw, U_full)
    plot_X = 4
    plot_Y = 3
    T = [(k-1)*tf/N for k=1:N]
    fig = figure(figsize=(3*plot_X,3*plot_Y))

    subplot(plot_Y, plot_X, 1)
    plot(T, X_cw[1,:], color="cornflowerblue", linewidth=1.0, linestyle="-", label=L"$x_1$ ego")
    plot(T, X_cw[2,:], color="darkorange", linewidth=1.0, linestyle="-", label=L"$x_2$ ego")
    plot(T, X_cw[3,:], color="forestgreen", linewidth=1.0, linestyle="-", label=L"$x_3$ ego")
    title("Positions CW")
    #grid("on")
    ticklabel_format(axis="both", style="sci", scilimits=(0,0))
    xlabel(L"Time in $s$")
    ylabel(L"Position in $m$")
    # legend()

    subplot(plot_Y, plot_X, 2)
    plot(T, X_cw[4,:], color="cornflowerblue", linewidth=1.0, linestyle="-", label=L"$\dot{x}_1$ ego")
    plot(T, X_cw[5,:], color="darkorange", linewidth=1.0, linestyle="-", label=L"$\dot{x}_2$ ego")
    plot(T, X_cw[6,:], color="forestgreen", linewidth=1.0, linestyle="-", label=L"$\dot{x}_3$ ego")
    title("Velocities CW")
    #grid("on")
    ticklabel_format(axis="both", style="sci", scilimits=(0,0))
    xlabel(L"Time in $s$")
    ylabel(L"Velocity in $m/s$")
    # legend()

    subplot(plot_Y, plot_X, 3)
    plot(T, X_cw_scaled[1,:], color="cornflowerblue", linewidth=1.0, linestyle="-", label=L"$x_1$ ego")
    plot(T, X_cw_scaled[2,:], color="darkorange", linewidth=1.0, linestyle="-", label=L"$x_2$ ego")
    plot(T, X_cw_scaled[3,:], color="forestgreen", linewidth=1.0, linestyle="-", label=L"$x_3$ ego")
    title("Positions CW scaled")
    #grid("on")
    ticklabel_format(axis="both", style="sci", scilimits=(0,0))
    xlabel(L"Time in $s$")
    ylabel(L"Position in $m$")
    # legend()

    subplot(plot_Y, plot_X, 4)
    plot(T, X_cw_scaled[4,:], color="cornflowerblue", linewidth=1.0, linestyle="-", label=L"$\dot{x}_1$ ego")
    plot(T, X_cw_scaled[5,:], color="darkorange", linewidth=1.0, linestyle="-", label=L"$\dot{x}_2$ ego")
    plot(T, X_cw_scaled[6,:], color="forestgreen", linewidth=1.0, linestyle="-", label=L"$\dot{x}_3$ ego")
    title("Velocities CW scaled")
    #grid("on")
    ticklabel_format(axis="both", style="sci", scilimits=(0,0))
    xlabel(L"Time in $s$")
    ylabel(L"Velocity in $m/s$")
    # legend()

    subplot(plot_Y, plot_X, 5)
    plot(T, X_converted[1,:], color="cornflowerblue", linewidth=2.0, linestyle="--", label=L"$x_1$ ego")
    plot(T, X_converted[2,:], color="darkorange", linewidth=2.0, linestyle="--", label=L"$x_2$ ego")
    plot(T, X_converted[3,:], color="forestgreen", linewidth=2.0, linestyle="--", label=L"$x_3$ ego")
    title("Positions conv")
    #grid("on")
    ticklabel_format(axis="both", style="sci", scilimits=(0,0))
    xlabel(L"Time in $s$")
    ylabel(L"Position in $m$")
    # legend()

    subplot(plot_Y, plot_X, 6)
    plot(T, X_converted[4,:], color="cornflowerblue", linewidth=2.0, linestyle="--", label=L"$\dot{x}_1$ ego")
    plot(T, X_converted[5,:], color="darkorange", linewidth=2.0, linestyle="--", label=L"$\dot{x}_2$ ego")
    plot(T, X_converted[6,:], color="forestgreen", linewidth=2.0, linestyle="--", label=L"$\dot{x}_3$ ego")
    title("Velocities conv")
    #grid("on")
    ticklabel_format(axis="both", style="sci", scilimits=(0,0))
    xlabel(L"Time in $s$")
    ylabel(L"Velocity in $m/s$")
    # legend()

    subplot(plot_Y, plot_X, 7)
    plot(T, X_converted_scaled[1,:], color="cornflowerblue", linewidth=2.0, linestyle="--", label=L"$x_1$ ego")
    plot(T, X_converted_scaled[2,:], color="darkorange", linewidth=2.0, linestyle="--", label=L"$x_2$ ego")
    plot(T, X_converted_scaled[3,:], color="forestgreen", linewidth=2.0, linestyle="--", label=L"$x_3$ ego")
    title("Positions conv scaled")
    #grid("on")
    ticklabel_format(axis="both", style="sci", scilimits=(0,0))
    xlabel(L"Time in $s$")
    ylabel(L"Position in $m$")
    # legend()

    subplot(plot_Y, plot_X, 8)
    plot(T, X_converted_scaled[4,:], color="cornflowerblue", linewidth=2.0, linestyle="--", label=L"$\dot{x}_1$ ego")
    plot(T, X_converted_scaled[5,:], color="darkorange", linewidth=2.0, linestyle="--", label=L"$\dot{x}_2$ ego")
    plot(T, X_converted_scaled[6,:], color="forestgreen", linewidth=2.0, linestyle="--", label=L"$\dot{x}_3$ ego")
    title("Velocities conv scaled")
    #grid("on")
    ticklabel_format(axis="both", style="sci", scilimits=(0,0))
    xlabel(L"Time in $s$")
    ylabel(L"Velocity in $m/s$")
    # legend()

    subplot(plot_Y, plot_X, 9)
    step(T[1:end-1],  - U_cw[1,:], color="cornflowerblue", linewidth=1.0, linestyle="-", label=L"$u_1$")
    step(T[1:end-1], U_cw[2,:], color="darkorange", linewidth=1.0, linestyle="-", label=L"$u_2$")
    step(T[1:end-1], U_cw[3,:], color="forestgreen", linewidth=1.0, linestyle="-", label=L"$u_3$")
    title("Control")
    #grid("on")
    ticklabel_format(axis="both", style="sci", scilimits=(0,0))
    xlabel(L"Time in $s$")
    ylabel(L"Controls in $N$")
    yscale("linear")
    # yscale("log")
    legend()

    subplot(plot_Y, plot_X, 10)
    step(T[1:end-1], U_full[1,:], color="cornflowerblue", linewidth=1.0, linestyle="-", label=L"$u_full_1$")
    step(T[1:end-1], U_full[2,:], color="darkorange", linewidth=1.0, linestyle="-", label=L"$u_full_2$")
    step(T[1:end-1], U_full[3,:], color="forestgreen", linewidth=1.0, linestyle="-", label=L"$u_full_3$")
    title("Control")
    #grid("on")
    ticklabel_format(axis="both", style="sci", scilimits=(0,0))
    xlabel(L"Time in $s$")
    ylabel(L"Controls in $N$")
    yscale("linear")
    # yscale("log")
    legend()

    subplot(plot_Y, plot_X, 11)
    step(T[1:end-1], U_full[1,:] - U_cw[1,:], color="cornflowerblue", linewidth=1.0, linestyle="-", label=L"$u_full_1$")
    step(T[1:end-1], U_full[2,:] - U_cw[2,:], color="darkorange", linewidth=1.0, linestyle="-", label=L"$u_full_2$")
    step(T[1:end-1], U_full[3,:] - U_cw[3,:], color="forestgreen", linewidth=1.0, linestyle="-", label=L"$u_full_3$")
    title("Control")
    #grid("on")
    ticklabel_format(axis="both", style="sci", scilimits=(0,0))
    xlabel(L"Time in $s$")
    ylabel(L"Controls in $N$")
    yscale("linear")
    # yscale("log")
    legend()

    tight_layout()
    savefig("result/" * "test.eps", format="eps", dpi=300)
    close()
end

function drifting_simulation(n, m, N, tf, x0, dynamics)
    X = zeros(n, N)
    X[:,1] = x0
    for k=2:N
        u = zeros(m)
        Δt = tf / (N-1)
        X[:,k] = rk4_step(dynamics, X[:,k-1], u, Δt)
    end
    return X
end

function rollout_dynamics(n, m, N, tf, x0, U, dynamics)
    X = zeros(n, N)
    X[:,1] = x0
    for k=2:N
        Δt = tf / (N-1)
        X[:,k] = rk4_step(dynamics, X[:,k-1], U[:,k-1], Δt)
    end
    return X
end

function initial_drift(parameters)
    non_lin_uncons_parameters = define_non_lin_unconstrained_parameters()
    tf_drift = parameters["tf_drift"]
    N_drift = parameters["N_drift"]
    x0_drift = parameters["x0_drift"]
    function non_linear_dynamics!(ẋ,x,u)
        non_linear_dynamics(ẋ, x, u, non_lin_uncons_parameters)
    end
    n = non_lin_uncons_parameters["n"]
    m = non_lin_uncons_parameters["m"]
    X_history = drifting_simulation(n, m, N_drift, tf_drift, x0_drift, non_linear_dynamics!)
    x0_full = X_history[:,N_drift]
    return x0_full
end

# test_linear_dynamics_scaling()
# test_non_linear_dynamics_scaling()
# test_dynamics_consistency()
