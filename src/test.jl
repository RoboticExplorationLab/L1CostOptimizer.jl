include("cost.jl")
include("dynamics.jl")
include("parameters.jl")
include("solver.jl")
include("utils.jl")

function test_linear_dynamics_scaling()
    lin_parameters = define_lin_parameters()
    scale_lin_parameters(lin_parameters)

    ẋ1 = zeros(lin_parameters["n"])
    x1 = ones(lin_parameters["n"])
    u1 = ones(lin_parameters["m"])
    cw_dynamics!(ẋ1, x1, u1, lin_parameters)
    Δt_rescaled = lin_parameters["Δt"] * lin_parameters["t_ref"]
    x1_next = x1 + Δt_rescaled * ẋ1

    ẋ2 = zeros(lin_parameters["n"])
    x2 = ones(lin_parameters["n"])
    u2 = ones(lin_parameters["m"])
    x2[1:3] /= lin_parameters["l_ref"]
    x2[4:6] /= lin_parameters["v_ref"]
    u2 /= lin_parameters["u_ref"]
    scaled_cw_dynamics!(ẋ2, x2, u2, lin_parameters)
    Δt = lin_parameters["Δt"]
    x2_next = x2 + Δt * ẋ2

    ẋ2_rescaled = zeros(lin_parameters["n"])
    ẋ2_rescaled[1:3] = ẋ2[1:3] * lin_parameters["v_ref"]
    ẋ2_rescaled[4:6] = ẋ2[4:6] * lin_parameters["a_ref"]

    x2_next_rescaled = zeros(lin_parameters["n"])
    x2_next_rescaled[1:3] = x2_next[1:3] * lin_parameters["l_ref"]
    x2_next_rescaled[4:6] = x2_next[4:6] * lin_parameters["v_ref"]

    println("**TEST 1** ", maximum((ẋ1 - ẋ2_rescaled) ./ (ẋ1 .+ 1e-5)))
    println("**TEST 2** ", maximum((x1_next - x2_next_rescaled) ./ (x1_next .+ 1e-5)))
end

function test_non_linear_dynamics_scaling()
    non_lin_parameters = define_non_lin_parameters()
    scale_non_lin_parameters(non_lin_parameters)

    ẋ1 = zeros(non_lin_parameters["n"])
    μ = non_lin_parameters["μ"]
    orbit_radius = non_lin_parameters["orbit_radius"]
    x1 = [1.0, 2.0, 3.0, orbit_radius, 0.0, 0.0, 10.0, 20.0, 30.0, 0.0, sqrt(μ/orbit_radius), 0.0]
    u1 = ones(non_lin_parameters["m"])
    non_linear_dynamics!(ẋ1, x1, u1, non_lin_parameters)
    Δt_rescaled = non_lin_parameters["Δt"] * non_lin_parameters["t_ref"]
    x1_next = x1 + Δt_rescaled * ẋ1

    ẋ2 = zeros(non_lin_parameters["n"])
    x2 = [1.0, 2.0, 3.0, orbit_radius, 0.0, 0.0, 10.0, 20.0, 30.0, 0.0, sqrt(μ/orbit_radius), 0.0]
    u2 = ones(non_lin_parameters["m"])
    x2[1:3] /= non_lin_parameters["l_ego_ref"]
    x2[4:6] /= non_lin_parameters["l_target_ref"]
    x2[7:9] /= non_lin_parameters["v_ego_ref"]
    x2[10:12] /= non_lin_parameters["v_target_ref"]
    u2 /= non_lin_parameters["u_ref"]
    scaled_non_linear_dynamics!(ẋ2, x2, u2, non_lin_parameters)
    Δt = non_lin_parameters["Δt"]
    x2_next = x2 + Δt * ẋ2

    ẋ2_rescaled = zeros(non_lin_parameters["n"])
    ẋ2_rescaled[1:3] = ẋ2[1:3] * non_lin_parameters["v_ego_ref"]
    ẋ2_rescaled[4:6] = ẋ2[4:6] * non_lin_parameters["v_target_ref"]
    ẋ2_rescaled[7:9] = ẋ2[7:9] * non_lin_parameters["a_ego_ref"]
    ẋ2_rescaled[10:12] = ẋ2[10:12] * non_lin_parameters["a_target_ref"]

    x2_next_rescaled = zeros(non_lin_parameters["n"])
    x2_next_rescaled[1:3] = x2_next[1:3] * non_lin_parameters["l_ego_ref"]
    x2_next_rescaled[4:6] = x2_next[4:6] * non_lin_parameters["l_target_ref"]
    x2_next_rescaled[7:9] = x2_next[7:9] * non_lin_parameters["v_ego_ref"]
    x2_next_rescaled[10:12] = x2_next[10:12] * non_lin_parameters["v_target_ref"]
    println("**TEST 1** ", maximum((ẋ1 - ẋ2_rescaled) ./ (ẋ1 .+ 1e-5)))
    println("**TEST 2** ", maximum((x1_next - x2_next_rescaled) ./ (x1_next .+ 1e-5)))
end

test_linear_dynamics_scaling()
test_non_linear_dynamics_scaling()


function test_dynamics_consistency()
    lin_uncons_parameters = define_lin_unconstrained_parameters()
    tf_drift = lin_uncons_parameters["tf_drift"]
    N_drift = lin_uncons_parameters["N_drift"]
    x0_full = lin_uncons_parameters["x0_drift"]
    x0_full[1:3] /= 1
    x0_full[7:9] /= 1000000

    x0_cw = full_to_cw(x0_full)
    X_full = drifting_simulation(N_drift, tf_drift, x0_full)
    X_cw = drifting_simulation_cw(N_drift, tf_drift, x0_cw)
    X_converted = zeros(lin_uncons_parameters["n"], N_drift)
    for k=1:N_drift
        X_converted[:,k] = full_to_cw(X_full[:,k])
    end
    # println("X_cw = ", X_cw)
    # println("X_full = ", X_converted)
    # println("delta = ", (X_converted .- X_cw)./ (1e-5 .+ X_cw))

    compare_plot(X_cw, X_converted)
    # println("**TEST ** ", maximum((ẋ1 - ẋ2_rescaled) ./ (ẋ1 .+ 1e-5)))
    # println("**TEST 2** ", maximum((x1_next - x2_next_rescaled) ./ (x1_next .+ 1e-5)))
end

function compare_plot(X_cw, X_converted)
    plot_X = 2
    plot_Y = 1
    fig = figure(figsize=(3*plot_X,3*plot_Y))
    subplot(plot_Y, plot_X, 1)
    println(size(X_cw[1,]))
    plot(X_cw[1,:], color="cornflowerblue", linewidth=1.0, linestyle="-", label=L"$x_1$ ego")
    plot(X_cw[2,:], color="darkorange", linewidth=1.0, linestyle="-", label=L"$x_2$ ego")
    plot(X_cw[3,:], color="forestgreen", linewidth=1.0, linestyle="-", label=L"$x_3$ ego")

    plot(X_converted[1,:], color="cornflowerblue", linewidth=2.0, linestyle="--", label=L"$x_1$ ego")
    plot(X_converted[2,:], color="darkorange", linewidth=2.0, linestyle="--", label=L"$x_2$ ego")
    plot(X_converted[3,:], color="forestgreen", linewidth=2.0, linestyle="--", label=L"$x_3$ ego")
    title("Positions")
    #grid("on")
    ticklabel_format(axis="both", style="sci", scilimits=(0,0))
    xlabel(L"Time in $s$")
    ylabel(L"Position in $m$")
    # legend()

    subplot(plot_Y, plot_X, 2)
    plot(X_cw[4,:], color="cornflowerblue", linewidth=1.0, linestyle="-", label=L"$\dot{x}_1$ ego")
    plot(X_cw[5,:], color="darkorange", linewidth=1.0, linestyle="-", label=L"$\dot{x}_2$ ego")
    plot(X_cw[6,:], color="forestgreen", linewidth=1.0, linestyle="-", label=L"$\dot{x}_3$ ego")

    plot(X_converted[4,:], color="cornflowerblue", linewidth=2.0, linestyle="--", label=L"$\dot{x}_1$ ego")
    plot(X_converted[5,:], color="darkorange", linewidth=2.0, linestyle="--", label=L"$\dot{x}_2$ ego")
    plot(X_converted[6,:], color="forestgreen", linewidth=2.0, linestyle="--", label=L"$\dot{x}_3$ ego")
    title("Velocities")
    #grid("on")
    ticklabel_format(axis="both", style="sci", scilimits=(0,0))
    xlabel(L"Time in $s$")
    ylabel(L"Velocity in $m/s$")
    # legend()

    tight_layout()
    savefig("result/" * "test.eps", format="eps", dpi=300)
    close()
end






function drifting_simulation_cw(N, tf, x0)
    lin_parameters = define_lin_parameters()
    n = lin_parameters["n"]
    m = lin_parameters["m"]
    X = zeros(n, N)
    X[:,1] = x0
    for k=2:N
        u = zeros(m)
        Δt = tf / (N-1)
        X[:,k] = rk4_step(cw_dynamics!, X[:,k-1], u, Δt)
    end
    return X
end
test_dynamics_consistency()
