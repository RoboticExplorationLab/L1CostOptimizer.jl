function lin_cons_example(ρ, stopping_criterion)
    # Runs the linear dynamics with control contraints example
    lin_cons_parameters = define_lin_constrained_parameters()
    N = lin_cons_parameters["N"]
    lin_cons_parameters["ρ"] = ρ
    lin_cons_parameters["stopping_criterion"] = stopping_criterion
    # Propagates the nonlinear dynamics forward
    # to let the 2 satellites drift apart from each other.
    x0_full = initial_drift(lin_cons_parameters)
    # We recover from the initial drift using a open loop controller
    # relying on the linear dynamics model.
    # we set the orbit radius of the cw model.
    lin_cons_parameters["orbit_radius"] = norm(x0_full[4:6])
    x0_cw = full_to_reduced_state(x0_full)
    lin_cons_parameters["x0"] = x0_cw
    # Scale the parameters of the linear model
    lin_cons_parameters = scale_lin_parameters(lin_cons_parameters)

    # Compute the control sequence
    X, U, Y, ν, cost_history, optimality_criterion, iter = l1_solver(lin_cons_parameters)
    filename = "constrained_linear_dynamics"
    save_results(X, U, Y, ν, cost_history, optimality_criterion,
        filename, iter, lin_cons_parameters)
    save_results_pyplot(X, U, Y, ν, cost_history, optimality_criterion,
        filename, iter, lin_cons_parameters)
end

function lin_uncons_example(ρ, stopping_criterion)
    # Runs the linear dynamics without control contraints example
    lin_uncons_parameters = define_lin_unconstrained_parameters()
    N = lin_uncons_parameters["N"]
    lin_uncons_parameters["ρ"] = ρ
    lin_uncons_parameters["stopping_criterion"] = stopping_criterion
    # Propagates the nonlinear dynamics forward
    # to let the 2 satellites drift apart from each other.
    x0_full = initial_drift(lin_uncons_parameters)
    # We recover from the initial drift using a open loop controller
    # relying on the linear dynamics model.
    # we set the orbit radius of the cw model.
    lin_uncons_parameters["orbit_radius"] = norm(x0_full[4:6])
    x0_cw = full_to_reduced_state(x0_full)
    lin_uncons_parameters["x0"] = x0_cw
    # Scale the parameters of the linear model
    lin_uncons_parameters = scale_lin_parameters(lin_uncons_parameters)

    # Compute the control sequence
    X, U, Y, ν, cost_history, optimality_criterion, iter = l1_solver(lin_uncons_parameters)
    filename = "unconstrained_linear_dynamics"
    save_results(X, U, Y, ν, cost_history, optimality_criterion,
        filename, iter, lin_uncons_parameters)
    save_results_pyplot(X, U, Y, ν, cost_history, optimality_criterion,
        filename, iter, lin_uncons_parameters)
end

function non_lin_uncons_example(ρ, stopping_criterion)
    # Runs the nonlinear dynamics without control contraints example
    non_lin_uncons_parameters = define_non_lin_unconstrained_parameters()
    N = non_lin_uncons_parameters["N"]
    non_lin_uncons_parameters["ρ"] = ρ
    non_lin_uncons_parameters["stopping_criterion"] = stopping_criterion
    # Propagates the nonlinear dynamics forward
    # to let the 2 satellites drift apart from each other.
    x0_full = initial_drift(non_lin_uncons_parameters)
    # We recover from the initial drift using a open loop controller
    # relying on the linear dynamics model.
    # we set the orbit radius of the cw model.
    non_lin_uncons_parameters["orbit_radius"] = norm(x0_full[4:6])
    non_lin_uncons_parameters["x0"] = x0_full
    println("x0_full = ", x0_full)
    # Scale the parameters of the linear model
    non_lin_uncons_parameters = scale_non_lin_parameters(non_lin_uncons_parameters)

    # Compute the control sequence
    X, U, Y, ν, cost_history, optimality_criterion, iter = l1_solver(non_lin_uncons_parameters)
    filename = "unconstrained_nonlinear_dynamics"
    save_results(X, U, Y, ν, cost_history, optimality_criterion,
        filename, iter, non_lin_uncons_parameters)
    save_results_pyplot(X, U, Y, ν, cost_history, optimality_criterion,
        filename, iter, non_lin_uncons_parameters)
end

function non_lin_cons_example(ρ, stopping_criterion)
    # Runs the nonlinear dynamics with control contraints example
    non_lin_cons_parameters = define_non_lin_constrained_parameters()
    N = non_lin_cons_parameters["N"]
    non_lin_cons_parameters["ρ"] = ρ
    non_lin_cons_parameters["stopping_criterion"] = stopping_criterion
    # Propagates the nonlinear dynamics forward
    # to let the 2 satellites drift apart from each other.
    x0_full = initial_drift(non_lin_cons_parameters)
    # We recover from the initial drift using a open loop controller
    # relying on the linear dynamics model.
    # we set the orbit radius of the cw model.
    non_lin_cons_parameters["orbit_radius"] = norm(x0_full[4:6])
    non_lin_cons_parameters["x0"] = x0_full
    # Scale the parameters of the linear model
    non_lin_cons_parameters = scale_non_lin_parameters(non_lin_cons_parameters)

    # Compute the control sequence
    X, U, Y, ν, cost_history, optimality_criterion, iter = l1_solver(non_lin_cons_parameters)
    filename = "constrained_nonlinear_dynamics"
    save_results(X, U, Y, ν, cost_history, optimality_criterion,
        filename, iter, non_lin_cons_parameters)
    save_results_pyplot(X, U, Y, ν, cost_history, optimality_criterion,
        filename, iter, non_lin_cons_parameters)
end
