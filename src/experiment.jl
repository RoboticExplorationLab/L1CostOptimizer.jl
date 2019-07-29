function lin_cons_example(ρ, stopping_criterion)
    lin_cons_parameters = define_lin_constrained_parameters()
    N = lin_cons_parameters["N"]
    lin_cons_parameters["ρ"] = ρ
    lin_cons_parameters["stopping_criterion"] = stopping_criterion

    x0_full = initial_drift(lin_cons_parameters)
    # We recover from the initial drift using a open loop controller reyling on the linear dynamics model.
    lin_cons_parameters["orbit_radius"] = norm(x0_full[4:6]) # we set the orbit radius of the cw model.
    x0_cw = full_to_cw(x0_full)
    lin_cons_parameters["x0"] = x0_cw
    println("x0_full = ", x0_full)
    println("x0_cw = ", x0_cw)

    # Scale the parameters of the linear model
    lin_cons_parameters = scale_lin_parameters(lin_cons_parameters)

    # Compute the control sequence
    X, U, Y, ν, cost_history, constraint_violation, optimality_criterion, iter = l1_solver(lin_cons_parameters)
    # filename = "lin_cons_" * "_rho_" * string(lin_cons_parameters["ρ"])
    # filename *= "_iter_" * string(lin_cons_parameters["num_iter"]) * "_Qf_" * string(lin_cons_parameters["Qf"][1,1])
    filename = "lin_cons"
    save_results(X, U, Y, ν, cost_history, constraint_violation, optimality_criterion, filename, iter, lin_cons_parameters)
end

function lin_uncons_example(ρ, stopping_criterion)
    lin_uncons_parameters = define_lin_unconstrained_parameters()
    N = lin_uncons_parameters["N"]
    lin_uncons_parameters["ρ"] = ρ
    lin_uncons_parameters["stopping_criterion"] = stopping_criterion

    x0_full = initial_drift(lin_uncons_parameters)
    # We recover from the initial drift using a open loop controller reyling on the linear dynamics model.
    lin_uncons_parameters["orbit_radius"] = norm(x0_full[4:6]) # we set the orbit radius of the cw model.
    x0_cw = full_to_cw(x0_full)
    lin_uncons_parameters["x0"] = x0_cw
    println("x0_full = ", x0_full)
    println("x0_cw = ", x0_cw)

    # Scale the parameters of the linear model
    lin_uncons_parameters = scale_lin_parameters(lin_uncons_parameters)

    # Compute the control sequence
    X, U, Y, ν, cost_history, constraint_violation, optimality_criterion, iter = l1_solver(lin_uncons_parameters)
    # filename = "lin_uncons_" * "_rho_" * string(lin_uncons_parameters["ρ"])
    # filename *= "_iter_" * string(lin_uncons_parameters["num_iter"]) * "_Qf_" * string(lin_uncons_parameters["Qf"][1,1])
    filename = "lin_uncons"
    save_results(X, U, Y, ν, cost_history, constraint_violation, optimality_criterion, filename, iter, lin_uncons_parameters)
end

function non_lin_uncons_example(ρ, stopping_criterion)
    non_lin_uncons_parameters = define_non_lin_unconstrained_parameters()
    N = non_lin_uncons_parameters["N"]
    non_lin_uncons_parameters["ρ"] = ρ
    non_lin_uncons_parameters["stopping_criterion"] = stopping_criterion

    x0_full = initial_drift(non_lin_uncons_parameters)
    # We recover from the initial drift using a open loop controller reyling on the linear dynamics model.
    non_lin_uncons_parameters["orbit_radius"] = norm(x0_full[4:6]) # we set the orbit radius of the cw model.
    non_lin_uncons_parameters["x0"] = x0_full
    println("x0_full = ", x0_full)

    # Scale the parameters of the linear model
    non_lin_uncons_parameters = scale_non_lin_parameters(non_lin_uncons_parameters)
    println("non_lin_uncons_parameters[x0] = ", non_lin_uncons_parameters["x0"])

    # Compute the control sequence
    X, U, Y, ν, cost_history, constraint_violation, optimality_criterion, iter = l1_solver(non_lin_uncons_parameters)
    # filename = "non_lin_uncons_" * "_rho_" * string(non_lin_uncons_parameters["ρ"])
    # filename *= "_iter_" * string(non_lin_uncons_parameters["num_iter"]) * "_Qf_" * string(non_lin_uncons_parameters["Qf"][1,1])
    filename = "non_lin_uncons"
    save_results(X, U, Y, ν, cost_history, constraint_violation, optimality_criterion, filename, iter, non_lin_uncons_parameters)
end

function non_lin_cons_example(ρ, stopping_criterion)
    non_lin_cons_parameters = define_non_lin_constrained_parameters()
    N = non_lin_cons_parameters["N"]
    non_lin_cons_parameters["ρ"] = ρ
    non_lin_cons_parameters["stopping_criterion"] = stopping_criterion

    x0_full = initial_drift(non_lin_cons_parameters)
    # We recover from the initial drift using a open loop controller reyling on the linear dynamics model.
    non_lin_cons_parameters["orbit_radius"] = norm(x0_full[4:6]) # we set the orbit radius of the cw model.
    non_lin_cons_parameters["x0"] = x0_full
    println("x0_full = ", x0_full)

    # Scale the parameters of the linear model
    non_lin_cons_parameters = scale_non_lin_parameters(non_lin_cons_parameters)

    # Compute the control sequence
    X, U, Y, ν, cost_history, constraint_violation, optimality_criterion, iter = l1_solver(non_lin_cons_parameters)
    # filename = "non_lin_cons_" * "_rho_" * string(non_lin_cons_parameters["ρ"])
    # filename *= "_iter_" * string(non_lin_cons_parameters["num_iter"]) * "_Qf_" * string(non_lin_cons_parameters["Qf"][1,1])
    filename = "non_lin_cons"
    save_results(X, U, Y, ν, cost_history, constraint_violation, optimality_criterion, filename, iter, non_lin_cons_parameters)
end
