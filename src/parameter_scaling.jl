function scale_lin_parameters(parameters)
    # Rescales the set of parameters for the linear dynamics experiments.
    x0 = parameters["x0"]
    xf = parameters["xf"]
    tf = parameters["tf"]
    Δt = parameters["Δt"]
    m_ego = parameters["m_ego"]

    # Scale
    l_ref = max(norm(x0[1:3] - xf[1:3]), 1)
    v_ref = max(norm(x0[4:6] - xf[4:6]), 1)
    t_ref = l_ref / v_ref
    a_ref = l_ref / (t_ref^2)
    u_ref = a_ref * m_ego
    # Scaling
    tf /= t_ref
    Δt /= t_ref
    x0[1:3] /= l_ref
    x0[4:6] /= v_ref
    xf[1:3] /= l_ref
    xf[4:6] /= v_ref

    parameters["Δt"] = Δt
    parameters["tf"] = tf
    parameters["x0"] = x0
    parameters["xf"] = xf
    parameters["t_ref"] = t_ref
    parameters["l_ref"] = l_ref
    parameters["v_ref"] = v_ref
    parameters["a_ref"] = a_ref
    parameters["u_ref"] = u_ref
    parameters["u_min"] /= u_ref
    parameters["u_max"] /= u_ref
    return parameters
end

function scale_non_lin_parameters(parameters)
    # Rescales the set of parameters for the nonlinear dynamics experiments.
    μ = parameters["μ"]
    x0 = parameters["x0"]
    xf = parameters["xf"]
    tf = parameters["tf"]
    Δt = parameters["Δt"]
    m_ego = parameters["m_ego"]

    # Scale
    l_ego_ref = max(norm(x0[1:3] - xf[1:3]), 1)
    l_target_ref =  norm(x0[4:6])
    # v_ego_ref = max(norm(x0[7:9] - xf[7:9]), 1)
    v_target_ref = 3*sqrt(μ/l_target_ref)
    t_ref_target = l_target_ref / v_target_ref
    t_ref = t_ref_target
    v_ego_ref = l_ego_ref / t_ref
    a_ego_ref = l_ego_ref / (t_ref^2)
    a_target_ref = l_target_ref / (t_ref^2)
    u_ref = a_ego_ref * m_ego

    # Scaling
    tf /= t_ref
    Δt /= t_ref
    x0[1:3] /= l_ego_ref
    x0[4:6] /= l_target_ref
    x0[7:9] /= v_ego_ref
    x0[10:12] /= v_target_ref
    xf[1:3] /= l_ego_ref
    xf[4:6] /= l_target_ref
    xf[7:9] /= v_ego_ref
    xf[10:12] /= v_target_ref

    parameters["Δt"] = Δt
    parameters["tf"] = tf
    parameters["x0"] = x0
    parameters["xf"] = xf
    parameters["t_ref"] = t_ref
    parameters["l_ego_ref"] = l_ego_ref
    parameters["l_target_ref"] = l_target_ref
    parameters["v_ego_ref"] = v_ego_ref
    parameters["v_target_ref"] = v_target_ref
    parameters["a_ego_ref"] = a_ego_ref
    parameters["a_target_ref"] = a_target_ref
    parameters["u_ref"] = u_ref
    parameters["u_min"] /= u_ref
    parameters["u_max"] /= u_ref
    return parameters
end
