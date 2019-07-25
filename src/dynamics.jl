function cw_dynamics!(ẋ, x, u, parameters)
    m_ego = parameters["m_ego"]
    n_ = sqrt(parameters["μ"]/parameters["orbit_radius"]^3)
    ẋ[1:3] = x[4:6]
    # ẋ[4] = 3*n_^2*x[1] + 2*n_*x[5] + u[1]/m_ego
    # ẋ[5] = -2*n_*x[4] + u[2]/m_ego
    # ẋ[6] = -n_^2*x[3] + u[3]/m_ego
    ẋ[4] =  2*n_*x[5] + u[1]/m_ego
    ẋ[5] = -2*n_*x[4] + 3*n_^2*x[2] + u[2]/m_ego
    ẋ[6] = -n_^2*x[3] + u[3]/m_ego
end

function cw_dynamics!(ẋ,x,u)
    cw_dynamics!(ẋ, x, u, lin_parameters)
end

function scaled_cw_dynamics!(ẋ, x, u, parameters)
    t_ref = parameters["t_ref"]
    l_ref = parameters["l_ref"]
    n_ = sqrt(parameters["μ"]/parameters["orbit_radius"]^3) * t_ref
    ẋ[1:3] = x[4:6]
    # ẋ[4] = 3*n_^2*x[1] + 2*n_*x[5] + u[1]
    # ẋ[5] = -2*n_*x[4] + u[2]
    # ẋ[6] = -n_^2*x[3] + u[3]
    ẋ[4] = 2*n_*x[5] + u[1]
    ẋ[5] = -2*n_*x[4] + 3*n_^2*x[2]+ u[2]
    ẋ[6] = -n_^2*x[3] + u[3]
end

function scaled_cw_dynamics!(ẋ,x,u)
    scaled_cw_dynamics!(ẋ, x, u, lin_parameters)
end

function scaled_non_linear_dynamics!(ẋ, x, u, parameters)
    # Definition of the state
    # x = [r_target-r_ego, r_target, rd_target-rd_ego, rd_target] ∈ R^12
    # r_ego = position vector of the ego satellite in the inertial frame.
    # r_ego = \vec{O_0 O_ego}
    # rd_ego = velocity vector of the ego satellite in the inertial frame.
    # rd_ego = V(O_ego, S_ego/0)
    # r_target = position vector of the target satellite in the inertial frame.
    # r_target = \vec{O_0 O_target}
    # rd_target = velocity vector of the target satellite in the inertial frame.
    # rd_target = V(O_target, S_target/0)

    # Definition of the control
    # u = [Fcx, Fcy, Fcz] \in R^3
    # Fc is the force applied by the control system of the ego satellite expressed in the inertial frame.

    # Defintion of the state derivative
    # ẋ = [rd_target-rd_ego, rd_target, rdd_target-rdd_ego, rdd_target] ∈ R^12
    # rdd_ego = acceleration vector of the ego satellite in the inertial frame.
    # rdd_ego = Γ(O_ego, S_ego/0) = F(->ego) / m_ego
    # rdd_target = acceleration vector of the target satellite in the inertial frame.
    # rdd_target = Γ(O_target, S_target/0) = F(->target) / m_target
    t_ref = parameters["t_ref"]
    l_ego_ref = parameters["l_ego_ref"]
    l_target_ref = parameters["l_target_ref"]
    v_ego_ref = parameters["v_ego_ref"]
    v_target_ref = parameters["v_target_ref"]
    a_ego_ref = parameters["a_ego_ref"]
    a_target_ref = parameters["a_target_ref"]
    u_ref = parameters["u_ref"]

    ẋ[1:6] = x[7:12] # scaled
    r_ego = x[4:6] * l_target_ref - x[1:3] * l_ego_ref # unscaled
    rd_ego = x[10:12] * v_target_ref - x[7:9] * v_ego_ref # unscaled
    r_target = x[4:6] * l_target_ref
    rd_target = x[10:12] * v_target_ref
    # The control that we give is expressed in the CW frame (without the translation).
    # We need to convert it to the world frame before applying it to the dynamics.
    # here x_cw is the vector X of the CW frame expressed in the world frame.
    y_cw = r_target / norm(r_target)
    z_cw = cross(y_cw, rd_target)
    z_cw = z_cw / norm(z_cw)
    x_cw = cross(y_cw, z_cw)
    u_world = [x_cw y_cw z_cw]*u
    F_ego = u_world * u_ref + gravitation_force(r_ego, "ego", parameters) ###+ drag_force(r_ego, rd_ego, "ego", parameters) # unscaled
    F_target = gravitation_force(r_target, "target", parameters) ###+ drag_force(r_target, rd_target, "target", parameters) # unscaled
    ẋ[10:12] = F_target / parameters["m_target"] / a_target_ref # scaled
    ẋ[7:9] = (F_target / parameters["m_target"] - F_ego / parameters["m_ego"]) / a_ego_ref # scaled
end

function scaled_non_linear_dynamics!(ẋ,x,u)
   scaled_non_linear_dynamics!(ẋ, x, u, non_lin_parameters)
end

function non_linear_dynamics!(ẋ, x, u, parameters)
    # Definition of the state
    # x = [r_target-r_ego, r_target, rd_target-rd_ego, rd_target] ∈ R^12
    # r_ego = position vector of the ego satellite in the inertial frame.
    # r_ego = \vec{O_0 O_ego}
    # rd_ego = velocity vector of the ego satellite in the inertial frame.
    # rd_ego = V(O_ego, S_ego/0)
    # r_target = position vector of the target satellite in the inertial frame.
    # r_target = \vec{O_0 O_target}
    # rd_target = velocity vector of the target satellite in the inertial frame.
    # rd_target = V(O_target, S_target/0)

    # Definition of the control
    # u = [Fcx, Fcy, Fcz] \in R^3
    # Fc is the force applied by the control system of the ego satellite expressed in the inertial frame.

    # Defintion of the state derivative
    # ẋ = [rd_target-rd_ego, rd_target, rdd_target-rdd_ego, rdd_target] ∈ R^12
    # rdd_ego = acceleration vector of the ego satellite in the inertial frame.
    # rdd_ego = Γ(O_ego, S_ego/0) = F(->ego) / m_ego
    # rdd_target = acceleration vector of the target satellite in the inertial frame.
    # rdd_target = Γ(O_target, S_target/0) = F(->target) / m_target

    ẋ[1:6] = x[7:12]
    r_ego = x[4:6] - x[1:3]
    rd_ego = x[10:12] - x[7:9]
    r_target = x[4:6]
    rd_target = x[10:12]
    # The control that we give is expressed in the CW frame (without the translation).
    # We need to convert it to the world frame before applying it to the dynamics.
    # here x_cw is the vector X of the CW frame expressed in the world frame.
    y_cw = r_target / norm(r_target)
    z_cw = cross(y_cw, rd_target)
    z_cw = z_cw / norm(z_cw)
    x_cw = cross(y_cw, z_cw)
    u_world = [x_cw y_cw z_cw]*u
    F_ego = u_world + gravitation_force(r_ego, "ego", parameters) ###+ drag_force(r_ego, rd_ego, "ego", parameters)
    F_target = gravitation_force(r_target, "target", parameters) ###+ drag_force(r_target, rd_target, "target", parameters)
    ẋ[10:12] = F_target / parameters["m_target"]
    ẋ[7:9] = ẋ[10:12] - F_ego / parameters["m_ego"]
end

function non_linear_dynamics!(ẋ,x,u)
    non_linear_dynamics!(ẋ, x, u, non_lin_parameters)
end

# function circular_dynamics!(ẋ, x, u, parameters)
#     # Definition of the state
#     # x = [r_target-r_ego, r_target, rd_target-rd_ego, rd_target] ∈ R^12
#     # r_ego = position vector of the ego satellite in the inertial frame.
#     # r_ego = \vec{O_0 O_ego}
#     # rd_ego = velocity vector of the ego satellite in the inertial frame.
#     # rd_ego = V(O_ego, S_ego/0)
#     # r_target = position vector of the target satellite in the inertial frame.
#     # r_target = \vec{O_0 O_target}
#     # rd_target = velocity vector of the target satellite in the inertial frame.
#     # rd_target = V(O_target, S_target/0)
#
#     # Definition of the control
#     # u = [Fcx, Fcy, Fcz] \in R^3
#     # Fc is the force applied by the control system of the ego satellite expressed in the inertial frame.
#
#     # Defintion of the state derivative
#     # ẋ = [rd_target-rd_ego, rd_target, rdd_target-rdd_ego, rdd_target] ∈ R^12
#     # rdd_ego = acceleration vector of the ego satellite in the inertial frame.
#     # rdd_ego = Γ(O_ego, S_ego/0) = F(->ego) / m_ego
#     # rdd_target = acceleration vector of the target satellite in the inertial frame.
#     # rdd_target = Γ(O_target, S_target/0) = F(->target) / m_target
#     ẋ[1:6] = x[7:12]
#     r_ego = x[4:6] - x[1:3]
#     rd_ego = x[10:12] - x[7:9]
#     r_target = x[4:6]
#     rd_target = x[10:12]
#     F_ego = u + gravitation_force(r_ego, "ego", parameters)  ### we shouldn't use J2
#     F_target = gravitation_force(r_target, "target", parameters) ### we shouldn't use J2
#     ẋ[10:12] = F_target / parameters["m_target"]
#     ẋ[7:9] = ẋ[10:12] - F_ego / parameters["m_ego"]
# end
#
# function circular_dynamics!(ẋ,x,u)
#     circular_dynamics!(ẋ, x, u, non_lin_parameters)
# end

function gravitation_force(r, id, parameters)
    μ = parameters["μ"]
    J2 = parameters["J2"]
    if id == "ego"
        mass = parameters["m_ego"]
    elseif id == "target"
        mass = parameters["m_target"]
    end
    rmag = norm(r)
    F_0 = - μ * mass / rmag^3 * r
    J2_term = [r[1]*(6*r[3]^2 - 3/2(r[1]^2 + r[2]^2)), r[2]*(6*r[3]^2 - 3/2(r[1]^2 + r[2]^2)), r[3]*(3*r[3]^2 - 9/2(r[1]^2 + r[2]^2))]
    F_J2 = J2 / rmag^7 * J2_term
    F = F_0 ###+ F_J2
    return F
end

function drag_force(r, v, id, parameters)
    if id == "ego"
        A = parameters["A_ego"]
        Cd = parameters["Cd_ego"]
    elseif id == "target"
        A = parameters["A_target"]
        Cd = parameters["Cd_target"]
    end
    ### a needs to be the earth radius
    alt = norm(r) - parameters["a"]
    density = atmospheric_density(alt)
    v_rel = v + cross(parameters["ω"], r)
    F = - 0.5 * Cd * A * density * norm(v_rel) * v_rel
    return F
end

function atmospheric_density(alt)
    # Data is from SMAD tables for density vs. altitude
    # Density is kg/m^3

    #  h = [100 150 200 250 300 350 400 450 500]'; %Altitude in km.
    #  rho_min = [4.61e-7 1.65e-9 1.78e-10 3.35e-11 8.19e-12 2.34e-12 7.32e-13 2.47e-13 8.98e-14]';
    #  rho_avg = [4.79e-7 1.81e-9 2.53e-10 6.24e-11 1.95e-11 6.98e-12 2.72e-12 1.13e-12 4.89e-13]';
    #  rho_max = [5.10e-7 2.04e-9 3.52e-10 1.06e-10 3.96e-11 1.68e-11 7.55e-12 3.61e-12 1.80e-12]';

    # Fits to SMAD data (for h in meters)
    density = 9.201e-05*exp(-5.301e-05*alt) # min
    # rho = 5.987e-05*exp(-4.836e-05*h); # avg
    # rho = 4.393e-05*exp(-4.467e-05*h); # max
    # if alt == 5e5
    #     density = 7.3e-13
    # end
    return density
end
