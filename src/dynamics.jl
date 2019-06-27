function cw_dynamics!(ẋ, x, u, parameters)
    n_ = sqrt(parameters["μ"]/parameters["a"]^3)
    ẋ[1:3] = x[4:6]
    ẋ[4] = 3*n_^2*x[1] + 2*n_*x[5] + u[1]
    ẋ[5] = -2*n_*x[4] + u[2]
    ẋ[6] = -n_^2*x[3] + u[3]
end

function cw_dynamics!(ẋ,x,u)
    cw_dynamics!(ẋ, x, u, lin_parameters)
end

function scaled_cw_dynamics!(ẋ, x, u, parameters)
    t_ref = parameters["t_ref"]
    l_ref = parameters["l_ref"]
    v_ref = parameters["v_ref"]
    a_ref = parameters["a_ref"]
    u_ref = parameters["u_ref"]
    n_ = sqrt(parameters["μ"]/parameters["radius"]^3)
    ẋ[1:3] = x[4:6]
    ẋ[4] = (3*n_^2*x[1]*l_ref + 2*n_*x[5]*v_ref + u[1]*u_ref) / a_ref
    ẋ[5] = (-2*n_*x[4]*v_ref + u[2]*u_ref) / a_ref
    ẋ[6] = (-n_^2*x[3]*l_ref + u[3]*u_ref) / a_ref
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
    F_ego = u * u_ref + gravitation_force(r_ego, "ego", parameters) + drag_force(r_ego, rd_ego, "ego", parameters) # unscaled
    F_target = gravitation_force(r_target, "target", parameters) + drag_force(r_target, rd_target, "target", parameters) # unscaled
    ẋ[10:12] = F_target / parameters["m_target"] / a_target_ref # scaled
    ẋ[7:9] = (ẋ[10:12] - F_ego / parameters["m_ego"]) / a_ego_ref # scaled
    # println("r_ego", r_ego)
    # println("rd_ego", rd_ego)
    # println("r_target", r_target)
    # println("rd_target", rd_target)
    # println("gravitation_force(r_ego, ego, parameters)", gravitation_force(r_ego, "ego", parameters))
    # println("gravitation_force(r_target, target, parameters)", gravitation_force(r_target, "target", parameters))
    # println("F_target / parameters[m_targe]", F_target / parameters["m_target"])
    # println("F_ego / parameters[m_ego]", F_ego / parameters["m_ego"])
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
    F_ego = u + gravitation_force(r_ego, "ego", parameters) + drag_force(r_ego, rd_ego, "ego", parameters)
    F_target = gravitation_force(r_target, "target", parameters) + drag_force(r_target, rd_target, "target", parameters)
    ẋ[10:12] = F_target / parameters["m_target"]
    ẋ[7:9] = ẋ[10:12] - F_ego / parameters["m_ego"]
    # println("r_ego", r_ego)
    # println("rd_ego", rd_ego)
    # println("r_target", r_target)
    # println("rd_target", rd_target)
    # println("gravitation_force(r_ego, ego, parameters)", gravitation_force(r_ego, "ego", parameters))
    # println("gravitation_force(r_target, target, parameters)", gravitation_force(r_target, "target", parameters))
    # println("F_target / parameters[m_targe]", F_target / parameters["m_target"])
    # println("F_ego / parameters[m_ego]", F_ego / parameters["m_ego"])
end

function non_linear_dynamics!(ẋ,x,u)
    non_linear_dynamics!(ẋ, x, u, non_lin_parameters)
end

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
    F = F_0 + F_J2
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
    return density
end

# function non_linear_dynamics!(ẋ, x, u, parameters::Dict{String,Any})
#     # This function gives the dynamics of a chipsat including the J2
#     # gravitational term and atmospheric drag.
#
#     # x = [position; velocity] is the spacecraft state vector in MKS units
#     # u is a scalar control input that is equal to the cosine of the angle
#     # between the spacecraft normal vector and the velocity vector. It should
#     # be between 0 and 1, and modulates the drag on the spacecraft.
#
#     # Constants
#     μ = parameters["μ"]
#     a = parameters["a"]
#     J2 = parameters["J2"]
#     A = parameters["A"]
#     ω = parameters["ω"]
#     Cd = parameters["Cd"]
#     x_ref = parameters["x_ref"]
#
#     r = x[1:3] + x_ref[1:3] # position in meters
#     v = x[4:6] # velocity in m/s
#     rmag = norm(r)
#     vmag = norm(v)
#
#     alt = rmag - a
#     density = atmospheric_density(alt)
#
#     a_spherical = -r * (μ/rmag^3)
#     J2_term = [(1-5*(r[3]/rmag)^2)*r[1], (1-5*(r[3]/rmag)^2)*r[2], (3-5*(r[3]/rmag)^2)*r[3]]
#     a_J2 = -(3/2) * J2 * (μ/rmag^5) *a^2 * J2_term
#     a_grav = a_spherical + a_J2;
#
#     v_rel = v + cross(ω, r)
#     a_drag = -0.5 * density * A * Cd * norm(v_rel) * v_rel
#     # a = a_grav + a_drag + u; # full drag (max drag: satellite facing air flow)
#     # a = a_grav + u*a_drag;
#     ẋ[1:3] = x[4:6]
#     ẋ[4:6] = a_grav + a_drag + u
#     return ẋ
# end



# function atmospheric_drag(x, parameters)
#     # Constants
#     a = parameters["a"]
#     A = parameters["A"]
#     mass = parameters["mass"]
#     ω = parameters["ω"]
#     Cd = parameters["Cd"]
#     # area = (.6)*.1*.1 + (1.2)*.35*.1; %Average surface area assuming tumbling
#     # Calculate Altitude
#     r = x[1:3]
#     v = x[4:6]
#     alt = norm(r) - a
#     v_rel = v + cross(ω, r)
#     # Calculate Atmospheric Density
#     density = atmospheric_density(alt)
#     # Calculate Acceleration
#     a = -0.5 * (A/mass) * density * Cd * norm(v_rel) * v_rel
#     # Return state derivative
#     x_dot = vcat(v, a)
#     return x_dot
# end
