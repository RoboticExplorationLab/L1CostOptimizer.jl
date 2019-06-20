function cw_dynamics!(ẋ, x, u, params)
   n_cst = sqrt(params["μ"]/params["a"]^3)
   ẋ[1:3] = x[4:6]
   ẋ[4] = 3*n_cst^2*x[1] + 2*n_cst*x[5] + u[1]
   ẋ[5] = -2*n_cst*x[4] + u[2]
   ẋ[6] = -n_cst^2*x[3] + u[3]
end

function cw_dynamics!(ẋ,x,u)
   cw_dynamics!(ẋ, x, u, params)
end

function non_linear_dynamics!(ẋ, x, u, params)
   n_cst = sqrt(params["μ"]/params["a"]^3)
   ẋ[1:3] = x[4:6]
   ẋ[4] = 3*n_cst^2*x[1] + 2*n_cst*x[5] + u[1]
   ẋ[5] = -2*n_cst*x[4] + u[2]
   ẋ[6] = -n_cst^2*x[3] + u[3]
end

function non_linear_dynamics!(ẋ,x,u)
   non_linear_dynamics!(ẋ, x, u, params)
end
