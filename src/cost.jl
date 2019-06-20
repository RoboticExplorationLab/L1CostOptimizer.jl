function cost_function(X, U, params)
    N = params["N"]
    Qf = params["Qf"]
    Q = params["Q"]
    α = params["α"]
    cost = 0
    for i=1:N-1
        cost += 1/2 * X[:,i]'*Q*X[:,i] + α * norm(U[:,i], 1)
    end
    cost += 1/2 * X[:,N]'*Qf*X[:,N]
    return cost
end

function lqr_cost_function(X, U, Y, ν, params)
    N = params["N"]
    Qf = params["Qf"]
    Q = params["Q"]
    ρ = params["ρ"]
    cost = 0
    for i=1:N-1
        cost += 1/2 * X[:,i]'*Q*X[:,i] + ρ * U[:,i]'*U[:,i] + U[:,i]'*(ν[:,i] - ρ * Y[:,i])
    end
    cost += 1/2 * X[:,N]'*Qf*X[:,N]
    return cost
end

function augmented_lagrangian(X, U, Y, ν, params)
    N = params["N"]
    ρ = params["ρ"]
    lagrangian = cost_function(X, Y, params)
    for i=1:N-1
        lagrangian += ν[:,i]' * (U[:,i] - Y[:,i])
        lagrangian += ρ / 2 * norm(U[:,i] - Y[:,i], 2)^2
    end
    return lagrangian
end
