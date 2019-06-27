function cost_function(X, U, parameters)
    N = parameters["N"]
    Qf = parameters["Qf"]
    Q = parameters["Q"]
    α = parameters["α"]
    cost = 0
    for i=1:N-1
        cost += 1/2 * X[:,i]'*Q*X[:,i] + α * norm(U[:,i], 1)
    end
    cost += 1/2 * X[:,N]'*Qf*X[:,N]
    return cost
end

function lqr_cost_function(X, U, Y, ν, parameters)
    N = parameters["N"]
    Qf = parameters["Qf"]
    Q = parameters["Q"]
    ρ = parameters["ρ"]
    cost = 0
    for i=1:N-1
        cost += 1/2 * X[:,i]'*Q*X[:,i] + ρ * U[:,i]'*U[:,i] + U[:,i]'*(ν[:,i] - ρ * Y[:,i])
    end
    cost += 1/2 * X[:,N]'*Qf*X[:,N]
    return cost
end

function augmented_lagrangian(X, U, Y, ν, parameters)
    N = parameters["N"]
    ρ = parameters["ρ"]
    lagrangian = cost_function(X, Y, parameters)
    for i=1:N-1
        lagrangian += ν[:,i]' * (U[:,i] - Y[:,i])
        lagrangian += ρ / 2 * norm(U[:,i] - Y[:,i], 2)^2
    end
    return lagrangian
end
