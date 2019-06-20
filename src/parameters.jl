function define_parameters()
    N = 100
    n = 6
    m = 3
    tf = 50.0
    Δt = tf/N
    x0 = [2000.0, 1500.0, 1000.0, 0.0, 0.0, 0.0]
    xf = zeros(n)
    params = Dict("num_iter" => 1100, # number of iteration of ADMM
                  "N" => N, # horizon
                  "n"=> n, # state size
                  "m"=> m, # control size
                  "Δt" => Δt, # time step
                  "tf" => tf, # final time
                  "ρ" => 1e-6, # weighting parameter for augmented lagrangian
                  "α" => 100.0, # weighting parameter
                  "X0" => zeros(n, N),#reshape(repeat(x0, outer = [N, 1]), (n, N)), # initial state trajectory
                  "U0" => zeros(m, N-1), # initial control trajectory
                  "ν0" => ones(m, N-1), # initial dual
                  "Y0" => zeros(m, N-1), # initial dummy
                  "x0" => x0, # initial state
                  "xf" => xf, # final state
                  "using_constraints" => true, # using constraints on control
                  "u_min" => -10.0, # lower bound on u
                  "u_max" => 10.0, # upper bound on u
                  "Q" => 0.0 * Matrix{Float64}(I, n, n), # Quadratic stage cost for states (n,n)
                  "R" => 1e-20 * Matrix{Float64}(I, m, m), # Quadratic stage cost for controls (m,m)
                  "H" => zeros(m, n), # Quadratic Cross-coupling for state and controls (m,n)
                  "q" => zeros(n), # Linear term on states (n,)
                  "r" => zeros(m), # Linear term on controls (m,)
                  "c" => 0.0, # constant term
                  "Qf" => 10.0 * Matrix{Float64}(I, n, n), # Quadratic final cost for terminal state (n,n)
                  "qf" => zeros(n), # Linear term on terminal state (n,)
                  "cf" => 0.0, # constant term (terminal)
                  "μ" => 3.99*10^14, # Standard gravitational parameter m^3 s^-2
                  "a" => 6731.0*10^3, # semi major axis m
                   )
    return params
end
