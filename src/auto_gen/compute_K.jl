using DataStructures
using Plots
using LaTeXStrings

include("quad_dynamics.jl")

<<<<<<< HEAD
# %% Examining Stability of Quad with injected delay on all elements of the state
dt = 0.001
Nt = 20000
K, _ = generate_LQR_hover_gains(HOVER_STATE, HOVER_INPUT, dt;
                                Qd=([10,10,10, 1,1,1, 1,1,1, 1,1,1]))
# Vector of latencies we will simulate with
latencies = [0.001, 0.003, 0.01, 0.03, 0.1]
times = 0:dt:((Nt-1)*dt)
xhist = zeros(length(latencies), Nt, length(HOVER_STATE))
# Initialize from starting point of the quadrotor
for i in 1:length(latencies)
    xhist[i,1,:] .= [+0.0266,  +0.1500, +0.8081,
                     -0.9983,  +0.0106, +0.0028, -0.0565,
                     +0.0468,  +0.0188, -0.0305,
                     -0.0019,  +0.0113, -0.0210]
=======
const r0 = [0.262, -0.555, 1.30]
const q0 = Rotations.params(UnitQuaternion(1., 0, 0, 0))
const v0 = [0.0; 0.0; 0.0]
const ω0 = [0.0; 0.0; 0.0]

# %%
function trim_controls()
    kf, bf = quad_motor_kf, quad_motor_bf
    g = 9.81
    m = quad_mass

    thrust = (g * m) / 4.0
    pwm = (thrust - bf) / kf

    return [pwm for _ in 1:4]
>>>>>>> refs/remotes/origin/master
end
const HOVER_STATE = [r0; q0; v0; ω0]
const HOVER_INPUT = trim_controls()

# Run a simulation for each latency duration
for (i, latency) in enumerate(latencies)
    u = HOVER_INPUT
    # Circular buffer for delay backlog of size latency duration/step length
    delay_xhist = CircularBuffer{Vector}(convert(Int, floor(latency / dt)))
    pushfirst!(delay_xhist, xhist[i,1,:])

    # Simulate for
    for k = 1:(Nt-1)
        if isfull(delay_xhist) # We can start up the delayed input
            delayed_xhist = pop!(delay_xhist)
            u = clamp.(HOVER_INPUT - K * state_error(delayed_xhist, HOVER_STATE),
                       quad_min_throttle, quad_max_throttle)
        else # No input due to inital delay
            u = fill(quad_min_throttle, 4)
            # u =
        end

        xhist[i,k+1,:] .= dynamics_rk4(xhist[i,k,:], u, dt)
        pushfirst!(delay_xhist, xhist[i,k+1,:])
    end
<<<<<<< HEAD
end

# %%
plots = [plot() for i in 1:length(latencies)]
for (i, plt) in enumerate(plots)
    plot!(plt, times, xhist[i,:,1] .- HOVER_STATE[1], label=L"p_x", title="Latency $(latencies[i])")
    plot!(plt, times, xhist[i,:,2] .- HOVER_STATE[2], label=L"p_y")
    plot!(plt, times, xhist[i,:,3] .- HOVER_STATE[3], label=L"p_z")
end
plot(plots..., layout=(5, 1), size=(600, 800))

# %%
plots = [plot() for i in 1:length(latencies)]
for (i, plt) in enumerate(plots)
    plot!(plt, times, xhist[i,:,4] .- HOVER_STATE[4], label=L"q_w", title="Latency $(latencies[i])")
    plot!(plt, times, xhist[i,:,5] .- HOVER_STATE[5], label=L"q_x")
    plot!(plt, times, xhist[i,:,6] .- HOVER_STATE[6], label=L"q_y")
    plot!(plt, times, xhist[i,:,7] .- HOVER_STATE[7], label=L"q_z")
end
plot(plots..., layout=(5, 1), size=(600, 800))

# %%
print_gains(K)

# %%
=======

    return torque
end

function cont_dynamics(x, u)
    p = x[1:3]
    q = Rotations.UnitQuaternion(x[4:7])
    v = x[8:10]
    ω = x[11:13]
    m = quad_mass
    J = quad_inertia

    dp = q * v
    dq = Rotations.kinematics(q, ω)
    dv = 1/m * q' * forces(x,u) - cross(ω, v)
    dω = J \ (moments(x,u) - cross(ω, J * ω))

    return [dp; dq; dv; dω]
end

function dynamics(x, u, dt)
    k1 = cont_dynamics(x, u)
    k2 = cont_dynamics(x + 0.5 * dt * k1, u)
    k3 = cont_dynamics(x + 0.5 * dt * k2, u)
    k4 = cont_dynamics(x + dt * k3, u)
    tmp = x + (dt/6.0) * (k1 + 2*k2 + 2*k3 + k4)

    tmp[4:7] .= Rotations.params(Rotations.UnitQuaternion(tmp[4:7]))

    return tmp
end

function state_error(x2, x1)
    p1 = x1[1:3]
    p2 = x2[1:3]
    q1 = Rotations.UnitQuaternion(x1[4:7])
    q2 = Rotations.UnitQuaternion(x2[4:7])
    all1 = x1[8:end]
    all2 = x2[8:end]

    ori_er = Rotations.rotation_error(q2, q1, Rotations.CayleyMap())

    dx =[p2-p1; ori_er; all2-all1]
    return dx
end

function error_state_jacobian(x)
    # Get various compoents
    q = Rotations.UnitQuaternion(x[4:7])
    # Build error state to state jacobian
    J = diagm(13, 12, [1 for _ in 1:12])
    J[4:7, 4:6] .= Rotations.∇differential(q)
    return J
end

function generate_LQR_hover_gains(
    Qd = ones(12),
    Rd = fill(0.1, 4);
    save_to_file::Bool = true,
)
    dt = 0.01 # time step (s)

    # Setting x₀ hover state
    xhover = copy(HOVER_STATE)
    uhover = trim_controls()

    A = ForwardDiff.jacobian(_x->dynamics(_x, uhover, dt), xhover)
    B = ForwardDiff.jacobian(_u->dynamics(xhover, _u, dt), uhover)

    E2 = error_state_jacobian(xhover)
    Ã = E2' * A * E2
    B̃ = E2' * B

    # Cost weights
    Q = Array(Diagonal(Qd))
    R = Array(Diagonal(Rd))

    # LQR Gain
    K = dlqr(Ã, B̃, Q, R)

    if (save_to_file)
        # Write gain to file
        file = open("$(@__DIR__)/gains.json", "w")
        JSON.print(file, K)
        close(file)

        file = open("$(@__DIR__)/hover_inputs.json", "w")
        JSON.print(file, uhover)
        close(file)
    end
    return K, uhover
end

# %%
K, _ = generate_LQR_hover_gains()

K * []
>>>>>>> refs/remotes/origin/master
