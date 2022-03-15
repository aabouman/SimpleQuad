using Rotations
using ForwardDiff
using LinearAlgebra
using ControlSystems
using JSON

# %%
const quad_mass = 1.776
const quad_inertia = [0.01566089 0.00000318037 0; 0.00000318037 0.01562078 0; 0 0 0.02226868]
const quad_motor_kf = 0.0244101
const quad_motor_bf = -30.48576
const quad_motor_km = 0.00029958
const quad_motor_bm = -0.367697
const quad_arm_len = 0.28

const r0 = [0.262, -0.555, 1.30]
const q0 = Rotations.params(UnitQuaternion(1., 0, 0, 0))
const v0 = [0.0; 0.0; 0.0]
const ω0 = [0.0; 0.0; 0.0]
const HOVER_STATE = [r0; q0; v0; ω0]
const HOVER_INPUT = trim_controls()

# %%
function trim_controls()
    kf, bf = quad_motor_kf, quad_motor_bf
    g = 9.81
    m = quad_mass

    thrust = (g * m) / 4.0
    pwm = (thrust - bf) / kf

    return [pwm for _ in 1:4]
end

"""
* `x` - Quadrotor state
* `u` - Motor PWM commands
"""
function forces(x, u)
    q = Rotations.UnitQuaternion(x[4:7])
    g = 9.81
    kf = quad_motor_kf
    bf = quad_motor_bf

    Kf = [0.0 0 0 0; 0 0 0 0; kf kf kf kf];
    Bf = [0; 0; bf];
    g = [0; 0; -quad_mass * g]

    force = Kf * u + Bf + q * g
    return force
end

function moments(x, u)
    km = quad_motor_km
    bm = quad_motor_bm
    kf = quad_motor_kf
    bf = quad_motor_bf

    Km = [0.0 0 0 0; 0 0 0 0; km -km km -km];
    Bm = [0; 0; bm];

    torque = Km * u + Bm

    ss = normalize.([[1.;1;0], [1.;-1;0], [-1.;-1;0], [-1.;1;0]])
    for (si, ui) in zip(ss, u)
        torque += cross(si, [0; 0; kf * ui + bf])
    end

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
    dt = 0.02 # time step (s)

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
generate_LQR_hover_gains()