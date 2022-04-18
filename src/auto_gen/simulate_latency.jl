using DataStructures
using StaticArrays
using Rotations
using Plots
using LaTeXStrings


include("quad_dynamics.jl")

# %% Examining Stability of Quad with injected delay on all elements of the state
dt = 0.001
Nt = 20000
K, _ = generate_LQR_hover_gains(HOVER_STATE, HOVER_INPUT, dt;
                                Qd=([10,10,10, 1,1,1, 1,1,1, 1,1,1]))
# Vector of latencies we will simulate with
latencies = [0.001, 0.003, 0.01, 0.03, 0.1]
times = 0:dt:((Nt-1)*dt)
p0 = [+0.0266,  +0.1500, +0.8081]
q0 = Rotations.params(Rotations.add_error(UnitQuaternion(1.0,0,0,0), Rotations.RotationError((@SVector randn(3)) * 0.3, MRPMap())))
v0 = [0,0,0]
ω0 = [0,0,0]

# %%

xhist = zeros(length(latencies), Nt, length(HOVER_STATE))
# Initialize from starting point of the quadrotor
for i in 1:length(latencies)
    xhist[i,1,:] .= [p0; q0; v0; ω0]
end

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
            u = clamp.(HOVER_INPUT, quad_min_throttle, quad_max_throttle)
        end

        xhist[i,k+1,:] .= dynamics_rk4(xhist[i,k,:], u, dt)
        pushfirst!(delay_xhist, xhist[i,k+1,:])
    end
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
