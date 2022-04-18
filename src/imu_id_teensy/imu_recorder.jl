using LibSerialPort
using DelimitedFiles

# %% Record IMU data
function record_data()
    sp = LibSerialPort.open("/dev/cu.usbmodem92225501", 115200)
    close(sp)
    dataBuffer = zeros(UInt8, 4 * 6 * 30000)

    start_time = time()
    open(sp)
    read!(sp, dataBuffer)
    close(sp)
    total_time = time() - start_time

    # Reinterpret
    tmp = split(String(dataBuffer), "End")
    num_measurements = length(tmp)
    float_data = map(_x->reinterpret(Float32, Vector{UInt8}(_x)), tmp[2:end-1])
    times = Vector(range(0, total_time; length=num_measurements))[2:end-1]

    # Write IMU data
    open(joinpath(@__DIR__, "delim_file.csv"), "w") do io
        writedlm(io, [["Time", "Acc_X", "Acc_Y", "Acc_Z", "GYR_X", "GYR_Y", "GYR_Z"]], ",\t")

        for i in 1:length(times)
            writedlm(io, [[times[i]; float_data[i]]], ",\t")
        end
    end

    return append!.(times, float_data)
end

logrange(x1, x2, n; base=10.0) = (base^y for y in range(x1, x2, length=n))

function computeAllanDev(freq::Float64, time_series::AbstractVector)
    dt = 1/freq
    N = length(time_series)
    M_max = 2^floor(log2(N / 2))
    M = logrange(log10(1), log10(M_max), 100)
    M = ceil.(M)
    M = unique(M)
    taus = M * dt
    allan_variance = zeros(length(M))

    for (i, mi) in enumerate(M)
        mi = Int(mi)
        twoMi = 2 * mi

        allan_variance[i] = sum((time_series[twoMi:end] - (2.0 * time_series[mi:(end-mi)]) + time_series[1:(end-(twoMi-1))]).^2)
    end

    allan_variance .= allan_variance ./ ((2.0 * taus.^2) .* (N .- 2.0 * M))
    return (taus, sqrt.(allan_variance))  # Return deviation (dev = sqrt(var))
end

# %%
imu_data = readdlm(joinpath(@__DIR__, "delim_file.csv"), ',', Float64; skipstart=1)

times = imu_data[:, 1]

acc_x = imu_data[:, 2]
acc_y = imu_data[:, 3]
acc_z = imu_data[:, 4]

gyr_x = imu_data[:, 5]
gyr_y = imu_data[:, 6]
gyr_z = imu_data[:, 7]

# %%
using Plots

taus, all_var_gyr_x = computeAllanDev(177.0, gyr_x)
plt = plot(taus, all_var_gyr_x, xscale=:log10, yscale=:log10,
           label="Gyr x", title="Allan Std Deviation of Gyro",
           xlabel="Observation Period (sec)", ylabel="Std Dev (rad / sec)"
           )
taus, all_var_gyr_y = computeAllanDev(177.0, gyr_y)
plot!(plt, taus, all_var_gyr_y, xscale=:log10, yscale=:log10, label="Gyr y")
taus, all_var_gyr_z = computeAllanDev(177.0, gyr_z)
plot!(plt, taus, all_var_gyr_z, xscale=:log10, yscale=:log10, label="Gyr z")
savefig(plt, joinpath(@__DIR__, "Allan_std_dev_gyro.png"))

# %%
taus, all_var_acc_x = computeAllanDev(177.0, acc_x)
plt = plot(taus, all_var_acc_x, xscale=:log10, yscale=:log10,
           label="Acc x", title="Allan Std Deviation of Accelerometer",
           xlabel="Observation Period (sec)", ylabel="Std Dev (rad / sec)"
           )
taus, all_var_acc_y = computeAllanDev(177.0, acc_y)
plot!(plt, taus, all_var_acc_y, xscale=:log10, yscale=:log10, label="Acc y")
taus, all_var_acc_z = computeAllanDev(177.0, acc_z)
plot!(plt, taus, all_var_acc_z, xscale=:log10, yscale=:log10, label="Acc z")
savefig(plt, joinpath(@__DIR__, "Allan_std_dev_accel.png"))

# %%
tau1_ind = argmin(abs.(taus .- 1))
σ_bias = [
    all_var_acc_x[tau1_ind],
    all_var_acc_y[tau1_ind],
    all_var_acc_z[tau1_ind],
    all_var_gyr_x[tau1_ind],
    all_var_gyr_y[tau1_ind],
    all_var_gyr_z[tau1_ind]
]