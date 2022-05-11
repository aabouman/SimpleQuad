using DelimitedFiles
using Statistics
using Printf

if isempty(ARGS)
    basename = "serial_latency"
else
    basename = ARGS[1]
end
outdata = readdlm(joinpath(@__DIR__, "$(basename)_test_out.txt"), ',')
indata = readdlm(joinpath(@__DIR__, "$(basename)_test_in.txt"), ',')

idx = map(eachrow(indata)) do (t,x) 
    idx = searchsortedfirst(outdata[:,2], x)
end
outdata[idx,2] ≈ indata[:,2]

tout = outdata[idx,1]
tin = indata[:,1]
all(tout .< tin)
latency = tin - tout

@printf("Latency = %0.3g ± %0.4f ms\n", mean(latency), std(latency))