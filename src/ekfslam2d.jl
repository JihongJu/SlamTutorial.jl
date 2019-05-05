module ekfslam2d
export Belief, initial_belief

include("data/sensor2d.jl")
using .sensor2d


mutable struct Belief
    mu::Array{Float32,1}
    sigma::Array{Float32,2}
end


function initial_belief(num_landmarks)
    num_landmarks = 9
    μ = zeros(Float32, 3 + 2*num_landmarks)
    Σ = Inf * eye(Float32, 3 + 2*num_landmarks)
    Σ[1:3,1:3]=0

    belief=Belief(μ, Σ)
    return belief

end


# landmarks = load_world()
# odometry, sensor = load_sensor_data()
#

end
