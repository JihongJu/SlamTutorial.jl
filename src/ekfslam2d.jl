using LinearAlgebra

mutable struct Belief
    mu::Array{Float32,1}
    sigma::Array{Float32,2}
end


function initial_belief(num_landmarks)
    μ = zeros(Float32, 3 + 2*num_landmarks)
    Σ = Matrix{Float32}(I, 3+2*num_landmarks, 3+2*num_landmarks)
    Σ[1:3,1:3] .= 0
    Σ[4:end, 4:end] .= Inf32

    belief=Belief(μ, Σ)
    return belief

end


function prediction_step(belief, odometry)
    return belief

end

function correction_step(belief, observations, observed_landmarks)
    return belief, observed_landmarks
end


include("src/data/slam_course_example.jl")
# import .slam_course_example

include("src/utils/visual2d.jl")
# import .visual2d


landmarks = slam_course_example.load_world()
num_landmarks = size(landmarks, 1)

odometries, sensor_readings = slam_course_example.load_sensor_data()

belief = initial_belief(3)
observed_landmarks = zeros(Bool, num_landmarks)

canvas = visual2d.make_canvas(-2, -2, 12, 12)

for t in 1:1
    belief = prediction_step(belief, odometries[t])
    belief, observed_landmarks = correction_step(belief, sensor_readings[t], observed_landmarks)
    visual2d.draw_state(canvas, t, belief, observed_landmarks, sensor_readings[t], landmarks)
end
