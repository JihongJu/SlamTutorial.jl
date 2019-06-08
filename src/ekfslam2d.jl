using LinearAlgebra


include("src/data/slam_course_example.jl")
include("src/utils/visual2d.jl")
include("src/data/sensor2d.jl")


mutable struct Belief
    mu::Array{Float32,1}
    sigma::Array{Float32,2}
end


function initial_belief(num_landmarks)
    μ = zeros(Float32, 3 + 2*num_landmarks)
    Σ = zeros(Float32, 3+2*num_landmarks, 3+2*num_landmarks)
    Σ[diagind(Σ)[1:3]] .= 0
    Σ[diagind(Σ)[4:end]] .= Inf32

    return Belief(μ, UpperTriangular(Σ))
end


function prediction_step(belief, odometry)
    # Compute the new mu based on the noise-free (odometry-based) motion model
    x, y, θ = belief.mu[1:3]
    belief.mu[1:3] = sensor2d.odometry_model([x, y, θ], odometry)

    # Compute the 3x3 Jacobian Gx of the motion model
    Gx = Matrix{Float32}(I, 3, 3)
    heading = θ + odometry.rot1
    Gx[1, 3] -= odometry.trans * sin(heading)  # ∂x'/∂θ
    Gx[2, 3] += odometry.trans * cos(heading)  # ∂y'/∂θ


    # Motion noise
    Rx = [
        0.1 0 0;
        0 0.1 0;
        0 0 0.01
    ]

    # Compute the predicted sigma after incorporating the motion
    Σxx = belief.sigma[1:3, 1:3]
    Σxm = belief.sigma[1:3, 4:end]

    belief.sigma[1:3, 1:3] = Gx * Σxx * Gx' + Rx
    belief.sigma[1:3, 4:end] = Gx * Σxm

    return belief
end

function correction_step(belief, observations, observed_landmarks)
    return belief, observed_landmarks
end


landmarks = slam_course_example.load_world()
num_landmarks = size(landmarks, 1)

odometries, sensor_readings = slam_course_example.load_sensor_data()

belief = initial_belief(num_landmarks)
observed_landmarks = zeros(Bool, num_landmarks)


for t in 1:20
    belief = prediction_step(belief, odometries[t])
    # belief, observed_landmarks = correction_step(belief, sensor_readings[t], observed_landmarks)
    canvas = visual2d.make_canvas(-2, -2, 12, 12)
    canvas.cla()
    visual2d.draw_state(canvas, t, belief, observed_landmarks, sensor_readings[t], landmarks)
end
