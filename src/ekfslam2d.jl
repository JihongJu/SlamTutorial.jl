using LinearAlgebra


include("src/data/slam_course_example.jl")
include("src/utils/visual2d.jl")
include("src/data/sensor2d.jl")


mutable struct Belief
    mean::Vector{Union{Float32, Missing}}
    covariance::UpperTriangular{Float32,Array{Float32,2}}
end


function initial_belief(num_landmarks)
    μ = Vector{Union{Float32, Missing}}(missing, 3 + 2*num_landmarks)
	μ[1:3] .= 0
    Σ = zeros(Float32, 3+2*num_landmarks, 3+2*num_landmarks)
    Σ[diagind(Σ)[1:3]] .= 0
    Σ[diagind(Σ)[4:end]] .= 0.1 # TODO back to Inf32

    return Belief(μ, UpperTriangular(Σ))
end


function prediction_step(belief, odometry)
    # Compute the new mu based on the noise-free (odometry-based) motion model
    rx, ry, rθ = belief.mean[1:3]
    belief.mean[1:3] = sensor2d.odometry_model([rx, ry, rθ], odometry)

    # Compute the 3x3 Jacobian Gx of the motion model
    Gx = Matrix{Float32}(I, 3, 3)
    heading = rθ + odometry.rot1
    Gx[1, 3] -= odometry.trans * sin(heading)  # ∂x'/∂θ
    Gx[2, 3] += odometry.trans * cos(heading)  # ∂y'/∂θ

    # Motion noise
    Rx = [
        0.1 0 0;
        0 0.1 0;
        0 0 0.01
    ]
    # Compute the predicted sigma after incorporating the motion
    Σxx = belief.covariance[1:3, 1:3]
    Σxm = belief.covariance[1:3, 4:end]

    belief.covariance[1:3, 1:3] = UpperTriangular(Gx * Σxx * Gx' + Rx)
    belief.covariance[1:3, 4:end] = Gx * Σxm

    return belief
end


function correction_step(belief, observations)
    # num_observations = size(observations, 2)

    for observation in observations
		rx, ry, rθ = belief.mean[1:3]
		mid = observation.landmark_id
        if ismissing(belief.mean[2*mid+2])
			# Initialize its pose in mu based on the measurement and the current robot pose
			mx, my = sensor2d.range_bearing_model([rx, ry, rθ], observation)
			belief.mean[2*mid+2:2*mid+3] = [mx, my]
        end
		# TODO: Add the landmark measurement to the Z vector

		# TODO: Use the current estimate of the landmark pose
		# to compute the corresponding expected measurement in expectedZ:

		# TODO: Compute the Jacobian Hi of the measurement function h for this observation

		# Augment H with the new Hi
		# H = [H;Hi];
    end

	# TODO: Construct the sensor noise matrix Q

	# TODO: Compute the Kalman gain

	# TODO: Compute the difference between the expected and recorded measurements.
	# Remember to normalize the bearings after subtracting!
	# (hint: use the normalize_all_bearings function available in tools)

	# TODO: Finish the correction step by computing the new mu and sigma.
	# Normalize theta in the robot pose.
    return belief
end


landmarks = slam_course_example.load_world()
num_landmarks = size(landmarks, 1)

odometries, observationss = slam_course_example.load_sensor_data()

belief = initial_belief(num_landmarks)
# observed_landmarks = zeros(Bool, num_landmarks)


for t in 1:1
    belief = prediction_step(belief, odometries[t])
    belief = correction_step(belief, observationss[t])
    canvas = visual2d.make_canvas(-2, -2, 12, 12)
    canvas.cla()
    visual2d.draw_state(canvas, t, belief, observationss[t], landmarks)
end
