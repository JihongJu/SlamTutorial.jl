using LinearAlgebra


include("src/data/slam_course_example.jl")
include("src/utils/visual2d.jl")
include("src/data/sensor2d.jl")


mutable struct Belief
    mean::Vector{Union{Float32, Missing}}
    covariance::UpperTriangular{Float32}
end


function initial_belief(num_landmarks)
    μ = Vector{Union{Float32, Missing}}(missing, 3 + 2*num_landmarks)
	μ[1:3] .= 0
    Σ = zeros(Float32, 3+2*num_landmarks, 3+2*num_landmarks)
    Σ[diagind(Σ)[1:3]] .= 0
    Σ[diagind(Σ)[4:end]] .= 1000

    return Belief(μ, UpperTriangular(Σ))
end


function prediction_step(belief::Belief, odometry)
    # Compute the new mu based on the noise-free (odometry-based) motion model
    rx, ry, rθ = belief.mean[1:3]
    belief.mean[1:3] = sensor2d.odometry_model([rx, ry, rθ], odometry)

    # Compute the 3x3 Jacobian Gx of the motion model
    Gx = Matrix{Float32}(I, 3, 3)
    heading = rθ + odometry.rot1
    Gx[1, 3] -= odometry.trans * sin(heading)  # ∂x'/∂θ
    Gx[2, 3] += odometry.trans * cos(heading)  # ∂y'/∂θ

    # Motion noise
    Rx = Diagonal{Float32}([0.1, 0.1, 0.01])

    # Compute the predicted sigma after incorporating the motion
    Σxx = belief.covariance[1:3, 1:3]
    Σxm = belief.covariance[1:3, 4:end]

    belief.covariance[1:3, 1:3] = UpperTriangular(Gx * Σxx * Gx' + Rx)
    belief.covariance[1:3, 4:end] = Gx * Σxm
	belief.covariance = UpperTriangular(belief.covariance)

    return belief
end


function correction_step(belief::Belief, observations)
	rx, ry, rθ = belief.mean[1:3]

	num_observations = length(observations)
	num_dim_state = length(belief.mean)

	H = Matrix{Float32}(undef, 2 * num_observations, num_dim_state) # Jacobian matrix ∂ẑ/∂(rx,ry)
	zs, ẑs = [], []  # true and predicted observations

    for (i, observation) in enumerate(observations)
		mid = observation.landmark_id
        if ismissing(belief.mean[2*mid+2])
			# Initialize its pose in mu based on the measurement and the current robot pose
			mx, my = sensor2d.range_bearing_model([rx, ry, rθ], observation)
			belief.mean[2*mid+2:2*mid+3] = [mx, my]
        end
		# Add the landmark measurement to the Z vector
		zs = [zs; observation.range; observation.bearing]

		# Use the current estimate of the landmark pose
		# to compute the corresponding expected measurement in z̄:
		mx, my = belief.mean[2*mid+2:2*mid+3]
		δ = [mx - rx, my - ry]
		q = dot(δ, δ)
		√q = sqrt(q)

	 	ẑs = [ẑs; √q; atan(δ[2], δ[1]) - rθ]

		# Compute the Jacobian Hi of the measurement function h for this observation
		δx, δy = δ
		Hi = zeros(Float32, 2, num_dim_state)
		Hi[1:2, 1:3] = [
			-√q * δx -√q * δy 0;
			δy     -δx    -q
		] / q
		Hi[1:2, 2*mid+2:2*mid+3] = [
			√q * δx √q * δy;
			-δy δx
		] / q

		# Augment H with the new Hi
		H[2*i-1:2*i, 1:end] = Hi
    end

	# Construct the sensor noise matrix Q
	Q = Diagonal{Float32}(ones(2 * num_observations) * 0.01)

	# Compute the Kalman gain K
	# print(size(belief.covariance), size(H'), size(Q))
	K = belief.covariance * H' * inv(H * belief.covariance * H' + Q)

	# Compute the difference between the expected and recorded measurements.
	# Remember to normalize the bearings after subtracting!
	Δz = zs - ẑs
	Δz[2:2:end] = map(bearing->rem2pi(bearing, RoundNearest), Δz[2:2:end])

	# Finish the correction step by computing the new mu and sigma.
	belief.mean += K * Δz
	I = Diagonal{Float32}(ones(num_dim_state))
	belief.covariance = UpperTriangular((I - K * H) * belief.covariance)

	# Normalize theta in the robot pose.
	belief.mean[3] = rem2pi(belief.mean[3], RoundNearest)
    return belief
end


landmarks = slam_course_example.load_world()
num_landmarks = size(landmarks, 1)

odometries, observationss = slam_course_example.load_sensor_data()

belief = initial_belief(num_landmarks)
# observed_landmarks = zeros(Bool, num_landmarks)


for t in 1:50
    belief = prediction_step(belief, odometries[t])
    belief = correction_step(belief, observationss[t])
    canvas = visual2d.make_canvas(-2, -2, 12, 12)
    canvas.cla()
    visual2d.draw_state(canvas, t, belief, observationss[t], landmarks)
end
