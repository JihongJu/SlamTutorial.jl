using LinearAlgebra


mutable struct Belief
    mean::Vector{Union{Float32, Missing}}
    covariance::Matrix{Float32}
end


function belief_init(num_landmarks)
    μ = Vector{Union{Float32, Missing}}(missing, 3 + 2*num_landmarks)
	μ[1:3] .= 0
    Σ = zeros(Float32, 3+2*num_landmarks, 3+2*num_landmarks)
    Σ[diagind(Σ)[1:3]] .= 0
    Σ[diagind(Σ)[4:end]] .= 1000

    return Belief(μ, Symmetric(Σ))
end


function prediction_step(belief, odometry)
    # Compute the new mu based on the noise-free (odometry-based) motion model
    rx, ry, rθ = belief.mean[1:3]
    belief.mean[1:3] = standard_odometry_model([rx, ry, rθ], odometry)

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

    Σ = Matrix(belief.covariance)
	Σ[1:3, 1:3] = Gx * Σxx * Gx' + Rx
    Σ[1:3, 4:end] = Gx * Σxm
	belief.covariance = Symmetric(Σ)

end


function correction_step(belief, range_bearings)
	rx, ry, rθ = belief.mean[1:3]

	num_range_bearings = length(range_bearings)
	num_dim_state = length(belief.mean)

	H = Matrix{Float32}(undef, 2 * num_range_bearings, num_dim_state) # Jacobian matrix ∂ẑ/∂(rx,ry)
	zs, ẑs = [], []  # true and predicted observations

    for (i, range_bearing) in enumerate(range_bearings)
		mid = range_bearing.landmark_id
        if ismissing(belief.mean[2*mid+2])
			# Initialize its pose in mu based on the measurement and the current robot pose
			mx, my = range_bearing_model([rx, ry, rθ], range_bearing)
			belief.mean[2*mid+2:2*mid+3] = [mx, my]
        end
		# Add the landmark measurement to the Z vector
		zs = [zs; range_bearing.range; range_bearing.bearing]

		# Use the current estimate of the landmark pose
		# to compute the corresponding expected measurement in z̄:
		mx, my = belief.mean[2*mid+2:2*mid+3]
		δ = [mx - rx, my - ry]
		q = dot(δ, δ)
		sqrtq = sqrt(q)

	 	ẑs = [ẑs; sqrtq; atan(δ[2], δ[1]) - rθ]

		# Compute the Jacobian Hi of the measurement function h for this observation
		δx, δy = δ
		Hi = zeros(Float32, 2, num_dim_state)
		Hi[1:2, 1:3] = [
			-sqrtq * δx  -sqrtq * δy   0;
			δy           -δx           -q
		] / q
		Hi[1:2, 2*mid+2:2*mid+3] = [
			sqrtq * δx sqrtq * δy;
			-δy δx
		] / q

		# Augment H with the new Hi
		H[2*i-1:2*i, 1:end] = Hi
    end

	# Construct the sensor noise matrix Q
	Q = Diagonal{Float32}(ones(2 * num_range_bearings) * 0.01)

	# Compute the Kalman gain K
	K = belief.covariance * H' * inv(H * belief.covariance * H' + Q)

	# Compute the difference between the expected and recorded measurements.
	Δz = zs - ẑs
	# Normalize the bearings
	Δz[2:2:end] = map(bearing->rem2pi(bearing, RoundNearest), Δz[2:2:end])

	# Finish the correction step by computing the new mu and sigma.
	belief.mean += K * Δz
	I = Diagonal{Float32}(ones(num_dim_state))
	belief.covariance = Symmetric((I - K * H) * belief.covariance)

	# Normalize theta in the robot pose.
	belief.mean[3] = rem2pi(belief.mean[3], RoundNearest)
end
