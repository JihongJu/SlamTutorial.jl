module sensor2d
export Odometry, odometry_model, SensorData

struct Odometry
    rot1::Float32
    trans::Float32
    rot2::Float32
end


function odometry_model(pose, odometry)
    x, y, θ = pose
    direction = θ + odometry.rot1
    x += odometry.trans * cos(direction)
    y += odometry.trans * sin(direction)
    θ += odometry.rot1 + odometry.rot2
    θ = rem2pi(θ, RoundNearest)  # Round to [-π, π]
    return [x, y, θ]
end


struct Observation
    landmark_id::Int8
    range::Float32
    bearing::Float32
end


function range_bearing_model(robot_pose, observation)
    x, y, θ = robot_pose
    mx = x + observation.range * cos(observation.bearing + θ)
    my = y + observation.range * sin(observation.bearing + θ)
    return [mx, my]
end

end
