module sensor2d
export Odometry, odometry_model, SensorData

struct Odometry
    rot1::Float32
    trans::Float32
    rot2::Float32
end


function odometry_model(pose, odometry)
    rx, ry, rθ = pose
    direction = rθ + odometry.rot1
    rx += odometry.trans * cos(direction)
    ry += odometry.trans * sin(direction)
    rθ += odometry.rot1 + odometry.rot2
    rθ = rem2pi(rθ, RoundNearest)  # Round to [-π, π]
    return [rx, ry, rθ]
end


struct Observation
    landmark_id::Int8
    range::Float32
    bearing::Float32
end


function range_bearing_model(robot_pose, observation)
    rx, ry, rθ = robot_pose
    mx = rx + observation.range * cos(observation.bearing + rθ)
    my = ry + observation.range * sin(observation.bearing + rθ)
    return [mx, my]
end

end
