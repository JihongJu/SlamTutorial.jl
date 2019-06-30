struct Odometry
    rot1::Float32
    trans::Float32
    rot2::Float32
end


function standard_odometry_model(pose, odometry)
    rx, ry, rθ = pose
    direction = rθ + odometry.rot1
    rx += odometry.trans * cos(direction)
    ry += odometry.trans * sin(direction)
    rθ += odometry.rot1 + odometry.rot2
    rθ = rem2pi(rθ, RoundNearest)  # Round to [-π, π]
    return [rx, ry, rθ]
end


struct RangeBearing
    landmark_id::Int8
    range::Float32
    bearing::Float32
end


function range_bearing_model(robot_pose, range_bearing)
    rx, ry, rθ = robot_pose
    range, bearing = range_bearing.range, range_bearing.bearing
    mx = rx + range * cos(bearing + rθ)
    my = ry + range * sin(bearing + rθ)
    return [mx, my]
end
