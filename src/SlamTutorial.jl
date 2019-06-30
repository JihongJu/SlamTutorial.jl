module SlamTutorial

export
    Odometry, standard_odometry_model, RangeBearing, range_bearing_model,
    Belief, belief_init, prediction_step, correction_step,
    make_canvas, make_animation, draw_2d_robot, draw_2d_gaussian, draw_kalman_state, animate_kalman_state,
    example2d_landmarks, example2d_sensor_data

include("animation2d.jl")
include("ekfslam2d.jl")
include("sensor2d.jl")
include("data/example2d.jl")

end # module
