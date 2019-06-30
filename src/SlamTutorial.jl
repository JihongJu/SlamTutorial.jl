module SlamTutorial

export
    Odometry, standard_odometry_model, RangeBearing, range_bearing_model,
    make_canvas, make_animation, draw_2d_robot, draw_2d_gaussian, draw_kalman_state

include("sensor2d.jl")
include("animation2d.jl")

end # module
