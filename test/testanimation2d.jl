using PyPlot
using PyCall
@pyimport matplotlib.animation as animation
@pyimport IPython.display as ipdisplay

using SlamTutorial


function test_draw_2d_robot()
    fig, ax = make_canvas(-2, -2, 12, 12)
    robot = draw_2d_robot(ax, [3, 4, π/6])
    return fig
end

function test_animate_2d_robot()
    fig, ax = make_canvas(-2, -2, 12, 12)
    poses = []
    for i in 1:10
        robot_pose = draw_2d_robot(ax, [i, i, π/6])
        push!(poses, robot_pose)
    end

    ani = animation.ArtistAnimation(fig, poses, interval=50, repeat_delay=100)
    return ani
end

display("text/html", test_animate_2d_robot().to_jshtml())

#
# function test_draw_gaussian()
#     canvas = visual2d.make_canvas(-2, -2, 12, 12)
#     μ = [5, 5]
#     Σ = [1 0.5; 0.5 3]
#     visual2d.draw_gaussian(canvas, μ, Σ)
#
#     return canvas
# end
#
# function test_draw_state()
#     canvas = visual2d.make_canvas(-2, -2, 12, 12)
#     landmarks = [5 5; 3 4]
#     num_landmarks = size(landmarks, 1)
#     belief = ekfslam2d.initial_belief(num_landmarks)
#
#     # observed_landmarks = zeros(Bool, num_landmarks)
#     # observed_landmarks[1] = true
#     RangeBearings = [sensor2d.RangeBearing(1, 0, 0)]
#
#     belief.covariance[1:2, 1:2] = [1 0.5; 0.5 2]
#     belief.mean[4:5] = [1, 1]
#     belief.covariance[4:5, 4:5] = [0.2 0.1; 0.1 0.5]
#     visual2d.draw_state(canvas, 1, belief, RangeBearings, landmarks)
# end
