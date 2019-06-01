using PyPlot
include("src/utils/visual2d.jl")
import .visual2d

function test_draw_robot()
    canvas = visual2d.make_canvas(-2, -2, 12, 12)
    visual2d.draw_robot(canvas, [3, 4, π/6])
    return canvas
end



function test_draw_gaussian()
    canvas = visual2d.make_canvas(-2, -2, 12, 12)
    μ = [5, 5]
    Σ = [1 0.5; 0.5 3]
    visual2d.draw_gaussian(canvas, μ, Σ)

    return canvas
end

function test_draw_state()
    canvas = visual2d.make_canvas(-2, -2, 12, 12)
    landmarks = [5 5; 3 4]
    num_landmarks = size(landmarks, 1)
    belief = ekfslam2d.initial_belief(num_landmarks)

    observed_landmarks = zeros(Bool, num_landmarks)
    observed_landmarks[1] = true
    sensor_data = [sensor2d.SensorData(1, 0, 0)]

    belief.sigma[1:2, 1:2] = [1 0.5; 0.5 2]
    belief.mu[4:5] = [1, 1]
    belief.sigma[4:5, 4:5] = [0.2 0.1; 0.1 0.5]
    visual2d.draw_state(canvas, 1, belief, observed_landmarks, sensor_data, landmarks)
end

test_draw_state()
