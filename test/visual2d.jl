include("src/utils/visual2d.jl")
using .visual2d


function test_draw_state()
    canvas = make_canvas(-2, -2, 12, 12)

    draw_robot(canvas, [3, 4, π/6])

    landmarks = [2 1; 0 4; 2 7; 9 2; 10 5; 9 8; 5 5; 5 3; 5 9]
    draw_landmarks(canvas, landmarks)

    μ = [5, 5]
    Σ = [1 0.5; 0.5 3]
    draw_gaussian(canvas, μ, Σ)

end


test_draw_state()
