using PyPlot
using LinearAlgebra


function make_canvas(xmin, ymin, xmax, ymax)
    ax = subplot(111)
    ax.set_aspect("equal")
    ax.set_xlim(xmin, xmax)
    ax.set_ylim(ymin, ymax)
    return ax
end


function draw_landmarks(canvas, landmarks)
    canvas.scatter(landmarks[:, 1], landmarks[:, 2], marker="+", color="r")
    return canvas
end


function draw_robot(canvas, pose)
    x, y, θ = pose

    canvas.scatter(x, y, marker="o", c="dodgerblue")
    head = matplotlib.markers.MarkerStyle(matplotlib.markers.TICKRIGHT)
    head._transform.rotate(θ)
    canvas.scatter(x, y, marker=head, color="dodgerblue")
end


function draw_gaussian(canvas, mu, sigma, nstd=1)
    factn = eigen(sigma)

    width, height = 2 * nstd * sqrt.(factn.values)

    firstvec = factn.vectors[:, 1]
    angle = atan(firstvec[2], firstvec[1]) * 360 /2π

    ell = matplotlib.patches.Ellipse(mu, width, height,
        angle=angle, linewidth=2, zorder=2, alpha=0.5)
    canvas.add_patch(ell)
end


function draw_state(canvas, timestep, belief, observed, sensor, landmarks)
    # Draw robot
    draw_robot(canvas, belief.μ[1:3])  # Turn into a proper robot drawing
    draw_gaussian(canvas, belief.μ[1:2], belief.Σ[1:2, 1:2])

    # Draw observed landmarks
    for i in 1:length(observed)
        canvas.scatter(belief.μ[2*i+2], belief.μ[2*i+3], marker='o')
        draw_gaussian(canvas, belief.μ[2*i+2:2*i+3], belief.Σ[2*i+2:2*i+3,2*i+2:2*i+3])
    end

    # Draw ground truth landmarks
    draw_landmarks(canvas, landmarks)

    return
end


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
