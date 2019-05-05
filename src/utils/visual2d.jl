module visual2d
export make_canvas, draw_landmarks, draw_robot, draw_gaussian, draw_state

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


function draw_robot(canvas, pose, color="royalblue")
    x, y, θ = pose

    canvas.scatter(x, y, marker="o", color=color)
    heading = matplotlib.markers.MarkerStyle(matplotlib.markers.TICKRIGHT)
    heading._transform.rotate(θ)
    canvas.scatter(x, y, marker=heading, color=color)
end


function draw_gaussian(canvas, mu, sigma, nstd=1, color="tomato")
    factn = eigen(sigma)

    width, height = 2 * nstd * sqrt.(factn.values)

    firstvec = factn.vectors[:, 1]
    angle = atan(firstvec[2], firstvec[1]) * 360 /2π

    ell = matplotlib.patches.Ellipse(mu, width, height,
        angle=angle, linewidth=2, zorder=2, alpha=0.5, color=color)
    canvas.add_patch(ell)
end


function draw_state(canvas, timestep, belief, observed, sensor_data, landmarks)
	robot_mu = belief.mu[1:3]
	robot_sigma = belief.sigma[1:3, 1:3]
    # Draw robot
    draw_robot(canvas, robot_mu, color="royalblue")
	draw_gaussian(canvas, robot_mu[1:2], robot_sigma[1:2, 1:2], color="royalblue")

    # Draw observed landmarks
    for i in 1:length(observed)
        canvas.scatter(belief.mu[2*i+2], belief.mu[2*i+3], marker='o')
        draw_gaussian(canvas, belief.mu[2*i+2:2*i+3], belief.sigma[2*i+2:2*i+3,2*i+2:2*i+3])
    end

    # Draw sensor rays
    for i in 1:size(sensor,2)
		mx = belief.μ(2*sensor_data(i).id+2);
		my = belief.μ(2*sensor_data(i).id+3);
    	line = matplotlib.lines.Line2D([belief.μ[1], mx], [belief.μ[2], my], color='k')
		canvas.add_line(line)
    end

    # Draw ground truth landmarks
    draw_landmarks(canvas, landmarks)

    return
end


end
