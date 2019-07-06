using PyPlot
using LinearAlgebra

using PyCall


function make_canvas(xmin, ymin, xmax, ymax)
	fig = figure()
	ax = fig.add_subplot(111, autoscale_on=false, xlim=(xmin, xmax), ylim=(ymin, ymax))
    ax.set_aspect("equal")
    return fig, ax
end


function make_animation(figure, artists; interval=500, repeat_delay=1000)
	animation = pyimport("matplotlib.animation")
	PyPlot.isjulia_display[1] = false
    return animation.ArtistAnimation(figure, artists, interval=interval, repeat_delay=repeat_delay)
end


function draw_2d_robot(ax, pose; sigma=nothing, color="royalblue")
    x, y, θ = pose

	artists = []
    robot = ax.scatter(x, y, marker="o", color=color, alpha=0.8, animated=true)
	push!(artists, robot)

    arrow = matplotlib.markers.MarkerStyle(matplotlib.markers.TICKRIGHT)
    arrow._transform.rotate(θ)
    heading = ax.scatter(x, y, marker=arrow, color=color, alpha=0.8, animated=true)
	push!(artists, heading)

	if !isnothing(sigma)
		prob = draw_2d_gaussian(ax, (x, y), sigma, color=color)
		push!(artists, prob)
	end
	return artists
end


function draw_2d_gaussian(ax, mu, sigma; nstd=1, color="tomato")
    factn = eigen(sigma)

    width, height = 2 * nstd * sqrt.(factn.values)

    firstvec = factn.vectors[:, 1]
    angle = atan(firstvec[2], firstvec[1]) * 360 /2π

    ell = matplotlib.patches.Ellipse(mu, width, height,
        angle=angle, linewidth=2, zorder=2, alpha=0.5, color=color)
    return ax.add_patch(ell)
end


function draw_kalman_state(ax, belief, range_bearings, landmarks)
    robot_mu = belief.mean[1:3]
    robot_sigma = belief.covariance[1:3, 1:3]
    artists = []

    # Draw robot
    robot_pred = draw_2d_robot(ax, robot_mu, sigma=robot_sigma[1:2, 1:2], color="royalblue")
    for pc in robot_pred
        push!(artists, pc)
    end

    # Draw observed landmarks
    num_landmarks = size(landmarks, 1)
    for i in 1:num_landmarks
        if !ismissing(belief.mean[2*i+2])
            landmark_pred = ax.scatter(belief.mean[2*i+2], belief.mean[2*i+3], marker="X", color="grey", alpha=0.8)
            push!(artists, landmark_pred)

            if belief.covariance[2*i+2,2*i+2] < 1000
                landmark_prob = draw_2d_gaussian(ax, belief.mean[2*i+2:2*i+3], belief.covariance[2*i+2:2*i+3,2*i+2:2*i+3], color="grey")
                push!(artists, landmark_prob)
            end
        end
    end

    # Draw observing rays
    lines = []
    for range_bearing in range_bearings
        landmark_mu = belief.mean[2*range_bearing.landmark_id+2:2*range_bearing.landmark_id+3]
        push!(lines, [robot_mu[1:2], landmark_mu])
    end
    rays = matplotlib.collections.LineCollection(lines, linestyle="dotted")
    push!(artists, ax.add_collection(rays))

    # Draw ground truth landmarks
    landmarks_true = ax.scatter(landmarks[:, 1], landmarks[:, 2], marker="*", color="r")
    push!(artists, landmarks_true)

    return artists
end


function animate_kalman_state(canvas, believes, range_bearingss, landmarks)
    fig, ax = canvas
    frames = []
    for t in 1:length(believes)
        frame = draw_kalman_state(ax, believes[t], range_bearingss[t], landmarks)
        push!(frames, frame)
    end
    return make_animation(fig, frames)
end
