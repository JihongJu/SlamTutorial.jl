module ekfslam2d
export Belief, initial_belief

using LinearAlgebra


mutable struct Belief
    mu::Array{Float32,1}
    sigma::Array{Float32,2}
end


function initial_belief(num_landmarks)
    μ = zeros(Float32, 3 + 2*num_landmarks)
    Σ = Matrix{Float32}(I, 3+2*num_landmarks, 3+2*num_landmarks)
    Σ[1:3,1:3] .= 0

    belief=Belief(μ, Σ)
    return belief

end
# belief = initial_belief(3)
# print(belief.mu)
# print(belief.sigma)

end
