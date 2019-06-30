using Test
using LinearAlgebra

using SlamTutorial

landmarks = example2d_landmarks()
num_landmarks = size(landmarks, 1)

odometries, range_bearingss = example2d_sensor_data()

believes = []

belief = initial_belief(num_landmarks)
for t in 1:1
    println(belief.mean)
    prediction_step(belief, odometries[t])
    println(belief.mean)
    correction_step(belief, range_bearingss[t])
    println(belief.mean)
    push!(believes, deepcopy(belief))
end
