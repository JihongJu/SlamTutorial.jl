using Test

include("src/ekfslam2d.jl")
using .ekfslam2d

function test_odometry_model()
    u=Odometry(π/2,3,-π/2)
    x=zeros(Float32, 3)
    @test odometry_model(x, u) ≈ [0, 3, 0]
end


function test_load_sensor_data()
    odometry, sensor = load_sensor_data()
    @test length(odometry) == length(sensor)
end

# test_odometry_model()
# test_load_sensor_data()

include("src/ekfslam2d.jl")
using .ekfslam2d

include("src/data/slam_course_example.jl")

include("src/utils/visual2d.jl")
using .visual2d


odometry, sensor_data = load_sensor_data()
draw_state()
