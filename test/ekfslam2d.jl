using Test
include("src/data/slam_course_example.jl")
# import .slam_course_example

include("src/data/sensor2d.jl")
# import .sensor2d


function test_odometry_model()
    u=sensor2d.Odometry(π/2,3,-π/2)
    x=zeros(Float32, 3)
    @test sensor2d.odometry_model(x, u) ≈ [0, 3, 0]
end


function test_load_sensor_data()
    odometries, observationss = slam_course_example.load_sensor_data()
    @test length(odometries) == length(observationss)
end


test_odometry_model()
test_load_sensor_data()
