using Test

using SlamTutorial


function test_standard_odometry_model()
    u=Odometry(π/2,3,-π/2)
    x=zeros(Float32, 3)
    @test standard_odometry_model(x, u) ≈ [0, 3, 0]
end

function test_range_bearing_model()
    rb=RangeBearing(0, √2, π/4)
    x=zeros(Float32, 3)
    @test range_bearing_model(x, rb) ≈ [1, 1]
end
