module slam_course_example
export load_world, load_sensor_data

include("sensor2d.jl")
# import .sensor2d


function load_world()
    landmarks = [2 1; 0 4; 2 7; 9 2; 10 5; 9 8; 5 5; 5 3; 5 9]
    return landmarks
end


function load_sensor_data()
    return open(joinpath(@__DIR__, "slam_course_example.dat")) do file
        odometry::Array{sensor2d.Odometry, 1} = []
        sensor::Array{Array{sensor2d.SensorData, 1}, 1} = []

        observations::Array{sensor2d.SensorData, 1} = []
        for (i, line) in enumerate(eachline(file))
            columns = split(line, ' ', keepempty=false)
            key, values = columns[1], columns[2:end]
            if key == "ODOMETRY"
                readings = (parse(Float32, value) for value in values)
                push!(odometry, sensor2d.Odometry(readings...))
                if i > 1
                    push!(sensor, observations)
                    observations = []
                end
            end

            if key == "SENSOR"
                id = parse(Int8, values[1])
                readings = (parse(Float32, value) for value in values[2:end])
                push!(observations, sensor2d.SensorData(id, readings...))
            end
        end
        push!(sensor, observations)
        return (odometry, sensor)
    end
end


end
