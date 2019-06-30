function example2d_landmarks()
    landmarks = [2 1; 0 4; 2 7; 9 2; 10 5; 9 8; 5 5; 5 3; 5 9]
    return landmarks
end


function example2d_sensor_data()
    data_path = joinpath(tempdir(), "slam_example2d.dat")
    if !isfile(data_path)
        url = "https://www.dropbox.com/s/8278yfc6rnnjuqp/slam_example2d.dat"
        download(url, data_path)
    end
    return open(data_path) do file
        odometries::Vector{Odometry} = []
        range_bearingss::Vector{Vector{RangeBearing}} = []

        range_bearings::Vector{RangeBearing} = []
        for (num, line) in enumerate(eachline(file))
            columns = split(line, ' ', keepempty=false)
            key, values = columns[1], columns[2:end]
            if key == "ODOMETRY"
                readings = (parse(Float32, value) for value in values)
                push!(odometries, Odometry(readings...))
                if num > 1
                    push!(range_bearingss, range_bearings)
                    range_bearings = []
                end
            end

            if key == "SENSOR"
                id = parse(Int8, values[1])
                readings = (parse(Float32, value) for value in values[2:end])
                push!(range_bearings, RangeBearing(id, readings...))
            end
        end
        push!(range_bearingss, range_bearings)
        return (odometries, range_bearingss)
    end
end
