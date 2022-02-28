function pos = polar_to_cartesian(theta, distance)
    x = sin(theta)*distance;
    y = cos(theta)*distance;
    pos = [transpose(x)  transpose(y)]
end