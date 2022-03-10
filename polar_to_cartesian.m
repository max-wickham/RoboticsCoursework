function pos = polar_to_cartesian(theta, distance)
    x = cos(theta)*distance;
    y = sin(theta)*distance;
    pos = [transpose(x)  transpose(y)]
end