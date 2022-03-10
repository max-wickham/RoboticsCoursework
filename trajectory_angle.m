function pos_array = trajectory_angle(current_pos, final_pos)
    if final_pos(4) > pi
        final_pos(4) = final_pos(4) - 2*pi;
    end
    if current_pos(4) > pi
        current_pos(4) = current_pos(4) - 2*pi;
    end
    N = round(norm(final_pos(4) - current_pos(4)) * 20 / pi);
    positions = zeros(N,4);
    angles = linspace(current_pos(4), final_pos(4), N);
    for i = 1:N
        positions(i,1:4) = [current_pos(1), current_pos(2), current_pos(3), angles(i)];
    end
    pos_array = positions;
end