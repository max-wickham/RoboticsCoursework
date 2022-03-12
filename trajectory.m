%this function takes the current and a goal position 
%and split the trajectory in smaller unevenly-distributed steps 
 
function pos_array = trajectory(current_pos, final_pos)
    
    angle = final_pos(4);
    current_pos = current_pos(1:3);
    final_pos = final_pos(1:3);
    delta_pos = final_pos-current_pos;
    N = round(norm(delta_pos));
    degree = 0.5;
    if mod(N,2) == 1
        N = N+1;
    end
    trans = linspace(-1, 1, N);
    trans = abs(trans.^(-degree));
    trans = trans./sum(trans, 'all');
    trans = transpose(cumsum(trans)) * delta_pos;
    positions = current_pos + trans;
    angle_column = zeros(N,1) + angle;
    pos_array = [positions angle_column]

    %N = 10; %use even number, could be adjusted depending on the distance
    % degree = 1; %find optimal distribution based on robot tests
    % distance = final_pos-current_pos;
    
    % N = round(norm(distance));
    % if mod(N,2) == 1
    %     N = N+1;
    % end
        
    % x_trans = linspace(-1, 1, N);
    % c = abs(x_trans.^(-degree));
    % y_trans = c./sum(c, 'all');
    
    % yx = y_trans.*distance(:,1);
    % yy = y_trans.*distance(:,2);
    % yz = y_trans.*distance(:,3);

    % pos_array =[];
    % a = current_pos(:,1);
    % b = current_pos(:,2);
    % c = current_pos(:,3);
    % for i=1:N
    %     pos_array(i,1) = a+yx(i);
    %     a = pos_array(i,1);
    %     pos_array(i,2) = b+yy(i);
    %     b = pos_array(i,2);
    %     pos_array(i,3) = c+yz(i);
    %     c = pos_array(i,3);
    %     pos_array(i,4) = final_pos(4);
    % end
    
    %figure()
    %plot(pos_array(1,:),pos_array(2,:), 'o')
end