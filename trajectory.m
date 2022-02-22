%this function takes the current and a goal position 
%and split the trajectory in smaller unevenly-distributed steps 
 
function pos_array = trajectory(current_pos, final_pos)
    N = 10; %use even number, could be adjusted depending on the distance
    degree = 1; %find optimal distribution based on robot tests
    
    x_trans = linspace(-1, 1, N);
    c = abs(x_trans.^(-degree));
    y_trans = c./sum(c, 'all');

    distance = final_pos-current_pos;
    yx = y_trans.*distance(:,1);
    yy = y_trans.*distance(:,2);

    pos_array =[];
    a = current_pos(:,1);
    b = current_pos(:,2);
    for i=1:N
        pos_array(1,i) = a+yx(i);
        a = x(i);
        pos_array(2,i) = b+yy(i);
        b = y(i);
    end
    
    %figure()
    %plot(pos_array(1,:),pos_array(2,:), 'o')
end