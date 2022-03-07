% main function for cube task2
% main_cube:
%       grid_to_cm()
%       flip() (compute flip value)
%       open gripper
%       reach_cube()
%               move_up(1cube_height)
%               move gripper to cube pos 1 (+flip position)
%               move_down 
%       close gripper 
%       move_cube()
%               move_up(1cube_height)
%               move to cube pos 2 (+flip position)
%               move_down  
%       open grippers
%       (flip again if requested)
%       (move away)


function main_cube(obj,cube_pos_grid, final_pos_grid, flip_angle)
%***INPUTS****% 
% cube pos, final pos = x,y in grid coords
% flip angle = angle diff in rad (inwards from top) 
% append height and angle to cube and final pos

%***PROCESS***%
%  convert to cm 
cube_pos = grid_to_cm(cube_pos_grid);
final_pos = grid_to_cm(final_pos_grid);

% compute flip given two positions (and robot range), 
% append to goal positions
flip = set_flip(cube_pos, final_pos, flip_angle);

% reach_cube
reach_cube();

% move cube
move_cube();
    
end

% function main_cube_stack(obj,cube_pos, final_pos, flip_angle)
%     
% end