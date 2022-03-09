% main function for cube task2
% main_cube:
%       grid_to_cm()
%       set_flip() (compute flip value)
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
%       (if requested)- flip_on_the_spot()
%                           up, orient, down, close, up orient, down, open
%       move up


function main_cube(obj,cube_pos_grid, final_pos_grid, flip_angle)
    %***INPUTS****% 
    % cube pos, final pos = x,y in grid coords
    % flip angle = angle diff in rad (inwards from top) 
    % append height and angle to cube and final pos

    %***PROCESS***%
    %  TO DO : convert to cm 
     cube_pos = grid_to_cm(cube_pos_grid);
     final_pos = grid_to_cm(final_pos_grid);

    % compute flip given two positions (and robot range), 
    % append to goal positions
    gripper_initial_angle, gripper_final_angle, flip_on_spot = set_flip(obj, cube_pos, final_pos, flip_angle);

    % reach cube and move, flip if possible
    reach_move_flip(obj,cube_pos, final_pos, gripper_initial_angle, gripper_final_angle);

    % rest of flipping performed on the spot
    flip_on_the_spot(obj,final_pos, flip_on_spot);
  
end

%to do after testing: stacking: same ration but different UP and DOWN_level
% function main_cube_stack(obj,cube_pos, final_pos, flip_angle)
%     
% end