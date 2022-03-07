% *********Functions for task 2 moving cubes **********
%to do in obj:
%up and down level, current position
%********************************REACH    

function reach_cube(gripper_angle_init)
    open_gripper(obj);
    move_and_flip();
    close_gripper(obj);
end

%********************************MOVE

function move_cube(gripper_angle_final)
    close_gripper(obj);
    move_and_flip()
    open_gripper(obj);
end

%********************************FLIP SETUP



%retun initial and final gripper orientation plus required flip on spot       
function gripper_angle_init, gripper_angle_final, overflow = set_flip(obj, cube_pos, final_pos, flip_angle)
    %**PARAMETERS**
    grip_inwards = pi;%-pi/2;
    grip_outwards = 0;%pi/2;
    grip_vertical = -pi/2;
    %**INPUT**
    %pos is x,y coords
    %flip_angle is amount of flip requested in outwards direction from top
    % 0 deg, no flip
    % allowed values for flip = 0, 90,180,270
    dir_init = turnability(cube_pos);
    dir_final = turnability(final_pos);
        %********compute flipping mode
        %turn = flip_angle/90;
        if flip_angle < 180 %neg direction
            if dir(2) % neg allowed
                gripper_final_angle = grip_inwards;
                turn = 1;
            else
                gripper_final_angle = grip_outwards;
                turn = 3;
            end
        elseif flip_angle == 180 %180°
            if dir(2) && dir(1) % neg allowed
                gripper_final_angle = grip_inwards;
                turn = 1;
            elseif dir(2)
                gripper_final_angle = grip_inwards;
                turn = 2;
            else
                gripper_final_angle = grip_outwards;
                turn = 2;   
            end
        elseif flip_angle > 180 %pos direction
            if dir(1) % neg allowed
                gripper_final_angle = grip_outwards;
                turn = 1;
            else
                gripper_final_angle = grip_inwards;
                turn = 3;
            end
        end

    %pos turn
    if flip_angle == 90 && dir_init(2)
         grip_angle_init = grip_inwards;
end

- if 90 and allowed 
  if 180 and allowed
  if -90 and allowed
  else flip on spot
      
%********************************MOVE and FLIP 

function reach_cube(obj,cube_pos)
       current_pos = robotController.current_position();
       if current_pos(3) <= UP_level
           current_pos(3) = obj.up_level;
       end
       up_cube_pos = [cube_pos(1), cube_pos(2), obj.up_level, gripper_angle_final];
       down_cube_pos = [cube_pos(1), cube_pos(2), obj.down_level, gripper_angle_final];
       pos_array = trajectory(current_pos, up_cube_pos);
       controller.move_to_positions(pos_array);
       controller.move_to_positions([down_cube_pos]);
end       

function move_and_flip(obj, final_pos, flip_angle):
    %open/close
    if
    reach_cube()
    close/open
    end
end
