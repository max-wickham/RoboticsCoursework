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

%given position it returns turning dir=(pos dir allowed, neg dir allowed)
function dir = turnability(pos)
    if cube_pos < min_range
        dir = [false, true]; %neg allowed
    elseif cube_pos > max_range
        dir = [true, false]; %pos allowed
    else 
        dir = [true, true]; %in range
    end
end

%retun initial and final gripper orientation plus required flip on spot       
function gripper_angle_init, gripper_angle_final, overflow = set_flip(obj, cube_pos, final_pos, flip_angle)
    %**PARAMETERS**
    grip_inwards = -pi/2;
    grip_outwards = pi/2;
    grip_vertical = 0;
    %**INPUT**
    %flip_angle is amount of flip requested in outwards direction from top
    % 0deg is red face up, no flip
    % from 0 to 270 
    %**OUTPUT**
    %overflow if cannot perform flip whist moving, same as flip_angle

    dir_init = turnability(cube_pos);
    dir_final = turnability(final_pos);
    %pos turn
    if flip_angle == 90 && dir_init(2)
         grip_angle_init = grip_inwards
end

- if 90 and allowed 
  if 180 and allowed
  if -90 and allowed
  else flip on spot
      
%********************************MOVE and FLIP 

function reach_cube(obj,cube_pos)
       current_pos = robotController.current_position();
       if current_pos(3) <= cube_height
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
%*******************FLIP ON THE SPOT
    function flip_on_the_spot(position, flip_angle)
        %**PARAMETERS**
        grip_inwards = pi;%-pi/2;
        grip_outwards = 0;%pi/2;
        grip_vertical = -pi/2;
        %**INPUT**
        %flip_angle is amount of flip requested in outwards direction from top
        % 0 deg is red face up, no flip
        % allowed values for flip = 0, 90,180,270
        dir = turnability(position)
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
        %********actual flipping
        %open and go up, turn gripper, down, close, go up, turn, down, open,
        %up
        for i=1:turn
            open_gripper(obj)
            current_pos = robotController.current_position();
            if current_pos(3) <= cube_height
                current_pos(3) = obj.up_level;
            end
            robotController.move_to_positions([position(1), position(2), current_pos(3), gripper_final_angle]);
            robotController.move_down();
            robotController.close_gripper();
            robotController.move_up();
            if flip_angle == 180 && turn == 1
                robotController.move_to_positions([position(1), position(2), current_pos(3), gripper_outwards]);
            else
                robotController.move_to_positions([position(1), position(2), current_pos(3), gripper_final_angle]);
            end
            robotController.move_down();          
            robotController.opem_gripper();
            robotController.move_up();
        end 
    end 