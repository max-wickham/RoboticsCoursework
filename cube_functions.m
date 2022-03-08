% *********Functions for task 2 moving cubes **********
%to do in obj:
%up and down level, current position
%********************************REACH    

function reach_cube(obj,gripper_angle_init)
    open_gripper(obj);
    move_and_flip();
    close_gripper(obj);
end

%********************************MOVE

function move_cube(obj,gripper_angle_final)
    close_gripper(obj);
    move_and_flip()
    open_gripper(obj);
end

%********************************FLIP SETUP



%retun initial and final gripper orientation plus required flip on spot       
function gripper_initial_angle, gripper_final_angle, flip_on_spot = set_flip(obj, cube_pos, final_pos, flip_angle)
       
    %**INPUTS**
    %pos is x,y coords
    %flip_angle is amount of flip requested in outwards direction from top
    % 0 deg, no flip
    % allowed values for flip = 0, 90,180,270
    
    %**OUTPUTS**
    %grip initial is angle at which grab cube, 
    %grip final is angle at which release the cube
    %flip_on_spot if couldnt perform all turn, flip still to perform a final position
    
    dir_init = turnability(cube_pos);
    dir_final = turnability(final_pos);
    flip_on_spot =0;
    %********compute flipping mode
    %--90deg turn
    if flip_angle < 180 %neg direction
        if dir_init(2) % neg allowed
            gripper_initial_angle = obj.grip_inwards;
            gripper_final_angle = obj.grip_vertical;
        elseif dir_final(1)
            gripper_final_angle = obj.grip_outwards;
            gripper_initial_angle = obj.grip_vertical;
        else
             gripper_initial_angle = obj.grip_vertical;
             gripper_final_angle = obj.grip_vertical;
             flip_on_spot = 90;
        end
    %--180deg turn
    elseif flip_angle == 180
        if dir_init(2) && dir_final(1) % neg allowed
            gripper_initial_angle = obj.grip_inwards;
            gripper_final_angle = obj.grip_outwards;
        elseif dir_init(1) && dir_final(2)
            gripper_final_angle = obj.grip_inwards;
            gripper_initial_angle = obj.grip_outwards;
        else
            % TODO
            % recursion or back 90/270
            % gripper_initial_angle, gripper_final_angle, flip_on_spot_2 = set_flip(cube_pos, final_pos, 90)
            
        
        end
    %--270deg turn
    elseif flip_angle > 180 %pos direction
        if dir_init(1) % pos allowed
            gripper_initial_angle = obj.grip_outwards;
            gripper_final_angle = obj.grip_vertical;
        elseif dir_final(2)
            gripper_final_angle = obj.grip_inwards;
            gripper_initial_angle = obj.grip_vertical;
        else
             gripper_initial_angle = obj.grip_vertical;
             gripper_final_angle = obj.grip_vertical;
             flip_on_spot = 270;
        end
    end 
end


%********************************MOVE and FLIP 

function reach_cube(obj,cube_pos)
       %open, up, new position and gripper orientation, down, close
       cubeController.open_gripper();
       current_pos = robotController.get_current_position();
       obj.robotController.move_to_positions([[position(1), position(2), obj.UP_level,current ]]);
       up_cube_pos = [cube_pos(1), cube_pos(2), current_pos(3), cu];
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
