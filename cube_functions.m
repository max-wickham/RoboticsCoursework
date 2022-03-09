% *********Functions for task 2 moving cubes **********

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
            gripper_init1, gripper_final1, flip_on_spot1 = set_flip(cube_pos, final_pos, 90);
            gripper_init2, gripper_final2, flip_on_spot2 = set_flip(cube_pos, final_pos, 270);
            if flip_on_spot1 == 0 %can do at least 90
                gripper_initial_angle = gripper_init1;
                gripper_final_angle = gripper_final1;
                flip_on_spot = 90;
            else % can do at least 270
                gripper_initial_angle = gripper_init2;
                gripper_final_angle = gripper_final2;
                flip_on_spot = 270;               
            end
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


%********************************REACH, MOVE and FLIP 

function reach_move_flip(obj,cube_pos, final_pos, gripper_initial_angle, gripper_final_angle)

       %****REACH: open, up, new position and gripper orientation, down, close
       obj.open_gripper();
       current_pos = obj.robotController.get_current_position();
       %up
       obj.robotController.move_to_positions([[current_pos(1), current_pos(2), obj.UP_level, current_pos(4)]]);
       %reach
       up_grip_pos = [cube_pos(1), cube_pos(2), obj.UP_level, gripper_initial_angle];
       pos_array = trajectory(current_pos, up_grip_pos);
       controller.move_to_positions(pos_array);
       %down
       obj.robotController.move_to_positions([[cube_pos(1), cube_pos(2), obj.DOWN_level, gripper_initial_angle]]);
       obj.close_gripper();
       
       %****MOVE CUBE: up, new position and orientation, down, open, up
       %up
       current_pos = obj.robotController.get_current_position();
       obj.robotController.move_to_positions([[current_pos(1), current_pos(2), obj.UP_level, current_pos(4)]]);
       %move
       up_cube_pos = [final_pos(1), final_pos(2), obj.UP_level, gripper_final_angle];
       pos_array = trajectory(current_pos, up_cube_pos);
       controller.move_to_positions(pos_array);
       %down
       obj.robotController.move_to_positions([[final_pos(1), final_pos(2), obj.DOWN_level, gripper_initial_angle]]);
       obj.open_gripper();
       %up
       current_pos = obj.robotController.get_current_position();
       obj.robotController.move_to_positions([[current_pos(1), current_pos(2), obj.UP_level, current_pos(4)]]);
       
end  

%*******************FLIP ON THE SPOT
function flip_on_the_spot(obj,position, flip_angle)

    %**INPUT**
    %pos is x,y coords
    %flip_angle is amount of flip requested in outwards direction from top
    % 0 deg is red face up, no flip
    % allowed values for flip = 0, 90,180,270
    dir = turnability(position);
    %********compute flipping mode
    %turn = flip_angle/90;
    if flip_angle < 180 %neg direction
        if dir(2) % neg allowed
            gripper_final_angle = obj.grip_inwards;
            turn = 1;
        else
            gripper_final_angle = obj.grip_outwards;
            turn = 3;
        end
    elseif flip_angle == 180 %180°
        if dir(2) && dir(1) % neg allowed
            gripper_final_angle = obj.grip_inwards;
            turn = 1;
        elseif dir(2)
            gripper_final_angle = obj.grip_inwards;
            turn = 2;
        else
            gripper_final_angle = obj.grip_outwards;
            turn = 2;   
        end
    elseif flip_angle > 180 %pos direction
        if dir(1) % neg allowed
            gripper_final_angle = obj.grip_outwards;
            turn = 1;
        else
            gripper_final_angle = obj.grip_inwards;
            turn = 3;
        end
    end
    %********actual flipping
    %open and go up, turn gripper, down, close, go up, turn, down, open,
    %up
    for i=1:turn
        open_gripper(obj);
        obj.move_up(obj.robotController.get_current_position());
        obj.robotController.move_to_positions([[position(1), position(2), obj.UP_level, gripper_final_angle]]);
        obj.robotController.move_to_positions([[position(1), position(2), obj.DOWN_level, gripper_final_angle]]);
        obj.close_gripper();
        obj.robotController.move_to_positions([[position(1), position(2), obj.UP_level, gripper_final_angle]]);
        if flip_angle == 180 && turn == 1
            obj.robotController.move_to_positions([[position(1), position(2), current_pos(3), obj.grip_outwards]]);
            obj.robotController.move_to_positions([[position(1), position(2), obj.DOWN_level, obj.grip_outwards]]);
            obj.open_gripper();
            obj.robotController.move_to_positions([[position(1), position(2), obj.UP_level, obj.grip_outwards]]);
        else
            obj.robotController.move_to_positions([[position(1), position(2), current_pos(3), obj.grip_vertical]]);
            obj.robotController.move_to_positions([[position(1), position(2), obj.DOWN_level, obj.grip_outwards]]);
            obj.open_gripper();
            obj.robotController.move_to_positions([[position(1), position(2), obj.UP_level, gripper_outwards]]);
        end
    end 
end 


        
        
        
        
 % ignore below
 
% %********************************REACH    
% 
% function reach_cube(obj,gripper_angle_init)
%     open_gripper(obj);
%     move_and_flip();
%     close_gripper(obj);
% end
% 
% %********************************MOVE
% 
% function move_cube(obj,gripper_angle_final)
%     close_gripper(obj);
%     move_and_flip()
%     open_gripper(obj);
% end

