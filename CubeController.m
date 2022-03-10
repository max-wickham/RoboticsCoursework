classdef CubeController
    properties
        robotController = RobotController()
        open_servo_val = 2000
        closed_servo_val = 2430
        grip_inwards = pi;
        grip_outwards = 0;
        grip_vertical = -pi/2;
        UP_level = 6
        DOWN_level = 3
        grid_to_cm = 2.5
        max_range = 20
        min_range = 8
    end
    methods

        function main_cube(obj,cube_pos_grid, final_pos_grid, flip_angle)
            %***INPUTS****% 
            % cube pos, final pos = x,y in grid coords
            % flip angle = angle diff in rad (inwards from top) 
            % append height and angle to cube and final pos
        
            %***PROCESS***%
            % convert to cm 
            cube_pos = obj.grid_to_cm*cube_pos_grid;
            final_pos = obj.grid_to_cm*final_pos_grid;
    
            % compute flip given two positions (and robot range), 
            % append to goal positions
            gripper_initial_angle, gripper_final_angle, flip_on_spot = obj.set_flip(cube_pos, final_pos, flip_angle);
        
            % reach cube and move, flip if possible
            obj.reach_move_flip(cube_pos, final_pos, gripper_initial_angle, gripper_final_angle);
        
            % rest of flipping performed on the spot
            obj.flip_on_the_spot(final_pos, flip_on_spot);
          
        end
        
        

        function setup_controller(obj)
            obj.robotController.init();
            obj.robotController.control_mode_setup();
        end

        function close_controller(obj)
            obj.robotController.close();
        end
        
        
        %--------------------------%
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
            
            dir_init = obj.turnability(cube_pos);
            dir_final = obj.turnability(final_pos);
            flip_on_spot = 0;
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

        function reach_move_flip(obj,cube_pos, final_pos, gripper_initial_angle, gripper_final_angle)

            %****REACH: open, up, new position and gripper orientation, down, close
            obj.open_gripper();
            current_pos = obj.robotController.get_current_position();
            %up
            obj.robotController.move_to_positions([[current_pos(1), current_pos(2), obj.UP_level, current_pos(4)]]);
            %reach
            up_grip_pos = [cube_pos(1), cube_pos(2), obj.UP_level, gripper_initial_angle];
            pos_array = trajectory(current_pos, up_grip_pos);
            obj.robotController.move_to_positions(pos_array);
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


        function flip_on_the_spot(obj,position, flip_angle)

            %**INPUT**
            %pos is x,y coords
            %flip_angle is amount of flip requested in outwards direction from top
            % 0 deg is red face up, no flip
            % allowed values for flip = 0, 90,180,270
            dir = obj.turnability(position);
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
                obj.open_gripper();
                pos = obj.robotController.get_current_position();
                obj.move_up(pos);
                obj.robotController.move_to_positions([[position(1), position(2), obj.UP_level, pos(4)]]);
                pos = trajectory_angle([position(1), position(2), obj.UP_level, pos(4)],[position(1), position(2), obj.UP_level, gripper_final_angle]);
                obj.robotController.move_to_positions(pos);
                obj.robotController.move_to_positions([[position(1), position(2), obj.DOWN_level, gripper_final_angle]]);
                obj.close_gripper();
                obj.robotController.move_to_positions([[position(1), position(2), obj.UP_level, gripper_final_angle]]);
                if flip_angle == 180 && turn == 1
                    pos = trajectory(obj.robotController.get_current_position(), [position(1), position(2), obj.UP_level, obj.grip_outwards]);
                    obj.robotController.move_to_positions(pos);
                    obj.robotController.move_to_positions([[position(1), position(2), obj.DOWN_level, obj.grip_outwards]]);
                    obj.open_gripper();
                    obj.robotController.move_to_positions([[position(1), position(2), obj.UP_level, obj.grip_outwards]]);
                else
                    pos_current = obj.robotController.get_current_position()
                    pos = trajectory_angle(pos_current,[pos_current(1), pos_current(2), pos_current(3),  obj.grip_vertical]);
                    obj.robotController.move_to_positions(pos);
                    obj.robotController.move_to_positions([[position(1), position(2), obj.DOWN_level, obj.grip_vertical]]);
                    obj.open_gripper();
                    obj.robotController.move_to_positions([[position(1), position(2), obj.UP_level,obj.grip_vertical ]]);
                end
            end 
        end 
        








        % function set_arm_speed(obj, speed, acc)
        %     robotController.set_arm_speed(speed,acc)
        % end

        % function set_gripper_speed(obj,speed)
        %     robotController.set_speed_gripper(speed)
        % end
        
        %given position it returns turning dir=(pos dir allowed, neg dir allowed)
        function dir = turnability(obj, pos)
        %pos is x,y coords
            if norm(pos) < obj.min_range
                dir = [false, true]; %neg allowed
            elseif norm(pos) > obj.max_range
                dir = [true, false]; %pos allowed
            else 
                dir = [false, false]; %in range
            end
        end
        
        function move_up(obj, pos)
            %pos array of 4 coords
            pos(3)= obj.UP_level;
            obj.robotController.move_to_positions([pos]);
        end

        function move_down(obj, pos)
            %pos array of 4 coords
            pos(3)= obj.DOWN_level;
            obj.robotController.move_to_positions([pos]);
        end

        function open_gripper(obj)
            obj.robotController.move_servo(5,obj.open_servo_val);
        end

        function close_gripper(obj)
            obj.robotController.move_servo(5,obj.closed_servo_val);
        end
    end

    methods (Access = private)

        
        
    end
end