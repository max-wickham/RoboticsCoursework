classdef CubeController
    properties
        robotController = RobotController()
        open_servo_val = 2000
        closed_servo_val = 2430
        grip_inwards = pi;
        grip_outwards = 0;
        grip_vertical = -pi/2;
        UP_level = 0
        DOWN_level = 0
        grid_to_cm = 2.5
    end
    methods

        function main_cube(obj,cube_pos_grid, final_pos_grid, flip_angle)
            %***INPUTS****% 
            % cube pos, final pos = x,y in grid coords
            % flip angle = angle diff in rad (inwards from top) 
            % append height and angle to cube and final pos
            
            %***PROCESS***%
            %  convert to cm 
            cube_pos = cube_pos_grid*grid_to_cm;
            final_pos = final_pos_grid*grid_to_cm;
            
            % compute flip given two positions (and robot range), 
            % append to goal positions
            flip = set_flip(cube_pos, final_pos, flip_angle);
            
            % reach_cube
            reach_cube();
            
            % move cube
            move_cube();
                
        end

    end

    methods (Access = private)
        function setup_controller(obj)
            obj.robotController.init();
            obj.robotController.control_mode_setup();
        end

        function close_controller(obj)
            obj.robotController.close();
        end

        function set_arm_speed(obj, speed, acc)
            robotController.set_arm_speed(speed,acc)
        end

        function set_gripper_speed(obj,speed)
            robotController.set_speed_gripper(speed)
        end
        
        %given position it returns turning dir=(pos dir allowed, neg dir allowed)
        function dir = turnability(pos)
        %pos is x,y coords
            if norm(pos) < min_range
                dir = [false, true]; %neg allowed
            elseif norm(pos) > max_range
                dir = [true, false]; %pos allowed
            else 
                dir = [true, true]; %in range
            end
        end
        
        function move_up(obj, pos)
            %pos array of 4 coords
            pos(3)= UP_level;
            robotController.move_to_positions([pos]);
        end
        function move_down(obj, pos)
            %pos array of 4 coords
            pos(3)= DOWN_level;
            robotController.move_to_positions([pos]);
        end
        function open_gripper(obj)
            robotController.move_servo(5,obj.open_servo_val);
        end

        function close_gripper(obj)
            robotController.move_servo(5,obj.closed_servo_val);
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
    end
end

%%%%%%%%%%%%%%% Example usage to rotate a cube %%%%%%%%%%%%%%%

% cubeController = CubeController()
% cubeController.setup_controller()

% cubeController.up_vertical_polar(1.4, 15) 
% cubeController.open_gripper()
% cubeController.down_vertical_polar(1.4, 15)
% cubeController.close_gripper()
% cubeController.up_vertical_polar(1.4, 15)
% cubeController.up_horizontal_polar(1.4, 15)
% cubeController.down_horizontal_polar(1.4, 15)
% cubeController.open_gripper()

% cubeController.close_controller()