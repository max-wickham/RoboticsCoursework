% may need speed to be implemented


classdef SushiController
    properties
        robotController = RobotController()

        stick_pos = [0,0] % current stick position need to be calebrated
        steps_per_radian_circle_move = 10
        table_x_pos = 10
        stick_radius = 10
        stick_down_height = 10
        stick_up_height = 10
        stick_gripper_open_val = 0
        stick_gripper_close_val = 0

        rice_up_height = 10
        rice_down_height = 10
        rice_gripper_open_val = 10
        rice_gripper_close_val = 10
        plate_rice_up_height = 10
        plate_rice_down_height = 10

        salmon_up_height = 10
        salmon_gripper_open_val = 0
        salmon_gripper_close_val = 0
        plate_salmon_up_height = 10
        plate_salmon_down_height = 10


    end
    methods

        function create_rice_block(obj, pos_plate, pos_bucket)
            % pos_plate = (x,y) coordinate to place on plate
            % pos_bucket = (x,y) coordinate to pick from bucket

            % turn table to new angle
            table_angle = calculate_table_turn(pos_plate)
            obj.turn_table(table_angle)
            % pick up rice from bucket
            obj.grab_rice(pos_bucket)
            % place rice on table
            obj.place_rice(new_plate_pos)
            % tur the table back
            obj.turn_table(2*pi - table_angle)
        end

    end
    methods (Access = private)

        function turn_table(obj, angle)
            % angle = angle in radians clockwise from positive x axis
                % calculate the array of positions

            % move gripper to above move stick
            pos = [stick_pos(1), stick_pos(2), obj.stick_up_height, -pi/2]                                           
            robotController.move_to_positions([pos])
            % open gripper
            robotController.move_servo(5,obj.stick_gripper_open_val);       
            % move gripper down
            pos = [stick_pos(1), stick_pos(2), obj.stick_down_height, -pi/2]                                           
            robotController.move_to_positions([pos])
            % close gripper
            robotController.move_servo(5,obj.stick_gripper_close_val);
            % move grippper in arc
            current_angle = obj.current_table_angle()
            delta_angle = mod(angle - current_angle + 2*pi,2*pi)
            num_steps = delta_angle / obj.steps_per_radian_circle_move

            % choose correct direction 
            diff = mod(angle - current_angle + 2*pi, 2*pi)
            if diff < pi
                % clockwise (angle is less than pi ahead of current_angle)
                if current_angle > angle
                    % if current_angle left of 0 and angle right of 0, make angle bigger than current angle
                    angle = angle + 2*pi
                end
            else
                % anticlockwise (angle is more than pi ahead of current_angle)
                if current_angle < angle
                    % if angle left of 0 and current_angle right of 0, make angle bigger than current angle
                    current_angle = current_angle + 2*pi
                end
            end
            angles_array = linspace(current_angle, angle, num_steps);

            pos_array = []
            for angle = angles_array
                % ensure angle is in range 0-2*pi
                angle = mod(angle + 2*pi, 2*pi)
                pos = [table_x_pos + sin(angle)*obj.stick_radius, cos(angle)*obj.stick_radius, obj.stick_down_height, -pi/2]
                append(pos_array,pos)
            end
            robotController.move_to_positions(pos_array)                                                              
            % update the current stick position
            pos = robotController.get_current_position()
            stick_pos = [pos(1),pos(2)]
            % open gripper
            robotController.move_servo(5,obj.stick_gripper_open_val);
            %move gripper up
            robotController.move_to_positions(pos_array(end,1),pos_array(end,2),obj.stick_up_height, -pi/2)
        end

        function grab_rice(obj, pos)
            % pos = (x,y)
            % move to pos above rice
            current_pos = robotController.get_current_position()
            pos = [pos(1), pos(2), obj.rice_up_height, -pi/2];                                           
            robotController.move_to_positions(trajectory(current_pos,pos));
            % open gripper
            robotController.move_servo(5,obj.rice_gripper_open_val);   
            % move down
            pos = [pos(1), pos(2), obj.rice_down_height, -pi/2];                                           
            robotController.move_to_positions([pos]);
            % close gripper
            robotController.move_servo(5,obj.rice_gripper_close_val);   
            % move up
            pos = [pos(1), pos(2), obj.rice_up_height, -pi/2];                                        
            robotController.move_to_positions([pos]);
        end

        function place_rice(obj, pos)
            % pos = (x,y)
            % calculate new pos given rotation
            new_pos = obj.calculate_adjusted_pos(pos);
            % move gripper above pos
            current_pos = robotController.get_current_position();
            pos = [new_pos(1), new_pos(2), obj.plate_rice_up_height, -pi/2];                                        
            robotController.move_to_positions([pos]);
            robotController.move_to_positions(trajectory(current_pos,pos));
            % move gripper down
            pos = [new_pos(1), new_pos(2), obj.plate_rice_down_height, -pi/2];                                     
            robotController.move_to_positions([pos]);
            % open gripper
            robotController.move_servo(5,obj.rice_gripper_open_val);  
            % move gripper up
            pos = [new_pos(1), new_pos(2), obj.plate_rice_up_height, -pi/2]  ;                                         
            robotController.move_to_positions([pos]);
        end

        function pick_salmon(obj,pos)
            % pos = (x,y,z) position of salmon
            place_pos = pos;
            current_pos = robotController.get_current_position();
            pos_up = [pos(1), pos(2), obj.salmon_up_height, -pi/2];                                           
            robotController.move_to_positions(trajectory(current_pos,pos_up));
            % open gripper
            robotController.move_servo(5,obj.salmon_gripper_open_val);   
            % move down
            robotController.move_to_positions([place_pos]);
            % close gripper
            robotController.move_servo(5,obj.rice_gripper_close_val);   
            % pos = (x,y,z) position of salmon
            pos_up = [pos(1), pos(2), obj.salmon_gripper_close_val, -pi/2];                                           
            robotController.move_to_positions([pos_up]);
        end

        function place_salmon(obj,pos)
            % pos = (x,y)
            % calculate new pos given rotation
            new_pos = obj.calculate_adjusted_pos(pos);
            % move gripper above pos
            current_pos = robotController.get_current_position();
            pos = [new_pos(1), new_pos(2), obj.plate_salmon_up_height, -pi/2];                                        
            robotController.move_to_positions([pos]);
            robotController.move_to_positions(trajectory(current_pos,pos));
            % move gripper down
            pos = [new_pos(1), new_pos(2), obj.plate_salmon_down_height, -pi/2];                                     
            robotController.move_to_positions([pos]);
            % open gripper
            robotController.move_servo(5,obj.salmon_gripper_open_val);  
            % move gripper up
            pos = [new_pos(1), new_pos(2), obj.plate_salmon_up_height , -pi/2]  ;                                         
            robotController.move_to_positions([pos]);
        end

        function theta = calculate_table_turn(obj, pos)
            % pos = (x,y)
            % theta = radians clockwise
            r = pos(1);
            theta = atan(r / obj.table_x_pos);
        end

        function new_pos = calculate_adjusted_pos(obj, pos, angle)
            % pos = (x,y)
            % new_pos = (x,y)
            % angle = obj.calculate_table_turn(pos)
            theta = pi/2 + angle * -1
            rotation = [cos(theta),-1* sin(theta) ; sin(theta),cos(theta)]
            new_pos = [pos(1) - obj.table_x_pos,pos(2)]
            new_pos = rotation * new_pos
            new_pos = [new_pos(1) + obj.table_x_pos, new_pos(2)]
        end

        function theta = current_table_angle(obj)
            % theta clockwise radians 
            x = obj.stick_pos(1) - obj.table_x_pos;
            y = x = obj.stick_pos(2);
            theta = mod(tan(y/x) + 2*pi,2*pi);
        end


    end

end

