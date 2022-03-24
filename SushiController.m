% may need speed to be implemented


classdef SushiController
    properties
        robotController = SushiRobotController()

        stick_pos = [22.5,0] % current stick position need to be calebrated
        steps_per_radian_circle_move = 10
        table_x_pos = 15
        stick_radius = 7.5
        stick_down_height = 3.6
        stick_up_height = 10
        stick_gripper_open_val = 2000
        stick_gripper_close_val = 2350

        rice_up_height = 10
        rice_down_height = -2
        rice_gripper_open_val =1800
        rice_gripper_close_val = 2370
        plate_rice_up_height = 10
        plate_rice_down_height = 0.5
        
        %width 2.7, 3.5 height ish vat kind of region
        weed_up_height = 15
        weed_down_height = 6.3%6.2
        weed_gripper_open_val = 1400
        weed_gripper_close_val = 1800
        grip_horizontal = -0.02;
        plate_weed_up_height = 15
        plate_weed_down_height = 11
        % -- ?
        weed_gripper_offsets = [4, 0,6]; %different tip of the gripper, offset w.r.t horizontal grip pos

        salmon_up_height = 15
        salmon_gripper_open_val = 1500
        salmon_gripper_close_val = 1850
        plate_salmon_up_height = 13
        plate_salmon_down_height = 11
        % -- ?
        salmon_gripper_offsets = [4, 0, 6];
        
        sushi_up_height = 10
        sushi_down_height = 2.5
        sushi_gripper_open_val =1300
        sushi_gripper_close_val = 2370
        plate_sushi_up_height = 10
        plate_sushi_down_height = 1

    end
    methods
        
        function setup_controller(obj)
            obj.robotController.init();
            obj.robotController.control_mode_setup();
        end

        function close_controller(obj)
            obj.robotController.close();
        end

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
    methods

        function turn_table(obj, angle, stick_pos)
            % angle = angle in radians clockwise from positive x axis
                % calculate the array of positions
            obj.stick_pos = stick_pos;
            % move gripper to above move stick
            pos = [obj.stick_pos(1), obj.stick_pos(2), obj.stick_up_height, -pi/2]     
            current_pos = obj.robotController.get_current_position()
            obj.robotController.move_to_positions(obj.robotController.trajectory_angle(current_pos,[current_pos(1),current_pos(2),current_pos(3),-pi/2]))
            obj.robotController.move_to_positions(obj.robotController.trajectory([current_pos(1),current_pos(2),current_pos(3),-pi/2],pos))
            % open gripper
            obj.robotController.move_servo(5,obj.stick_gripper_open_val);       
            % move gripper down
            pos = [obj.stick_pos(1), obj.stick_pos(2), obj.stick_down_height, -pi/2]                                           
            obj.robotController.move_to_positions(obj.robotController.trajectory(obj.robotController.get_current_position(),pos));
            % close gripper
            obj.robotController.move_servo(5,obj.stick_gripper_close_val);
%             % move grippper in arc
            current_angle = obj.current_table_angle()
            delta_angle = mod(angle - current_angle + 2*pi,2*pi)
            num_steps = delta_angle * obj.steps_per_radian_circle_move

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
                pos = [obj.table_x_pos + cos(angle)*obj.stick_radius, sin(angle)*obj.stick_radius, obj.stick_down_height, -pi/2]
                pos_array = [ pos_array ; pos];
                
            end
%             figure()
%             pos_t = transpose(pos_array)
%             plot(pos_t(1, :), pos_t(2, :), 'o')
            obj.robotController.move_to_positions(pos_array)                                                              
            % update the current stick position
            pos = obj.robotController.get_current_position()
            obj.stick_pos = [pos(1),pos(2)]
            % open gripper
            obj.robotController.move_servo(5,obj.stick_gripper_open_val);
            %move gripper up
            len = length(pos_array)
            obj.robotController.move_to_positions(obj.robotController.trajectory(obj.robotController.get_current_position(),[pos_array(len(1),1),pos_array(len(1),2),obj.stick_up_height, -pi/2]));
        end

        function grab_rice(obj, pos)
            % pos = (x,y)
            % move to pos above rice
            current_pos = obj.robotController.get_current_position();
            pos = [pos(1), pos(2), obj.rice_up_height, -pi/2];                                           
            % robotController.move_to_positions(obj.robotController.trajectory(current_pos,pos));
            obj.robotController.move_to_positions([pos]);
            % open gripper
            obj.robotController.move_servo(5,obj.rice_gripper_open_val);   
            % move down
            current_pos = obj.robotController.get_current_position();
            pos = [pos(1), pos(2), obj.rice_down_height, -pi/2];                                           
            % robotController.move_to_positions([pos]);
            obj.robotController.move_to_positions(obj.robotController.trajectory(current_pos,pos));
            % close gripper
            obj.robotController.move_servo(5,obj.rice_gripper_close_val);   
            % move up
            pos = [pos(1), pos(2), obj.rice_up_height, -pi/2];           
            current_pos = obj.robotController.get_current_position();                             
            % robotController.move_to_positions([pos]);
            obj.robotController.move_to_positions(obj.robotController.trajectory(current_pos,pos));
        end

        function place_rice(obj, pos)
            % pos = (x,y)
            % calculate new pos given rotation
            new_pos = pos;%obj.calculate_adjusted_pos(pos,obj.current_table_angle());
            % move gripper above pos
            % current_pos = robotController.get_current_position();
            pos = [new_pos(1), new_pos(2), obj.plate_rice_up_height, -pi/2];                                        
            obj.robotController.move_to_positions([pos]);
            % robotController.move_to_positions(trajectory(current_pos,pos));
            % move gripper down
            pos = [new_pos(1), new_pos(2), obj.plate_rice_down_height, -pi/2];  
            current_pos = obj.robotController.get_current_position();                                   
            % robotController.move_to_positions([pos]);
            obj.robotController.move_to_positions(obj.robotController.trajectory(current_pos,pos));
            % open gripper
            obj.robotController.move_servo(5,obj.rice_gripper_open_val);  
            % move gripper up
            pos = [new_pos(1), new_pos(2), obj.plate_rice_up_height, -pi/2];  
            current_pos = obj.robotController.get_current_position();                                         
            % robotController.move_to_positions([pos]);
            obj.robotController.move_to_positions(obj.robotController.trajectory(current_pos,pos));
        end

        function grab_salmon(obj,pos)
            % pos = (x,y,z) position of salmon + offset new gripper
            %----? 
            place_pos = pos ;
            %----?
            current_pos = obj.robotController.get_current_position();
            pos_up = [pos(1), pos(2), obj.salmon_up_height, -pi/2];                                           
            obj.robotController.move_to_positions([pos_up]);
            obj.robotController.move_to_positions(obj.robotController.trajectory_angle(pos_up,[pos(1), pos(2), obj.salmon_up_height, obj.grip_horizontal] ));
            % open gripper
            obj.robotController.move_servo(5,obj.salmon_gripper_open_val);
            % move down
            current_pos = obj.robotController.get_current_position();
            salmon_place = [place_pos , obj.grip_horizontal];
            obj.robotController.move_to_positions(obj.robotController.trajectory(current_pos,salmon_place));
            % close gripper
            obj.robotController.move_servo(5,obj.salmon_gripper_close_val);   
            % pos = (x,y,z) position of salmon
            pos_up = [pos(1), pos(2), obj.salmon_up_height,  obj.grip_horizontal]; 
            current_pos = obj.robotController.get_current_position();                                          
            % robotController.move_to_positions([pos_up]);
            obj.robotController.move_to_positions(obj.robotController.trajectory(current_pos,pos_up));
        end

        function place_salmon(obj,pos)
            % pos = (x,y)
            % calculate new pos given rotation
            %----?  new gripper offsets
            new_pos = pos;%obj.calculate_adjusted_pos(pos);
            %----?
            % move gripper above pos
            current_pos = obj.robotController.get_current_position();
            pos = [new_pos(1), new_pos(2), obj.plate_salmon_up_height, current_pos(4)];                                        
            % robotController.move_to_positions([pos]);
            obj.robotController.move_to_positions([pos]);
            % move gripper down
            pos = [new_pos(1), new_pos(2), obj.plate_salmon_down_height, pos(4)];   
            current_pos = obj.robotController.get_current_position();                                  
            % robotController.move_to_positions([pos]);
            obj.robotController.move_to_positions(obj.robotController.trajectory(current_pos,pos));
            % open gripper
            obj.robotController.move_servo(5,obj.salmon_gripper_open_val);  
            % move gripper up
            pos = [new_pos(1), new_pos(2), obj.plate_salmon_up_height , pos(4)];  
            current_pos = obj.robotController.get_current_position();                                           
            % robotController.move_to_positions([pos]);
            obj.robotController.move_to_positions(obj.robotController.trajectory(current_pos,pos));
            current_pos = obj.robotController.get_current_position(); 
            obj.robotController.move_to_positions(obj.robotController.trajectory_angle(current_pos,[current_pos(1), current_pos(2), current_pos(3), -pi/2] ));
            
        end

        function grab_weed(obj,pos)
            % pos = (x,y,z) position of salmon + offset new gripper
            %----?
            place_pos = pos;
            %----?
            current_pos = obj.robotController.get_current_position();
            pos_up = [pos(1), pos(2), obj.weed_up_height, -pi/2];                                           
            obj.robotController.move_to_positions([pos_up]);
            obj.robotController.move_to_positions(obj.robotController.trajectory_angle(pos_up,[pos(1), pos(2), obj.weed_up_height, obj.grip_horizontal] ));
            % open gripper
            obj.robotController.move_servo(5,obj.weed_gripper_open_val);
            % move down
            current_pos = obj.robotController.get_current_position();
            pos_down = [place_pos(1), place_pos(2), obj.weed_down_height, obj.grip_horizontal];
            obj.robotController.move_to_positions(obj.robotController.trajectory(current_pos,pos_down));
            % close gripper
            obj.robotController.move_servo(5,obj.weed_gripper_close_val);   
            % pos = (x,y,z) position of salmon
            current_pos = obj.robotController.get_current_position();
            pos_up = [current_pos(1), current_pos(2), obj.weed_up_height,  obj.grip_horizontal]; 
            current_pos = obj.robotController.get_current_position();                                          
            % robotController.move_to_positions([pos_up]);
            obj.robotController.move_to_positions(obj.robotController.trajectory(current_pos,pos_up));
        end
         
        function place_weed(obj,pos)
            % pos = (x,y)
            % calculate new pos given rotation
            %----?  new gripper offsets
            new_pos = pos;%obj.calculate_adjusted_pos(pos);
            %----?
            % move gripper above pos
            current_pos = obj.robotController.get_current_position();
            pos = [new_pos(1), new_pos(2), obj.plate_weed_up_height, current_pos(4)];                                        
            % robotController.move_to_positions([pos]);
            obj.robotController.move_to_positions([pos]);
            % move gripper down
            pos = [new_pos(1), new_pos(2), obj.plate_weed_down_height, pos(4)];   
            current_pos = obj.robotController.get_current_position();                                  
            % robotController.move_to_positions([pos]);
            obj.robotController.move_to_positions(obj.robotController.trajectory(current_pos,pos));
            % open gripper
            obj.robotController.move_servo(5,obj.weed_gripper_open_val);  
            % move gripper up
            pos = [new_pos(1), new_pos(2), obj.plate_weed_up_height , pos(4)];  
            current_pos = obj.robotController.get_current_position();                                           
            % robotController.move_to_positions([pos]);
            obj.robotController.move_to_positions(obj.robotController.trajectory(current_pos,pos));
        end
        
        function theta = calculate_table_turn(obj, pos)
            % pos = (x,y)
            % theta = radians clockwise
            r = norm(pos);
            theta = atan(r / obj.table_x_pos);
        end

        function new_pos = calculate_adjusted_pos(obj, pos, angle)
            % pos = (x,y)
            % new_pos = (x,y)
            % angle = obj.calculate_table_turn(pos)
            theta = pi/2 + angle * -1
            rotation = [cos(theta),-1* sin(theta) ; sin(theta),cos(theta)]
            new_pos = [pos(1) - obj.table_x_pos,pos(2)]
            new_pos = transpose(new_pos)
            new_pos = rotation * new_pos
            new_pos = [new_pos(1) + obj.table_x_pos, new_pos(2)]
        end

        function theta = current_table_angle(obj)
            % theta clockwise radians 
            x = obj.stick_pos(1) - obj.table_x_pos;
            y = obj.stick_pos(2);
            theta = mod(atan(y/x) + 2*pi,2*pi);
        end
        
        function move_sushi(obj, pos_grab, pos_place)
            %----GRAB FROM ROTATING PLATE
            % pos = (x,y)
            % move to pos above rice
            current_pos = obj.robotController.get_current_position();
            pos = [pos_grab(1), pos_grab(2), obj.sushi_up_height, -pi/2];                                           
            % robotController.move_to_positions(obj.robotController.trajectory(current_pos,pos));
            obj.robotController.move_to_positions([pos]);
            % open gripper
            obj.robotController.move_servo(5,obj.sushi_gripper_open_val);   
            % move down
            current_pos = obj.robotController.get_current_position();
            pos = [pos(1), pos(2), obj.sushi_down_height, -pi/2];                                           
            % robotController.move_to_positions([pos]);
            obj.robotController.move_to_positions(obj.robotController.trajectory(current_pos,pos));
            % close gripper
            obj.robotController.move_servo(5,obj.sushi_gripper_close_val);
            obj.robotController.move_servo(5,obj.sushi_gripper_open_val);
            % move up
            pos = [pos(1), pos(2), obj.sushi_up_height, -pi/2];           
            current_pos = obj.robotController.get_current_position();                             
            % robotController.move_to_positions([pos]);
            obj.robotController.move_to_positions(obj.robotController.trajectory(current_pos,pos));
%             
%             %----PLACE TO SERVING PLATE
%             
%             % current_pos = robotController.get_current_position();
%             pos = [pos_place(1), pos_place(2), obj.plate_sushi_up_height, -pi/2];                                        
%             obj.robotController.move_to_positions([pos]);
%             % robotController.move_to_positions(trajectory(current_pos,pos));
%             % move gripper down
%             pos = [pos_place(1), pos_place(2), obj.plate_sushi_down_height, -pi/2];  
%             current_pos = obj.robotController.get_current_position();                                   
%             % robotController.move_to_positions([pos]);
%             obj.robotController.move_to_positions(obj.robotController.trajectory(current_pos,pos));
%             % open gripper
%             obj.robotController.move_servo(5,obj.sushi_gripper_open_val);  
%             % move gripper up
%             pos = [pos_place(1), pos_place(2), obj.plate_sushi_up_height, -pi/2];  
%             current_pos = obj.robotController.get_current_position();                                         
%             % robotController.move_to_positions([pos]);
%             obj.robotController.move_to_positions(obj.robotController.trajectory(current_pos,pos));
        end 

    end

end

