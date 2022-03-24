classdef DrawingController
    properties
        robotController = NewRobotController()
        grip_angle = 0
        pen_pick_angle = -pi/2
        close_value = 2450%correct value2600
        open_value = 2000
        lift_height = 16%11
        lower_height = 12%8.8
        steps_per_cm_circle = 2
        pen_pos_upper = [0,6*2.5,16]
        pen_pos_lower = [0,6*2.5,0.5]
        stretch_scale = -5
    end

    methods
        function setup_controller(obj)
            obj.robotController.init();
            obj.robotController.control_mode_setup();
        end

        function close_controller(obj)
            obj.robotController.close();
        end

       function open_gripper(obj)
            obj.robotController.move_servo(5,obj.open_value);
        end

        function close_gripper(obj)
            obj.robotController.move_servo(5,obj.close_value);
        end
        
        function grab_pen(obj)
            % Move to above pen
            % Lower to pen
            % Grab pen
            % Lift pen
            % Rotate pen
            obj.robotController.move_to_positions([[obj.pen_pos_upper obj.pen_pick_angle]]);
            obj.robotController.move_servo(5,obj.open_value);
            pos_array = obj.robotController.trajectory([obj.pen_pos_upper obj.pen_pick_angle],[obj.pen_pos_lower obj.pen_pick_angle]);
            obj.robotController.move_to_positions(pos_array);
            obj.robotController.move_servo(5,obj.close_value);
            pos_array = obj.robotController.trajectory([obj.pen_pos_lower obj.pen_pick_angle],[obj.pen_pos_upper obj.pen_pick_angle]);
            obj.robotController.move_to_positions(pos_array);
            pos_array = obj.robotController.trajectory_angle([obj.pen_pos_upper obj.pen_pick_angle], [obj.pen_pos_upper obj.grip_angle]);
            obj.robotController.move_to_positions(pos_array);
        end

        function draw_line(obj, start_pos, end_pos, continuos_start, continuos_end) % start_pos and end_pos are 2 dimensional
            % Move above draw start
            % Move down to draw start
            % Draw line
            % Lift back up 
            lower_start_pos = [start_pos(1), start_pos(2), obj.lower_height, obj.grip_angle];
            lower_end_pos = [end_pos(1), end_pos(2), obj.lower_height, obj.grip_angle];

            % adjust start and end height
            if continuos_start
                upper_start_pos = [start_pos(1), start_pos(2), obj.lower_height, obj.grip_angle];
            else
                upper_start_pos = [start_pos(1), start_pos(2), obj.lift_height, obj.grip_angle];
            end
            if continuos_end
                upper_end_pos = [end_pos(1), end_pos(2), obj.lower_height, obj.grip_angle];
            else
                upper_end_pos = [end_pos(1), end_pos(2), obj.lift_height, obj.grip_angle];
            end

            
            obj.robotController.move_to_positions([upper_start_pos]);
            drawing_positions = obj.robotController.trajectory(lower_start_pos, lower_end_pos);
            lower_array =  obj.robotController.trajectory(upper_start_pos, lower_start_pos);
            raise_array = obj.robotController.trajectory(lower_end_pos, upper_end_pos);
            positions = [lower_array ; drawing_positions ; raise_array];
            obj.robotController.move_to_positions(positions);
        end

        function draw_circle_segment(obj, center, radius, start_angle, end_angle, continuos_start, continuos_end, clockwise) % center is 2D, start angle and end angle in radians measured clockwise from forward, always draw circle clockwise
            % Move above draw start
            % Move down to draw start
            % Draw segment
            % Lift back up 
            
            start_pos = [center(1)+cos(start_angle)*radius, center(2)+sin(start_angle)*radius];
            end_pos = [center(1)+cos(end_angle)*radius, center(2)+sin(end_angle)*radius];

            if continuos_start
                upper_start_pos = [start_pos(1), start_pos(2), obj.lower_height, obj.grip_angle];
            else
                upper_start_pos = [start_pos(1), start_pos(2), obj.lift_height, obj.grip_angle];
            end

            if continuos_end
                upper_end_pos = [end_pos(1), end_pos(2), obj.lower_height, obj.grip_angle];
            else
                upper_end_pos = [end_pos(1), end_pos(2), obj.lift_height, obj.grip_angle];
            end

            if clockwise == false
                temp = upper_end_pos;
                upper_end_pos = upper_start_pos;
                upper_start_pos = temp;
            end


            if start_angle > end_angle
                start_angle = start_angle - 2*pi;
            end
            delta_angle = mod(end_angle - start_angle, 2*pi);
            num_steps = round(radius * delta_angle * obj.steps_per_cm_circle);


            if mod(num_steps,2) == 1
                num_steps = num_steps + 1;
            end

            if clockwise
                angles = linspace(0, 1, num_steps) ;  
            else
                angles = linspace(1, 0, num_steps) ;  
            end

            angles = angles * delta_angle;
            angles = angles + start_angle;
            positions = polar_to_cartesian(angles, radius) + center;

            %adjust circle
            % find normal
            len = length(positions);
            normal = [center(2), -center(1)];
            normal = normal / norm(normal);
            for i=1:len(1)
                dot_product = (center(1)/center(2)) > (positions(i,1)/positions(i,2));
                numerator = abs((center(1) - 0) * (0 - positions(i,2)) - (0 - positions(i,1)) * (center(2) - 0));
	
                % Find the denominator for our point-to-line distance formula.
                denominator = sqrt((center(1) - 0) ^ 2 + (center(2) - 0) ^ 2);
                
                % Compute the distance.
                distance = numerator ./ denominator;
                if dot_product > 0
                    positions(i,:) = positions(i,:) -  normal*distance / obj.stretch_scale;
                else
                    positions(i,:) = positions(i,:) +  normal*distance / obj.stretch_scale;
                end
            end

%             figure()
%             pos_t = transpose(positions)
%             plot(pos_t(1, :), pos_t(2, :), 'o')
            angle_column = zeros(num_steps,1) + obj.grip_angle;
            height_column = zeros(num_steps,1) + obj.lower_height;
%             positions = [positions height_column angle_column];
%             positions = [[upper_start_pos]; positions; [upper_end_pos]];
%             obj.robotController.move_to_positions([upper_start_pos]);
%             obj.robotController.move_to_positions(positions);

            obj.robotController.move_to_positions([upper_start_pos]);
            drawing_positions = [positions height_column angle_column];
            lower_array =  obj.robotController.trajectory(upper_start_pos, drawing_positions(1,:));
            raise_array = obj.robotController.trajectory(drawing_positions(end,:), upper_end_pos);
            obj.robotController.move_to_positions(lower_array);
            positions = drawing_positions;
%             split up positions into multiple arrays and carry out each one individually
%             cells = [];
%             x = size(positions,1);
%             while x > 5
%                 cells(end+1) = 5;
%                 x = x - 5;
%             end
%             cells(end+1) = x;
%             positions = mat2cell( positions  , cells);
%             len = length(positions);
%             for i=1:len(1)
%                 x = positions(i,:,:);
%                 obj.robotController.move_to_positions(x);
%             end
            obj.robotController.move_to_positions(positions)
            obj.robotController.move_to_positions(raise_array);

        end
    end
end