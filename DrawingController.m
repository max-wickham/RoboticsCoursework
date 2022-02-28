classdef CubeController
    properties
        robotController = RobotController()
        grip_angle = 0
        close_value = 0
        open_value = 0
        lift_height = 0
        lower_height = 0
        steps_per_cm_circle = 1
        pen_pos_upper = [0,0,0]
        pen_pos_lower = [0,0,0]
    end

    methods
        function setup_controller(obj)
            obj.robotController.init();
            obj.robotController.control_mode_setup();
        end

        function close_controller(obj)
            obj.robotController.close();
        end

        function grab_pen(obj)
            % Move to above pen
            % Lower to pen
            % Grab pen
            % Lift pen
            robotController.move_to_positions([[pen_pos_upper grip_angle]])
            robotController.move_servo(5,obj.open_value);
            robotController.move_to_positions([[pen_pos_lower grip_angle]])
            robotController.move_servo(5,obj.close_value);
            robotController.move_to_positions([[pen_pos_upper grip_angle]])
        end

        function draw_line(obj, start_pos, end_pos) % start_pos and end_pos are 2 dimensional
            % Move above draw start
            % Move down to draw start
            % Draw line
            % Lift back up 
            upper_start_pos = [start_pos(0), start_pos(1), obj.lift_height, obj.grip_angle]
            lower_start_pos = [start_pos(0), start_pos(1), obj.lower_height, obj.grip_angle]
            lower_end_pos = [end_pos(0), end_pos(1), obj.lower_height, obj.grip_angle]
            upper_end_pos = [end_pos(0), end_pos(1), obj.lift_height, obj.grip_angle]
            drawing_positions = trajectory(lower_start_pos, lower_end_pos)
            positions = [[upper_start_pos] ; drawing_positions ; [upper_end_pos]]
            robotController.move_to_positions(positions)
        end

        function draw_circle_segment(obj, center, radius, start_angle, end_angle) % center is 2D, start angle and end angle in radians measured clockwise from forward, always draw circle clockwise
            % Move above draw start
            % Move down to draw start
            % Draw segment
            % Lift back up  
            upper_start_pos = [start_pos(0), start_pos(1), obj.lift_height, obj.grip_angle]
            upper_end_pos = [end_pos(0), end_pos(1), obj.lift_height, obj.grip_angle]
            if start_angle > end_angle
                start_angle = start_angle - 2*pi
            end
            delta_angle = mod(end_angle - start_angle, 2*pi)
            num_steps = round(radius * delta_angle * obj.steps_per_cm_circle)
            angles = zeros(1,num_steps) * (delta_angle / num_steps)
            angles = cumsum(angles) + start_angle
            positions = polar_to_cartesian(angles, radius) + center
            angle_column = zeros(N,1) + obj.grip_angle;
            height_column = zeros(N,1) + obj.lower_height;
            positions = [positions height_column angle_column]
            positions = [[upper_start_pos]; positions; [upper_end_pos]]
            robotController.move_to_positions(positions)
    end
end