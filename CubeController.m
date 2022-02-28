classdef CubeController
    properties
        robotController = RobotController()
        open_servo_val = 2000
        closed_servo_val = 2430
        up_vertical_height = 0
        down_vertical_height = 0
        up_horizontal_height = 0
        down_horizontal_height = 0
    end
    methods
        function set_arm_speed(obj, speed, acc)
            robotController.set_arm_speed(speed,acc)
        end

        function set_gripper_speed(obj,speed)
            robotController.set_speed_gripper(speed)
        end

        function up_vertical_polar(obj, theta, distance)
            obj.go_to_pos(theta,distance,-pi/2,obj.up_vertical_height)
        end

        function down_vertical_polar(obj, theta, distance)
            obj.go_to_pos(theta,distance,-pi/2,obj.down_vertical_height)
        end

        function up_horizontal_polar(obj, theta, distance)
            obj.go_to_pos(theta,distance,0,obj.up_horizontal_height)
        end

        function down_horizontal_polar(obj, theta, distance)
            obj.go_to_pos(theta,distance,0,obj.down_horizontal_height)
        end

        function open(obj)
            robotController.move_servo(5,obj.open_servo_val);
        end

        function close(obj)
            robotController.move_servo(5,obj.closed_servo_val);
        end
    end

    methods (Access = private)
        function pos = polar_to_cartesian(obj, theta, distance)
            x = sin(theta)*distance
            y = cos(theta)*distance
            pos = [x,y]
        end

        function go_to_pos(obj, theta, postiion, angle, height)
            pos = obj.polar_to_cartesian(theta,distance)
            end_pos = [pos[1],pos[2],height,angle]
            obj.robotController.move_to_positions([end_pos])
        end
    end
end