classdef RobotModel
    properties
        kinematic_model = KinematicModel()
        angle_scalar = [1,-1,-1,-1]
        angle_offset = [0,-0.18356-pi/2,pi/2+0.18356,pi] %radians
        max_servo_val = 4096 %val for 2pi radians
        pos_scalar = [-1,-1,1]
        pos_offset = [0,0,0]
    end
    methods
        function r = servo_vals(obj, pos, theta)
            % Generate the servo values needded to achieve a given position
            new_pos = [0,0,0];
            len = length(new_pos);
            for i=1:len
                new_pos(i) = pos(i) * obj.pos_scalar(i) + obj.pos_offset(i);
            end
            angles = obj.kinematic_model.angles(new_pos(1),new_pos(2),new_pos(3),theta)
            new_servo_vals = [0,0,0,0];
            len = length(new_servo_vals);
            for i=1:len
                new_servo_vals(i) = angles(i) * (obj.angle_scalar(i) * obj.max_servo_val / (2*pi)) + obj.angle_offset(i) * obj.max_servo_val / (2*pi) %convert from rad to servo val
                new_servo_vals(i) = mod((new_servo_vals(i)+4096), 4096) %remove neg
            end
            r = new_servo_vals;
        end

        function position = current_position(obj, servo_vals)
            % Get the current position given servo vals
            angles =[0,0,0,0]
            for i=1:4
                angles(i) = (servo_vals(i) - obj.angle_offset(i) * obj.max_servo_val / (2*pi)) /  (obj.angle_scalar(i) * obj.max_servo_val / (2*pi));
            end
            position = kinematic_model.forward(angles(1),angles(2),angles(3),angles(4));
        end
    end
end



