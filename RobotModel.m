classdef RobotModel
    properties
        kinematic_model = KinematicModel()
        angle_scalar = [1,1,1,1]
        angle_offset = [0,0,0,0]
        max_servo_val = 2000
        pos_scalar = [1,-1,1]
        pos_offset = [0,0,0]
    end
    methods
        function r = servo_vals(obj, pos, theta)
            % Generate the servo values needded to achieve a given position
            new_pos=[0,0,0];
            len =length(new_pos);
            for i=1:len
                new_pos(i) = pos(i) * obj.pos_scalar(i) + obj.pos_offset(i);
            end
            angles = obj.kinematic_model.angles(new_pos(1),new_pos(2),new_pos(2),theta);
            new_servo_vals = [0,0,0,0];
            len =length(new_servo_vals);
            for i=1:len
                new_servo_vals(i) = angles(i) * obj.angle_scalar(i) + obj.angle_offset(i);
            end
            r = new_servo_vals;
        end
    end
end

