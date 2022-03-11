classdef KinematicModel
    properties
        A1 = 13
        A2 = 12.4
        A3 = 14.6
        D0 = 7.7
        Up = true
    end
    methods
        function r = angles(obj, x , y, z, theta)
%             obj.A1 = 13;
%             obj.A2 = 12.4;
%             obj.A3 = 14.6;
%             obj.D0 = 7.7;
            theta123 = theta;
            theta0 = atan2(y,x);
            C0 = cos(theta0);
            S0 = sin(theta0);
            C123 = cos(theta123);
            S123 = sin(theta123);
            if C0 ~= 0
                a =  (x - C0*C123*obj.A3); 
                a = a / C0;
            else
                a =  (y - S0*C123*obj.A3); 
                a = a / S0;
            end
            b = z - S123*obj.A3 - obj.D0; 
            C2 = ( (a^2+b^2) - (obj.A1^2 + obj.A2^2));
            C2 = C2 / (2*obj.A1*obj.A2);
            theta2 = acos(C2);
            theta1 = atan(b/a) - atan(obj.A2*sin(theta2) / (obj.A1 + obj.A2*C2));
            if obj.Up == true
                theta1 = theta1 + theta2;
                theta2 = theta2 * -1;
            end
            theta3 = (theta123 - theta1 - theta2);
            r = [theta0, theta1, theta2, theta3];
        end
        function r = positions(obj, x , y, z, theta)
            angles = obj.angles(x,y,z,theta);
            theta0 = angles(1);
            theta1 = angles(2);
            theta2 = angles(3);
            theta3 = angles(4);
            matrix0 = [1,0,0; 0, cos(theta0), -1*sin(theta0); 0, sin(theta0), cos(theta0)];
            matrix05 = [1,0,0; 0,0,-1; 0,1,0];
            matrix1 = [cos(theta1),-1*sin(theta1),0;sin(theta1),cos(theta1),0;0,0,1];
            matrix2 = [cos(theta2),-1*sin(theta2),0;sin(theta2),cos(theta2),0;0,0,1];
            matrix3 = [cos(theta3),-1*sin(theta3),0;sin(theta3),cos(theta3),0;0,0,1];
            p0 = [0,0,obj.D0];
            p1 = [obj.A1,0,0];
            p2 = [obj.A2,0,0];
            p3 = [obj.A3,0,0];
            pos0 = [0,0,obj.D0];
            pos1 = (matrix0*(matrix05*((matrix1*p1.'))+p0.')).';
            pos2 = (matrix0*(matrix05*((matrix1*(p1.' + matrix2*p2.')))+p0.')).';
            pos3 = (matrix0*(matrix05*((matrix1*(p1.' + matrix2*(p2.'+matrix3*p3.'))))+p0.')).';
            r = [pos0,pos1,pos2,pos3];
        end

        function position = forward(obj, theta0, theta1, theta2, theta3)
            C0 = cos(theta0);
            C1 = cos(theta1);
            C12 = cos(theta1+theta2);
            C123 = cos(theta1+theta2+theta3);
            S0 = sin(theta0);
            S1 = sin(theta1);
            S12 = sin(theta1+theta2);
            S123 = sin(theta1+theta2+theta3);
            x = C0*(C123*obj.A3+C12*obj.A2+C1*obj.A1);
            y = S0*(C123*obj.A3+C12*obj.A2+C1*obj.A1);
            z = S123*obj.A3+S12*obj.A2+S1*obj.A1 + obj.D0;
            gripper_angle = mod(theta1+theta2+theta3+8*pi, 2*pi)
            position = [x,y,z,gripper_angle];
        end
    end
end

