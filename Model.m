classdef Model
    properties
        A1 = 20
        A2 = 20
        A3 = 5
        D0 = 10
        Up = true
    end
    methods
        function obj = BasicClass(a1,a2,a3,d0,up)
            obj.A1 = a1;
            obj.A2 = a2;
            obj.a3 = a3;
            obj.D0 = d0;
            obj.Up = up;
        end
        function r = angles(obj, x , y, z, theta)
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
            %S1 = (b*(obj.A1+obj.A2*C2) - a*obj.A2*(1-C2^2)^0.5);
            %S1 = S1 / (a^2+b^2);
            %theta1 = asin(S1);
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
    end
end

