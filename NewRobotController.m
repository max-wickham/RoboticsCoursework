%%%%%%%% setup 
% robotController = RobotController()
% robotController.init()
%%%%%%%% usage 
% robotController.move_servo(index, val)
% robotController.move_to_positions(positions)
% position = robotController.get_current_position()
%%%%%%%% clean up
% robotController.close()
classdef NewRobotController
    properties
        robot_model = RobotModel()

        port_num = 0
        lib_name = ''
        max_angle_error = 50%100

        %% ---- Control Table Addresses ---- %%

        ADDR_PRO_TORQUE_ENABLE       = 64;           % Control table address is different in Dynamixel model
        ADDR_PRO_GOAL_POSITION       = 116; 
        ADDR_PRO_PRESENT_POSITION    = 132; 
        ADDR_PRO_OPERATING_MODE      = 11;
        ADDR_PROFILE_VELOCITY            = 112;

        %% ---- Other Settings ---- %%

        % Protocol version
        PROTOCOL_VERSION            = 2.0;          % See which protocol version is used in the Dynamixel

        % Default setting
        DXL_ID                       = [11,12,13,14,15];
        DXL_ID1                      = 11;            % Dynamixel ID: BASE
        DXL_ID2                      = 12;            % Dynamixel ID: SHOULDER
        DXL_ID3                      = 13;            % Dynamixel ID: ELBOW
        DXL_ID4                      = 14;            % Dynamixel ID: WRIST
        DXL_ID5                      = 15;            % Dynamixel ID: HAND
        BAUDRATE                    = 115200;
        DEVICENAME                  = 'COM10';       % Check which port is being used on your controller
                                                    % ex) Windows: 'COM1'   Linux: '/dev/ttyUSB0' Mac: '/dev/tty.usbserial-*'
                                                    
        TORQUE_ENABLE               = 1;            % Value for enabling the torque
        TORQUE_DISABLE              = 0;            % Value for disabling the torque
        DXL_MINIMUM_POSITION_VALUE  = -150000;      % Dynamixel will rotate between this value
        DXL_MAXIMUM_POSITION_VALUE  = 150000;       % and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
        DXL_MOVING_STATUS_THRESHOLD = 20;           % Dynamixel moving status threshold

        ESC_CHARACTER               = 'e';          % Key for escaping loop

        COMM_SUCCESS                = 0;            % Communication Success result value
        COMM_TX_FAIL                = -1001;        % Communication Tx Failed

        SPEED_VAL                   = 5000;
        %% ------------------ %%
    end
    methods

        function readPID(obj)
            moving_threshold = read4ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, obj.DXL_ID(1), 24); % default  10
            pos_p = read2ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, obj.DXL_ID(1), 80); % 0
            velocity_p = read2ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, obj.DXL_ID(1), 76); % 1920
        end
        
        function setThreshold(obj, threshold)
            for i = 1:4
                write1ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, obj.DXL_ID(i), 24, threshold);
            end
        end
        
        function set_speed_arm(obj,speed, acc)
            for i=1:4
                write1ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, obj.DXL_ID(i), 10, 4);
                write4ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, obj.DXL_ID(i), 112, speed);
                write4ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, obj.DXL_ID(i), 108, acc);
            end
        end
        function set_arm_speed_mode(obj,speed, acc)
            for i=1:4
                write1ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, obj.DXL_ID(i), 10, 0);
                write4ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, obj.DXL_ID(i), 112, speed);
                write4ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, obj.DXL_ID(i), 108, acc);
            end
        end
        function set_speed_gripper(obj,speed)
            write1ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, obj.DXL_ID(5), 10, 0);
            write4ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, obj.DXL_ID(5), 112, speed);
            write4ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, obj.DXL_ID(5), 108, 1);
        end
        function move_servo(obj, index, val)
            % sends a specific value directly to a servo, between 0 and 4096, should be used for controlling the gripper
            write4ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, obj.DXL_ID(index), obj.ADDR_PRO_GOAL_POSITION, val);   
            while 1
                dxl_present_position = read4ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, obj.DXL_ID(index), obj.ADDR_PRO_PRESENT_POSITION); 
                not_correct_position = abs(dxl_present_position - val) > obj.max_angle_error;
                if not_correct_position == false
                    break;
                end
            end
        end

        function move_to_positions(obj, positions)
            % moves the arm gripper to a series of positions,
            % positions should be an array of arrays that each have 4 values, the x y z oordinate
            % and then the angle in radians of the gripper
            
            % !! 
            % set speed or time mode depending on positions length
            % !!


            % Try 

            len = size(positions);


            % set speed
%             with DRIVE = 4 and error = 50
            if len == 1
                obj.set_arm_speed_mode(35,5);
                adjust = true;
            else
                adjust = true;
%                 obj.set_speed_arm(1000,500);
            end
            % with drive = 0 and error < 20
%             if len == 1
%                 obj.set_arm_speed_mode(35,5);
%             else
%                 obj.set_speed_arm(35,5);
%             end

            servo_vals = zeros(len(1),4);
            for i=1:len(1)
                servo_vals(i,:) = obj.robot_model.servo_vals(positions(i, 1:3),positions(i,4));
            end
                
            for i=1:len(1)
%                 servo_vals = obj.robot_model.servo_vals(positions(i, 1:3),positions(i,4))
%                 obj.move_servo_to_val(servo_vals, adjust);

                if (i-1) / len(1) < 0.2
                    obj.move_servo_to_val(servo_vals(i,:), adjust, 1);
                elseif (i) / len(1) > 0.8
                    obj.move_servo_to_val(servo_vals(i,:), adjust, 1);
                else
                    obj.move_servo_to_val(servo_vals(i,:), adjust, 0);
                end

                % if i == 1
                %     obj.move_servo_to_val(servo_vals(i,:), adjust, 1);
                % elseif i == len(1)
                %     obj.move_servo_to_val(servo_vals(i,:), adjust, 1);
                % else
                %     obj.move_servo_to_val(servo_vals(i,:), adjust, 0);
                % end
            end
        end

        function pos = get_current_position(obj)
            % returns the current position and gripper angle
            servo_vals = zeros(4);
            for i=1:4
                servo_vals(i) = read4ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, obj.DXL_ID(i), obj.ADDR_PRO_PRESENT_POSITION); 
            end
            pos = obj.robot_model.current_position(servo_vals);
        end

        function move_servo_to_val(obj, servo_vals, adjust, correct)
            % if adjust == false
            %     start_time = tic;
            %     len =length(servo_vals);
            %     for i=1:len
            %         write4ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, obj.DXL_ID(i), obj.ADDR_PRO_GOAL_POSITION, servo_vals(i));
            %     end
            %     while 1
            %         if toc(start_time) > 1.5
            %             break
            %         end
            %         test = false
            %         len = length(servo_vals);
            %         for i=1:len
            %             % check if not at correct position and if so continue
            %             dxl_present_position = read4ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, obj.DXL_ID(i), obj.ADDR_PRO_PRESENT_POSITION); 
            %             correct_position = abs(dxl_present_position - servo_vals(i)) < obj.max_angle_error;
            %             if correct_position == false
            %                 test = true
            %             end
            %             %write4ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, obj.DXL_ID(i), obj.ADDR_PRO_GOAL_POSITION, servo_vals(i));
            %         end 
            %         if test == false
            %             break
            %         end
            %     end
            % else
                max_error = 10;
                if correct == false
                    max_error = 100;
                end
                count = 0;
                current_servo_vals = zeros(1,4);
                current_goal = servo_vals;
                for i=1:4
                    current_servo_vals(i) = read4ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, obj.DXL_ID(i), obj.ADDR_PRO_PRESENT_POSITION); 
                end
                diff = abs(norm(current_servo_vals-servo_vals));
                if correct
                    max_time = diff / 800
                else
                    max_time = diff / 800
                end
                start_time = tic;
                len = length(servo_vals);
                for i=1:len
                    write4ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, obj.DXL_ID(i), obj.ADDR_PRO_GOAL_POSITION, servo_vals(i));
                end
                while 1
                    if toc(start_time) > max_time
                        for i=1:4
                            current_servo_vals(i) = read4ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, obj.DXL_ID(i), obj.ADDR_PRO_PRESENT_POSITION); 
                        end
                        diff = abs(norm(current_servo_vals-servo_vals));
                        max_time = diff / 800
                        if count > 2
                            for i=1:4
                                dxl_present_position = read4ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, obj.DXL_ID(i), obj.ADDR_PRO_PRESENT_POSITION); 
                                write4ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, obj.DXL_ID(i), obj.ADDR_PRO_GOAL_POSITION, dxl_present_position);
                            end
                            break
                        end
                        if correct == false
                            for i=1:4
                                dxl_present_position = read4ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, obj.DXL_ID(i), obj.ADDR_PRO_PRESENT_POSITION); 
                                write4ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, obj.DXL_ID(i), obj.ADDR_PRO_GOAL_POSITION, dxl_present_position);
                            end
                            break
                        end
                        count = count + 1;
                        start_time = tic;
                        for i =1:4
                            dxl_present_position = read4ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, obj.DXL_ID(i), obj.ADDR_PRO_PRESENT_POSITION); 
                            current_goal(i) = dxl_present_position + 1.5*(servo_vals(i) - dxl_present_position);
                            if abs(dxl_present_position - servo_vals(i)) > max_error
                                write4ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, obj.DXL_ID(i), obj.ADDR_PRO_GOAL_POSITION, current_goal(i));
                            end
                        end
                        
                    end
                    test = false
                    len = length(servo_vals);
                    for i=1:len
                        % check if not at correct position and if so continue
                        dxl_present_position = read4ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, obj.DXL_ID(i), obj.ADDR_PRO_PRESENT_POSITION); 
                        correct_position = abs(dxl_present_position - servo_vals(i)) < max_error;
                        if correct_position == false
                            test = true
                        end
                        %write4ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, obj.DXL_ID(i), obj.ADDR_PRO_GOAL_POSITION, servo_vals(i));
                    end 
                    if test == false
                        if correct == false
                        for i=1:4
                            dxl_present_position = read4ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, obj.DXL_ID(i), obj.ADDR_PRO_PRESENT_POSITION); 
                            write4ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, obj.DXL_ID(i), obj.ADDR_PRO_GOAL_POSITION, dxl_present_position);
                        end
                        end
                        break
                    end
                % end
            end
        end

        function control_mode_setup(obj)
            % Put actuator into Position Control Mode
            write1ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, obj.DXL_ID1, obj.ADDR_PRO_OPERATING_MODE, 3);
            write1ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, obj.DXL_ID2, obj.ADDR_PRO_OPERATING_MODE, 3);
            write1ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, obj.DXL_ID3, obj.ADDR_PRO_OPERATING_MODE, 3);
            write1ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, obj.DXL_ID4, obj.ADDR_PRO_OPERATING_MODE, 3);
            write1ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, obj.DXL_ID5, obj.ADDR_PRO_OPERATING_MODE, 3);

            % set actuator movement speed
            write4ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, obj.DXL_ID1, obj.ADDR_PROFILE_VELOCITY, obj.SPEED_VAL);
            write4ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, obj.DXL_ID2, obj.ADDR_PROFILE_VELOCITY, obj.SPEED_VAL);
            write4ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, obj.DXL_ID3, obj.ADDR_PROFILE_VELOCITY, obj.SPEED_VAL);
            write4ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, obj.DXL_ID4, obj.ADDR_PROFILE_VELOCITY, obj.SPEED_VAL);
            write4ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, obj.DXL_ID5, obj.ADDR_PROFILE_VELOCITY, obj.SPEED_VAL);

            % enable dynamixle torque
            write1ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, obj.DXL_ID1, obj.ADDR_PRO_TORQUE_ENABLE, 1);
            write1ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, obj.DXL_ID2, obj.ADDR_PRO_TORQUE_ENABLE, 1);
            write1ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, obj.DXL_ID3, obj.ADDR_PRO_TORQUE_ENABLE, 1);
            write1ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, obj.DXL_ID4, obj.ADDR_PRO_TORQUE_ENABLE, 1);
            write1ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, obj.DXL_ID5, obj.ADDR_PRO_TORQUE_ENABLE, 1);

            dxl_comm_result = getLastTxRxResult(obj.port_num, obj.PROTOCOL_VERSION);
            dxl_error = getLastRxPacketError(obj.port_num, obj.PROTOCOL_VERSION);

            if dxl_comm_result ~= obj.COMM_SUCCESS
                fprintf('%s\n', getTxRxResult(obj.PROTOCOL_VERSION, dxl_comm_result));
            elseif dxl_error ~= 0
                fprintf('%s\n', getRxPacketError(obj.PROTOCOL_VERSION, dxl_error));
            else
                fprintf('Dynamixel has been successfully connected \n');
            end
            
            obj.setThreshold(10);
            for i=1:4
                write1ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, obj.DXL_ID(i), 10, 4);
                write4ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, obj.DXL_ID(i), 112, 1000);
                write4ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, obj.DXL_ID(i), 108, 500);
            end
        end

        function close(obj)
            % Close port
            closePort(obj.port_num);
            fprintf('Port Closed \n');
            clear all;
            % Unload Library
            %unloadlibrary(obj.lib_name);
        end

        function init(obj)

            obj.lib_name = '';

            if strcmp(computer, 'PCWIN')
                obj.lib_name = 'dxl_x86_c';
            elseif strcmp(computer, 'PCWIN64')
                obj.lib_name = 'dxl_x64_c';
            elseif strcmp(computer, 'GLNX86')
                obj.lib_name = 'libdxl_x86_c';
            elseif strcmp(computer, 'GLNXA64')
                obj.lib_name = 'libdxl_x64_c';
            elseif strcmp(computer, 'MACI64')
                obj.lib_name = 'libdxl_mac_c';
            end

            % Load Libraries
            if ~libisloaded(obj.lib_name)
                [~, ~] = loadlibrary(obj.lib_name, 'dynamixel_sdk.h', 'addheader', 'port_handler.h', 'addheader', 'packet_handler.h');
            end
                        
            % Initialize PortHandler Structs
            % Set the port path
            % Get methods and members of PortHandlerLinux or PortHandlerWindows
            obj.port_num = portHandler(obj.DEVICENAME);

            % Initialize PacketHandler Structs
            packetHandler();
            % Open port
            if (openPort(obj.port_num))
                fprintf('Port Open\n');
            else
                unloadlibrary(obj.lib_name);
                fprintf('Failed to open the port\n');
                input('Press any key to terminate...\n');
                return;
            end


            % Set port baudrate
            if (setBaudRate(obj.port_num, obj.BAUDRATE))
                fprintf('Baudrate Set\n');
            else
                unloadlibrary(obj.lib_name);
                fprintf('Failed to change the baudrate!\n');
                input('Press any key to terminate...\n');
                return;
            end

            obj.control_mode_setup()

        end
        

                %this function takes the current and a goal position 
        %and split the trajectory in smaller unevenly-distributed steps 

        function pos_array = trajectory(obj,current_pos, final_pos)
            obj.set_speed_arm(1000,500);
            angle = final_pos(4);
            current_pos = current_pos(1:3);
            final_pos = final_pos(1:3);
            delta_pos = final_pos-current_pos;
            N = round(norm(delta_pos));
            degree = 0.3;
            if mod(N,2) == 1
                N = N+1;
            end
            trans = linspace(-1, 1, N);
            trans = abs(trans.^(-degree));
            trans = trans./sum(trans, 'all');
            trans = transpose(cumsum(trans)) * delta_pos;
            positions = current_pos + trans;
            angle_column = zeros(N,1) + angle;
            pos_array = [positions angle_column];

            %N = 10; %use even number, could be adjusted depending on the distance
            % degree = 1; %find optimal distribution based on robot tests
            % distance = final_pos-current_pos;

            % N = round(norm(distance));
            % if mod(N,2) == 1
            %     N = N+1;
            % end

            % x_trans = linspace(-1, 1, N);
            % c = abs(x_trans.^(-degree));
            % y_trans = c./sum(c, 'all');

            % yx = y_trans.*distance(:,1);
            % yy = y_trans.*distance(:,2);
            % yz = y_trans.*distance(:,3);

            % pos_array =[];
            % a = current_pos(:,1);
            % b = current_pos(:,2);
            % c = current_pos(:,3);
            % for i=1:N
            %     pos_array(i,1) = a+yx(i);
            %     a = pos_array(i,1);
            %     pos_array(i,2) = b+yy(i);
            %     b = pos_array(i,2);
            %     pos_array(i,3) = c+yz(i);
            %     c = pos_array(i,3);
            %     pos_array(i,4) = final_pos(4);
            % end

            %figure()
            %plot(pos_array(1,:),pos_array(2,:), 'o')
        end
        
        function pos_array = trajectory_angle(obj,current_pos, final_pos)
            
            
            obj.set_speed_arm(1000,10);
            if final_pos(4) > pi
                final_pos(4) = final_pos(4) - 2*pi;
            end
            if current_pos(4) > pi
                current_pos(4) = current_pos(4) - 2*pi;
            end
            N = round(norm(final_pos(4) - current_pos(4)) * 10 / pi);
            positions = zeros(N,4);
            angles = linspace(current_pos(4), final_pos(4), N);
            for i = 1:N
                positions(i,1:4) = [current_pos(1), current_pos(2), current_pos(3), angles(i)];
            end
            pos_array = positions;
        end
    end
end

