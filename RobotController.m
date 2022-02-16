classdef RobotController
    properties
        robot_model = RobotModel()

        port_num = 0
        lib_name = ''
        max_angle_error = 3

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
        BAUDRATE                    = 1000000;
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
        function move_servo(obj, index, val)
            write4ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, obj.DXL_ID(index), obj.ADDR_PRO_GOAL_POSITION, val);   
        end
        function move_to_positions(obj, positions)
            % Iterate through psoition, create servo vals and move to each one
            len =length(positions);
            for i=1:len
                % check the current position if the change in position is too large create an array of smaller moves
                servo_vals = obj.robot_model.servo_vals(positions(i, 1:3),positions(i,4))
                obj.move_servo_to_val(servo_vals);
            end
        end

        function move_servo_to_val(obj, servo_vals)
            len =length(servo_vals);
            for i=1:len
                write4ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, obj.DXL_ID(i), obj.ADDR_PRO_GOAL_POSITION, servo_vals(i));
            end
            while 1
                len =length(servo_vals);
                for i=1:len
                    % check if not at correct position and if so continue
                    dxl_present_position = read4ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, obj.DXL_ID(i), obj.ADDR_PRO_PRESENT_POSITION); 
                    correct_position = abs(dxl_present_position - servo_vals(i)) < obj.max_angle_error;
                    if correct_position == false
                        continue
                    end
                    write4ByteTxRx(obj.port_num, obj.PROTOCOL_VERSION, obj.DXL_ID(i), obj.ADDR_PRO_GOAL_POSITION, servo_vals(i));
                end 
                break
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
        end

        function close(obj)
            % Close port
            closePort(obj.port_num);
            fprintf('Port Closed \n');
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

        end
    end
end

