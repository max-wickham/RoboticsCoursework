%model = KinematicModel();
%model.positions(10,0,10,0)

controller = Controller();
controller.init();
controller.control_mode_setup();
% controller.move_servo(5,2000);
% % controller.move_servo(5,2430);
%%controller.set_speed_arm(1000,100);
%controller.set_speed_gripper();
%try
    pos = trajectory([10,10,10,-pi/2],[10,0,10,-pi/2]);
    controller.move_to_positions(pos);
    %%controller.set_speed_arm(3000, 100);
    %%controller.move_to_positions(pos);
    % controller.move_servo(5,2430);
    % controller.move_to_positions([10,10,10, -pi/2; 10,0,10,-pi/2 ]);
    % controller.move_servo(5,2000);
    %controller.move_servo(2, 3072);
    %model2 = KinematicModel();
    %model2.angles(20,0,20,0)
    %model = RobotModel();
    %model.servo_vals([20,0,20],0)
    controller.close();
    %catch ME
        controller.close();

%end 
%trajectory([10,10,10, pi/2],[10,0,10, pi/2])

% polar_to_cartesian([0,0.1,0.2], 5) + [0, 2]
